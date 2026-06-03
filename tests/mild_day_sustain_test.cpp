// Mild-day sustain-gear regression test (2026-06-03)
// ---------------------------------------------------------------------------
// Guards the Phase-1 fix that makes gear 2 (not gear 1) the cool-mode sustain
// floor on mild days, so the controller settles into a quiet 2<->3 duty cycle
// instead of churning 1<->2<->3<->4.
//
// Grounded in measured data from a real mild cool cycle on 2026-06-03
// (camper HA, target 68F, outside 72-78F, 11:46-18:30 ET):
//   - 38 gear changes in 6.6 h; inside-temp sawtooth 66.0-69.7F (3.7F swing).
//   - Per-gear room drift rate (F/min): g1 +0.065, g2 +0.047, g3 -0.061,
//     g4 -0.093. The equilibrium load sits BETWEEN gear 2 (gently under-cools)
//     and gear 3 (gently over-cools) -> the natural steady state is a 2<->3
//     toggle. g1 and g2 drift nearly identically (the "1&2 collapse" effect).
//   - Pathology: gear 3's thermal coast carried the room ~0.20C past the 3->2
//     downshift and tripped the old 2->1 threshold (C_DN_21 = -0.10C, only
//     0.09F below the 3->2 point). The resulting gear-1 dip under-cools, the
//     room makes a long HOLD_MS-gated climb back, and overshoots into a gear-4
//     spike, which over-cools and slams back down. Vicious cycle.
//
// Fix under test: C_DN_21 -0.10 -> -0.40C, C_DN_10 -0.50 -> -0.55C.
//
// Build: g++ -std=c++17 -o tests/mild_test tests/mild_day_sustain_test.cpp && ./tests/mild_test
// ---------------------------------------------------------------------------

#include <cstdio>
#include <cstdint>
#include <cmath>
#include <algorithm>

// ============================================================
// Cooling constants — current production values (post-2026-06-03).
// C_DN_21 / C_DN_10 are passed in per-scenario so the same logic can be
// A/B-compared against the OLD values to prove the regression.
// ============================================================

static const uint32_t HOLD_MS[] = {0, 180000, 180000, 300000, 300000, 600000};

static const float C_UP_01 =  0.15f;
static const float C_UP_12 =  0.25f;
static const float C_UP_23 =  0.40f;
static const float C_UP_34 =  0.85f;
static const float C_UP_45 =  1.00f;
static const float C_DN_54 =  0.45f;
static const float C_DN_43 =  0.40f;
static const float C_DN_32 = -0.05f;
static const float C_IDLE  = -0.15f;

// Values under test (NEW) vs the regression baseline (OLD).
struct CoolTune { float c_dn_21; float c_dn_10; };
static const CoolTune NEW_TUNE = {-0.40f, -0.55f};   // Phase-1 fix
static const CoolTune OLD_TUNE = {-0.10f, -0.50f};   // pre-fix baseline

float f_to_c(float f) { return (f - 32.0f) * (5.0f / 9.0f); }
float c_to_f_delta(float c) { return c * (9.0f / 5.0f); }

// ============================================================
// Cool gear selection — faithful copy of the steady-state cool path from
// furrion_chill_cube.cpp run_gear_controller_() (switch on gear, gear>=0,
// non-user-input). The -1/user-input/first-compute branches are not exercised
// by this steady-state mild-day scenario and are omitted for clarity.
// ============================================================

int select_cool_gear(int gear, float diff, uint32_t time_in_gear, const CoolTune &t) {
    int new_gear = gear;
    auto can_upshift_to = [&](int tg) -> bool { return time_in_gear >= HOLD_MS[tg]; };
    switch (gear) {
        case 0:
            if (can_upshift_to(1) && diff > C_UP_01) new_gear = 1;
            else if (diff < C_IDLE)                  new_gear = -1;
            break;
        case 1:
            if (can_upshift_to(2) && diff > C_UP_12)  new_gear = 2;
            else if (diff < t.c_dn_10)                new_gear = 0;
            break;
        case 2:
            if (can_upshift_to(3) && diff > C_UP_23)  new_gear = 3;
            else if (diff < t.c_dn_21)                new_gear = 1;   // <-- the fix
            break;
        case 3:
            if (can_upshift_to(4) && diff > C_UP_34)  new_gear = 4;
            else if (diff < C_DN_32)                  new_gear = 2;
            break;
        case 4:
            if (can_upshift_to(5) && diff > C_UP_45)  new_gear = 5;
            else if (diff < C_DN_43)                  new_gear = 3;
            break;
        case 5:
            if (diff < C_DN_54)                       new_gear = 4;
            break;
    }
    return new_gear;
}

// ============================================================
// Thermal plant — first-order-lag model calibrated to the 2026-06-03 data.
//
// Steady-state room drift per gear (F/min) reproduces the MEASURED net rates;
// a first-order lag on compressor output (tau ~4 min) reproduces the thermal
// COAST that carries the room past a downshift threshold (the observed ~0.20C
// over-travel that triggered the old 2->1 collapse). Envelope warming is the
// idle drift; per-gear compressor contribution = measured_net - envelope.
// ============================================================

struct Plant {
    static constexpr float ENVELOPE_WARM = 0.120f;   // F/min, compressor off (mild day)
    static constexpr float TAU_MIN = 4.0f;           // compressor-output lag
    // measured net drift (F/min): idle/g0 ~ +envelope; g5 estimated
    float net_drift(int g) const {
        switch (g) { case 1: return 0.065f; case 2: return 0.047f;
                     case 3: return -0.061f; case 4: return -0.093f;
                     case 5: return -0.140f; default: return ENVELOPE_WARM; }
    }
    float comp_target(int g) const { return (g <= 0) ? 0.0f : net_drift(g) - ENVELOPE_WARM; }
    float comp_now = 0.0f;   // current (lagged) compressor contribution, F/min
    void step(int gear) { // advance 1 minute
        float tc = comp_target(gear);
        comp_now += (tc - comp_now) * (1.0f - std::exp(-1.0f / TAU_MIN));
    }
    float room_drift() const { return ENVELOPE_WARM + comp_now; }
};

// ============================================================
// Closed-loop simulation over `minutes`, returns summary stats.
// ============================================================

struct SimResult {
    int changes = 0;
    int gear1_entries = 0;
    int gear4plus_entries = 0;
    float tmin = 1e9f, tmax = -1e9f;
};

SimResult simulate(const CoolTune &t, int minutes, float target_f, bool verbose) {
    SimResult r;
    Plant plant;
    float room_f = target_f;          // start at setpoint
    int gear = 2;                      // start in the sustain gear
    plant.comp_now = plant.comp_target(2);
    uint32_t time_in_gear = 999999;    // settled
    if (verbose) printf("  t(min) gear  room   diff(C)\n");
    for (int m = 0; m < minutes; m++) {
        float diff_c = f_to_c(room_f) - f_to_c(target_f);
        int ng = select_cool_gear(gear, diff_c, time_in_gear, t);
        if (ng != gear) {
            r.changes++;
            if (ng == 1) r.gear1_entries++;
            if (ng >= 4) r.gear4plus_entries++;
            gear = ng;
            time_in_gear = 0;
            if (verbose) printf("  %4d   ->%d  %.2f  %+.3f\n", m, gear, room_f, diff_c);
        } else {
            time_in_gear += 60000;
        }
        plant.step(gear);
        room_f += plant.room_drift();          // 1-minute advance
        if (m > 15) { r.tmin = std::min(r.tmin, room_f); r.tmax = std::max(r.tmax, room_f); }
    }
    return r;
}

// ============================================================
// Test infra
// ============================================================

int tests_run = 0, tests_passed = 0, tests_failed = 0;
#define CHECK(cond, msg) do { tests_run++; if (cond) { tests_passed++; \
    printf("  PASS  %s\n", msg); } else { tests_failed++; \
    printf("  FAIL  %s\n", msg); } } while(0)

// ============================================================
// Tests
// ============================================================

// 1) Unit-level: the crisp regression boundary. At gear 2, the deepest coast
//    observed on 2026-06-03 was diff = -0.20C. The fix must HOLD gear 2 there;
//    the old threshold dropped to gear 1.
void test_sustain_floor_boundary() {
    printf("\n=== Gear-2 sustain floor: -0.20C undershoot holds (was a 2->1 drop) ===\n");
    float diff = -0.20f;  // deepest measured coast-down past the 3->2 point
    int g_new = select_cool_gear(2, diff, 999999, NEW_TUNE);
    int g_old = select_cool_gear(2, diff, 999999, OLD_TUNE);
    CHECK(g_new == 2, "NEW: gear 2 holds at diff=-0.20C (no collapse to gear 1)");
    CHECK(g_old == 1, "OLD: gear 2 collapsed to gear 1 at diff=-0.20C (the bug)");

    // Gear 2 still yields to a GENUINE deep undershoot (room truly cold).
    int g_deep = select_cool_gear(2, -0.45f, 999999, NEW_TUNE);
    CHECK(g_deep == 1, "NEW: a genuine -0.45C undershoot still steps gear 2 -> 1");

    // The 2<->3 toggle band is untouched by the fix.
    CHECK(select_cool_gear(2, 0.50f, 999999, NEW_TUNE) == 3, "gear 2 -> 3 upshift unchanged (diff 0.50C)");
    CHECK(select_cool_gear(3, -0.10f, 999999, NEW_TUNE) == 2, "gear 3 -> 2 downshift unchanged (diff -0.10C)");
}

// 2) Down-ladder must stay strictly monotonic after the change.
void test_down_ladder_monotonic() {
    printf("\n=== Down-ladder monotonicity ===\n");
    bool mono = (C_DN_54 > C_DN_43) && (C_DN_43 > C_DN_32) &&
                (C_DN_32 > NEW_TUNE.c_dn_21) && (NEW_TUNE.c_dn_21 > NEW_TUNE.c_dn_10);
    CHECK(mono, "0.45 > 0.40 > -0.05 > -0.40 > -0.55 strictly decreasing");
}

// 3) Closed-loop A/B: the fix must turn the mild-day churn into a clean 2<->3
//    cycle — no gear-1 dips, no gear-4 spikes, far fewer changes, smaller swing.
void test_closed_loop_ab() {
    printf("\n=== Closed-loop mild-day A/B (target 68F, 6h) ===\n");
    SimResult old_r = simulate(OLD_TUNE, 360, 68.0f, false);
    SimResult new_r = simulate(NEW_TUNE, 360, 68.0f, false);
    printf("  OLD: %d changes, %d gear-1 dips, %d gear-4 spikes, swing %.2fF\n",
           old_r.changes, old_r.gear1_entries, old_r.gear4plus_entries, old_r.tmax - old_r.tmin);
    printf("  NEW: %d changes, %d gear-1 dips, %d gear-4 spikes, swing %.2fF\n",
           new_r.changes, new_r.gear1_entries, new_r.gear4plus_entries, new_r.tmax - new_r.tmin);

    CHECK(old_r.gear1_entries > 0, "OLD baseline reproduces the gear-1 dips");
    CHECK(new_r.gear1_entries == 0, "NEW: zero gear-1 dips on a mild day");
    CHECK(new_r.gear4plus_entries == 0, "NEW: zero gear-4 spikes on a mild day");
    CHECK(new_r.changes < old_r.changes, "NEW: fewer total gear changes than OLD");
    CHECK((new_r.tmax - new_r.tmin) <= (old_r.tmax - old_r.tmin) + 0.01f,
          "NEW: temperature swing no larger than OLD");
    CHECK((new_r.tmax - new_r.tmin) < 1.5f, "NEW: swing stays under 1.5F (tight regulation)");
}

int main() {
    test_sustain_floor_boundary();
    test_down_ladder_monotonic();
    test_closed_loop_ab();
    printf("\n========================================\n");
    printf("Results: %d/%d passed, %d failed\n", tests_passed, tests_run, tests_failed);
    printf("========================================\n");
    return tests_failed > 0 ? 1 : 0;
}
