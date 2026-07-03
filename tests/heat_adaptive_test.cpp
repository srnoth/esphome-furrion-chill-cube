// Phase 2 adaptive equilibrium-gear controller test (HEAT mode)
// ---------------------------------------------------------------------------
// Validates the heat-side integral added 2026-07-03 — the SIGN-MIRROR of the cool
// controller. Heat demand = -diff (room below target = need heat); bias_h winds POSITIVE
// when persistently cold; eff = real_diff - bias_h (more negative → higher heat gear).
// Mirrors adaptive_heat_eff_diff_ + run_heat_mode_'s switch in furrion_chill_cube.cpp.
//
// Heat can't be field-tested outside heating season, so these unit-level invariants are the
// primary guard: a future edit that inverts a sign or moves the pinned stop will trip here.
//
// Build: g++ -std=c++17 -O2 -o tests/heat_adaptive_test tests/heat_adaptive_test.cpp && ./tests/heat_adaptive_test
// ---------------------------------------------------------------------------

#include <cstdio>
#include <cstdint>
#include <cmath>
#include <algorithm>

// ── Current heat ladder constants (2026-07-03 uniform redesign) ─────────────────
static const uint32_t HOLD_MS[] = {0, 180000, 180000, 300000, 300000, 600000};
static const float H_UP_01 = -0.35f, H_UP_12 = -0.60f, H_UP_23 = -0.85f;
static const float H_DN_10 = -0.15f, H_DN_21 = -0.40f, H_DN_32 = -0.65f;

// ── Adaptive constants (mirror the cpp) ─────────────────────────────────────────
static const float ADAPT_KI = 0.06f, ADAPT_BIAS_C_MAX = 2.0f, ADAPT_DEADBAND_C = 0.15f;
static const float ADAPT_DECAY_TAU_MIN = 30.0f, ADAPT_DT_CAP_MIN = 5.0f;
static const float ADAPT_UPSHIFT_DRIFT_MIN_CPM = 0.0f;

// ── Heat adaptive state (mirrors adaptive_heat_eff_diff_) ────────────────────────
struct HeatAdaptive {
    bool enable = false;
    float bias_h = 0.0f;
    uint32_t last_advance = 0;
    float eff_up_diff = 0.0f;   // eff for UPSHIFT decisions (bias gated out while not cooling)

    // room_drift_cpm: room dT/dt (°C/min). For heat, "cooling" (drift < -threshold) is the
    // gear-raising-allowed condition — the mirror of cool's "warming".
    float eff_diff(float real_diff, int heat_gear, bool kickstart, uint32_t now,
                   uint32_t time_in_gear, float room_drift_cpm, bool drift_fresh) {
        if (!enable) { last_advance = now; eff_up_diff = real_diff; return real_diff; }
        float dt_min = 0.0f;
        if (last_advance != 0) {
            dt_min = (now - last_advance) / 60000.0f;
            if (dt_min < 0) dt_min = 0;
            if (dt_min > ADAPT_DT_CAP_MIN) dt_min = ADAPT_DT_CAP_MIN;
        }
        last_advance = now;
        float e = -real_diff;                                   // + = too cold = need heat
        if (e > -ADAPT_DEADBAND_C && e < ADAPT_DEADBAND_C) e = 0.0f;
        bool idle = (heat_gear <= 0);
        bool upshift_held = (heat_gear < 3) && (time_in_gear < HOLD_MS[heat_gear + 1]);
        bool cooling = std::isnan(room_drift_cpm) ||
                       (room_drift_cpm < -ADAPT_UPSHIFT_DRIFT_MIN_CPM && drift_fresh);
        bool block_up = (e > 0.0f) && (heat_gear >= 3 || upshift_held || !cooling);
        bool freeze = kickstart || block_up;
        if (idle) {
            if (dt_min > 0) bias_h *= std::exp(-dt_min / ADAPT_DECAY_TAU_MIN);
        } else if (!freeze && dt_min > 0) {
            bias_h += ADAPT_KI * e * dt_min;
            bias_h = std::clamp(bias_h, -ADAPT_BIAS_C_MAX, ADAPT_BIAS_C_MAX);
        }
        float eff = real_diff - bias_h;                         // more negative → higher heat gear
        eff_up_diff = cooling ? eff : std::max(eff, real_diff); // fmaxf: gate can only SUPPRESS
        return eff;
    }
};

// Heat gear selection mirroring run_heat_mode_'s switch: upshift on rate-gated up_diff,
// modulation downshift on eff_diff, and the pong-critical 1→0 STOP on REAL diff.
int select_heat_gear(int gear, float real_diff, float eff_diff, float up_diff, uint32_t time_in_gear) {
    int new_gear = gear;
    auto can_up = [&](int tg) { return time_in_gear >= HOLD_MS[tg]; };
    switch (gear) {
        case 1:
            if (can_up(2) && up_diff < H_UP_12)  new_gear = 2;
            else if (real_diff > H_DN_10)        new_gear = 0;   // STOP on REAL diff (pinned)
            break;
        case 2:
            if (can_up(3) && up_diff < H_UP_23)  new_gear = 3;
            else if (eff_diff > H_DN_21)         new_gear = 1;
            break;
        case 3:
            if (eff_diff > H_DN_32)              new_gear = 2;
            break;
    }
    return new_gear;
}

// ── Test infra ──────────────────────────────────────────────────────────────────
int run=0, pass=0, fail=0;
#define CHECK(c,msg) do{ run++; if(c){pass++; printf("  PASS  %s\n",msg);} \
    else{fail++; printf("  FAIL  %s\n",msg);} }while(0)

// ── Tests ─────────────────────────────────────────────────────────────────────
void test_disabled_identical() {
    printf("\n=== Heat adaptive disabled == static ladder (regression) ===\n");
    HeatAdaptive off; off.enable = false;
    bool identical = true;
    for (int i = 0; i < 50; i++) {
        float d = -2.0f + i * 0.08f;
        float eff = off.eff_diff(d, 2, false, (uint32_t)i * 60000u, 999999, NAN, true);
        if (eff != d) identical = false;
    }
    CHECK(identical, "disabled: eff_diff == real_diff for every input (static heat ladder unchanged)");
    CHECK(off.bias_h == 0.0f, "disabled: bias_h never moves");
}

void test_sign_cold_raises_gear() {
    printf("\n=== Sign: sustained cold + cooling winds bias_h POSITIVE → higher gear ===\n");
    HeatAdaptive a; a.enable = true;
    float eff = 0;
    // room 0.6°C below setpoint, gear 1, room still cooling (drift < 0, fresh), no hold block
    for (int m = 0; m < 40; m++)
        eff = a.eff_diff(-0.6f, 1, false, (uint32_t)(m+1)*60000u, 999999, -0.05f, true);
    CHECK(a.bias_h > 0.3f, "cold+cooling: bias_h winds positive");
    CHECK(eff < H_UP_12, "cold: eff_diff drops below 1→2 threshold (upshift warranted)");
    CHECK(a.eff_up_diff < H_UP_12, "cold+cooling: rate gate PERMITS the upshift (up_diff also clears)");
}

void test_antiwindup_rails() {
    printf("\n=== Anti-windup: freeze positive demand at max gear / when warming; unwind when warm ===\n");
    // At gear 3 (max heat), a still-cold room must NOT keep winding bias up.
    HeatAdaptive a; a.enable = true;
    a.eff_diff(-1.0f, 3, false, 60000u, 999999, -0.05f, true);   // prime last_advance
    float b0 = a.bias_h;
    for (int m = 2; m < 30; m++) a.eff_diff(-1.0f, 3, false, (uint32_t)m*60000u, 999999, -0.05f, true);
    CHECK(std::fabs(a.bias_h - b0) < 1e-6f, "gear 3 (max heat): positive accumulation frozen (no windup)");

    // Rate gate: cold but room WARMING (drift > 0) → upshift not warranted → freeze positive too.
    HeatAdaptive w; w.enable = true;
    w.eff_diff(-0.6f, 1, false, 60000u, 999999, +0.05f, true);
    float w0 = w.bias_h;
    for (int m = 2; m < 30; m++) w.eff_diff(-0.6f, 1, false, (uint32_t)m*60000u, 999999, +0.05f, true);
    CHECK(std::fabs(w.bias_h - w0) < 1e-6f, "cold but warming: bias frozen (gate suppresses over-heat windup)");

    // A stale positive bias must UNWIND when the room goes warm (negative demand never frozen).
    HeatAdaptive u; u.enable = true; u.bias_h = 1.0f;
    u.eff_diff(+0.4f, 2, false, 60000u, 999999, +0.05f, true);
    for (int m = 2; m < 40; m++) u.eff_diff(+0.4f, 2, false, (uint32_t)m*60000u, 999999, +0.05f, true);
    CHECK(u.bias_h < 0.5f, "warm room: stale positive bias_h unwinds (not rail-frozen)");
}

void test_stop_on_real_diff() {
    printf("\n=== Pong safety: 1→0 STOP fires on REAL diff even with a large heat bias ===\n");
    // Pong scenario: heat has driven the room UP to just below setpoint and it's still warming
    // (drift ≥ 0 → NOT cooling). A large stale positive bias_h wants more heat. The rate gate must
    // suppress the spurious upshift (up_diff = fmaxf(eff, real) = real), and the 1→0 stop must fire
    // on REAL diff — if it used eff_diff (= real - bias, deeply negative) the unit would never stop
    // and would overshoot past the mode-switch offset into a heat→cool pong.
    HeatAdaptive a; a.enable = true; a.bias_h = 1.5f;
    float real_diff = -0.10f;                       // room 0.10°C below setpoint: ABOVE H_DN_10=-0.15
    float eff = a.eff_diff(real_diff, 1, false, 60000u, 999999, +0.05f, true);  // warming, not cooling
    int ng = select_heat_gear(1, real_diff, eff, a.eff_up_diff, 999999);
    CHECK(eff < H_DN_10, "sanity: eff_diff is far below the stop threshold (would block stop if used)");
    CHECK(a.eff_up_diff >= real_diff - 1e-6f, "not-cooling: rate gate clamps up_diff to real (suppresses bias upshift)");
    CHECK(ng == 0, "1→0 stops on REAL diff (real -0.10 > H_DN_10 -0.15) despite bias wanting more heat");
}

void test_no_cascade() {
    printf("\n=== No cascade: one downshift from gear 3 lands in gear 2, not through to 1 ===\n");
    // Simulate a strong heat pulse overshooting warm, then handing down. With bias ~0 the down-rail
    // spacing (0.25°C) must contain a single-pass coast. Drive real_diff up past H_DN_32 by a small
    // overshoot and confirm gear 3→2, and that gear 2 then HOLDS (does not immediately trip 2→1).
    HeatAdaptive a; a.enable = true;   // bias ~0 (fresh)
    float real_diff = -0.55f;          // above H_DN_32(-0.65): gear 3 should shed to 2
    float eff = a.eff_diff(real_diff, 3, false, 60000u, 999999, +0.05f, true);
    int ng = select_heat_gear(3, real_diff, eff, a.eff_up_diff, 999999);
    CHECK(ng == 2, "gear 3 sheds to gear 2 when room warms past H_DN_32");
    // Now at gear 2, same-ish temp (coast) — must NOT immediately shed to 1 (needs eff > H_DN_21=-0.40)
    float eff2 = a.eff_diff(-0.55f, 2, false, 120000u, 0, +0.05f, true);
    int ng2 = select_heat_gear(2, -0.55f, eff2, a.eff_up_diff, 0);
    CHECK(ng2 == 2, "gear 2 HOLDS after the 3→2 shed (down-rail spacing contains the coast — no cascade)");
}

int main() {
    printf("Heat adaptive controller tests\n==============================\n");
    test_disabled_identical();
    test_sign_cold_raises_gear();
    test_antiwindup_rails();
    test_stop_on_real_diff();
    test_no_cascade();
    printf("\n========================================\n");
    printf("Results: %d/%d passed, %d failed\n", pass, run, fail);
    printf("========================================\n");
    return fail == 0 ? 0 : 1;
}
