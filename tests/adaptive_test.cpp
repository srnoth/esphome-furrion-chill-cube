// Phase 2 adaptive equilibrium-gear controller test (cool mode)
// ---------------------------------------------------------------------------
// Validates the adaptive layer that floats the cool ladder's operating point to the
// gear that sustains the current load (PI control). Mirrors the logic in
// furrion_chill_cube.cpp: eff_diff = diff + bias_c + fan_ff is used ONLY for active-gear
// switch cases 1-5; the integral advances with anti-windup; bias_c decays while idle.
// See PHASE2_ADAPTIVE_DESIGN.md.
//
// Build: g++ -std=c++17 -O2 -o tests/adaptive_test tests/adaptive_test.cpp && ./tests/adaptive_test
// ---------------------------------------------------------------------------

#include <cstdio>
#include <cstdint>
#include <cmath>
#include <algorithm>
#include <vector>

// ── Current cool ladder constants (post-2026-06-03) ─────────────────────────────
static const uint32_t HOLD_MS[] = {0, 180000, 180000, 300000, 300000, 600000};
static const float C_UP_01 =  0.15f, C_UP_12 = 0.25f, C_UP_23 = 0.40f, C_UP_34 = 0.85f, C_UP_45 = 1.00f;
static const float C_DN_54 =  0.45f, C_DN_43 = 0.40f, C_DN_32 = -0.05f, C_DN_21 = -0.40f, C_DN_10 = -0.55f;
static const float C_IDLE  = -0.15f;

// ── Adaptive constants (mirror the cpp) ─────────────────────────────────────────
static const float ADAPT_KI = 0.06f, ADAPT_BIAS_C_MAX = 2.0f, ADAPT_DEADBAND_C = 0.15f;
static const float ADAPT_DECAY_TAU_MIN = 30.0f, ADAPT_DT_CAP_MIN = 5.0f, GEAR_STEP_C = 0.85f;
static const uint32_t FAN_EDGE_FREEZE_MS = 180000;

float f_to_c(float f) { return (f - 32.0f) * (5.0f / 9.0f); }

// ── Adaptive state (mirrors the member vars) ────────────────────────────────────
struct Adaptive {
    bool enable = false;
    float bias_c = 0.0f;
    uint32_t last_advance = 0;
    uint32_t fan_changed_at = 0;
    bool fan_on = false;
    int fan_ff_gears = 1;

    // mirror of adaptive_cool_eff_diff_ — cool_gear is the CURRENT gear (pre-selection)
    float eff_diff(float real_diff, int cool_gear, bool kickstart, uint32_t now) {
        if (!enable) { last_advance = now; return real_diff; }
        float dt_min = 0.0f;
        if (last_advance != 0) {
            dt_min = (now - last_advance) / 60000.0f;
            if (dt_min < 0) dt_min = 0;
            if (dt_min > ADAPT_DT_CAP_MIN) dt_min = ADAPT_DT_CAP_MIN;
        }
        last_advance = now;
        float e = real_diff;
        if (e > -ADAPT_DEADBAND_C && e < ADAPT_DEADBAND_C) e = 0.0f;
        bool fan_edge_freeze = (now - fan_changed_at) < FAN_EDGE_FREEZE_MS;
        bool idle = (cool_gear <= 0);
        bool sat_high = (cool_gear >= 5 && e > 0.0f);
        bool sat_low = (cool_gear <= 1 && e < 0.0f);
        bool freeze = kickstart || fan_edge_freeze || sat_high || sat_low;
        if (idle) {
            if (dt_min > 0) bias_c *= std::exp(-dt_min / ADAPT_DECAY_TAU_MIN);
        } else if (!freeze && dt_min > 0) {
            bias_c += ADAPT_KI * e * dt_min;
            bias_c = std::clamp(bias_c, -ADAPT_BIAS_C_MAX, ADAPT_BIAS_C_MAX);
        }
        float ff_c = fan_on ? (fan_ff_gears * GEAR_STEP_C) : 0.0f;
        return real_diff + bias_c + ff_c;
    }
};

// Cool gear selection mirroring run_cool_mode_: eff_diff for active cases 1-5, real diff
// for fresh-start/idle. Steady-state path only (gear>=0, no user_input) — what the sim drives.
int select_cool_gear(int gear, float real_diff, float eff_diff, uint32_t time_in_gear) {
    int new_gear = gear;
    auto can_up = [&](int tg) { return time_in_gear >= HOLD_MS[tg]; };
    switch (gear) {
        case 0:
            if (can_up(1) && real_diff > C_UP_01) new_gear = 1;   // re-engage on REAL diff
            else if (real_diff < C_IDLE)          new_gear = -1;
            break;
        case 1:
            if (can_up(2) && eff_diff > C_UP_12)  new_gear = 2;
            else if (eff_diff < C_DN_10)          new_gear = 0;
            break;
        case 2:
            if (can_up(3) && eff_diff > C_UP_23)  new_gear = 3;
            else if (eff_diff < C_DN_21)          new_gear = 1;
            break;
        case 3:
            if (can_up(4) && eff_diff > C_UP_34)  new_gear = 4;
            else if (eff_diff < C_DN_32)          new_gear = 2;
            break;
        case 4:
            if (can_up(5) && eff_diff > C_UP_45)  new_gear = 5;
            else if (eff_diff < C_DN_43)          new_gear = 3;
            break;
        case 5:
            if (eff_diff < C_DN_54)               new_gear = 4;
            break;
    }
    return new_gear;
}

// ── Thermal plant: room drift = load - cooling(gear), with first-order output lag ───
// cooling[] are fixed per-gear capacities (°F/min); `load` (°F/min warming) sets the regime.
struct Plant {
    float cooling(int g) const {
        switch (g) { case 1: return 0.055f; case 2: return 0.073f; case 3: return 0.181f;
                     case 4: return 0.213f; case 5: return 0.300f; default: return 0.0f; }
    }
    static constexpr float TAU_MIN = 4.0f;
    float c_now = 0.0f;
    void step(int gear) {
        float t = cooling(gear);
        c_now += (t - c_now) * (1.0f - std::exp(-1.0f / TAU_MIN));
    }
    float drift(float load) const { return load - c_now; }
};

struct SimResult { int changes=0; float tmin=1e9f, tmax=-1e9f, tsum=0; int n=0;
                   float bias_end=0; std::vector<int> gear_hist;
                   // steady-state window (last third of the run) — excludes startup transient
                   int ss_changes=0; float ss_tmin=1e9f, ss_tmax=-1e9f, ss_tsum=0; int ss_n=0;
                   std::vector<int> ss_gear_hist; };

// Run a closed-loop cool sim. load_at(minute) returns the warming load.
template <typename LoadFn>
SimResult simulate(bool adaptive_on, LoadFn load_at, int minutes, float target_f, int fan_ff=0) {
    SimResult r; Plant plant; Adaptive a; a.enable = adaptive_on; a.fan_ff_gears = fan_ff;
    float room = target_f; int gear = 2; plant.c_now = plant.cooling(2);
    uint32_t time_in_gear = 999999; uint32_t now = 0;
    int ss_start = minutes * 2 / 3;          // steady-state = last third
    for (int m = 0; m < minutes; m++) {
        now = (uint32_t)m * 60000u;
        float diff_c = f_to_c(room) - f_to_c(target_f);
        float eff = a.eff_diff(diff_c, gear, /*kickstart=*/false, now);
        int ng = select_cool_gear(gear, diff_c, eff, time_in_gear);
        bool changed = (ng != gear);
        if (changed) { r.changes++; gear = ng; time_in_gear = 0; }
        else time_in_gear += 60000;
        if (gear < 0) gear = 0;            // sim never fully shuts the unit off
        plant.step(gear);
        room += plant.drift(load_at(m));
        if (m > 20) { r.tmin=std::min(r.tmin,room); r.tmax=std::max(r.tmax,room);
                      r.tsum+=room; r.n++; r.gear_hist.push_back(gear); }
        if (m >= ss_start) { r.ss_tmin=std::min(r.ss_tmin,room); r.ss_tmax=std::max(r.ss_tmax,room);
                             r.ss_tsum+=room; r.ss_n++; r.ss_gear_hist.push_back(gear);
                             if (changed) r.ss_changes++; }
    }
    r.bias_end = a.bias_c;
    return r;
}

// ── Test infra ──────────────────────────────────────────────────────────────────
int run=0, pass=0, fail=0;
#define CHECK(c,msg) do{ run++; if(c){pass++; printf("  PASS  %s\n",msg);} \
    else{fail++; printf("  FAIL  %s\n",msg);} }while(0)

float mean_gear(const SimResult&r){ float s=0; for(int g:r.gear_hist) s+=g; return r.gear_hist.empty()?0:s/r.gear_hist.size(); }
float ss_mean_gear(const SimResult&r){ float s=0; for(int g:r.ss_gear_hist) s+=g; return r.ss_gear_hist.empty()?0:s/r.ss_gear_hist.size(); }

// ── Tests ─────────────────────────────────────────────────────────────────────
void test_disabled_is_identical() {
    printf("\n=== Adaptive disabled == static ladder (regression) ===\n");
    auto mild = [](int){ return 0.12f; };
    SimResult off = simulate(false, mild, 240, 68.0f);
    CHECK(off.bias_end == 0.0f, "disabled: bias_c stays exactly 0");
    // identical gear history to a pure-static run (same plant, same seed) — trivially true
    // since eff_diff==diff when disabled; assert the sim is well-formed.
    CHECK(off.changes > 0 && off.n > 0, "disabled: sim ran and cycled");
}

void test_hot_day_centers_high() {
    printf("\n=== Hot day: adaptive centers on the sustaining gear, holds setpoint ===\n");
    auto hot = [](int){ return 0.213f; };  // equilibrium ~gear 4
    SimResult st = simulate(false, hot, 480, 68.0f);
    SimResult ad = simulate(true,  hot, 480, 68.0f);
    float st_off = st.tsum/st.n - 68.0f, ad_off = ad.tsum/ad.n - 68.0f;
    float st_ss_off = st.ss_tsum/st.ss_n - 68.0f, ad_ss_off = ad.ss_tsum/ad.ss_n - 68.0f;
    printf("  static  : full off %+.2fF | steady off %+.2fF swing %.2fF gear %.2f chg %d\n",
           st_off, st_ss_off, st.ss_tmax-st.ss_tmin, ss_mean_gear(st), st.ss_changes);
    printf("  adaptive: full off %+.2fF | steady off %+.2fF swing %.2fF gear %.2f chg %d | bias_end %.2fC\n",
           ad_off, ad_ss_off, ad.ss_tmax-ad.ss_tmin, ss_mean_gear(ad), ad.ss_changes, ad.bias_end);
    // Core wins: adaptive eliminates the static ladder's hot-running steady-state offset...
    // (bias settles modestly — the deadband freezes it once the room is within ±0.27F.)
    CHECK(ad.bias_end > 0.1f, "adaptive: bias_c climbed positive on a hot load");
    CHECK(std::fabs(ad_ss_off) < std::fabs(st_ss_off) - 0.5f,
          "adaptive: steady-state offset far tighter than static (kills the hot-running error)");
    CHECK(ss_mean_gear(ad) >= 3.6f, "adaptive: steady-state centers on the sustaining gear (~4)");
    // ...without trading it for a worse swing than the original hunting problem (3.7F on 6/3).
    CHECK(ad.ss_tmax - ad.ss_tmin < 2.0f, "adaptive: steady-state swing stays bounded (< 2F)");
}

void test_bias_clamped() {
    printf("\n=== Extreme load: bias_c clamps, never runs away ===\n");
    auto blazing = [](int){ return 0.50f; };  // beyond gear-5 capacity
    SimResult ad = simulate(true, blazing, 600, 68.0f);
    printf("  bias_end %.3fC (max %.2f)\n", ad.bias_end, ADAPT_BIAS_C_MAX);
    CHECK(ad.bias_end <= ADAPT_BIAS_C_MAX + 1e-4f, "bias_c never exceeds ADAPT_BIAS_C_MAX");
}

void test_idle_decay() {
    printf("\n=== Idle decay: bias_c relaxes toward 0 while compressor off ===\n");
    Adaptive a; a.enable = true; a.bias_c = 1.5f;
    uint32_t now = 0;
    a.last_advance = 0;
    a.eff_diff(0.0f, 2, false, now);          // prime last_advance
    // 30 min idle (gear 0), 1-min steps
    for (int m = 1; m <= 30; m++) { now = (uint32_t)m*60000u; a.eff_diff(0.0f, /*gear=*/0, false, now); }
    printf("  bias after 30min idle: %.3fC (started 1.5)\n", a.bias_c);
    CHECK(a.bias_c < 1.5f * 0.5f, "bias decayed by >half over one tau (30min)");
    CHECK(a.bias_c > 0.0f, "bias decays toward 0, doesn't overshoot negative");
}

void test_fan_edge_freeze_and_ff() {
    printf("\n=== Fan: feedforward raises eff_diff; integral frozen across the edge ===\n");
    Adaptive a; a.enable = true; a.fan_ff_gears = 1; a.last_advance = 0;
    a.eff_diff(0.0f, 3, false, 0);
    // fan turns on at t=60s
    a.fan_on = true; a.fan_changed_at = 60000;
    float eff = a.eff_diff(0.0f, 3, false, 120000);   // 1 min after edge, room at setpoint
    CHECK(std::fabs(eff - GEAR_STEP_C) < 1e-4f, "fan on: eff_diff = +1 gear of feedforward at zero error");
    float bias_during_freeze = a.bias_c;
    a.eff_diff(0.5f, 3, false, 150000);               // warm error, but within freeze window
    CHECK(a.bias_c == bias_during_freeze, "integral frozen within FAN_EDGE_FREEZE_MS of the edge");
    // past the freeze window, integration resumes
    a.eff_diff(0.5f, 3, false, 60000 + FAN_EDGE_FREEZE_MS + 60000);
    CHECK(a.bias_c > bias_during_freeze, "integral resumes after the freeze window");
}

void test_mild_day_low_bias() {
    printf("\n=== Mild day: bias stays small (no needless escalation) ===\n");
    auto mild = [](int){ return 0.12f; };  // equilibrium between gear 2 and 3
    SimResult ad = simulate(true, mild, 360, 68.0f);
    printf("  mild bias_end %.3fC, mean gear %.2f, swing %.2fF\n",
           ad.bias_end, mean_gear(ad), ad.tmax-ad.tmin);
    CHECK(std::fabs(ad.bias_end) < 1.0f, "mild: bias stays modest (< 1 gear-ish)");
    CHECK(ad.tmax-ad.tmin < 1.5f, "mild: room swing stays tight");
}

int main() {
    test_disabled_is_identical();
    test_hot_day_centers_high();
    test_bias_clamped();
    test_idle_decay();
    test_fan_edge_freeze_and_ff();
    test_mild_day_low_bias();
    printf("\n========================================\n");
    printf("Results: %d/%d passed, %d failed\n", pass, run, fail);
    printf("========================================\n");
    return fail > 0 ? 1 : 0;
}
