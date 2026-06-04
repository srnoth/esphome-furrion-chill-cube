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
    uint32_t fan_changed_at = 0;     // 0 = no real edge seen yet (mirrors the cpp guard)
    bool fan_present = false;        // a vent_fan sensor is configured
    bool fan_on = false;
    int fan_ff_gears = 1;

    // mirror of adaptive_cool_eff_diff_ — cool_gear is the CURRENT gear (pre-selection),
    // time_in_gear gates the conditional-integration anti-windup.
    float eff_diff(float real_diff, int cool_gear, bool kickstart, uint32_t now, uint32_t time_in_gear) {
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
        bool fan_edge_freeze = fan_present && fan_changed_at != 0 && (now - fan_changed_at) < FAN_EDGE_FREEZE_MS;
        bool idle = (cool_gear <= 0);
        bool upshift_held = (cool_gear < 5) && (time_in_gear < HOLD_MS[cool_gear + 1]);
        bool block_up = (e > 0.0f) && (cool_gear >= 5 || upshift_held);
        bool freeze = kickstart || fan_edge_freeze || block_up;
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
SimResult simulate(bool adaptive_on, LoadFn load_at, int minutes, float target_f,
                   int fan_ff=0, int fan_start=-1, int fan_end=-1) {
    SimResult r; Plant plant; Adaptive a; a.enable = adaptive_on; a.fan_ff_gears = fan_ff;
    a.fan_present = (fan_start >= 0);
    float room = target_f; int gear = 2; plant.c_now = plant.cooling(2);
    uint32_t time_in_gear = 999999; uint32_t now = 0;
    int ss_start = minutes * 2 / 3;          // steady-state = last third
    for (int m = 0; m < minutes; m++) {
        now = (uint32_t)m * 60000u;
        bool fan = (m >= fan_start && m < fan_end);
        if (fan != a.fan_on) { a.fan_on = fan; a.fan_changed_at = now; }  // record real edges
        float diff_c = f_to_c(room) - f_to_c(target_f);
        float eff = a.eff_diff(diff_c, gear, /*kickstart=*/false, now, time_in_gear);
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
    // Unit-level identity: with adaptive off, eff_diff must equal real_diff for ANY input,
    // so the cool ladder (cases 1-5) sees exactly the static value → bit-identical selection.
    Adaptive off; off.enable = false;
    bool identical = true;
    for (int i = 0; i < 50; i++) {
        float d = -2.0f + i * 0.08f;
        float eff = off.eff_diff(d, 3, false, (uint32_t)i * 60000u, 999999);
        if (eff != d) identical = false;
    }
    CHECK(identical, "disabled: eff_diff == real_diff for every input (static ladder unchanged)");
    CHECK(off.bias_c == 0.0f, "disabled: bias_c never moves");
    // And enabling MUST change behaviour on a non-trivial load (else the feature is a no-op).
    auto hot = [](int){ return 0.213f; };
    SimResult on = simulate(true, hot, 360, 68.0f);
    SimResult st = simulate(false, hot, 360, 68.0f);
    CHECK(std::fabs(on.tsum/on.n - 68.0f) < std::fabs(st.tsum/st.n - 68.0f),
          "enabled: measurably tightens setpoint tracking vs disabled (not a no-op)");
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
    // Load sits exactly at gear-4 capacity, so holding setpoint requires a gentle 3<->4 duty
    // cycle (a single fixed gear would hold, but only at the wrong temperature) → mean ~3.5.
    CHECK(ss_mean_gear(ad) >= 3.4f, "adaptive: steady-state centers on the sustaining gear (3<->4 cycle)");
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
    a.eff_diff(0.0f, 2, false, now, 999999);          // prime last_advance
    // 30 min idle (gear 0), 1-min steps
    for (int m = 1; m <= 30; m++) { now = (uint32_t)m*60000u; a.eff_diff(0.0f, /*gear=*/0, false, now, 999999); }
    printf("  bias after 30min idle: %.3fC (started 1.5)\n", a.bias_c);
    CHECK(a.bias_c < 1.5f * 0.5f, "bias decayed by >half over one tau (30min)");
    CHECK(a.bias_c > 0.0f, "bias decays toward 0, doesn't overshoot negative");
}

void test_fan_edge_freeze_and_ff() {
    printf("\n=== Fan: feedforward raises eff_diff; integral frozen across the edge ===\n");
    Adaptive a; a.enable = true; a.fan_ff_gears = 1; a.fan_present = true; a.last_advance = 0;
    a.eff_diff(0.0f, 3, false, 0, 999999);
    // fan turns on at t=60s
    a.fan_on = true; a.fan_changed_at = 60000;
    float eff = a.eff_diff(0.0f, 3, false, 120000, 999999);   // 1 min after edge, room at setpoint
    CHECK(std::fabs(eff - GEAR_STEP_C) < 1e-4f, "fan on: eff_diff = +1 gear of feedforward at zero error");
    float bias_during_freeze = a.bias_c;
    a.eff_diff(0.5f, 3, false, 150000, 999999);              // warm error, but within freeze window
    CHECK(a.bias_c == bias_during_freeze, "integral frozen within FAN_EDGE_FREEZE_MS of the edge");
    // past the freeze window, integration resumes (gear settled so no upshift-hold block)
    a.eff_diff(0.5f, 3, false, 60000 + FAN_EDGE_FREEZE_MS + 60000, 999999);
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

// Round-1 regression: a stale POSITIVE bias at gear 1 with a cold room must UNWIND.
// (The old sat_low froze it → indefinite overcooling.)
void test_positive_bias_unwinds_at_gear1() {
    printf("\n=== Anti-windup: stale +bias at gear 1 + cold room unwinds (not trapped) ===\n");
    Adaptive a; a.enable = true; a.bias_c = 1.0f; a.last_advance = 0;
    a.eff_diff(-0.5f, 1, false, 0, 999999);   // prime
    for (int m = 1; m <= 20; m++) a.eff_diff(-0.5f, /*gear=*/1, false, (uint32_t)m*60000u, 999999);
    printf("  bias after 20min at gear1, room cold: %.3fC (started 1.0)\n", a.bias_c);
    CHECK(a.bias_c < 1.0f, "positive bias unwinds at gear 1 with e<0 (no sat_low trap)");
}

// Round-1 regression: while warm AND the upshift is hold-blocked, the integral must NOT wind up.
void test_no_windup_behind_held_upshift() {
    printf("\n=== Anti-windup: no accumulation behind a hold-blocked upshift ===\n");
    Adaptive a; a.enable = true; a.last_advance = 0;
    // gear 2, warm (e=+0.8), but time_in_gear small → upshift 2->3 is hold-blocked (HOLD_MS[3]=300s)
    a.eff_diff(0.8f, 2, false, 0, 0);
    float b0 = a.bias_c;
    for (int m = 1; m <= 4; m++) a.eff_diff(0.8f, /*gear=*/2, false, (uint32_t)m*60000u, /*time_in_gear=*/0);
    printf("  bias after 4min warm w/ held upshift: %.3fC\n", a.bias_c);
    CHECK(a.bias_c == b0, "integral frozen while warm + upshift hold-blocked");
    // once the hold clears (time_in_gear past HOLD_MS[3]=300s), it resumes
    a.eff_diff(0.8f, 2, false, 360000, 360000);
    CHECK(a.bias_c > b0, "integral resumes once the upshift hold clears");
}

void test_varying_load_tracks_setpoint() {
    printf("\n=== Varying load: adaptive holds setpoint tighter than static ===\n");
    // Slowly oscillating load (sun moving, intermittent occupancy) makes the static ladder
    // ride a varying offset; adaptive should keep the mean near setpoint.
    auto vary = [](int m){ return 0.17f + 0.05f * std::sin(m / 25.0f); };
    SimResult st = simulate(false, vary, 600, 68.0f);
    SimResult ad = simulate(true,  vary, 600, 68.0f);
    printf("  static  steady off %+.2fF | adaptive steady off %+.2fF\n",
           st.ss_tsum/st.ss_n - 68.0f, ad.ss_tsum/ad.ss_n - 68.0f);
    CHECK(std::fabs(ad.ss_tsum/ad.ss_n - 68.0f) < std::fabs(st.ss_tsum/st.ss_n - 68.0f),
          "adaptive tracks setpoint tighter than static under a varying load");
}

// Mirror of the user_input/fresh-start recompute. Round-3: this path uses REAL diff only
// (bias-independent) — the adaptive bias governs only the steady-state switch cases. Identical
// to the non-adaptive ladder.
int user_recompute(int gear, float real_diff, bool user_input) {
    if (real_diff > C_UP_45) return 5;
    if (real_diff > C_UP_34) return 4;
    if (real_diff > C_UP_23) return 3;
    if (real_diff > C_UP_12) return 2;
    if (real_diff > C_UP_01) return (gear == -1) ? 2 : 1;
    if (gear == -1)          return -1;
    if (user_input && real_diff < C_IDLE) return -1;
    return 0;
}

// Round-3 regressions: the user_input path is conservative (real diff, bias-independent), and
// the gear-0 user-event shutoff works.
void test_user_event_conservative() {
    printf("\n=== User event recompute uses real diff only (bias-independent) ===\n");
    // gear 0, user event, room below setpoint → must shut OFF (the Round-3 gear-0 regression).
    CHECK(user_recompute(0, -0.30f, true) == -1, "gear-0 user event below setpoint idles (-1)");
    // A negative bias must NOT collapse the gear on a user tap — the path ignores bias entirely,
    // so a warm room (real diff +0.20) recomputes to a cooling gear regardless of any bias.
    CHECK(user_recompute(3, 0.20f, true) == 1, "user tap, room warm: real-diff recompute (bias ignored)");
    // Big setpoint drop still jumps straight to a high gear (responsiveness preserved).
    CHECK(user_recompute(2, 1.50f, true) == 5, "big setpoint drop jumps to gear 5");
    // Cold re-engage from -1 uses real diff, gear-2 minimum.
    CHECK(user_recompute(-1, 0.30f, false) == 2, "cold re-engage from -1 -> gear 2");
}

// Closed-loop: a sustained vent-fan burst raises the load; adaptive (bias + feedforward)
// should ride it without losing setpoint or winding up wildly.
void test_hot_plus_fan_closed_loop() {
    printf("\n=== Hot + sustained vent fan (closed loop): holds setpoint, bias bounded ===\n");
    // base hot load; fan ON minutes 200-360 adds load (dumps conditioned air).
    auto load = [](int m){ return (m >= 200 && m < 360) ? 0.245f : 0.18f; };
    SimResult ad = simulate(true, load, 480, 68.0f, /*fan_ff=*/1, /*fan_start=*/200, /*fan_end=*/360);
    float ss_off = ad.ss_tsum/ad.ss_n - 68.0f;
    printf("  steady off %+.2fF, swing %.2fF, bias_end %.2fC\n", ss_off, ad.ss_tmax-ad.ss_tmin, ad.bias_end);
    CHECK(std::fabs(ss_off) < 0.6f, "hot+fan: holds setpoint within 0.6F after the burst");
    CHECK(ad.bias_end <= ADAPT_BIAS_C_MAX + 1e-4f, "hot+fan: bias stays clamped (no runaway)");
}

int main() {
    test_disabled_is_identical();
    test_user_event_conservative();
    test_hot_plus_fan_closed_loop();
    test_hot_day_centers_high();
    test_bias_clamped();
    test_idle_decay();
    test_fan_edge_freeze_and_ff();
    test_mild_day_low_bias();
    test_positive_bias_unwinds_at_gear1();
    test_no_windup_behind_held_upshift();
    test_varying_load_tracks_setpoint();
    printf("\n========================================\n");
    printf("Results: %d/%d passed, %d failed\n", pass, run, fail);
    printf("========================================\n");
    return fail > 0 ? 1 : 0;
}
