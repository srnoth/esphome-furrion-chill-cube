// Standalone boot gear-restore test harness
// Mirrors the warm-reset gear+bias persistence logic added to furrion_chill_cube.cpp:
//   - is_warm_reset_() gating (which esp_reset_reason values trust the saved gear)
//   - setup()'s restore decision (validity / clamping / fallback to gear 0)
//   - save_gear_pref_()'s change-detection + bias quantization (flash-write dedup)
//   - first cool-pass dynamics at a restored gear: WITH the restored bias the gear
//     holds; with bias 0 it would collapse — the reason bias is persisted at all
// Build: g++ -std=c++17 -o boot_restore_test tests/boot_restore_test.cpp && ./boot_restore_test

#include <cstdio>
#include <cstdint>
#include <cmath>
#include <vector>

// ============================================================
// Constants (copied from furrion_chill_cube.cpp)
// ============================================================

static const uint32_t HOLD_MS[] = {0, 180000, 180000, 300000, 300000, 600000};

static const float C_UP_01 =  0.15f;
static const float C_UP_12 =  0.25f;
static const float C_UP_23 =  0.4f;
static const float C_UP_34 =  0.85f;
static const float C_UP_45 =  1.0f;
static const float C_DN_54 =  0.45f;
static const float C_DN_43 =  0.40f;
static const float C_DN_32 = -0.05f;
static const float C_DN_21 = -0.40f;
static const float C_DN_10 = -0.55f;
static const float C_IDLE  = -0.15f;

static const float ADAPT_BIAS_C_MAX = 2.0f;
static const float GEAR_PREF_BIAS_QUANTUM_C = 0.1f;

// ============================================================
// Mirrored logic
// ============================================================

// esp_reset_reason() values (esp_system.h)
enum ResetReason {
    RST_UNKNOWN = 0, RST_POWERON, RST_EXT, RST_SW, RST_PANIC,
    RST_INT_WDT, RST_TASK_WDT, RST_WDT, RST_DEEPSLEEP, RST_BROWNOUT, RST_SDIO,
};

// Mirror of is_warm_reset_()
static bool is_warm_reset(ResetReason r) {
    switch (r) {
        case RST_SW:
        case RST_PANIC:
        case RST_INT_WDT:
        case RST_TASK_WDT:
        case RST_WDT:
            return true;
        default:
            return false;
    }
}

// Mirror of setup()'s restore decision for an active saved mode (1=heat, 2=cool).
struct RestoreResult {
    int gear;      // restored gear for the saved mode
    float bias_c;  // restored adaptive bias (cool only)
};
static RestoreResult decide_restore(bool is_heat, ResetReason reason,
                                    bool have_gear_pref, int8_t saved_gear, float saved_bias,
                                    bool adaptive_enabled = true) {
    RestoreResult r{0, 0.0f};
    if (is_warm_reset(reason) && have_gear_pref) {
        int max_gear = is_heat ? 3 : 5;
        if (saved_gear >= 0 && saved_gear <= max_gear) {
            r.gear = saved_gear;
            // Bias only when adaptive is running: disabled means it neither
            // integrates nor decays, so a restored value would sit frozen and
            // spring back stale whenever adaptive is re-enabled.
            if (!is_heat && adaptive_enabled && !std::isnan(saved_bias)) {
                r.bias_c = std::fmax(-ADAPT_BIAS_C_MAX, std::fmin(ADAPT_BIAS_C_MAX, saved_bias));
            }
        }
    }
    return r;
}

// Mirror of save_gear_pref_()'s change detection. Returns true if a flash write
// would be queued, and updates the trackers.
struct SaveTracker {
    int8_t last_gear = -128;
    float last_bias = 0.0f;
    int writes = 0;
};
static bool save_gear_pref(SaveTracker &t, int8_t active_gear, float bias_c) {
    float bias_q = std::isnan(bias_c)
                       ? 0.0f
                       : roundf(bias_c / GEAR_PREF_BIAS_QUANTUM_C) * GEAR_PREF_BIAS_QUANTUM_C;
    if (active_gear == t.last_gear && bias_q == t.last_bias) return false;
    t.last_gear = active_gear;
    t.last_bias = bias_q;
    t.writes++;
    return true;
}

// Mirror of run_cool_mode_'s active-gear switch (cases 0-5), non-user path.
// eff_diff = diff + bias (fan feedforward omitted — not persisted/restored).
static int select_cool_gear(int gear, float diff, float bias_c,
                            uint32_t time_in_gear, uint32_t last_gear_change) {
    auto can_upshift_to = [&](int target_gear) -> bool {
        return time_in_gear >= HOLD_MS[target_gear];
    };
    float eff_diff = diff + bias_c;
    int new_gear = gear;
    switch (gear) {
        case 0: {
            // First compute post-restore: skip the HOLD_MS ladder
            if (last_gear_change == 0) {
                if (diff > C_UP_45)         new_gear = 5;
                else if (diff > C_UP_34)    new_gear = 4;
                else if (diff > C_UP_23)    new_gear = 3;
                else if (diff > C_UP_12)    new_gear = 2;
                else if (diff > C_UP_01)    new_gear = 1;
            } else {
                if (can_upshift_to(1) && diff > C_UP_01) new_gear = 1;
            }
            break;
        }
        case 1:
            if (can_upshift_to(2) && eff_diff > C_UP_12)  new_gear = 2;
            else if (eff_diff < C_DN_10)                   new_gear = 0;
            break;
        case 2:
            if (can_upshift_to(3) && eff_diff > C_UP_23)  new_gear = 3;
            else if (eff_diff < C_DN_21)                   new_gear = 1;
            break;
        case 3:
            if (can_upshift_to(4) && eff_diff > C_UP_34)  new_gear = 4;
            else if (eff_diff < C_DN_32)                   new_gear = 2;
            break;
        case 4:
            if (can_upshift_to(5) && eff_diff > C_UP_45)  new_gear = 5;
            else if (eff_diff < C_DN_43)                   new_gear = 3;
            break;
        case 5:
            if (eff_diff < C_DN_54)                        new_gear = 4;
            break;
    }
    return new_gear;
}

// ============================================================
// Test infrastructure
// ============================================================

int tests_run = 0;
int tests_passed = 0;
int tests_failed = 0;

struct TestEntry {
    const char *name;
    void (*fn)();
};
std::vector<TestEntry> test_registry;

#define TEST(name) \
    void test_##name(); \
    struct Register_##name { Register_##name() { test_registry.push_back({#name, test_##name}); } } reg_##name; \
    void test_##name()

#define EXPECT_EQ(expected, actual, msg) do { \
    tests_run++; \
    if ((expected) == (actual)) { \
        tests_passed++; \
        printf("  ✓ %s\n", msg); \
    } else { \
        tests_failed++; \
        printf("  ✗ %s — expected %d, got %d\n", msg, (int)(expected), (int)(actual)); \
    } \
} while(0)

#define EXPECT_NEAR(expected, actual, tol, msg) do { \
    tests_run++; \
    if (std::fabs((expected) - (actual)) <= (tol)) { \
        tests_passed++; \
        printf("  ✓ %s\n", msg); \
    } else { \
        tests_failed++; \
        printf("  ✗ %s — expected %.3f, got %.3f\n", msg, (double)(expected), (double)(actual)); \
    } \
} while(0)

// ============================================================
// Tests
// ============================================================

TEST(warm_reset_classification) {
    printf("\n=== Reset-reason gating ===\n");
    EXPECT_EQ(true,  is_warm_reset(RST_SW),       "SW reset (OTA/esp_restart) is warm");
    EXPECT_EQ(true,  is_warm_reset(RST_PANIC),    "Panic reset is warm (unit kept power)");
    EXPECT_EQ(true,  is_warm_reset(RST_INT_WDT),  "Interrupt WDT reset is warm");
    EXPECT_EQ(true,  is_warm_reset(RST_TASK_WDT), "Task WDT reset is warm");
    EXPECT_EQ(true,  is_warm_reset(RST_WDT),      "Other WDT reset is warm");
    EXPECT_EQ(false, is_warm_reset(RST_POWERON),  "Power-on reset is cold");
    EXPECT_EQ(false, is_warm_reset(RST_BROWNOUT), "Brownout reset is cold");
    EXPECT_EQ(false, is_warm_reset(RST_UNKNOWN),  "Unknown reset defaults cold");
    EXPECT_EQ(false, is_warm_reset(RST_EXT),      "External reset defaults cold");
}

TEST(restore_decision_table) {
    printf("\n=== Restore decision (saved mode active) ===\n");

    // Warm reset, valid saved cool gear + bias → resume both
    RestoreResult r = decide_restore(false, RST_SW, true, 3, 0.7f);
    EXPECT_EQ(3, r.gear, "Warm SW reset: cool gear 3 restored");
    EXPECT_NEAR(0.7f, r.bias_c, 1e-6f, "Warm SW reset: bias 0.7 restored");

    // Cold boot: same saved data ignored → gear 0, bias 0 (failover invariant)
    r = decide_restore(false, RST_POWERON, true, 3, 0.7f);
    EXPECT_EQ(0, r.gear, "Cold boot: saved gear ignored, restore to idle");
    EXPECT_NEAR(0.0f, r.bias_c, 1e-6f, "Cold boot: saved bias ignored");

    // Warm but no gear pref stored yet (first boot after this firmware) → gear 0
    r = decide_restore(false, RST_SW, false, 3, 0.7f);
    EXPECT_EQ(0, r.gear, "No gear pref stored: restore to idle");

    // Inconsistent pair: mode says active but saved gear is -1 → gear 0
    r = decide_restore(false, RST_SW, true, -1, 0.7f);
    EXPECT_EQ(0, r.gear, "Saved gear -1 under active mode: restore to idle");
    EXPECT_NEAR(0.0f, r.bias_c, 1e-6f, "Inconsistent pair: bias not restored either");

    // Out-of-range gears (heat max 3, cool max 5) → gear 0
    r = decide_restore(true, RST_SW, true, 4, 0.0f);
    EXPECT_EQ(0, r.gear, "Heat gear 4 (out of range): restore to idle");
    r = decide_restore(false, RST_SW, true, 6, 0.0f);
    EXPECT_EQ(0, r.gear, "Cool gear 6 (out of range): restore to idle");
    r = decide_restore(true, RST_SW, true, 3, 0.0f);
    EXPECT_EQ(3, r.gear, "Heat gear 3 (max valid): restored");
    r = decide_restore(false, RST_SW, true, 5, 0.0f);
    EXPECT_EQ(5, r.gear, "Cool gear 5 (max valid): restored");

    // Bias hygiene: NaN dropped, magnitude clamped, heat never takes a bias
    r = decide_restore(false, RST_SW, true, 2, NAN);
    EXPECT_EQ(2, r.gear, "NaN bias: gear still restored");
    EXPECT_NEAR(0.0f, r.bias_c, 1e-6f, "NaN bias: restored as 0");
    r = decide_restore(false, RST_SW, true, 2, 9.9f);
    EXPECT_NEAR(ADAPT_BIAS_C_MAX, r.bias_c, 1e-6f, "Oversized bias clamped to +2.0");
    r = decide_restore(false, RST_SW, true, 2, -9.9f);
    EXPECT_NEAR(-ADAPT_BIAS_C_MAX, r.bias_c, 1e-6f, "Oversized bias clamped to -2.0");
    r = decide_restore(true, RST_PANIC, true, 2, 1.5f);
    EXPECT_NEAR(0.0f, r.bias_c, 1e-6f, "Heat restore ignores saved bias");

    // Adaptive disabled: gear still resumes, but the bias stays 0 — a frozen
    // bias must not survive a disabled period and re-arm later.
    r = decide_restore(false, RST_SW, true, 3, 0.7f, /*adaptive_enabled=*/false);
    EXPECT_EQ(3, r.gear, "Adaptive disabled: gear still restored");
    EXPECT_NEAR(0.0f, r.bias_c, 1e-6f, "Adaptive disabled: bias NOT restored");
}

TEST(first_pass_holds_restored_gear) {
    printf("\n=== First cool pass after warm reboot (the actual fix) ===\n");

    // Scenario: mild-day equilibrium before reboot — sustaining gear 3, room pinned
    // slightly below target (diff ≈ -0.10, gear 3 gently over-cools) because
    // bias_c ≈ +0.55 holds eff_diff inside gear 3's band (C_DN_32=-0.05 .. C_UP_34=0.85).
    // OTA reflash → warm reset.
    float diff = -0.10f;
    float bias = 0.55f;

    // NEW behavior: gear 3 + bias restored → eff_diff 0.45 stays in band → gear holds
    int g = select_cool_gear(3, diff, bias, /*time_in_gear=*/999999999, /*last_change=*/0);
    EXPECT_EQ(3, g, "Restored gear 3 + restored bias: holds gear 3 (no swing)");

    // Gear-only restore (bias lost): eff_diff -0.10 < C_DN_32 → immediate collapse.
    // This is WHY bias_c_ is persisted alongside the gear.
    g = select_cool_gear(3, diff, 0.0f, 999999999, 0);
    EXPECT_EQ(2, g, "Restored gear 3 WITHOUT bias: collapses to 2 (documents bias need)");

    // OLD behavior (pre-change): restore lands at gear 0; diff < C_UP_01 means
    // the ladder-skip finds no demand → controller idles while the load is real.
    // The room then has to drift past +0.15°C before gear 1 even engages — the swing.
    g = select_cool_gear(0, diff, 0.0f, 999999999, 0);
    EXPECT_EQ(0, g, "Old gear-0 restore at diff -0.10: idles (documents the old swing)");

    // Warm restore must still react if the world changed during the reboot:
    // room got hot while we were down → upshift allowed immediately (sentinel hold)
    g = select_cool_gear(3, 0.9f, 0.55f, 999999999, 0);
    EXPECT_EQ(4, g, "Restored gear 3, room hot (eff 1.45): upshifts immediately");
    // room got cold → downshift cascades naturally (downshifts never hold-gated)
    g = select_cool_gear(3, -0.8f, 0.0f, 999999999, 0);
    EXPECT_EQ(2, g, "Restored gear 3, room cold: downshifts immediately");
}

TEST(save_dedup_and_quantization) {
    printf("\n=== save_gear_pref_ change detection (flash-write hygiene) ===\n");

    SaveTracker t;
    // First call after boot always writes (tracker starts at -128 sentinel)
    EXPECT_EQ(true, save_gear_pref(t, 0, 0.0f), "First save after boot writes");
    // Same state again → no write
    EXPECT_EQ(false, save_gear_pref(t, 0, 0.0f), "Unchanged state: no write");
    // Bias drifting within one quantum → no write
    EXPECT_EQ(false, save_gear_pref(t, 0, 0.04f), "Bias +0.04 (same 0.1 quantum): no write");
    EXPECT_EQ(false, save_gear_pref(t, 0, -0.04f), "Bias -0.04 (same quantum): no write");
    // Crossing a quantum boundary → one write
    EXPECT_EQ(true, save_gear_pref(t, 0, 0.07f), "Bias 0.07 rounds to 0.1: writes");
    EXPECT_EQ(false, save_gear_pref(t, 0, 0.11f), "Bias 0.11 still 0.1: no write");
    // Gear change → write
    EXPECT_EQ(true, save_gear_pref(t, 2, 0.11f), "Gear 0→2: writes");
    // NaN bias treated as 0
    EXPECT_EQ(true, save_gear_pref(t, 2, NAN), "NaN bias saves as 0: writes once");
    EXPECT_EQ(false, save_gear_pref(t, 2, NAN), "NaN bias again: no write");
    EXPECT_EQ(4, t.writes, "Total flash writes across 9 passes: 4");

    // Simulate a steady hour at equilibrium: gear 3, bias wandering ±0.03 around 0.6
    SaveTracker steady;
    save_gear_pref(steady, 3, 0.60f);
    for (int i = 0; i < 120; i++) {  // 120 passes (~1 hr at 30s cadence)
        float wobble = 0.60f + 0.03f * ((i % 3) - 1);
        save_gear_pref(steady, 3, wobble);
    }
    EXPECT_EQ(1, steady.writes, "1 hr at equilibrium: exactly 1 flash write (the first)");
}

// ============================================================
// Main
// ============================================================

int main() {
    printf("Furrion Chill Cube — boot gear-restore tests\n");
    printf("=============================================\n");

    for (auto &t : test_registry) {
        t.fn();
    }

    printf("\n=============================================\n");
    printf("Results: %d run, %d passed, %d failed\n", tests_run, tests_passed, tests_failed);
    return tests_failed > 0 ? 1 : 0;
}
