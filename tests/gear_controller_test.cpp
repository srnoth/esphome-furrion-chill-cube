// Standalone gear controller test harness
// Extracts the pure gear selection logic from furrion_chill_cube.cpp
// Build: g++ -std=c++17 -o gear_test tests/gear_controller_test.cpp && ./gear_test

#include <cstdio>
#include <cstdint>
#include <cmath>
#include <cassert>
#include <vector>
#include <string>

// ============================================================
// Constants (copied from furrion_chill_cube.cpp)
// ============================================================

static const uint32_t HOLD_MS[] = {0, 180000, 180000, 300000, 300000, 600000};
static const uint32_t IDLE_BEFORE_OFF_MS = 900000;

// Heating deadbands (diff = room - target, negative = cold)
static const float H_UP_01 = -0.3f;
static const float H_UP_12 = -0.8f;
static const float H_UP_23 = -1.5f;
static const float H_DN_32 = -0.8f;
static const float H_DN_21 = -0.3f;
static const float H_DN_10 =  0.3f;
static const float H_IDLE  =  0.3f;

// Cooling deadbands (diff = room - target, positive = hot)
static const float C_UP_01 =  0.15f;
static const float C_UP_12 =  0.25f;
static const float C_UP_23 =  0.4f;
static const float C_UP_34 =  0.6f;
static const float C_UP_45 =  0.8f;
static const float C_DN_54 =  0.45f;
static const float C_DN_43 =  0.25f;
static const float C_DN_32 =  0.10f;
static const float C_DN_21 =  0.0f;
static const float C_DN_10 = -0.15f;
static const float C_IDLE  = -0.15f;

// ============================================================
// Gear selection functions (extracted from run_gear_controller_)
// ============================================================

float f_to_c(float f) { return (f - 32.0f) * (5.0f / 9.0f); }

int select_cool_gear(int current_gear, float room_c, float target_c,
                     bool user_input, uint32_t time_in_gear, bool boot_ready,
                     uint32_t idle_since, uint32_t now) {
    float diff = room_c - target_c;
    int gear = current_gear;
    int new_gear = gear;

    bool idle_long_enough = (idle_since > 0) && (now - idle_since >= IDLE_BEFORE_OFF_MS);

    auto can_upshift_to = [&](int target_gear) -> bool {
        return user_input || (time_in_gear >= HOLD_MS[target_gear]);
    };

    if (gear == -1 || user_input) {
        // Fresh start or user change: jump directly to appropriate gear
        // From -1, only go to 1+ (never 0 — gear 0 is only reachable by downshift from 1)
        if (diff > C_UP_45)         new_gear = 5;
        else if (diff > C_UP_34)    new_gear = 4;
        else if (diff > C_UP_23)    new_gear = 3;
        else if (diff > C_UP_12)    new_gear = 2;
        else if (diff > C_UP_01)    new_gear = 1;
        else                        new_gear = (gear == -1) ? -1 : 0;
    } else {
        switch (gear) {
            case 0: {
                if (can_upshift_to(1) && diff > C_UP_01) new_gear = 1;
                bool imm_off = !boot_ready;
                bool far_from_setpoint = (diff < -1.1f);
                if ((imm_off || (idle_long_enough && far_from_setpoint)) && diff < C_IDLE) new_gear = -1;
                break;
            }
            case 1:
                if (can_upshift_to(2) && diff > C_UP_12)  new_gear = 2;
                else if (diff < C_DN_10)                   new_gear = 0;
                break;
            case 2:
                if (can_upshift_to(3) && diff > C_UP_23)  new_gear = 3;
                else if (diff < C_DN_21)                   new_gear = 1;
                break;
            case 3:
                if (can_upshift_to(4) && diff > C_UP_34)  new_gear = 4;
                else if (diff < C_DN_32)                   new_gear = 2;
                break;
            case 4:
                if (can_upshift_to(5) && diff > C_UP_45)  new_gear = 5;
                else if (diff < C_DN_43)                   new_gear = 3;
                break;
            case 5:
                if (diff < C_DN_54)                        new_gear = 4;
                break;
        }
    }
    return new_gear;
}

int select_heat_gear(int current_gear, float room_c, float target_c,
                     bool user_input, uint32_t time_in_gear, bool boot_ready,
                     uint32_t idle_since, uint32_t now) {
    float diff = room_c - target_c;
    int gear = current_gear;
    int new_gear = gear;

    bool idle_long_enough = (idle_since > 0) && (now - idle_since >= IDLE_BEFORE_OFF_MS);

    auto can_upshift_to = [&](int target_gear) -> bool {
        return user_input || (time_in_gear >= HOLD_MS[target_gear]);
    };

    if (gear == -1 || user_input) {
        // Fresh start or user change: jump directly to appropriate gear
        // From -1, only go to 1+ (never 0 — gear 0 is only reachable by downshift from 1)
        if (diff < H_UP_23)         new_gear = 3;
        else if (diff < H_UP_12)    new_gear = 2;
        else if (diff < H_UP_01)    new_gear = 1;
        else                        new_gear = (gear == -1) ? -1 : 0;
    } else {
        switch (gear) {
            case 0: {
                if (can_upshift_to(1) && diff < H_UP_01) new_gear = 1;
                bool imm_off = !boot_ready;
                bool far_from_setpoint = (diff > 1.1f);
                if ((imm_off || (idle_long_enough && far_from_setpoint)) && diff > H_IDLE) new_gear = -1;
                break;
            }
            case 1:
                if (can_upshift_to(2) && diff < H_UP_12)  new_gear = 2;
                else if (diff > H_DN_10)                   new_gear = 0;
                break;
            case 2:
                if (can_upshift_to(3) && diff < H_UP_23)  new_gear = 3;
                else if (diff > H_DN_21)                   new_gear = 1;
                break;
            case 3:
                if (diff > H_DN_32)                        new_gear = 2;
                break;
        }
    }
    return new_gear;
}

// ============================================================
// Test infrastructure
// ============================================================

int tests_run = 0;
int tests_passed = 0;
int tests_failed = 0;

#define TEST(name) \
    void test_##name(); \
    struct Register_##name { Register_##name() { test_registry.push_back({#name, test_##name}); } } reg_##name; \
    void test_##name()

#define EXPECT_GEAR(expected, actual, msg) do { \
    tests_run++; \
    if ((expected) == (actual)) { \
        tests_passed++; \
        printf("  ✓ %s (gear %d)\n", msg, actual); \
    } else { \
        tests_failed++; \
        printf("  ✗ %s — expected gear %d, got %d\n", msg, expected, actual); \
    } \
} while(0)

struct TestEntry {
    const char *name;
    void (*fn)();
};
std::vector<TestEntry> test_registry;

// ============================================================
// Tests
// ============================================================

// --- Scenario: Last night's incident (2026-04-04) ---

TEST(cool_setpoint_drop_user_input_jumps_to_gear5) {
    printf("\n=== Setpoint drop 66→63°F at gear 2 (user input) ===\n");

    float target_66 = f_to_c(66.0f);
    float target_63 = f_to_c(63.0f);
    float room = f_to_c(66.2f);  // Room temp when setpoint changed

    // Before change: gear 2, target 66°F, room 66.2°F
    // diff = 66.2 - 66 = 0.2°F = 0.11°C — at gear 2, no change expected
    int gear = select_cool_gear(2, room, target_66, false, 999999, true, 0, 100000);
    EXPECT_GEAR(2, gear, "Before setpoint change: gear 2 stable");

    // User drops setpoint to 63°F — user_input=true
    // diff = 66.2 - 63 = 3.2°F = 1.78°C — should jump to gear 5
    gear = select_cool_gear(2, room, target_63, true, 0, true, 0, 100000);
    EXPECT_GEAR(5, gear, "After setpoint drop 66→63: should jump to gear 5");
}

TEST(cool_setpoint_drop_without_user_input_OLD_behavior) {
    printf("\n=== Same scenario WITHOUT user_input (old behavior) ===\n");

    float target_63 = f_to_c(63.0f);
    float room = f_to_c(66.2f);

    // Without user_input, hold timers apply — can only step one gear
    // time_in_gear=0 means just changed, HOLD_MS[3]=300000 not met
    int gear = select_cool_gear(2, room, target_63, false, 0, true, 0, 100000);
    EXPECT_GEAR(2, gear, "Without user_input, time_in_gear=0: stuck at gear 2");

    // With enough time elapsed for hold timer
    gear = select_cool_gear(2, room, target_63, false, 300001, true, 0, 100000);
    EXPECT_GEAR(3, gear, "Without user_input, hold timer passed: steps to gear 3 (not 5)");
}

TEST(cool_gear5_downshift_near_setpoint) {
    printf("\n=== Gear 5 downshift when approaching setpoint ===\n");

    float target = f_to_c(63.0f);

    // Room at 64°F — diff = 1°F = 0.56°C > C_DN_54 (0.45) — stay at 5
    int gear = select_cool_gear(5, f_to_c(64.0f), target, false, 999999, true, 0, 100000);
    EXPECT_GEAR(5, gear, "Room 64°F, target 63°F (diff 0.56°C): stay at gear 5");

    // Room at 63.8°F — diff = 0.8°F = 0.44°C < C_DN_54 (0.45) — downshift!
    gear = select_cool_gear(5, f_to_c(63.8f), target, false, 999999, true, 0, 100000);
    EXPECT_GEAR(4, gear, "Room 63.8°F, target 63°F (diff 0.44°C): downshift to gear 4");

    // Room at 63.3°F — diff = 0.3°F = 0.17°C — definitely downshift
    gear = select_cool_gear(5, f_to_c(63.3f), target, false, 999999, true, 0, 100000);
    EXPECT_GEAR(4, gear, "Room 63.3°F, target 63°F (diff 0.17°C): downshift to gear 4");

    // Room at 63.0°F — diff = 0 — downshift
    gear = select_cool_gear(5, f_to_c(63.0f), target, false, 999999, true, 0, 100000);
    EXPECT_GEAR(4, gear, "Room 63.0°F, target 63°F (diff 0.0°C): downshift to gear 4");
}

TEST(cool_gear5_stale_target_reproduces_bug) {
    printf("\n=== Gear 5 with STALE target (reproducing bug) ===\n");

    // Bug scenario: target_temperature stuck at 66°F (heat target)
    // while target_temperature_high is correctly 63°F
    float stale_target = f_to_c(66.0f);  // WRONG — stale from before setpoint change
    float correct_target = f_to_c(63.0f);

    float room = f_to_c(63.3f);

    // With stale target: diff = 63.3 - 66 = -2.7°F = -1.5°C
    // -1.5 < C_DN_54 (0.45)? YES — would actually downshift!
    // Wait, that's negative which IS less than 0.45...
    int gear = select_cool_gear(5, room, stale_target, false, 999999, true, 0, 100000);
    EXPECT_GEAR(4, gear, "Stale target 66°F, room 63.3°F: diff=-1.5°C, still downshifts");

    // What if stale target was the HEAT target (60°F)?
    float heat_target = f_to_c(60.0f);
    // diff = 63.3 - 60 = 3.3°F = 1.83°C > C_DN_54 (0.45) — STUCK!
    gear = select_cool_gear(5, room, heat_target, false, 999999, true, 0, 100000);
    EXPECT_GEAR(5, gear, "Stale heat target 60°F, room 63.3°F: diff=1.83°C — STUCK at gear 5!");

    // With correct target it would downshift
    gear = select_cool_gear(5, room, correct_target, false, 999999, true, 0, 100000);
    EXPECT_GEAR(4, gear, "Correct target 63°F, room 63.3°F: diff=0.17°C — downshifts to 4");
}

// --- User input: setpoint increase (should downshift) ---

TEST(cool_user_raises_setpoint_above_room) {
    printf("\n=== User raises cooling setpoint above room temp ===\n");

    float room = f_to_c(63.0f);
    float new_target = f_to_c(68.0f);  // Raised well above room

    // diff = 63 - 68 = -5°F = -2.78°C — should go to idle
    int gear = select_cool_gear(5, room, new_target, true, 0, true, 0, 100000);
    EXPECT_GEAR(0, gear, "Gear 5, raise setpoint to 68°F (above room): should go to idle");

    gear = select_cool_gear(3, room, new_target, true, 0, true, 0, 100000);
    EXPECT_GEAR(0, gear, "Gear 3, raise setpoint to 68°F (above room): should go to idle");
}

// --- Fresh start ---

TEST(cool_fresh_start_various_diffs) {
    printf("\n=== Fresh start (gear -1) gear selection ===\n");

    float target = f_to_c(63.0f);

    int gear = select_cool_gear(-1, f_to_c(70.0f), target, false, 0, true, 0, 100000);
    EXPECT_GEAR(5, gear, "Fresh start, room 70°F (diff=3.89°C): gear 5");

    gear = select_cool_gear(-1, f_to_c(64.5f), target, false, 0, true, 0, 100000);
    EXPECT_GEAR(5, gear, "Fresh start, room 64.5°F (diff=0.83°C): gear 5");

    gear = select_cool_gear(-1, f_to_c(64.0f), target, false, 0, true, 0, 100000);
    EXPECT_GEAR(3, gear, "Fresh start, room 64.0°F (diff=0.56°C): gear 3");

    gear = select_cool_gear(-1, f_to_c(63.5f), target, false, 0, true, 0, 100000);
    EXPECT_GEAR(2, gear, "Fresh start, room 63.5°F (diff=0.28°C): gear 2");

    gear = select_cool_gear(-1, f_to_c(63.0f), target, false, 0, true, 0, 100000);
    EXPECT_GEAR(-1, gear, "Fresh start, room at setpoint (diff=0): stay off");

    gear = select_cool_gear(-1, f_to_c(62.0f), target, false, 0, true, 0, 100000);
    EXPECT_GEAR(-1, gear, "Fresh start, room below setpoint: stay off");
}

// --- Heating ---

TEST(heat_setpoint_raise_user_input) {
    printf("\n=== Heating: raise setpoint 63→68°F at gear 1 ===\n");

    float target_63 = f_to_c(63.0f);
    float target_68 = f_to_c(68.0f);
    float room = f_to_c(63.5f);

    // At gear 1, target 63, room 63.5: diff = +0.28°C, H_DN_10=0.3 — not met yet
    int gear = select_heat_gear(1, room, target_63, false, 999999, true, 0, 100000);
    EXPECT_GEAR(1, gear, "Heat gear 1, room 0.28°C above target (< H_DN_10=0.3): hold gear 1");

    // User raises setpoint to 68: diff = 63.5 - 68 = -4.5°F = -2.5°C — max heat
    gear = select_heat_gear(1, room, target_68, true, 0, true, 0, 100000);
    EXPECT_GEAR(3, gear, "Heat setpoint raise to 68°F: jump to gear 3 (max heat)");
}

TEST(heat_user_lowers_setpoint_below_room) {
    printf("\n=== Heating: lower setpoint below room ===\n");

    float room = f_to_c(68.0f);
    float new_target = f_to_c(63.0f);

    // diff = 68 - 63 = 5°F = 2.78°C — well above target, should idle
    int gear = select_heat_gear(3, room, new_target, true, 0, true, 0, 100000);
    EXPECT_GEAR(0, gear, "Heat gear 3, lower setpoint below room: should go to idle");
}

// --- Replay of last night's full timeline ---

TEST(replay_april4_incident) {
    printf("\n=== Full replay: April 4 incident timeline ===\n");

    float target_66 = f_to_c(66.0f);
    float target_63 = f_to_c(63.0f);
    int gear;
    uint32_t now = 1000000;  // arbitrary start

    // 11:00pm: gear 4, target 66, room 68.84
    gear = 4;
    printf("  [11:00] gear=%d, room=68.84, target=66\n", gear);

    // 11:08: gear->5 (normal upshift, hold timer passed)
    gear = select_cool_gear(gear, f_to_c(68.18f), target_66, false, 600001, true, 0, now);
    EXPECT_GEAR(5, gear, "[11:08] room 68.18, hold timer passed: upshift to 5");

    // 11:24: temp dropping, downshift
    gear = select_cool_gear(gear, f_to_c(66.68f), target_66, false, 999999, true, 0, now);
    EXPECT_GEAR(4, gear, "[11:24] room 66.68 (diff=0.38°C < C_DN_54=0.45): downshift");

    // Continue downshifting...
    gear = select_cool_gear(gear, f_to_c(66.44f), target_66, false, 300001, true, 0, now);
    EXPECT_GEAR(3, gear, "[11:30] room 66.44 (diff=0.24°C < C_DN_43=0.25): downshift");

    gear = select_cool_gear(gear, f_to_c(66.14f), target_66, false, 300001, true, 0, now);
    EXPECT_GEAR(2, gear, "[11:33] room 66.14 (diff=0.08°C < C_DN_32=0.10): downshift");

    // 11:40: USER DROPS SETPOINT TO 63°F
    printf("\n  >>> USER CHANGES SETPOINT 66→63°F <<<\n\n");
    gear = select_cool_gear(gear, f_to_c(66.2f), target_63, true, 0, true, 0, now);
    EXPECT_GEAR(5, gear, "[11:40] user_input=true, room 66.2 (diff=1.78°C): JUMP to gear 5");

    // 11:55: temp dropping at gear 5
    gear = select_cool_gear(gear, f_to_c(65.1f), target_63, false, 999999, true, 0, now);
    EXPECT_GEAR(5, gear, "[11:55] room 65.1 (diff=1.17°C > 0.45): stay at 5");

    // 12:05: approaching setpoint
    gear = select_cool_gear(gear, f_to_c(63.92f), target_63, false, 999999, true, 0, now);
    EXPECT_GEAR(5, gear, "[12:05] room 63.92 (diff=0.51°C > 0.45): stay at 5");

    // 12:08: should downshift!
    gear = select_cool_gear(gear, f_to_c(63.74f), target_63, false, 999999, true, 0, now);
    EXPECT_GEAR(4, gear, "[12:08] room 63.74 (diff=0.41°C < 0.45): DOWNSHIFT to 4");

    // Continue downshifting
    gear = select_cool_gear(gear, f_to_c(63.5f), target_63, false, 300001, true, 0, now);
    EXPECT_GEAR(4, gear, "[12:10] room 63.5 (diff=0.28°C > C_DN_43=0.25): stay at 4");

    gear = select_cool_gear(gear, f_to_c(63.3f), target_63, false, 300001, true, 0, now);
    EXPECT_GEAR(3, gear, "[12:12] room 63.3 (diff=0.17°C < C_DN_43=0.25): downshift to 3");

    gear = select_cool_gear(gear, f_to_c(63.1f), target_63, false, 300001, true, 0, now);
    EXPECT_GEAR(2, gear, "[12:13] room 63.1 (diff=0.06°C < C_DN_32=0.10): downshift to 2");
}

// ============================================================
// Main
// ============================================================

int main() {
    // Register all tests
    test_cool_setpoint_drop_user_input_jumps_to_gear5();
    test_cool_setpoint_drop_without_user_input_OLD_behavior();
    test_cool_gear5_downshift_near_setpoint();
    test_cool_gear5_stale_target_reproduces_bug();
    test_cool_user_raises_setpoint_above_room();
    test_cool_fresh_start_various_diffs();
    test_heat_setpoint_raise_user_input();
    test_heat_user_lowers_setpoint_below_room();
    test_replay_april4_incident();

    printf("\n========================================\n");
    printf("Results: %d/%d passed, %d failed\n", tests_passed, tests_run, tests_failed);
    printf("========================================\n");

    return tests_failed > 0 ? 1 : 0;
}
