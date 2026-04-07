// 24-hour gear controller simulation
// Models thermal behavior of a small camper with 18kBTU Furrion Chill Cube
// Build: g++ -std=c++17 -o tests/sim_test tests/simulation_test.cpp && ./tests/sim_test

#include <cstdio>
#include <cstdint>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>

// ============================================================
// Constants (from furrion_chill_cube.cpp)
// ============================================================

static const uint32_t HOLD_MS[] = {0, 180000, 180000, 300000, 300000, 600000};
static const uint32_t IDLE_BEFORE_OFF_MS = 900000;

static const float H_UP_01 = -0.3f;
static const float H_UP_12 = -0.8f;
static const float H_UP_23 = -1.5f;
static const float H_DN_32 = -0.8f;
static const float H_DN_21 = -0.3f;
static const float H_DN_10 =  0.3f;
static const float H_IDLE  =  0.3f;

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
// Helpers
// ============================================================

float f_to_c(float f) { return (f - 32.0f) * (5.0f / 9.0f); }
float c_to_f(float c) { return c * (9.0f / 5.0f) + 32.0f; }

// ============================================================
// Gear selection (from furrion_chill_cube.cpp, current code)
// ============================================================

int select_cool_gear(int current_gear, float room_c, float target_c,
                     bool user_input, uint32_t time_in_gear, bool boot_ready,
                     uint32_t idle_since, uint32_t now) {
    float diff = room_c - target_c;
    int gear = current_gear;
    int new_gear = gear;
    bool idle_long_enough = (idle_since > 0) && (now - idle_since >= IDLE_BEFORE_OFF_MS);
    auto can_upshift_to = [&](int tg) -> bool {
        return user_input || (time_in_gear >= HOLD_MS[tg]);
    };

    if (gear == -1 || user_input) {
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
                bool far = (diff < -1.1f);
                if ((imm_off || (idle_long_enough && far)) && diff < C_IDLE) new_gear = -1;
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
    auto can_upshift_to = [&](int tg) -> bool {
        return user_input || (time_in_gear >= HOLD_MS[tg]);
    };

    if (gear == -1 || user_input) {
        if (diff < H_UP_23)         new_gear = 3;
        else if (diff < H_UP_12)    new_gear = 2;
        else if (diff < H_UP_01)    new_gear = 1;
        else                        new_gear = (gear == -1) ? -1 : 0;
    } else {
        switch (gear) {
            case 0: {
                if (can_upshift_to(1) && diff < H_UP_01) new_gear = 1;
                bool imm_off = !boot_ready;
                bool far = (diff > 1.1f);
                if ((imm_off || (idle_long_enough && far)) && diff > H_IDLE) new_gear = -1;
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
// Thermal model — small camper
// ============================================================

// Approximate cooling/heating power per gear (°F/min change to room temp)
// Based on observed data: gear 5 cools ~0.5°F/min in a small camper
static const float COOL_RATE[] = {0.0f, -0.10f, -0.18f, -0.28f, -0.40f, -0.55f};  // gear 0-5
static const float HEAT_RATE[] = {0.0f,  0.12f,  0.22f,  0.35f};                   // gear 0-3

// Natural drift toward outside temp (°F/min per °F of inside-outside delta)
static const float THERMAL_LEAK_RATE = 0.003f;  // Small camper, moderate insulation

struct SimState {
    float room_f;
    float target_f;
    int cool_gear;
    int heat_gear;
    bool cooling_active;  // vs heating
    uint32_t now_ms;
    uint32_t last_gear_change_ms;
    uint32_t idle_since_ms;

    // Stats
    float min_temp_f;
    float max_temp_f;
    float max_overshoot_f;
    int gear_changes;
    int max_gear_reached;
    std::vector<std::string> events;
};

void log_event(SimState &s, const char *fmt, ...) {
    char buf[256];
    int hrs = (s.now_ms / 3600000) % 24;
    int mins = (s.now_ms / 60000) % 60;
    int offset = snprintf(buf, sizeof(buf), "  %02d:%02d | room=%5.1f°F gear=%s%d | ",
                          hrs, mins, s.room_f,
                          s.cooling_active ? "C" : "H",
                          s.cooling_active ? s.cool_gear : s.heat_gear);
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf + offset, sizeof(buf) - offset, fmt, args);
    va_end(args);
    s.events.push_back(buf);
}

void simulate_step(SimState &s, float outside_f, float dt_min) {
    // Apply thermal model: HVAC effect + natural drift
    float hvac_effect = 0.0f;
    if (s.cooling_active && s.cool_gear >= 1) {
        hvac_effect = COOL_RATE[s.cool_gear] * dt_min;
    } else if (!s.cooling_active && s.heat_gear >= 1) {
        hvac_effect = HEAT_RATE[s.heat_gear] * dt_min;
    }
    float drift = THERMAL_LEAK_RATE * (outside_f - s.room_f) * dt_min;
    s.room_f += hvac_effect + drift;

    // Update stats
    s.min_temp_f = std::min(s.min_temp_f, s.room_f);
    s.max_temp_f = std::max(s.max_temp_f, s.room_f);

    // Run gear controller
    float room_c = f_to_c(s.room_f);
    float target_c = f_to_c(s.target_f);
    uint32_t time_in_gear = (s.last_gear_change_ms == 0) ? 999999999 :
                            (s.now_ms - s.last_gear_change_ms);

    int old_cool = s.cool_gear;
    int old_heat = s.heat_gear;

    if (s.cooling_active) {
        int new_gear = select_cool_gear(s.cool_gear, room_c, target_c,
                                        false, time_in_gear, true,
                                        s.idle_since_ms, s.now_ms);
        if (new_gear != s.cool_gear) {
            s.cool_gear = new_gear;
            s.last_gear_change_ms = s.now_ms;
            s.gear_changes++;
            if (new_gear > s.max_gear_reached) s.max_gear_reached = new_gear;
            if (new_gear == 0 && old_cool != 0) s.idle_since_ms = s.now_ms;
            else if (new_gear != 0) s.idle_since_ms = 0;
            log_event(s, "gear %d → %d (diff=%.2f°C)", old_cool, new_gear, room_c - target_c);
        }
    } else {
        int new_gear = select_heat_gear(s.heat_gear, room_c, target_c,
                                        false, time_in_gear, true,
                                        s.idle_since_ms, s.now_ms);
        if (new_gear != s.heat_gear) {
            s.heat_gear = new_gear;
            s.last_gear_change_ms = s.now_ms;
            s.gear_changes++;
            if (new_gear > s.max_gear_reached) s.max_gear_reached = new_gear;
            if (new_gear == 0 && old_heat != 0) s.idle_since_ms = s.now_ms;
            else if (new_gear != 0) s.idle_since_ms = 0;
            log_event(s, "gear %d → %d (diff=%.2f°C)", old_heat, new_gear, room_c - target_c);
        }
    }
}

void user_setpoint_change(SimState &s, float new_target_f) {
    float old = s.target_f;
    s.target_f = new_target_f;

    float room_c = f_to_c(s.room_f);
    float target_c = f_to_c(new_target_f);

    if (s.cooling_active) {
        int old_gear = s.cool_gear;
        s.cool_gear = select_cool_gear(s.cool_gear, room_c, target_c,
                                       true, 0, true, s.idle_since_ms, s.now_ms);
        if (s.cool_gear != old_gear) {
            s.last_gear_change_ms = s.now_ms;
            s.gear_changes++;
            if (s.cool_gear == 0 && old_gear != 0) s.idle_since_ms = s.now_ms;
            else if (s.cool_gear != 0) s.idle_since_ms = 0;
        }
        log_event(s, "USER setpoint %.0f→%.0f°F → gear %d→%d", old, new_target_f, old_gear, s.cool_gear);
    } else {
        int old_gear = s.heat_gear;
        s.heat_gear = select_heat_gear(s.heat_gear, room_c, target_c,
                                       true, 0, true, s.idle_since_ms, s.now_ms);
        if (s.heat_gear != old_gear) {
            s.last_gear_change_ms = s.now_ms;
            s.gear_changes++;
            if (s.heat_gear == 0 && old_gear != 0) s.idle_since_ms = s.now_ms;
            else if (s.heat_gear != 0) s.idle_since_ms = 0;
        }
        log_event(s, "USER setpoint %.0f→%.0f°F → gear %d→%d", old, new_target_f, old_gear, s.heat_gear);
    }
}

// Outside temperature profile: sinusoidal with min at 5am, max at 3pm
float outside_temp(float hour, float daily_low, float daily_high) {
    float mid = (daily_low + daily_high) / 2.0f;
    float amp = (daily_high - daily_low) / 2.0f;
    // Phase: minimum at 5am (hour 5), maximum at 5pm (hour 17)
    float phase = (hour - 5.0f) / 24.0f * 2.0f * 3.14159f;
    return mid - amp * cosf(phase);
}

struct SimResult {
    std::string name;
    float min_temp_f;
    float max_temp_f;
    float max_overshoot_below_f;
    float max_overshoot_above_f;
    int total_gear_changes;
    int max_gear;
    std::vector<std::string> events;
    bool passed;
    std::string failure_reason;
};

SimResult run_simulation(const char *name, float daily_low_outside, float daily_high_outside,
                         bool start_cooling, float start_room_f, float start_target_f,
                         int start_hour) {
    printf("\n╔══════════════════════════════════════════════════════════╗\n");
    printf("║  %s\n", name);
    printf("║  Outside: %.0f-%.0f°F  |  Start: %.0f°F room, %.0f°F target\n",
           daily_low_outside, daily_high_outside, start_room_f, start_target_f);
    printf("╚══════════════════════════════════════════════════════════╝\n");

    SimState s = {};
    s.room_f = start_room_f;
    s.target_f = start_target_f;
    s.cool_gear = start_cooling ? 0 : -1;
    s.heat_gear = start_cooling ? -1 : 0;
    s.cooling_active = start_cooling;
    s.now_ms = start_hour * 3600000;
    s.last_gear_change_ms = s.now_ms;
    s.idle_since_ms = s.now_ms;
    s.min_temp_f = start_room_f;
    s.max_temp_f = start_room_f;
    s.gear_changes = 0;
    s.max_gear_reached = 0;

    float dt_min = 1.0f;  // 1-minute simulation steps
    uint32_t step_ms = (uint32_t)(dt_min * 60000);

    // Track overshoot relative to setpoint
    float max_overshoot_below = 0.0f;  // how far below setpoint (cooling overshoot)
    float max_overshoot_above = 0.0f;  // how far above setpoint (heating overshoot)

    // Simulate 24 hours
    for (int step = 0; step < 24 * 60; step++) {
        float hour = (float)((s.now_ms / 60000) % (24 * 60)) / 60.0f;
        float outside_f = outside_temp(hour, daily_low_outside, daily_high_outside);

        // Setpoint schedule: 63°F from 11pm-8am, 68°F from 8am-11pm
        float scheduled_target;
        if (hour >= 23.0f || hour < 8.0f) {
            scheduled_target = 63.0f;
        } else {
            scheduled_target = 68.0f;
        }

        // Apply setpoint change if different
        if (fabsf(scheduled_target - s.target_f) > 0.1f) {
            user_setpoint_change(s, scheduled_target);
        }

        // Simulate one step
        simulate_step(s, outside_f, dt_min);

        // Track overshoot
        float below = s.target_f - s.room_f;  // positive = room below setpoint
        float above = s.room_f - s.target_f;  // positive = room above setpoint
        if (below > max_overshoot_below) max_overshoot_below = below;
        if (above > max_overshoot_above) max_overshoot_above = above;

        // Hourly summary
        if (step > 0 && step % 60 == 0) {
            int active_gear = s.cooling_active ? s.cool_gear : s.heat_gear;
            printf("  %02d:00 | room=%5.1f°F | target=%2.0f°F | outside=%4.1f°F | %s gear %d\n",
                   ((int)hour) % 24, s.room_f, s.target_f, outside_f,
                   s.cooling_active ? "COOL" : "HEAT", active_gear);
        }

        s.now_ms += step_ms;
    }

    // Print notable events (gear changes, setpoint changes)
    printf("\n  --- Key Events ---\n");
    for (auto &e : s.events) {
        printf("%s\n", e.c_str());
    }

    // Results
    SimResult r;
    r.name = name;
    r.min_temp_f = s.min_temp_f;
    r.max_temp_f = s.max_temp_f;
    r.max_overshoot_below_f = max_overshoot_below;
    r.max_overshoot_above_f = max_overshoot_above;
    r.total_gear_changes = s.gear_changes;
    r.max_gear = s.max_gear_reached;
    r.events = s.events;
    r.passed = true;
    r.failure_reason = "";

    // Validation checks
    if (max_overshoot_below > 5.0f) {
        r.passed = false;
        r.failure_reason += "EXCESSIVE cooling overshoot: " + std::to_string(max_overshoot_below) + "°F below setpoint. ";
    }
    if (max_overshoot_above > 5.0f) {
        r.passed = false;
        r.failure_reason += "EXCESSIVE heating overshoot: " + std::to_string(max_overshoot_above) + "°F above setpoint. ";
    }
    if (s.min_temp_f < 55.0f) {
        r.passed = false;
        r.failure_reason += "Room dropped to " + std::to_string(s.min_temp_f) + "°F (dangerously cold). ";
    }
    if (s.max_temp_f > 80.0f) {
        r.passed = false;
        r.failure_reason += "Room reached " + std::to_string(s.max_temp_f) + "°F (dangerously hot). ";
    }

    printf("\n  --- Summary ---\n");
    printf("  Room range: %.1f - %.1f°F\n", s.min_temp_f, s.max_temp_f);
    printf("  Max overshoot: %.1f°F below setpoint, %.1f°F above setpoint\n",
           max_overshoot_below, max_overshoot_above);
    printf("  Gear changes: %d  |  Max gear: %d\n", s.gear_changes, s.max_gear_reached);
    printf("  Result: %s%s\n", r.passed ? "PASS" : "FAIL",
           r.passed ? "" : (" — " + r.failure_reason).c_str());

    return r;
}

// ============================================================
// Scenarios
// ============================================================

int main() {
    std::vector<SimResult> results;

    // Scenario 1: Cool fall day — heating only
    // Outside: 42-58°F. Heat pump operates within range (>40°F lockout).
    results.push_back(run_simulation(
        "Cool Fall Day (heating only)",
        42.0f, 58.0f,    // outside low/high
        false,            // start in heating mode
        65.0f, 68.0f,     // start room/target
        8                 // start at 8am
    ));

    // Scenario 2: Hot summer day — cooling only
    // Outside: 75-95°F. Cooling all day.
    results.push_back(run_simulation(
        "Hot Summer Day (cooling only)",
        75.0f, 95.0f,    // outside low/high
        true,             // start in cooling mode
        70.0f, 68.0f,     // start room/target (slightly warm)
        8                 // start at 8am
    ));

    // Scenario 3: Spring transition day — heat AM, cool PM
    // Outside: 45-78°F. Need heat in early morning, cooling by afternoon.
    results.push_back(run_simulation(
        "Spring Day (heat morning, cool afternoon)",
        45.0f, 78.0f,    // outside low/high
        false,            // start in heating mode
        63.0f, 63.0f,     // start at sleep temp
        0                 // start at midnight
    ));

    // Scenario 4: Mild day — minimal HVAC needed
    // Outside: 60-72°F. Room stays near setpoint naturally.
    results.push_back(run_simulation(
        "Mild Day (minimal HVAC)",
        60.0f, 72.0f,
        true,             // cooling mode
        68.0f, 68.0f,
        8
    ));

    // Scenario 5: Cold spring morning — heat needed overnight, warms by afternoon
    // Outside: 40-65°F. Tests heat at lower boundary of operating range.
    results.push_back(run_simulation(
        "Cold Spring (heat boundary, 40-65°F outside)",
        40.0f, 65.0f,
        false,
        63.0f, 63.0f,    // start at sleep temp
        0                 // start at midnight
    ));

    // Scenario 6: Extreme heat — stress test cooling
    // Outside: 85-105°F. Cooling hard all day.
    results.push_back(run_simulation(
        "Extreme Heat (stress test)",
        85.0f, 105.0f,
        true,
        75.0f, 68.0f,
        8
    ));

    // Final summary
    printf("\n╔══════════════════════════════════════════════════════════╗\n");
    printf("║  SIMULATION SUMMARY                                      ║\n");
    printf("╠══════════════════════════════════════════════════════════╣\n");
    int pass = 0, fail = 0;
    for (auto &r : results) {
        printf("║  %s%-45s%s  %-4s ║\n",
               r.passed ? "" : "",
               r.name.c_str(),
               r.passed ? "" : "",
               r.passed ? "PASS" : "FAIL");
        printf("║    Room: %5.1f-%5.1f°F | Overshoot: ↓%.1f ↑%.1f | Gear Δ: %3d  ║\n",
               r.min_temp_f, r.max_temp_f,
               r.max_overshoot_below_f, r.max_overshoot_above_f,
               r.total_gear_changes);
        if (!r.passed) {
            printf("║    ⚠ %s\n", r.failure_reason.c_str());
            fail++;
        } else {
            pass++;
        }
    }
    printf("╠══════════════════════════════════════════════════════════╣\n");
    printf("║  %d passed, %d failed                                      ║\n", pass, fail);
    printf("╚══════════════════════════════════════════════════════════╝\n");

    return fail > 0 ? 1 : 0;
}
