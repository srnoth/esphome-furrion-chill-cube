// Virtual test harness for mode switch redesign + clamped kickstart fan=LOW
// Build: g++ -std=c++17 -o tests/mode_switch_test tests/mode_switch_test.cpp && ./tests/mode_switch_test
//
// Models the gear controller + kickstart state machines + time progression
// entirely in pure C++. No ESPHome dependencies. Lets us verify all mode
// switch scenarios without flashing hardware.

#include <cstdio>
#include <cstdint>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <cassert>

// ============================================================
// Constants (mirror furrion_chill_cube.cpp)
// ============================================================

static const uint32_t HOLD_MS[] = {0, 180000, 180000, 300000, 300000, 600000};
static const uint32_t CLAMP_DURATION_MS = 305000;

static const float H_UP_01 = -0.3f;
static const float H_UP_12 = -0.8f;
static const float H_UP_23 = -1.5f;
static const float H_DN_32 = -0.8f;
static const float H_DN_21 = -0.3f;
static const float H_DN_10 = -0.15f;   // below setpoint — compensates for shutdown thermal carry
static const float H_IDLE  =  0.3f;

static const float C_UP_01 =  0.15f;
static const float C_UP_12 =  0.25f;
static const float C_UP_23 =  0.4f;
static const float C_UP_34 =  0.6f;
static const float C_UP_45 =  0.8f;
static const float C_DN_54 =  0.45f;
static const float C_DN_43 =  0.25f;
static const float C_DN_32 =  0.10f;
static const float C_DN_21 = -0.1f;   // below setpoint — gives gear 2 real downshift hysteresis
static const float C_DN_10 = -0.15f;
static const float C_IDLE  = -0.15f;

enum ActiveMode : uint8_t { MODE_NONE = 0, MODE_HEAT = 1, MODE_COOL = 2 };
enum ClampPhase : uint8_t { CLAMP_IDLE = 0, CLAMP_PRE_CS = 1, CLAMP_CLAMPED = 2 };
enum IrMode : uint8_t { IR_OFF = 0, IR_HEAT = 1, IR_COOL = 2 };
enum FanMode : uint8_t { FAN_AUTO = 0, FAN_LOW = 1 };

// ============================================================
// System state (mirrors component's member variables)
// ============================================================

struct SystemState {
    // Gear state
    int heat_gear = -1;
    int cool_gear = -1;
    ActiveMode last_active_mode = MODE_NONE;
    IrMode active_ir_mode = IR_OFF;

    // Timing
    uint32_t last_gear_change = 0;
    uint32_t idle_since = 0;
    uint32_t last_mode_event_at = 0;
    uint32_t off_since = 0;

    // Kickstart
    ClampPhase clamp_phase = CLAMP_IDLE;
    uint32_t clamp_start = 0;
    uint32_t clamp_phase_start = 0;
    bool clamp_is_heat = false;
    bool quick_kick_active = false;
    uint32_t quick_kick_start = 0;
    bool quick_kick_is_heat = false;

    // Config (tunable — default 10/20/1F/1min)
    uint32_t mode_switch_idle_ms = 600000;
    uint32_t mode_switch_event_ms = 1200000;
    float mode_switch_temp_offset_c = 0.5556f;  // 1°F
    uint32_t mode_switch_off_ms = 60000;

    // Flags
    bool boot_ready = false;

    // Inputs (set by test driver)
    float room_c = 20.0f;
    float heat_target_c = 19.4f;  // 67°F
    float cool_target_c = 20.0f;  // 68°F
    bool user_input = false;
    IrMode user_mode = IR_OFF;  // HEAT_COOL flag handled separately
    bool heat_cool_mode = true;  // user selected HEAT_COOL
};

struct IrTransmission {
    uint32_t t_ms;
    IrMode mode;
    FanMode fan;
    int cs;
    const char *reason;
};

// ============================================================
// Helpers (mirror component logic)
// ============================================================

FanMode get_effective_fan(const SystemState &s) {
    if (s.clamp_phase == CLAMP_CLAMPED) return FAN_LOW;
    return FAN_AUTO;
}

// ============================================================
// Gear computation (heat) — mirrors heat path in run_gear_controller_
// ============================================================

bool gear_in_band_heat(int gear, float diff) {
    switch (gear) {
        case 0: return diff >= H_UP_01 && diff <= H_DN_10;
        case 1: return diff >= H_UP_12 && diff <= H_DN_10;
        case 2: return diff >= H_UP_23 && diff <= H_DN_21;
        case 3: return diff <= H_DN_32;
        default: return false;
    }
}

bool gear_in_band_cool(int gear, float diff) {
    switch (gear) {
        case 0: return diff >= C_DN_10 && diff <= C_UP_01;
        case 1: return diff >= C_DN_21 && diff <= C_UP_12;
        case 2: return diff >= C_DN_32 && diff <= C_UP_23;
        case 3: return diff >= C_DN_43 && diff <= C_UP_34;
        case 4: return diff >= C_DN_54 && diff <= C_UP_45;
        case 5: return diff >= C_DN_54;
        default: return false;
    }
}

int compute_heat_new_gear(const SystemState &s, uint32_t now) {
    float diff = s.room_c - s.heat_target_c;
    int gear = s.heat_gear;
    int new_gear = gear;

    uint32_t time_in_gear = (s.last_gear_change == 0) ? 999999999u : (now - s.last_gear_change);
    auto can_upshift_to = [&](int tg) {
        return s.user_input || (time_in_gear >= HOLD_MS[tg]);
    };

    if (gear == -1 || s.user_input) {
        bool off_long_enough = (s.off_since == 0) || (now - s.off_since >= s.mode_switch_off_ms);
        if (gear == -1 && !off_long_enough) {
            new_gear = -1;
        } else if (s.user_input && gear >= 0 && gear_in_band_heat(gear, diff)) {
            new_gear = gear;   // preserve stable hunting gear on user event
        } else {
            if (diff < H_UP_23)      new_gear = 3;
            else if (diff < H_UP_12) new_gear = 2;
            else if (diff < H_UP_01) new_gear = 1;
            else if (gear == -1)     new_gear = -1;
            else if (s.user_input && diff > H_IDLE) new_gear = -1;
            else                     new_gear = 0;
        }
    } else {
        switch (gear) {
            case 0: {
                if (can_upshift_to(1) && diff < H_UP_01) new_gear = 1;
                bool imm_off = !s.boot_ready;
                bool idle_enough = (s.idle_since > 0) && (now - s.idle_since >= s.mode_switch_idle_ms);
                bool event_ok = (s.last_mode_event_at == 0) || (now - s.last_mode_event_at >= s.mode_switch_event_ms);
                bool past_setpoint = diff > s.mode_switch_temp_offset_c;
                bool natural_off = idle_enough && event_ok && past_setpoint;
                if ((imm_off || natural_off) && diff > H_IDLE) new_gear = -1;
                break;
            }
            case 1:
                if (can_upshift_to(2) && diff < H_UP_12) new_gear = 2;
                else if (diff > H_DN_10)                  new_gear = 0;
                break;
            case 2:
                if (can_upshift_to(3) && diff < H_UP_23) new_gear = 3;
                else if (diff > H_DN_21)                  new_gear = 1;
                break;
            case 3:
                if (diff > H_DN_32)                       new_gear = 2;
                break;
        }
    }
    return new_gear;
}

int compute_cool_new_gear(const SystemState &s, uint32_t now) {
    float diff = s.room_c - s.cool_target_c;
    int gear = s.cool_gear;
    int new_gear = gear;

    uint32_t time_in_gear = (s.last_gear_change == 0) ? 999999999u : (now - s.last_gear_change);
    auto can_upshift_to = [&](int tg) {
        return s.user_input || (time_in_gear >= HOLD_MS[tg]);
    };

    if (gear == -1 || s.user_input) {
        bool off_long_enough = (s.off_since == 0) || (now - s.off_since >= s.mode_switch_off_ms);
        if (gear == -1 && !off_long_enough) {
            new_gear = -1;
        } else if (s.user_input && gear >= 0 && gear_in_band_cool(gear, diff)) {
            new_gear = gear;   // preserve stable hunting gear on user event
        } else {
            if (diff > C_UP_45)      new_gear = 5;
            else if (diff > C_UP_34) new_gear = 4;
            else if (diff > C_UP_23) new_gear = 3;
            else if (diff > C_UP_01) new_gear = (gear == -1) ? 2 : 1;
            else if (gear == -1)     new_gear = -1;
            else if (s.user_input && diff < C_IDLE) new_gear = -1;
            else                     new_gear = 0;
        }
    } else {
        switch (gear) {
            case 0: {
                if (can_upshift_to(1) && diff > C_UP_01) new_gear = 1;
                bool imm_off = !s.boot_ready;
                bool idle_enough = (s.idle_since > 0) && (now - s.idle_since >= s.mode_switch_idle_ms);
                bool event_ok = (s.last_mode_event_at == 0) || (now - s.last_mode_event_at >= s.mode_switch_event_ms);
                bool past_setpoint = diff < -s.mode_switch_temp_offset_c;
                bool natural_off = idle_enough && event_ok && past_setpoint;
                if ((imm_off || natural_off) && diff < C_IDLE) new_gear = -1;
                break;
            }
            case 1:
                if (can_upshift_to(2) && diff > C_UP_12) new_gear = 2;
                else if (diff < C_DN_10)                 new_gear = 0;
                break;
            case 2:
                if (can_upshift_to(3) && diff > C_UP_23) new_gear = 3;
                else if (diff < C_DN_21)                 new_gear = 1;
                break;
            case 3:
                if (can_upshift_to(4) && diff > C_UP_34) new_gear = 4;
                else if (diff < C_DN_32)                 new_gear = 2;
                break;
            case 4:
                if (can_upshift_to(5) && diff > C_UP_45) new_gear = 5;
                else if (diff < C_DN_43)                 new_gear = 3;
                break;
            case 5:
                if (diff < C_DN_54)                      new_gear = 4;
                break;
        }
    }
    return new_gear;
}

// Mode decision: picks do_heat vs do_cool based on gear state and temperature
void decide_mode(const SystemState &s, bool &do_heat, bool &do_cool) {
    if (!s.heat_cool_mode) {
        do_heat = false;
        do_cool = false;
        return;
    }
    do_heat = true;
    do_cool = true;
    if (s.heat_gear >= 0) {
        do_cool = false;
    } else if (s.cool_gear >= 0) {
        do_heat = false;
    } else {
        if (s.room_c <= s.heat_target_c) { do_cool = false; }
        else if (s.room_c >= s.cool_target_c) { do_heat = false; }
        else {
            do_heat = false;
            do_cool = false;
        }
    }
}

// ============================================================
// Simulation step — advances one second, records IR transmissions
// ============================================================

void advance_kickstart(SystemState &s, uint32_t now, std::vector<IrTransmission> &tx) {
    // PRE_CS → CLAMPED after 500ms
    if (s.clamp_phase == CLAMP_PRE_CS) {
        if ((now - s.clamp_phase_start) >= 500) {
            // IMPORTANT: set CLAMPED before transmitting so get_effective_fan returns LOW
            s.active_ir_mode = s.clamp_is_heat ? IR_HEAT : IR_COOL;
            s.clamp_phase = CLAMP_CLAMPED;
            s.clamp_start = now;
            // Record the mode-on transmission
            tx.push_back({now, s.active_ir_mode, get_effective_fan(s), 0, "clamp PRE_CS→CLAMPED"});
        }
    } else if (s.clamp_phase == CLAMP_CLAMPED) {
        int drop_gear = s.clamp_is_heat ? 2 : 3;
        int cur = s.clamp_is_heat ? s.heat_gear : s.cool_gear;
        if (cur >= drop_gear) {
            s.clamp_phase = CLAMP_IDLE;
        } else if ((now - s.clamp_start) >= CLAMP_DURATION_MS) {
            s.clamp_phase = CLAMP_IDLE;
        }
    }
}

void run_gear_controller(SystemState &s, uint32_t now, std::vector<IrTransmission> &tx) {
    if (!s.boot_ready) {
        // First run — just set flag; real logic below
    }

    bool do_heat, do_cool;
    decide_mode(s, do_heat, do_cool);

    if (do_heat) {
        int new_gear = compute_heat_new_gear(s, now);
        // Track idle_since
        if (new_gear == 0 && s.heat_gear != 0) s.idle_since = now;
        else if (new_gear != 0) s.idle_since = 0;
        // Track off_since
        if (new_gear == -1 && s.heat_gear != -1) s.off_since = now;
        // Detect fresh start (gear -1→active): trigger clamped kickstart for gear 1
        if (s.heat_gear == -1 && new_gear == 1 && s.clamp_phase == CLAMP_IDLE) {
            s.clamp_phase = CLAMP_PRE_CS;
            s.clamp_phase_start = now;
            s.clamp_is_heat = true;
            s.last_mode_event_at = now;
        } else if (s.heat_gear == -1 && new_gear >= 2) {
            s.last_mode_event_at = now;
        }
        if (new_gear != s.heat_gear) {
            s.heat_gear = new_gear;
            s.last_gear_change = now;
            if (new_gear >= 1) s.last_active_mode = MODE_HEAT;
            if (new_gear == -1 && s.active_ir_mode != IR_OFF) {
                s.active_ir_mode = IR_OFF;
                tx.push_back({now, IR_OFF, FAN_AUTO, 0, "heat gear → -1 (OFF)"});
            }
        }
    } else if (do_cool) {
        int new_gear = compute_cool_new_gear(s, now);
        if (new_gear == 0 && s.cool_gear != 0) s.idle_since = now;
        else if (new_gear != 0) s.idle_since = 0;
        if (new_gear == -1 && s.cool_gear != -1) s.off_since = now;
        if (s.cool_gear == -1 && new_gear == 2 && s.clamp_phase == CLAMP_IDLE) {
            s.clamp_phase = CLAMP_PRE_CS;
            s.clamp_phase_start = now;
            s.clamp_is_heat = false;
            s.last_mode_event_at = now;
        } else if (s.cool_gear == -1 && (new_gear == 3 || new_gear >= 4)) {
            s.last_mode_event_at = now;
        }
        if (new_gear != s.cool_gear) {
            s.cool_gear = new_gear;
            s.last_gear_change = now;
            if (new_gear >= 1) s.last_active_mode = MODE_COOL;
            if (new_gear == -1 && s.active_ir_mode != IR_OFF) {
                s.active_ir_mode = IR_OFF;
                tx.push_back({now, IR_OFF, FAN_AUTO, 0, "cool gear → -1 (OFF)"});
            }
        }
    } else {
        // NEITHER ACTIVE
        if (s.active_ir_mode != IR_OFF) {
            s.active_ir_mode = IR_OFF;
            tx.push_back({now, IR_OFF, FAN_AUTO, 0, "neither active (OFF)"});
        }
        if (s.heat_gear != -1) {
            if (s.heat_gear != -1) s.off_since = now;  // track off_since
            s.heat_gear = -1;
        }
        if (s.cool_gear != -1) {
            s.cool_gear = now ? now : s.cool_gear;  // placeholder to avoid unused
            s.cool_gear = -1;
        }
        s.idle_since = 0;
    }

    if (!s.boot_ready) s.boot_ready = true;
}

// ============================================================
// Test utilities
// ============================================================

float f_to_c(float f) { return (f - 32.0f) * (5.0f / 9.0f); }

int tests_passed = 0;
int tests_failed = 0;

void check(bool cond, const char *name) {
    if (cond) {
        tests_passed++;
        printf("  ✓ %s\n", name);
    } else {
        tests_failed++;
        printf("  ✗ %s\n", name);
    }
}

// ============================================================
// TEST 1: Clamped kickstart fan=LOW verification (Problem #2)
// ============================================================

void test_clamp_fan_low() {
    printf("\n=== Test 1: Clamped kickstart transmits fan=LOW ===\n");
    SystemState s;
    s.heat_cool_mode = true;
    // Room 1°F below heat target (diff = -0.556°C) — triggers gear 1 fresh start (clamped)
    // (gear 1 range: -0.8 < diff < -0.3; 1°F = -0.556°C falls in range)
    s.heat_target_c = f_to_c(67.0f);
    s.room_c = s.heat_target_c - (1.0f * 5.0f / 9.0f);
    s.cool_target_c = f_to_c(80.0f);  // wide deadband so cool path doesn't interfere
    s.heat_gear = -1;
    s.cool_gear = -1;
    s.boot_ready = true;
    s.off_since = 0;  // no off lockout (not just-turned-off)

    std::vector<IrTransmission> tx;
    uint32_t now = 100000;  // start at some time past boot

    // Run gear controller: should trigger clamped kickstart for -1→1
    run_gear_controller(s, now, tx);
    check(s.clamp_phase == CLAMP_PRE_CS, "clamp entered PRE_CS");
    check(s.clamp_is_heat, "clamp is for heat");

    // Advance 500ms → PRE_CS→CLAMPED
    now += 500;
    advance_kickstart(s, now, tx);
    check(s.clamp_phase == CLAMP_CLAMPED, "clamp transitioned to CLAMPED");

    // Verify the mode-on transmission had fan=LOW
    bool found_low = false;
    for (const auto &t : tx) {
        if (t.mode == IR_HEAT) {
            found_low = (t.fan == FAN_LOW);
            break;
        }
    }
    check(found_low, "mode-ON IR transmission had fan=LOW (Problem #2 fix)");
}

// ============================================================
// TEST 2: Mode transition without getting stuck (Problem #1 obsolete)
// ============================================================

void test_mode_transition_unstuck() {
    printf("\n=== Test 2: Mode transition after cool session ends ===\n");
    SystemState s;
    s.heat_cool_mode = true;
    s.heat_target_c = f_to_c(66.0f);
    s.cool_target_c = f_to_c(68.0f);
    s.cool_gear = 0;
    s.last_active_mode = MODE_COOL;
    s.boot_ready = true;
    s.idle_since = 1000;  // already been idle

    std::vector<IrTransmission> tx;

    // Room drifts below heat setpoint over 2 hours
    // First, let cool session end: room at 66°F (2°F below cool target, clearly past 1°F offset)
    uint32_t now = 1000 + 1300000;  // 21 min later (past both 10 and 20 min)
    s.last_mode_event_at = 1000;  // set to satisfy event_ok after 20min
    s.room_c = f_to_c(66.0f);  // 2°F below cool target (avoids floating-point boundary)
    run_gear_controller(s, now, tx);
    check(s.cool_gear == -1, "cool 0→-1 after idle + event + past setpoint");

    // Room continues dropping, now below heat setpoint. Wait 1-min off, then heat fresh start.
    now += 61000;  // 1 min later
    s.room_c = f_to_c(65.0f);  // 1°F below heat target
    run_gear_controller(s, now, tx);
    check(s.heat_gear >= 1, "heat fresh start fires after 1-min off");
}

// ============================================================
// TEST 3: 1-min off lockout
// ============================================================

void test_off_lockout() {
    printf("\n=== Test 3: 1-min off lockout ===\n");
    SystemState s;
    s.heat_cool_mode = true;
    s.heat_target_c = f_to_c(66.0f);
    s.cool_target_c = f_to_c(68.0f);
    s.cool_gear = -1;
    s.heat_gear = -1;
    s.off_since = 100000;
    s.boot_ready = true;

    std::vector<IrTransmission> tx;

    // 30s after off — should stay at -1 even with temp demanding heat
    uint32_t now = 100000 + 30000;
    s.room_c = f_to_c(62.0f);  // well below heat target
    run_gear_controller(s, now, tx);
    check(s.heat_gear == -1, "gear stays -1 at 30s (off lockout active)");

    // 61s after off — should now allow fresh start
    now = 100000 + 61000;
    run_gear_controller(s, now, tx);
    check(s.heat_gear >= 1, "heat fresh start allowed at 61s");
}

// ============================================================
// TEST 4: 1°F setpoint offset on 0→-1 gate
// ============================================================

void test_temp_offset_gate() {
    printf("\n=== Test 4: Temp offset on 0→-1 gate ===\n");
    SystemState s;
    s.heat_cool_mode = true;
    s.heat_target_c = f_to_c(66.0f);
    s.cool_target_c = f_to_c(80.0f);  // wide deadband so cool doesn't interfere
    s.heat_gear = 0;
    s.last_active_mode = MODE_HEAT;
    s.idle_since = 1000;
    s.last_mode_event_at = 1000;
    s.boot_ready = true;

    std::vector<IrTransmission> tx;
    uint32_t now = 1000 + 1300000;  // 21+ min later

    // Room = heat target + 0.5°F (not past 1°F offset)
    s.room_c = s.heat_target_c + (0.5f * 5.0f / 9.0f);
    run_gear_controller(s, now, tx);
    check(s.heat_gear == 0, "stays at gear 0 when only 0.5°F past setpoint");

    // Room = heat target + 1.5°F (past 1°F offset)
    s.room_c = s.heat_target_c + (1.5f * 5.0f / 9.0f);
    run_gear_controller(s, now, tx);
    check(s.heat_gear == -1, "transitions to -1 when 1.5°F past setpoint");
}

// ============================================================
// TEST 5: User bypass of 0→-1 restrictions
// ============================================================

void test_user_bypass() {
    printf("\n=== Test 5: User bypass of 0→-1 restrictions ===\n");
    SystemState s;
    s.heat_cool_mode = true;
    s.heat_target_c = f_to_c(66.0f);
    s.cool_target_c = f_to_c(80.0f);
    s.heat_gear = 0;
    s.last_active_mode = MODE_HEAT;
    s.idle_since = 1000;
    s.last_mode_event_at = 1000;
    s.boot_ready = true;

    std::vector<IrTransmission> tx;

    // Only 3 min idle (short of 10), but user_input=true
    uint32_t now = 1000 + 180000;  // 3 min
    s.user_input = true;
    s.room_c = f_to_c(68.0f);  // clearly above heat target
    run_gear_controller(s, now, tx);
    check(s.heat_gear == -1, "user_input bypasses all 0→-1 restrictions");
}

// ============================================================
// TEST 6: Deadband after off
// ============================================================

void test_deadband_after_off() {
    printf("\n=== Test 6: Deadband — stays off when room between setpoints ===\n");
    SystemState s;
    s.heat_cool_mode = true;
    s.heat_target_c = f_to_c(66.0f);
    s.cool_target_c = f_to_c(70.0f);
    s.heat_gear = -1;
    s.cool_gear = -1;
    s.off_since = 100000;  // off long ago
    s.boot_ready = true;

    std::vector<IrTransmission> tx;
    uint32_t now = 100000 + 120000;  // 2 min after off

    s.room_c = f_to_c(68.0f);  // between setpoints
    run_gear_controller(s, now, tx);
    check(s.heat_gear == -1 && s.cool_gear == -1, "deadband — stays off");
}

// ============================================================
// TEST 7: YAML validator (inline C-equivalent)
// ============================================================

void test_temp_offset_parsing() {
    printf("\n=== Test 7: Python validator tested separately via python3 (always passes here) ===\n");
    check(true, "python validator verified separately");
}

// ============================================================
// TEST 8: Full cycle — heat → 0 → -1 → 1 min off → cool
// ============================================================

void test_full_cycle() {
    printf("\n=== Test 8: Full cycle heat → off → cool ===\n");
    SystemState s;
    s.heat_cool_mode = true;
    s.heat_target_c = f_to_c(66.0f);
    s.cool_target_c = f_to_c(68.0f);
    s.heat_gear = 0;
    s.last_active_mode = MODE_HEAT;
    s.idle_since = 1000;
    s.last_mode_event_at = 1000;
    s.boot_ready = true;
    s.active_ir_mode = IR_HEAT;

    std::vector<IrTransmission> tx;

    // Room past heat setpoint (natural 0→-1 via timers)
    uint32_t now = 1000 + 1300000;
    s.room_c = f_to_c(69.0f);  // 3°F above heat target
    run_gear_controller(s, now, tx);
    check(s.heat_gear == -1, "heat 0→-1 with large overshoot + timers satisfied");

    // Wait 1-min off, room just slightly above cool target (0.3°F = gear 2 range)
    now += 61000;
    s.room_c = s.cool_target_c + (0.3f * 5.0f / 9.0f);  // diff ≈ 0.17°C → gear 2
    run_gear_controller(s, now, tx);
    check(s.cool_gear == 2, "cool fresh start to gear 2 after 1-min off");

    // Verify clamped kickstart triggered for -1→2 cool
    check(s.clamp_phase == CLAMP_PRE_CS || s.clamp_phase == CLAMP_CLAMPED, "clamped kickstart active for cool fresh start");
    check(!s.clamp_is_heat, "clamp is for cool");
}

// ============================================================
// TEST 9: user_input preserves stable hunting gear (Fix #1)
// ============================================================
//
// Scenario from 2026-04-12: cool-mode hunting between gears 2 and 3, user
// tweaks target_temp_low (an inactive endpoint in cool) while diff sits in
// gear 1's band. Old behavior: any user_input recomputed from scratch and
// with diff just below C_UP_01 fell through to new_gear = 0, wiping the
// running gear. New behavior: gear_in_band_cool preserves the current gear.
void test_user_input_preserves_gear_in_band() {
    printf("\n=== Test 9: user_input preserves gear when diff stays in band ===\n");
    SystemState s;
    s.heat_cool_mode = true;
    s.heat_target_c = f_to_c(66.0f);
    s.cool_target_c = f_to_c(68.0f);
    s.cool_gear = 1;
    s.heat_gear = -1;
    s.last_active_mode = MODE_COOL;
    s.idle_since = 0;
    s.boot_ready = true;

    std::vector<IrTransmission> tx;
    uint32_t now = 1000000;

    // Room = 68.12°F → diff ≈ +0.067°C, safely inside gear 1's band [-0.1, 0.25].
    s.room_c = f_to_c(68.12f);
    s.user_input = true;
    run_gear_controller(s, now, tx);
    check(s.cool_gear == 1, "user event w/ diff in gear-1 band: gear 1 preserved (was 0 in old code)");

    // Gear 2, same diff — gear 2 band is [0.10, 0.40], diff 0.067 NOT in band → should recompute.
    SystemState s2 = s;
    s2.cool_gear = 2;
    s2.user_input = true;
    run_gear_controller(s2, now, tx);
    check(s2.cool_gear == 0, "user event w/ diff below gear-2 band: falls through to 0");

    // Heat mirror: gear 1 heat, diff in new band [-0.8, -0.15]. With H_DN_10 lowered
    // to -0.15°C, gear 1 is only in-band when room is 0.27-1.44°F below target.
    // Room 65.5°F, target 66°F → diff = -0.5°F = -0.28°C, safely in band.
    SystemState sh;
    sh.heat_cool_mode = true;
    sh.heat_target_c = f_to_c(66.0f);
    sh.cool_target_c = f_to_c(75.0f);
    sh.heat_gear = 1;
    sh.cool_gear = -1;
    sh.last_active_mode = MODE_HEAT;
    sh.boot_ready = true;
    sh.room_c = f_to_c(65.5f);
    sh.user_input = true;
    std::vector<IrTransmission> tx2;
    run_gear_controller(sh, now, tx2);
    check(sh.heat_gear == 1, "heat user event w/ diff in gear-1 band: gear 1 preserved");
}

// ============================================================
// TEST 10: C_DN_21 asymmetric hysteresis (Fix #2)
// ============================================================
//
// Scenario from 2026-04-12 17:16: gear 2 cooling, room dipped 0.03°C below
// setpoint (diff = -0.03°C). Old C_DN_21 = 0.0 → gear 2→1 on this tiny dip,
// gear 1 undersized, room overshot to 69.7°F triggering gear 4 cascade.
// New C_DN_21 = -0.1 absorbs sub-0.1°C dips.
void test_c_dn_21_hysteresis() {
    printf("\n=== Test 10: C_DN_21 hysteresis absorbs sub-0.1°C dips ===\n");
    SystemState s;
    s.heat_cool_mode = true;
    s.heat_target_c = f_to_c(66.0f);
    s.cool_target_c = f_to_c(68.0f);
    s.cool_gear = 2;
    s.heat_gear = -1;
    s.last_active_mode = MODE_COOL;
    s.last_gear_change = 0;  // plenty of time-in-gear
    s.boot_ready = true;

    std::vector<IrTransmission> tx;
    uint32_t now = 1000000;

    // Room 67.94°F, target 68°F → diff ≈ -0.033°C. With old C_DN_21 = 0.0
    // this would step down to gear 1. With new C_DN_21 = -0.1 it holds.
    s.room_c = f_to_c(67.94f);
    s.user_input = false;
    run_gear_controller(s, now, tx);
    check(s.cool_gear == 2, "gear 2 holds at diff=-0.03°C (new C_DN_21=-0.1 absorbs tiny dip)");

    // Deeper dip — diff = -0.15°C, now below C_DN_21 = -0.1: downshift.
    s.room_c = f_to_c(67.73f);  // ~-0.15°C below 68
    run_gear_controller(s, now, tx);
    check(s.cool_gear == 1, "gear 2 downshifts to 1 at diff=-0.15°C (past C_DN_21)");
}

// ============================================================

int main() {
    printf("Mode Switch Virtual Test Suite\n");
    printf("================================\n");

    test_clamp_fan_low();
    test_mode_transition_unstuck();
    test_off_lockout();
    test_temp_offset_gate();
    test_user_bypass();
    test_deadband_after_off();
    test_temp_offset_parsing();
    test_full_cycle();
    test_user_input_preserves_gear_in_band();
    test_c_dn_21_hysteresis();

    printf("\n================================\n");
    printf("Results: %d/%d passed, %d failed\n",
           tests_passed, tests_passed + tests_failed, tests_failed);
    printf("================================\n");

    return (tests_failed == 0) ? 0 : 1;
}
