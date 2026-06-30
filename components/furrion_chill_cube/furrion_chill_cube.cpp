#include "furrion_chill_cube.h"
#include "esphome/core/log.h"
#include <cmath>  // expf, isnan (adaptive integral decay)

#ifdef USE_API
#include "esphome/components/api/api_server.h"
#endif

#ifdef USE_ESP32
#include <esp_system.h>  // esp_reset_reason() — gates the gear restore on warm vs cold boot
#endif

namespace esphome {
namespace furrion_chill_cube {

static const char *const TAG = "furrion_chill_cube";

// ============================================================
// IR Protocol Constants — RAC-PT1411HWRU
// ============================================================

static const uint16_t IR_HEADER_MARK = 4380;
static const uint16_t IR_HEADER_SPACE = 4370;
static const uint16_t IR_BIT_MARK = 540;
static const uint16_t IR_ZERO_SPACE = 540;
static const uint16_t IR_ONE_SPACE = 1620;
static const uint16_t IR_GAP_SPACE = 5480;
static const uint16_t IR_PACKET_SPACE = 10500;
static const uint16_t IR_CARRIER_FREQ = 38000;
// Inter-MESSAGE guard idle, prepended to each CS / mode frame. Distinct from
// IR_GAP_SPACE (5480µs, the INTRA-message gap between a frame's repeats) and
// larger than the 10500µs that already delimits the swing frame reliably. With
// non_blocking:false, consecutive transmit.perform() calls leave no idle between
// them, so a CS→MODE-ON→CS bracket (or the quick-kickstart's CS→MODE-ON) arrives
// jammed at ~intra-message spacing and the unit can merge/misparse the mode-on.
// That is invisible on a running unit (the mode frame is redundant there) but
// makes a cold OFF→ON power-on intermittently fail — the mode-on is its only
// shot. A leading idle on every frame guarantees a clean message boundary.
//
// 2026-06-20: raised 25ms → 100ms. 25ms reliably fixed the OFF→ON case but a
// running setpoint change (CS→MODE→CS bracket, where the new setpoint rides the
// middle MODE frame sandwiched between two CS frames) still intermittently failed
// to register — confirmed from the camper's recorder DB by repeated up/down
// jiggling on climate.camper_ac. 100ms is closer to the idle a physical remote
// leaves between distinct commands; the unit parses each frame cleanly. With
// non_blocking:false the CPU idle-waits while the RMT peripheral clocks each frame
// out in hardware (so IR timing is unaffected — only CPU responsiveness): a full
// CS→MODE→CS bracket is ~430ms (CS + MODE + the MODE's swing sub-frame + CS, each
// with a 100ms lead), and a fresh-start pass that also hits the HVAC-on path can
// stack two brackets (~0.8s) in one loop iteration. Acceptable on this ESP32-C3
// (RMT-driven IR, no hard real-time work; ESPHome feeds the task WDT) but worth a
// hardware check that a worst-case pass doesn't hiccup the API. A bracket only fires
// on a settled setpoint/mode change; the 30s heartbeat is a single frame.
static const uint32_t IR_INTER_MSG_GAP = 100000;  // 100ms

// Temperature encoding: non-linear Gray code lookup
// RAC-PT1411HWRU Celsius table (index 0=16°C, 14=30°C)
// Bits: [5]=FRAC, [4]=NEG, [3:0]=temp nibble
static const uint8_t TEMP_C_TABLE[] = {
    0x10, 0x00, 0x01, 0x03, 0x02, 0x06, 0x07, 0x05,
    0x04, 0x0C, 0x0D, 0x09, 0x08, 0x0A, 0x0B};

// RAC-PT1411HWRU Fahrenheit table (index 0=60°F, 26=86°F)
// Same flag bits as the C table. Source: esphome PR #14666
// (esphome/components/toshiba/toshiba.cpp RAC_PT1411HWRU_TEMPERATURE_F).
// In F-mode, message[9] also gets bit 0x01 (FLAG_FAH) set so the unit's
// panel displays in °F.
static const uint8_t TEMP_F_TABLE[] = {
    0x10, 0x30, 0x00, 0x20, 0x01, 0x21, 0x03, 0x23, 0x02,
    0x22, 0x06, 0x26, 0x07, 0x05, 0x25, 0x04, 0x24, 0x0C,
    0x2C, 0x0D, 0x2D, 0x09, 0x08, 0x28, 0x0A, 0x2A, 0x0B};

// Celsius setpoint range (Furrion accepts 16-30°C)
static const int FURRION_MIN_TEMP_C = 16;
static const int FURRION_MAX_TEMP_C = 30;

// Fahrenheit setpoint range (Toshiba F-protocol bounds, 60-86°F).
// Approximately equivalent to the C range; F resolution is 1°F.
static const int FURRION_MIN_TEMP_F = 60;
static const int FURRION_MAX_TEMP_F = 86;

// Gear controller constants — no fixed anchors; setpoint is dynamic

// Per-gear upshift hold times (ms) — indexed by target gear number.
// HOLD_MS[0] is unused (gear 0 never upshifts via can_upshift_to) but
// kept for direct array indexing by gear number without offset.
//                        0      1       2       3        4        5
static const uint32_t HOLD_MS[] = {0, 180000, 180000, 300000, 300000, 600000};
static const uint32_t CLAMP_DURATION_MS = 305000;  // 5 min 5s (matches unit's internal 5-min enforced startup)
// Quick-kickstart hold windows (ms). OFF→gear-3 needs only a brief CS kick because
// start_quick_kickstart_ also sends a MODE-ON command from OFF, which does the heavy
// lifting of waking the compressor. The idle→gear-1/2 path is CS-only (unit already in
// COOL mode — no MODE-ON), so its kickstart CS must stay asserted long enough to outlast
// the compressor's anti-short-cycle restart lockout (~3 min). See session_log 2026-05-20:
// a 10s idle kick fired inside the lockout shadow and had zero effect (unit dead ~11 min).
static const uint32_t QUICK_KICK_HOLD_MS = 10000;  // OFF→gear-3
static const uint32_t IDLE_KICK_HOLD_MS = 90000;   // idle→gear-1/2
static const uint32_t CS_HEARTBEAT_MS = 30000;  // 30s
static const uint32_t GEAR_INTERVAL_MS = 60000;  // 60s fallback
// Setpoint debounce: a temperature change is held this long (steady state, no further
// change) before it is committed to IR. Coalesces a rapid multi-step adjustment (user
// holding temp-down: 68→67→66→65→64→63 as 5 separate control() calls) into ONE transmit
// of the final value, instead of a burst of overlapping CS→MODE→CS brackets that collide
// on the wire and cause steps to be missed. Only the final setpoint matters to the
// open-loop unit. Tune here: long enough to swallow a button-mash, short enough that a
// single deliberate change still acts promptly. Does NOT delay HA's display (the climate
// state updates immediately in control()); only the IR is deferred. Mode/fan changes are
// NOT debounced — they flush immediately.
static const uint32_t SETPOINT_SETTLE_MS = 2500;  // 2.5s
static const uint32_t KEEPALIVE_INTERVAL_MS = 300000;  // 5 min
static const uint32_t KEEPALIVE_STEP_MS = 5000;  // 5s between steps
// Mode switch lockouts are now configurable member variables (mode_switch_*_ms_, mode_switch_temp_offset_c_)

// Heating deadbands (diff = room - target, negative = cold)
static const float H_UP_01 = -0.3f;   // 0->1: room 0.5F below target
static const float H_UP_12 = -0.8f;   // 1->2
static const float H_UP_23 = -1.5f;   // 2->3
static const float H_DN_32 = -0.8f;   // 3->2
static const float H_DN_21 = -0.3f;   // 2->1
// H_DN_10 triggers gear 1 → 0 before reaching setpoint, compensating for
// the Furrion's post-drop thermal carry (~0.85°F observed on 2026-04-13).
// Old value (+0.3°C / room 0.54°F above target) let peak reach +1.44°F
// above setpoint — exceeds mode_switch_temp_offset_c_, triggered heat→cool
// pong. New value drops gear at 0.27°F *below* setpoint; thermal carry
// brings peak to ~+0.58°F, well inside mode-switch-offset safety margin.
static const float H_DN_10 = -0.15f;  // 1->0: room 0.27F below target
static const float H_IDLE  =  0.3f;   // 0->-1 threshold

// Cooling deadbands (diff = room - target, positive = hot)
static const float C_UP_01 =  0.15f;
static const float C_UP_12 =  0.25f;
static const float C_UP_23 =  0.4f;
// C_UP_34 / C_DN_32 / C_DN_43 widened on 2026-05-18 to break a 2↔4 hunt
// observed on the production unit. Old (0.60/0.10/0.25) let gear 3 act only
// as a transient: HOLD_MS[4]=5min forced 3→4 once diff cleared 0.60, and
// post-step-down thermal carry from gear 4 dragged diff straight through
// gear 3's 0.15°C window into gear 2. New values give gear 3 a 0.90°C
// hysteresis window and step gear 4 down ~0.15°C earlier so residual cool
// lands inside gear 3's range instead of below it. Power swings ~3A↔11A
// → expected to settle near gear 3 (~7A) with brief 2/4 excursions only.
static const float C_UP_34 =  0.85f;
static const float C_UP_45 =  1.0f;   // bumped from 0.80 to preserve up-ladder monotonicity
static const float C_DN_54 =  0.45f;
static const float C_DN_43 =  0.40f;
static const float C_DN_32 = -0.05f;
// C_DN_21 is the gear-2 sustain floor — deliberately deep (room ~0.7°F below setpoint).
// Gear 2 (not gear 1) is the sustain gear on mild days. 2026-06-03 drift analysis of a
// full mild cool cycle (target 68°F, outside 72-78°F) measured the equilibrium load
// sitting BETWEEN gear 2 (+0.047°F/min, gently under-cools) and gear 3 (-0.061°F/min,
// gently over-cools) — the natural steady state is a quiet 2↔3 duty cycle. The old -0.10
// (3→2 at -0.05 and 2→1 at -0.10 only 0.09°F apart) let gear 3's downward momentum coast
// the room ~0.27°F past the 3→2 point and trip 2→1 immediately, so gear 2 never held (a
// 6-min waypoint, 18 visits). Dropping to gear 1 buys nothing physically — g1 drift
// (+0.065°F/min) ≈ g2's — but it deepens the bottom of the swing, and the long under-cooled
// climb back overshoots (HOLD_MS-gated) all the way to a gear-4 spike on a mild day. -0.40
// keeps gear 2 holding through a normal undershoot (deepest observed coast-down was only
// -0.20°C), collapsing the 1↔2↔3↔4 churn into a clean 2↔3 toggle and removing the gear-4
// trigger. Hot-day operation is untouched (2→1 rarely triggers when the room stays warm).
static const float C_DN_21 = -0.40f;
// C_DN_10 is the gear-1→idle floor. With gear 2 now the sustain floor (C_DN_21=-0.40),
// gear 1 is a thin deep-undershoot-only sliver: it is entered only when the room is already
// >0.7°F cold, and -0.55 gives it a small (0.15°C) band before idle rather than collapsing
// straight to a stop. Down-ladder stays monotonic: 0.45 / 0.40 / -0.05 / -0.40 / -0.55.
static const float C_DN_10 = -0.55f;
static const float C_IDLE  = -0.15f;

// ── Phase 2 adaptive equilibrium-gear controller (cool mode) ────────────────────
// A slow integral floats the cool ladder's operating point to the gear that sustains
// the current load (which is NOT observable from outside temp — bodies, the CO2 vent
// fan, solar gain). PI control: the static ladder is the proportional inner loop; this
// integral removes its steady-state offset. See PHASE2_ADAPTIVE_DESIGN.md.
static const float ADAPT_KI = 0.06f;          // bias_c (°C) per (°C-error · min). Primary tuning knob.
static const float ADAPT_BIAS_C_MAX = 2.0f;   // authority clamp (~2-3 gears) + runaway backstop
static const float ADAPT_DEADBAND_C = 0.15f;  // don't integrate noise / tiny offset
static const float ADAPT_DECAY_TAU_MIN = 30.0f; // idle: forget a stale equilibrium with this time const
static const float ADAPT_DT_CAP_MIN = 5.0f;   // clamp dt across stalls/reboots
static const float GEAR_STEP_C = 0.85f;       // eff_diff per gear-equivalent of fan feedforward
                                              // (= C_UP_34; a deliberately firm one-gear shove)
static const uint32_t FAN_EDGE_FREEZE_MS = 180000; // freeze integral 3 min after a vent-fan edge
// Quantum for persisting bias_c_ to flash: the integral drifts a tiny amount every
// pass, so saving it raw would queue an NVS write per pass. Quantized to 0.1°C a
// write happens only every few minutes even while the bias is actively ramping
// (ADAPT_KI=0.06 → ≥~28 min per step at typical sub-deadband-adjacent errors),
// and a restore lands within half a quantum of the true equilibrium.
static const float GEAR_PREF_BIAS_QUANTUM_C = 0.1f;
static const float ADAPT_DRIFT_ALPHA = 0.3f;  // EMA smoothing for room dT/dt
// Rate-gated upshift (derivative action). The learned integral bias may DRIVE an upshift only while
// the room is strictly WARMING (drift > this). When the current gear is already holding or cooling
// the room, a stronger gear isn't needed, so upshift decisions fall back to the unbiased real diff —
// a wound-up integral can't grab a higher gear straight into an overshoot. This is the fix for the
// hot-day "gear-5 windup": the integral would wind to ~0.6°C and leap to the 13 A top gear while
// gear 4 was already holding/cooling the room, overshoot ~1°F, then collapse back down through 3+
// gears (analysis 2026-06-30). Effect, by closed-loop sim sweep (tests/adaptive_test.cpp):
//   • load < gear-4 capacity → collapses the "gear-3 + gear-5 pulses" pattern into a clean 3↔4 pair
//     (fewer transitions, tighter swing, SAME steady offset);
//   • load > gear-4 capacity → no effect: the room keeps warming at gear 4 (drift > 0) so the gate
//     stays open and the genuinely-needed gear 5 / 4↔5 hunt is untouched.
// Threshold is exactly 0, NOT positive: a positive value locks gear 4 and rides a large steady
// offset at the gear-4-capacity knife-edge (it suppresses the corrective top-gear pulses too — the
// sweep showed +0.01 → +1.45°F offset). 0 stays permeable there. Negative values are inert
// (equilibrium warming is ≥ 0). Downshifts and the integral's own error term are NOT gated.
static const float ADAPT_UPSHIFT_DRIFT_MIN_CPM = 0.0f;

// Mode-switching protection lives in the 0→-1 gate and -1→active gate
// (see mode_switch_idle_ms_, mode_switch_event_ms_, mode_switch_temp_offset_c_, mode_switch_off_ms_)

// Diagnostic compressor-output percentage for a gear. Single source for the
// gear-pass publishes AND setup()'s boot-restore publish, so a restored gear
// can't show a mismatched percentage.
static float gear_output_pct(bool is_heat, int gear) {
  if (gear <= 0) return 0.0f;
  if (is_heat) {
    switch (gear) {
      case 3:  return 100.0f;
      case 2:  return 66.6f;
      default: return 33.3f;
    }
  }
  switch (gear) {
    case 5:  return 100.0f;
    case 4:  return 80.0f;
    case 3:  return 60.0f;
    case 2:  return 40.0f;
    default: return 20.0f;
  }
}

// ============================================================
// Configuration Setters
// ============================================================

void FurrionChillCube::set_outside_lockout_temp(float temp_f) {
  // Convert Fahrenheit to Celsius
  outside_lockout_temp_c_ = (temp_f - 32.0f) * (5.0f / 9.0f);
}

void FurrionChillCube::set_mode_switch_idle_min(int min) {
  mode_switch_idle_ms_ = (uint32_t)min * 60000;
}
void FurrionChillCube::set_mode_switch_event_min(int min) {
  mode_switch_event_ms_ = (uint32_t)min * 60000;
}
void FurrionChillCube::set_mode_switch_temp_offset(float offset_c) {
  // Value already converted to °C at config time (Python validator handles F/C suffix)
  mode_switch_temp_offset_c_ = offset_c;
}
// set_mode_switch_off_ms is inline in the header (time-typed via YAML).

// ============================================================
// Active IR Mode Persistence
// ============================================================

// A "warm" reset is one where only the ESP rebooted (OTA, esp_restart, crash/WDT):
// the Furrion kept power and is still running the last IR-commanded mode/gear/CS,
// so the saved gear IS the unit's true state. A cold boot (power-on, brownout)
// means the camper likely lost power too — the unit's internal board resumed at
// the user's real target (the failover anchor, see project_failover_invariant),
// so gear 0 — not the saved gear — matches the hardware. Default cold when unsure.
static bool is_warm_reset_() {
#ifdef USE_ESP32
  switch (esp_reset_reason()) {
    case ESP_RST_SW:
    case ESP_RST_PANIC:
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:
      return true;
    default:
      return false;
  }
#else
  return false;
#endif
}

void FurrionChillCube::set_active_ir_mode_(climate::ClimateMode mode) {
  climate::ClimateMode prev = active_ir_mode_;
  if (prev == mode) return;
  active_ir_mode_ = mode;
  uint8_t m = (mode == climate::CLIMATE_MODE_HEAT) ? 1 :
              (mode == climate::CLIMATE_MODE_COOL) ? 2 : 0;
  mode_pref_.save(&m);

  // Timed vane positioning hook. The ONLY known vane anchor is the unit's power-on
  // re-home, so we trigger strictly on an OFF->active transition (Stephen's call).
  // Leaving an active mode (-> OFF, or a mode change that routes through OFF) aborts
  // any in-progress homing. Note: setup()'s boot-restore assigns active_ir_mode_
  // DIRECTLY (not via this setter), so a reboot never spuriously re-homes the vane.
  if (mode == climate::CLIMATE_MODE_OFF) {
    abort_vent_positioning_();
  } else if (prev == climate::CLIMATE_MODE_OFF) {
    maybe_start_vent_positioning_(mode == climate::CLIMATE_MODE_HEAT);
  }
}

// Persist the active mode's gear + the adaptive bias for the warm-reboot restore.
// Called at the end of every gear pass: every gear mutation inside the pass funnels
// through here, and the paths that zero the gear OUTSIDE a pass (failsafe, forced
// OFF, user OFF, kickstart abort) all save mode OFF via set_active_ir_mode_() —
// which makes the gear pref irrelevant on restore — so one call site suffices.
// No-op when neither field moved, so steady state costs zero flash writes; on the
// ESP32's wear-leveled NVS even the worst case (a gear change every HOLD_MS plus a
// bias quantum step every few minutes) is decades below the flash endurance budget.
// ESP32-only, matching is_warm_reset_(): on other platforms (ESP8266) preferences
// live in a single fixed flash sector with NO wear leveling — these writes would be
// real sector wear — and is_warm_reset_() always reports cold there, so the saved
// gear would never be read back anyway. Skip the writes; behavior = pre-persistence.
void FurrionChillCube::save_gear_pref_() {
#ifdef USE_ESP32
  int8_t g = -1;
  if (active_ir_mode_ == climate::CLIMATE_MODE_HEAT) {
    g = (int8_t) heat_gear_;
  } else if (active_ir_mode_ == climate::CLIMATE_MODE_COOL) {
    g = (int8_t) cool_gear_;
  }
  float bias_q = isnan(bias_c_)
                     ? 0.0f
                     : roundf(bias_c_ / GEAR_PREF_BIAS_QUANTUM_C) * GEAR_PREF_BIAS_QUANTUM_C;
  if (g == last_saved_gear_ && bias_q == last_saved_bias_c_) return;
  GearPrefData d{g, bias_q};
  gear_pref_.save(&d);
  last_saved_gear_ = g;
  last_saved_bias_c_ = bias_q;
#endif
}

// ============================================================
// IR Protocol Encoding
// ============================================================

void FurrionChillCube::encode_(remote_base::RemoteTransmitData *data,
                                const uint8_t *msg, uint8_t len, uint8_t repeat) {
  data->set_carrier_frequency(IR_CARRIER_FREQ);
  for (uint8_t copy = 0; copy <= repeat; copy++) {
    data->item(IR_HEADER_MARK, IR_HEADER_SPACE);
    for (uint8_t byte = 0; byte < len; byte++) {
      for (uint8_t bit = 0; bit < 8; bit++) {
        data->mark(IR_BIT_MARK);
        if (msg[byte] & (1 << (7 - bit))) {
          data->space(IR_ONE_SPACE);
        } else {
          data->space(IR_ZERO_SPACE);
        }
      }
    }
    data->item(IR_BIT_MARK, IR_GAP_SPACE);
  }
}

// ============================================================
// IR Transmission Methods
// ============================================================

void FurrionChillCube::transmit_mode_command_() {
  // Don't broadcast a mode/setpoint command for an active mode without a valid
  // setpoint — incomplete info. The OFF command carries no setpoint and is
  // always allowed. A transient gap is harmless (the unit coasts); a gap longer
  // than the unit's ~7-min CS-mode timeout hands control back to its internal
  // controller. See project_failover_invariant.
  if (active_ir_mode_ != climate::CLIMATE_MODE_OFF && isnan(get_active_ir_target_())) {
    return;
  }

  auto transmit = this->transmitter_->transmit();
  auto *data = transmit.get_data();
  // Inter-message guard idle so this mode frame is never jammed against a
  // preceding CS frame (CS→MODE-ON bracket / kickstart power-on). See IR_INTER_MSG_GAP.
  data->space(IR_INTER_MSG_GAP);

  uint8_t message[12] = {0};

  // Packet 1 header
  message[0] = 0xB2;
  message[1] = ~message[0];

  // Temperature code (dynamic setpoint → Gray code lookup).
  // F-mode encodes the user's precise °F target (1°F resolution, ranges
  // 60–86°F). C-mode encodes furrion_setpoint_c_ (the °C-rounded HA target,
  // ranges 16–30°C). Gear-CS math elsewhere always uses furrion_setpoint_c_
  // regardless of which path runs here — the only difference is what gets
  // shown on the unit's panel and which protocol the byte rides on.
  uint8_t temp_code;
  if (active_ir_mode_ == climate::CLIMATE_MODE_OFF) {
    temp_code = 0x0E;  // OFF special value (same for both protocols)
  } else if (use_fahrenheit_) {
    // target_c is guaranteed non-NaN — the gate at the top of this function
    // returns early for an active mode without a valid setpoint.
    float target_c = get_active_ir_target_();
    int target_f = (int)roundf(target_c * 1.8f + 32.0f);
    target_f = std::max(FURRION_MIN_TEMP_F, std::min(FURRION_MAX_TEMP_F, target_f));
    temp_code = TEMP_F_TABLE[target_f - FURRION_MIN_TEMP_F];
    last_tx_target_f_ = target_f;  // record the °F target actually put on the wire
  } else {
    int temp_c = std::max(FURRION_MIN_TEMP_C, std::min(FURRION_MAX_TEMP_C, furrion_setpoint_c_));
    temp_code = TEMP_C_TABLE[temp_c - FURRION_MIN_TEMP_C];
  }
  // Record the °C anchor actually put on the wire (both protocols ride furrion_setpoint_c_ as
  // the anchor). This is the baseline update_furrion_setpoint_() diffs against for sp_changed.
  if (active_ir_mode_ != climate::CLIMATE_MODE_OFF) {
    last_tx_setpoint_c_ = furrion_setpoint_c_;
  }

  // Fan speed
  auto fan = get_effective_fan_mode_();
  if (active_ir_mode_ == climate::CLIMATE_MODE_OFF) {
    message[2] = 0x7B;  // FAN_OFF
    message[7] = 0x00;
  } else {
    switch (fan) {
      case climate::CLIMATE_FAN_LOW:
        message[2] = 0x9F;
        message[7] = 0x28;
        break;
      case climate::CLIMATE_FAN_MEDIUM:
        message[2] = 0x5F;
        message[7] = 0x3C;
        break;
      case climate::CLIMATE_FAN_HIGH:
        message[2] = 0x3F;
        message[7] = 0x64;
        break;
      case climate::CLIMATE_FAN_AUTO:
      default:
        message[2] = 0xBF;
        message[7] = 0x66;
        break;
    }
  }
  message[3] = ~message[2];

  // Temperature (upper nibble) + Mode (lower nibble)
  message[4] = (temp_code & 0x0F) << 4;
  switch (active_ir_mode_) {
    case climate::CLIMATE_MODE_HEAT:
      message[4] |= 0x0C;
      break;
    case climate::CLIMATE_MODE_COOL:
    case climate::CLIMATE_MODE_OFF:
    default:
      message[4] |= 0x00;
      break;
  }
  message[5] = ~message[4];

  // Packet 2 (only if not OFF)
  if (active_ir_mode_ != climate::CLIMATE_MODE_OFF) {
    message[6] = 0xD5;
    // message[7] already set (fan code2)
    if (temp_code & 0x20) message[8] |= 0x20;  // FRAC flag
    if (temp_code & 0x10) message[9] |= 0x10;  // NEG flag
    if (use_fahrenheit_) message[9] |= 0x01;   // FAH flag — F-mode display
    message[10] = 0x00;
    message[11] = 0;
    for (int i = 6; i <= 10; i++) message[11] += message[i];
  }

  // === Transmission 1: Mode command (B2 + D5) ===
  this->encode_(data, &message[0], 6, 1);
  bool send_packet2 = (active_ir_mode_ != climate::CLIMATE_MODE_OFF);
  if (send_packet2) {
    this->encode_(data, &message[6], 6, 0);
  }
  transmit.perform();

  // === Transmission 2: Swing (B9) — separate transmit call for clean state ===
  // Suppressed while the vane positioner is MOVING or a manual step is in flight: those
  // own the physical swing during their window, and a swing frame here (SWING_OFF, since
  // swing_mode stays OFF) would stop the vane early. Each sends its own stop frame.
  if (vent_phase_ != VentPhase::MOVING && !vane_step_active_) {
    auto swing_tx = this->transmitter_->transmit();
    auto *swing_data = swing_tx.get_data();
    swing_data->space(IR_PACKET_SPACE);
    if (this->swing_mode == climate::CLIMATE_SWING_VERTICAL) {
      static const uint8_t SWING_ON[] = {0xB9, 0x46, 0xF5, 0x0A, 0x04, 0xFB};
      this->encode_(swing_data, SWING_ON, 6, 1);
    } else {
      static const uint8_t SWING_OFF[] = {0xB9, 0x46, 0xF5, 0x0A, 0x05, 0xFA};
      this->encode_(swing_data, SWING_OFF, 6, 1);
    }
    swing_data->space(IR_PACKET_SPACE);
    swing_tx.perform();
  }

  ESP_LOGD(TAG, "IR mode=%d fan=%d swing=%d",
           (int)active_ir_mode_, (int)fan, (int)this->swing_mode);
}

void FurrionChillCube::transmit_cs_update_() {
  if (active_ir_mode_ == climate::CLIMATE_MODE_OFF) return;
  // Don't broadcast a CS demand on incomplete info: the gear-CS signal needs
  // both a valid inside temperature and a valid setpoint. On a transient gap
  // the unit just coasts; on a gap longer than its ~7-min CS-mode timeout it
  // hands control back to its internal sensor. See project_failover_invariant.
  if (isnan(inside_temp_c_)) return;
  if (isnan(get_active_ir_target_())) return;

  auto transmit = this->transmitter_->transmit();
  auto *data = transmit.get_data();
  // Inter-message guard idle so this CS frame never jams the frame before/after
  // it in a CS→MODE-ON→CS bracket. See IR_INTER_MSG_GAP.
  data->space(IR_INTER_MSG_GAP);

  uint8_t message[6] = {0};
  message[0] = 0xBA;
  message[1] = ~message[0];  // 0x45

  // CS temperature — transmitted raw. Empirically the Furrion accepts
  // out-of-nominal-range values and treats them as stronger signals in
  // the same direction (tested on similar Midea units up to 55-95°F).
  // Gear 0 uses setpoint±5 which can land at CS=11 (setpoint 16°C cool)
  // or CS=35 (setpoint 30°C heat); both are transmitted unmodified.
  message[2] = (uint8_t)current_cs_;

  // Feature flags: always CS_ENABLED + CS_DATA. The value-bearing frame is the
  // only CS packet the unit acts on — a CS_DATA-clear frame is inert — and it
  // implicitly keeps the unit in Comfort Sense mode. No separate "enable" frame.
  message[2] |= 0xC0;  // CS_ENABLED + CS_DATA
  message[3] = ~message[2];

  // Mode footer
  switch (active_ir_mode_) {
    case climate::CLIMATE_MODE_HEAT:
      message[4] = 0x7E;
      break;
    case climate::CLIMATE_MODE_COOL:
      message[4] = 0x72;
      break;
    default:
      message[4] = 0x7A;  // AUTO footer
      break;
  }
  message[5] = ~message[4];

  this->encode_(data, message, 6, 1);
  transmit.perform();
}

// Bracket a mode/setpoint command with real CS_DATA frames so the unit is never
// left holding a stale CS against a new setpoint. On a running setpoint change
// the pre-CS makes the change atomic; on OFF→ON the pre-CS is transmitted but
// ignored by the unit (a CS before mode-on has no effect) — the post-CS is the
// one that matters there. transmit_cs_update_() no-ops only while active_ir_mode_
// is OFF.
void FurrionChillCube::transmit_mode_with_cs_() {
  // Deliberately NOT gated on boot_ready_: the OFF→ON mode command must still go
  // out on the first gear-controller pass, and boot_ready_ is not set until the
  // end of that pass. transmit_cs_update_() has its own isnan(inside_temp_c_)
  // gate against a boot-time CS leak, so a boot_ready_ gate here would be both
  // unnecessary and harmful — it would silently drop the first mode-on.
  if (failsafe_active_) return;  // no IR during failsafe — unit runs on its own sensor
  transmit_cs_update_();
  transmit_mode_command_();
  transmit_cs_update_();
  last_cs_heartbeat_ = millis();  // a CS just went out — restart the 30s heartbeat clock
}

void FurrionChillCube::transmit_raw_6byte_(const uint8_t *msg) {
  auto transmit = this->transmitter_->transmit();
  auto *data = transmit.get_data();
  // Inter-message guard idle so a raw command (swing/turbo/display) is never jammed
  // against an adjacent CS/mode frame. See IR_INTER_MSG_GAP.
  data->space(IR_INTER_MSG_GAP);
  this->encode_(data, msg, 6, 1);
  transmit.perform();
}

// ============================================================
// Button IR Commands
// ============================================================

void FurrionChillCube::send_display_toggle() {
  static const uint8_t CMD[] = {0xB9, 0x46, 0xF5, 0x0A, 0x09, 0xF6};
  this->transmit_raw_6byte_(CMD);
}
void FurrionChillCube::send_turbo_on() {
  static const uint8_t CMD[] = {0xB9, 0x46, 0xF5, 0x0A, 0x01, 0xFE};
  this->transmit_raw_6byte_(CMD);
}
void FurrionChillCube::send_turbo_off() {
  static const uint8_t CMD[] = {0xB9, 0x46, 0xF5, 0x0A, 0x02, 0xFD};
  this->transmit_raw_6byte_(CMD);
}
void FurrionChillCube::send_swing_on() {
  static const uint8_t CMD[] = {0xB9, 0x46, 0xF5, 0x0A, 0x04, 0xFB};
  this->transmit_raw_6byte_(CMD);
}
void FurrionChillCube::send_swing_off() {
  static const uint8_t CMD[] = {0xB9, 0x46, 0xF5, 0x0A, 0x05, 0xFA};
  this->transmit_raw_6byte_(CMD);
}

void FurrionChillCube::send_vane_step() {
  // Step is a manual-positioning tool: meaningless while the unit is off (the vane
  // re-homes on power-on anyway, wiping any step) and while swing oscillation owns
  // the vane — a pulse there is a no-op SWING_ON followed by a SWING_OFF that stops
  // the user's oscillation while swing_mode (and the HA switch) still shows ON.
  // Same active-mode convention as send_swing_state_().
  if (active_ir_mode_ == climate::CLIMATE_MODE_OFF) {
    ESP_LOGI(TAG, "Vane: step ignored (unit off)");
    return;
  }
  if (this->swing_mode != climate::CLIMATE_SWING_OFF) {
    ESP_LOGI(TAG, "Vane: step ignored (swing active)");
    return;
  }
  // One uniform manual nudge: SWING_ON now, SWING_OFF after vane_step_duration_ms_
  // (completed non-blocking in loop()). Abort any in-progress auto-positioning so a
  // manual step takes over and the positioner's later SWING_OFF can't clip this pulse.
  abort_vent_positioning_();
  send_swing_on();
  vane_step_active_ = true;
  vane_step_start_ = millis();  // self-clock on millis() — pressed outside the loop-now context
  ESP_LOGI(TAG, "Vane: step ON (%lums)", (unsigned long) vane_step_duration_ms_);
}

// ============================================================
// Component Lifecycle
// ============================================================

void FurrionChillCube::setup() {
  // Restore mode, targets, fan, swing from flash
  auto restore = this->restore_state_();
  if (restore.has_value()) {
    restore->apply(this);
    ESP_LOGI(TAG, "Restored state: mode=%d temp=%.1f lo=%.1f hi=%.1f fan=%d swing=%d",
             (int)this->mode, this->target_temperature,
             this->target_temperature_low, this->target_temperature_high,
             (int)this->fan_mode.value_or(climate::CLIMATE_FAN_AUTO),
             (int)this->swing_mode);
  }

  // Restore active IR mode from flash (survives reboot)
  mode_pref_ = global_preferences->make_preference<uint8_t>(this->get_object_id_hash() ^ 0x4D4F4445);
  // Gear + adaptive bias, saved by save_gear_pref_() at the end of every gear pass.
  gear_pref_ = global_preferences->make_preference<GearPrefData>(this->get_object_id_hash() ^ 0x47454152);
  uint8_t saved_mode = 0;
  if (mode_pref_.load(&saved_mode) && (saved_mode == 1 || saved_mode == 2)) {
    bool is_heat = (saved_mode == 1);
    // The saved gear is the unit's true state only on a WARM reset (ESP-only reboot:
    // the Furrion kept power and is still at the last commanded gear CS) — resume
    // there directly instead of re-climbing from idle, and resume the adaptive bias
    // so the cool ladder doesn't immediately collapse the restored gear (bias 0 at
    // equilibrium diff≈0 reads as "no demand"). On a cold boot, with no saved gear,
    // or an inconsistent pair (gear -1 under an active saved mode — crash between
    // the two pref saves), fall back to gear 0: the pre-change behavior.
    int g = 0;
    GearPrefData saved_gear{};
    if (is_warm_reset_() && gear_pref_.load(&saved_gear)) {
      int max_gear = is_heat ? 3 : 5;
      if (saved_gear.gear >= 0 && saved_gear.gear <= max_gear) {
        g = saved_gear.gear;
        // Bias only when the adaptive controller is actually running: with adaptive
        // disabled the bias never integrates OR decays, so a restored value would sit
        // frozen across reboots and spring back stale (≤±2.0°C) whenever adaptive is
        // re-enabled — possibly a season later, against a different equilibrium.
        if (!is_heat && adaptive_enable_ && !isnan(saved_gear.bias_c)) {
          bias_c_ = std::max(-ADAPT_BIAS_C_MAX, std::min(ADAPT_BIAS_C_MAX, saved_gear.bias_c));
        }
      }
    }
    // Restore active_ir_mode_ to match the gear state — otherwise the first
    // controller run sees gear≥0 with active_ir_mode_=OFF and sends a spurious
    // MODE_ON IR command, waking the Furrion from idle.
    // Also sync furrion_setpoint_c_ and current_cs_ to the restored target +
    // gear CS, so update_furrion_setpoint_() and the CS check on first run
    // don't detect a bogus mismatch (default 22 vs. actual) and transmit a
    // spurious MODE_ON / CS frame the unit didn't ask for.
    if (is_heat) {
      heat_gear_ = g;
      active_ir_mode_ = climate::CLIMATE_MODE_HEAT;
      last_active_mode_ = MODE_HEAT;
    } else {
      cool_gear_ = g;
      active_ir_mode_ = climate::CLIMATE_MODE_COOL;
      last_active_mode_ = MODE_COOL;
    }
    furrion_setpoint_c_ = compute_setpoint_c_(is_heat);
    last_tx_setpoint_c_ = furrion_setpoint_c_;  // seed sp_changed baseline (avoid spurious 1st-pass tx)
    current_cs_ = compute_gear_cs_(is_heat, g);
    // Publish the restored CS so the diagnostic doesn't sit "unknown" until the
    // next CS change (no IR goes out — this is state-tracking only).
    if (cs_value_sensor_) cs_value_sensor_->publish_state(current_cs_);
    seed_last_tx_target_f_();   // F-protocol: avoid a bogus f_changed on first pass
    boot_ready_ = true;          // restored state is valid — skip imm_off
    if (g == 0) {
      idle_since_ = millis();    // 10-min lockout before mode switch allowed
    } else {
      // Active gear restored — not idle. Stamp a fresh keep-alive window (mirrors
      // the on-gear-change stamp): without it, a restored sustaining gear leaves
      // keepalive_last_ AND last_gear_change_ at 0, so the low-gear pulse clock
      // would stay unarmed until the next gear change.
      keepalive_last_ = millis();
    }
    ESP_LOGI(TAG, "Restored prior mode: %s (gear=%d%s, skip kickstart, sp=%d°C cs=%d bias=%.2f)",
             is_heat ? "HEAT" : "COOL", g,
             g == 0 ? " → idle" : " (warm reset — resume gear)",
             furrion_setpoint_c_, current_cs_, bias_c_);
  }

  // Register temperature sensor callbacks
  if (inside_temp_sensor_) {
    inside_temp_sensor_->add_on_state_callback([this](float value) {
      uint32_t cb_now = millis();
      if (isnan(value)) {
        inside_temp_c_ = NAN;
        // Reset drift history so the next valid sample starts a fresh interval rather than
        // computing a rate across the NaN gap.
        prev_inside_temp_c_ = NAN;
        prev_inside_temp_at_ = 0;
        room_drift_cpm_ = NAN;
      } else {
        inside_temp_c_ = inside_temp_fahrenheit_ ? (value - 32.0f) * (5.0f / 9.0f) : value;
        // Room drift EMA (°C/min) — observability + future rate-gated upshift. Only when we
        // have a valid prior sample at a sane interval (5s–5min); NaN gaps reset the history.
        if (!isnan(prev_inside_temp_c_) && prev_inside_temp_at_ != 0) {
          float dt_min = (cb_now - prev_inside_temp_at_) / 60000.0f;
          if (dt_min >= (5.0f / 60.0f) && dt_min <= 5.0f) {
            float inst = (inside_temp_c_ - prev_inside_temp_c_) / dt_min;
            room_drift_cpm_ = isnan(room_drift_cpm_)
                                  ? inst
                                  : ADAPT_DRIFT_ALPHA * inst + (1.0f - ADAPT_DRIFT_ALPHA) * room_drift_cpm_;
          }
        }
        prev_inside_temp_c_ = inside_temp_c_;
        prev_inside_temp_at_ = cb_now;
      }
      this->current_temperature = inside_temp_c_;
      this->last_temp_update_ = cb_now;
      this->temp_dirty_ = true;
      this->publish_state();
    });
  }

  // Vent-fan disturbance input: record each edge so the adaptive integral can freeze
  // across it (the burst is handled by feedforward, not by re-learning the equilibrium).
  if (vent_fan_sensor_) {
    // Leave vent_fan_changed_at_ at 0 until a REAL edge fires — the freeze window is gated on
    // (!= 0) so boot doesn't look like a fan edge.
    vent_fan_sensor_->add_on_state_callback([this](bool state) {
      this->vent_fan_changed_at_ = millis();
      // Re-run the controller promptly on a fan edge — but only when adaptive is enabled, so a
      // disabled build is strictly identical to main (no extra prompted passes).
      if (this->adaptive_enable_) this->temp_dirty_ = true;
    });
  }
  if (outside_temp_sensor_) {
    outside_temp_sensor_->add_on_state_callback([this](float value) {
      if (isnan(value)) {
        outside_temp_c_ = NAN;
      } else {
        outside_temp_c_ = outside_temp_fahrenheit_ ? (value - 32.0f) * (5.0f / 9.0f) : value;
      }
    });
  }

  // Initialize boot time and diagnostic sensors. Compressor output reflects the
  // restored gear (a warm reset can resume at gear N — 0% there would contradict
  // the gear sensor until the next gear change).
  boot_time_ = millis();
  if (heat_gear_sensor_) heat_gear_sensor_->publish_state(heat_gear_);
  if (cool_gear_sensor_) cool_gear_sensor_->publish_state(cool_gear_);
  if (compressor_output_sensor_) {
    float pct = (heat_gear_ > 0) ? gear_output_pct(true, heat_gear_)
              : (cool_gear_ > 0) ? gear_output_pct(false, cool_gear_)
                                 : 0.0f;
    compressor_output_sensor_->publish_state(pct);
  }
}

void FurrionChillCube::loop() {
  uint32_t now = millis();

  // 1. Advance keep-alive state machine
  if (keepalive_phase_ != KeepAlivePhase::IDLE) {
    advance_keepalive_(now);
  }

  // 1b. Commit a settled setpoint change (debounce). A temp change arms setpoint_pending_
  // in control() but does NOT transmit; once the user stops stepping (SETPOINT_SETTLE_MS of
  // no further change) we commit here by setting user_changed_, so the gear controller runs
  // once on the FINAL value and emits a single clean transmit. abort_keepalive_ so the gear
  // pass actually runs this iteration (step 2 is gated on keepalive_phase_ == IDLE).
  // Self-clock on millis() (NOT loop's cached `now`): setpoint_pending_since_ is armed in
  // control(), outside the loop-`now` context, so per reference_furrion_millis_now_footgun a
  // timer armed from a callback must compare against its own clock to avoid any underflow.
  if (setpoint_pending_ && (millis() - setpoint_pending_since_) >= SETPOINT_SETTLE_MS) {
    setpoint_pending_ = false;
    user_changed_ = true;
    if (keepalive_phase_ != KeepAlivePhase::IDLE) abort_keepalive_();
  }

  // 2. Run gear controller if triggered (not blocked by kickstart — kickstart suppresses CS writes)
  bool should_run = false;
  if (temp_dirty_) should_run = true;
  if (last_gear_run_ == 0 || (now - last_gear_run_) >= GEAR_INTERVAL_MS) should_run = true;
  if (user_changed_) should_run = true;

  if (should_run && keepalive_phase_ == KeepAlivePhase::IDLE) {
    temp_dirty_ = false;  // only consume when gear controller actually runs
    run_gear_controller_();
    last_gear_run_ = now;
  }

  // Re-sample the clock after the gear pass. run_gear_controller_() reads its OWN
  // millis() and may spend hundreds of ms in blocking IR transmits; the timers it
  // arms (kickstart/keepalive) and last_cs_heartbeat_ (stamped at the END of a mode
  // bracket, after those transmits) are therefore NEWER than the `now` captured at
  // loop entry. Comparing steps 3-5 below against that stale `now` underflows:
  // a redundant CS fires after every bracket, and a kickstart armed this pass can
  // collapse instantly. Re-reading keeps the comparator >= any gear-pass stamp.
  // See reference_furrion_millis_now_footgun.
  now = millis();

  // 3. Advance kickstart (after gear controller so it can check gear thresholds)
  if (kickstart_active_()) {
    advance_kickstart_(now);
  }

  // 3b. Advance timed vane positioning (independent state machine; raw swing IR).
  // Aborted via set_active_ir_mode_(OFF) / user swing toggle, so it only runs while
  // the unit is on; the failsafe guard is belt-and-suspenders. It self-clocks on
  // millis() (does NOT take loop's cached `now`) — see advance_vent_positioning_.
  if (vent_positioning_active_() && !failsafe_active_) {
    advance_vent_positioning_();
  }

  // 3c. Complete a manual vane-step pulse: SWING_OFF after vane_step_duration_ms_.
  // Self-clocked on millis() (vane_step_start_ was set on the button press); runs to
  // completion even across a failsafe so the swing is never left stuck ON.
  if (vane_step_active_ && (millis() - vane_step_start_) >= vane_step_duration_ms_) {
    send_swing_off();
    vane_step_active_ = false;
    ESP_LOGI(TAG, "Vane: step OFF");
  }

  // 4. Keep-alive trigger for low-CS gears (cool 1-2, heat 1)
  if (keepalive_enable_ && !kickstart_active_() && keepalive_phase_ == KeepAlivePhase::IDLE &&
      boot_ready_ && !failsafe_active_) {
    bool cool_eligible = (cool_gear_ >= 1 && cool_gear_ <= 2 &&
                          active_ir_mode_ == climate::CLIMATE_MODE_COOL);
    bool heat_eligible = (heat_gear_ == 1 &&
                          active_ir_mode_ == climate::CLIMATE_MODE_HEAT);
    if (cool_eligible || heat_eligible) {
      uint32_t since = keepalive_last_ > 0 ? keepalive_last_ : last_gear_change_;
      if (since > 0 && (now - since) >= KEEPALIVE_INTERVAL_MS) {
        start_keepalive_(heat_eligible, now);
      }
    }
  }

  // 5. CS heartbeat every 30s (current_cs_ is kickstart CS during kickstart — correct)
  if (boot_ready_ && !failsafe_active_ &&
      active_ir_mode_ != climate::CLIMATE_MODE_OFF &&
      (now - last_cs_heartbeat_) >= CS_HEARTBEAT_MS) {
    transmit_cs_update_();
    last_cs_heartbeat_ = now;
  }
}

climate::ClimateTraits FurrionChillCube::traits() {
  auto traits = climate::ClimateTraits();
  traits.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE |
                           climate::CLIMATE_SUPPORTS_TWO_POINT_TARGET_TEMPERATURE |
                           climate::CLIMATE_SUPPORTS_ACTION);
  traits.add_supported_mode(climate::CLIMATE_MODE_OFF);
  traits.add_supported_mode(climate::CLIMATE_MODE_HEAT);
  traits.add_supported_mode(climate::CLIMATE_MODE_COOL);
  traits.add_supported_mode(climate::CLIMATE_MODE_HEAT_COOL);
  traits.set_visual_min_temperature(4.4f);    // 40°F
  traits.set_visual_max_temperature(30.0f);   // 86°F
  traits.set_visual_current_temperature_step(0.1f);
  traits.set_visual_target_temperature_step(1.0f);
  traits.add_supported_fan_mode(climate::CLIMATE_FAN_AUTO);
  traits.add_supported_fan_mode(climate::CLIMATE_FAN_LOW);
  traits.add_supported_fan_mode(climate::CLIMATE_FAN_MEDIUM);
  traits.add_supported_fan_mode(climate::CLIMATE_FAN_HIGH);
  traits.add_supported_swing_mode(climate::CLIMATE_SWING_OFF);
  traits.add_supported_swing_mode(climate::CLIMATE_SWING_VERTICAL);
  return traits;
}

void FurrionChillCube::control(const climate::ClimateCall &call) {
  // Track whether each field *actually* changed. user_changed_ is set only for
  // real, mode-relevant changes — not for redundant HA re-syncs after reconnect
  // (which send the same values we already have) and not for tweaks to the
  // inactive endpoint (low in cool mode, high in heat mode).
  bool mode_changed = false;
  bool temp_changed = false;
  bool fan_changed = false;

  if (call.get_mode().has_value()) {
    auto new_mode = *call.get_mode();

    // Ensure two-point values valid for first boot (BEFORE sync to avoid NaN copy)
    if (isnan(this->target_temperature_low)) this->target_temperature_low = 20.0f;
    if (isnan(this->target_temperature_high)) this->target_temperature_high = 25.0f;

    // Sync target_temperature for HA single-slider display in HEAT/COOL modes
    if (new_mode == climate::CLIMATE_MODE_HEAT) {
      this->target_temperature = this->target_temperature_low;
    } else if (new_mode == climate::CLIMATE_MODE_COOL) {
      this->target_temperature = this->target_temperature_high;
    }

    mode_changed = (new_mode != this->mode);
    this->mode = new_mode;

    if (mode_changed) {
      // Abort keepalive immediately so gear controller can respond on next loop
      if (keepalive_phase_ != KeepAlivePhase::IDLE) {
        abort_keepalive_();
      }

      // Abort kickstart only if new mode is incompatible with the active IR mode.
      // Clear state directly (don't call end_kickstart_ which would re-send old mode).
      if (kickstart_active_()) {
        // Determine the kickstart's intended mode (active_ir_mode_ may still be OFF during PRE_CS)
        bool kick_is_cool = (active_ir_mode_ == climate::CLIMATE_MODE_COOL) ||
                            (clamp_phase_ != ClampPhase::IDLE && !clamp_is_heat_) ||
                            (quick_kick_active_ && !quick_kick_is_heat_);
        bool kick_is_heat = (active_ir_mode_ == climate::CLIMATE_MODE_HEAT) ||
                            (clamp_phase_ != ClampPhase::IDLE && clamp_is_heat_) ||
                            (quick_kick_active_ && quick_kick_is_heat_);
        bool compatible = false;
        if (kick_is_cool) {
          compatible = (new_mode == climate::CLIMATE_MODE_COOL || new_mode == climate::CLIMATE_MODE_HEAT_COOL);
        } else if (kick_is_heat) {
          compatible = (new_mode == climate::CLIMATE_MODE_HEAT || new_mode == climate::CLIMATE_MODE_HEAT_COOL);
        }
        if (!compatible) {
          uint32_t now = millis();
          ESP_LOGI(TAG, "Kickstart aborted — mode %d incompatible with IR mode %d",
                   (int)new_mode, (int)active_ir_mode_);
          // Clear kickstart state, force to OFF. All mode changes (including
          // COOL↔HEAT and to OFF) now go through -1 with 1-min off_since_ lockout.
          clamp_phase_ = ClampPhase::IDLE;
          quick_kick_active_ = false;
          heat_gear_ = -1;
          cool_gear_ = -1;
          off_since_ = now;  // start 1-min off lockout
          if (heat_gear_sensor_) heat_gear_sensor_->publish_state(-1);
          if (cool_gear_sensor_) cool_gear_sensor_->publish_state(-1);
          if (active_ir_mode_ != climate::CLIMATE_MODE_OFF) {
            set_active_ir_mode_(climate::CLIMATE_MODE_OFF);
            transmit_mode_command_();
          }
        }
      }
    }
  }

  // Target temperature changes. Only flag as user-relevant when:
  //   (a) the value actually changed (idempotent re-sync from HA is a no-op), AND
  //   (b) the endpoint drives the active mode's gear math:
  //         target_temperature_low  → heat target (HEAT, HEAT_COOL)
  //         target_temperature_high → cool target (COOL, HEAT_COOL)
  //         target_temperature      → active target in HEAT or COOL single modes
  bool is_heat_mode = (this->mode == climate::CLIMATE_MODE_HEAT ||
                       this->mode == climate::CLIMATE_MODE_HEAT_COOL);
  bool is_cool_mode = (this->mode == climate::CLIMATE_MODE_COOL ||
                       this->mode == climate::CLIMATE_MODE_HEAT_COOL);
  bool is_single_active_mode = (this->mode == climate::CLIMATE_MODE_HEAT ||
                                this->mode == climate::CLIMATE_MODE_COOL);

  if (call.get_target_temperature().has_value()) {
    float v = *call.get_target_temperature();
    bool changed = isnan(this->target_temperature) || v != this->target_temperature;
    if (changed && is_single_active_mode) temp_changed = true;
    this->target_temperature = v;
  }
  if (call.get_target_temperature_low().has_value()) {
    float v = *call.get_target_temperature_low();
    bool changed = isnan(this->target_temperature_low) || v != this->target_temperature_low;
    if (changed && is_heat_mode) temp_changed = true;
    this->target_temperature_low = v;
  }
  if (call.get_target_temperature_high().has_value()) {
    float v = *call.get_target_temperature_high();
    bool changed = isnan(this->target_temperature_high) || v != this->target_temperature_high;
    if (changed && is_cool_mode) temp_changed = true;
    this->target_temperature_high = v;
  }
  // Sync target_temperature when low/high change in single modes.
  // Without this, get_heat/cool_target_() returns a stale value in HEAT/COOL modes.
  if (call.get_target_temperature_low().has_value() &&
      this->mode == climate::CLIMATE_MODE_HEAT) {
    this->target_temperature = this->target_temperature_low;
  }
  if (call.get_target_temperature_high().has_value() &&
      this->mode == climate::CLIMATE_MODE_COOL) {
    this->target_temperature = this->target_temperature_high;
  }

  // Fan mode change — idempotent: same-value sync is a no-op (no IR, no user flag)
  if (call.get_fan_mode().has_value()) {
    auto new_fan = *call.get_fan_mode();
    auto cur_fan = this->fan_mode.value_or(climate::CLIMATE_FAN_AUTO);
    fan_changed = (new_fan != cur_fan);
    this->fan_mode = new_fan;
    if (fan_changed && active_ir_mode_ != climate::CLIMATE_MODE_OFF && !kickstart_active_()) {
      transmit_mode_command_();
      ESP_LOGI(TAG, "User fan change → %d, mode command sent", (int)new_fan);
    }
  }

  // Swing mode change — standalone swing frame, works during kickstart
  // Does NOT set user_changed_: vent direction is cosmetic and must never
  // trigger gear recalculation, timer resets, or immediate-off logic.
  if (call.get_swing_mode().has_value()) {
    this->swing_mode = *call.get_swing_mode();
    // User took manual vane control — exit any in-progress timed homing cleanly and
    // immediately. abort is a pure state reset (no IR); send_swing_state_() below puts
    // the vane in the user's requested state, so there is no competing frame.
    abort_vent_positioning_();
    // Also cancel an in-flight manual step pulse: the user's swing frame supersedes
    // it, and the pulse's deferred SWING_OFF (loop 3c) would otherwise stop a
    // just-enabled oscillation ~vane_step_duration later while HA shows it ON.
    vane_step_active_ = false;
    send_swing_state_();
    ESP_LOGI(TAG, "User swing change → %d", (int)*call.get_swing_mode());
  }

  // Flag gear recalculation for real, gear-relevant changes. Swing is cosmetic;
  // same-value re-syncs from HA are filtered above.
  //
  // Mode/fan changes are discrete, single-shot actions → act immediately, and flush any
  // pending setpoint (the immediate gear run commits the latest target too). A pure temp
  // change is DEBOUNCED instead: arm setpoint_pending_ and let loop() commit it after the
  // user stops stepping (SETPOINT_SETTLE_MS). target_temperature is already updated above,
  // so HA's card shows the new value instantly — only the IR transmit is deferred.
  if (mode_changed || fan_changed) {
    this->user_changed_ = true;
    this->setpoint_pending_ = false;  // flush: the immediate run handles the current target
    if (keepalive_phase_ != KeepAlivePhase::IDLE) {
      abort_keepalive_();
    }
  } else if (temp_changed) {
    this->setpoint_pending_ = true;
    this->setpoint_pending_since_ = millis();
  }
  this->publish_state();
}

// ============================================================
// Helpers
// ============================================================

float FurrionChillCube::get_heat_target_() {
  return this->target_temperature_low;
}

float FurrionChillCube::get_cool_target_() {
  return this->target_temperature_high;
}

// Target for whichever mode the unit's IR state is in. Keyed on active_ir_mode_
// (distinct from get_heat/cool_target_, which are keyed on the user's climate
// mode). Returns NAN when OFF — an OFF command carries no setpoint.
float FurrionChillCube::get_active_ir_target_() {
  if (active_ir_mode_ == climate::CLIMATE_MODE_HEAT) return get_heat_target_();
  if (active_ir_mode_ == climate::CLIMATE_MODE_COOL) return get_cool_target_();
  return NAN;
}

// Seed last_tx_target_f_ to the °F byte a real transmit would put on the wire for
// the currently active target — same math as transmit_mode_command_()'s F path.
// Called from setup()'s restore branches: without it last_tx_target_f_ stays 0, and
// update_furrion_setpoint_()'s f_changed check (target_f != 0, always true in the
// default use_fahrenheit_ build) fires a spurious MODE-ON+CS bracket on the first
// gear pass after every reboot — the exact un-commanded wake the restore setpoint/CS
// sync exists to prevent. NaN target (no restored climate state) → leave 0; the
// isnan gates downstream already suppress the transmit in that case.
void FurrionChillCube::seed_last_tx_target_f_() {
  float t = get_active_ir_target_();
  if (isnan(t)) return;
  int tf = (int) roundf(t * 1.8f + 32.0f);
  last_tx_target_f_ = std::max(FURRION_MIN_TEMP_F, std::min(FURRION_MAX_TEMP_F, tf));
}

// Returns true iff the given heat gear's hysteresis band covers `diff`.
// Used on user_input events to avoid collapsing a stable hunting gear when
// the user change doesn't actually move the room out of the current gear's band.
bool FurrionChillCube::gear_in_band_heat_(int gear, float diff) {
  // Gear N stays in (H_UP_N(N+1), H_DN_N(N-1)) — its own transition thresholds.
  // (For heat, diff is negative when cold, so H_UP_* are the lower bounds.)
  switch (gear) {
    case 0: return diff >= H_UP_01 && diff <= H_IDLE;       // (-0.3, 0.3)
    case 1: return diff >= H_UP_12 && diff <= H_DN_10;      // (-0.8, -0.15)
    case 2: return diff >= H_UP_23 && diff <= H_DN_21;      // (-1.5, -0.3)
    case 3: return diff <= H_DN_32;                          // max heat, no lower bound
    default: return false;
  }
}

bool FurrionChillCube::gear_in_band_cool_(int gear, float diff) {
  // Gear N stays in (C_DN_N(N-1), C_UP_N(N+1)) — its own transition thresholds.
  switch (gear) {
    case 0: return diff >= C_IDLE  && diff <= C_UP_01;      // (-0.15, 0.15)
    case 1: return diff >= C_DN_10 && diff <= C_UP_12;      // (-0.55, 0.25)
    case 2: return diff >= C_DN_21 && diff <= C_UP_23;      // (-0.40, 0.40)
    case 3: return diff >= C_DN_32 && diff <= C_UP_34;      // (-0.05, 0.85)
    case 4: return diff >= C_DN_43 && diff <= C_UP_45;      // (0.40, 1.00)
    case 5: return diff >= C_DN_54;                          // max cool, no upper bound
    default: return false;
  }
}

bool FurrionChillCube::vent_fan_on_() {
  return vent_fan_sensor_ != nullptr && vent_fan_sensor_->state;
}

// Phase 2 adaptive (cool): advance the integral bias (bias_c_) with anti-windup, then return
// the effective diff the cool ladder's ACTIVE-gear thresholds select on
// (real_diff + bias_c_ + vent-fan feedforward). Returns real_diff unchanged when adaptive is
// disabled. The caller uses this ONLY for switch cases 1-5 — re-engage/idle decisions stay on
// real diff. Must be called once per cool pass (even at gear 0) so the integral advances/decays.
// See PHASE2_ADAPTIVE_DESIGN.md §2-3.
float FurrionChillCube::adaptive_cool_eff_diff_(float real_diff, uint32_t now, uint32_t time_in_gear) {
  if (!adaptive_enable_) {
    adaptive_last_advance_ = now;  // keep dt fresh so a later enable doesn't see a huge gap
    cool_eff_up_diff_ = real_diff; // no bias → upshift path is the static ladder (bit-identical)
    return real_diff;
  }

  // dt since last advance, clamped (first pass, stalls, mode gaps, millis wrap-after-reboot)
  float dt_min = 0.0f;
  if (adaptive_last_advance_ != 0) {
    dt_min = (now - adaptive_last_advance_) / 60000.0f;
    if (dt_min < 0.0f) dt_min = 0.0f;
    if (dt_min > ADAPT_DT_CAP_MIN) dt_min = ADAPT_DT_CAP_MIN;
  }
  adaptive_last_advance_ = now;

  // Error with deadband (+ = room too warm = need more cooling). REAL diff drives the integral.
  float e = real_diff;
  if (e > -ADAPT_DEADBAND_C && e < ADAPT_DEADBAND_C) e = 0.0f;

  // Freeze the 3-min window after a vent-fan state publish. The (sensor != null && != 0) guard
  // fully suppresses the case with NO fan configured (callback never fires → stays 0). With a
  // fan configured, the binary_sensor's initial-state publish does set this at boot, so the
  // window can be active for the first ~3 min of uptime — harmless, since bias_c_ starts at 0
  // and there is nothing to freeze.
  bool fan_edge_freeze = (vent_fan_sensor_ != nullptr) && (vent_fan_changed_at_ != 0) &&
                         ((now - vent_fan_changed_at_) < FAN_EDGE_FREEZE_MS);
  bool idle = (cool_gear_ <= 0);  // compressor off/idle — error not controllable

  // Conditional integration anti-windup: freeze POSITIVE accumulation only when the gear cannot
  // rise right now — at gear 5, or while an upshift is hold-blocked (can_upshift_to). NEGATIVE
  // accumulation is never rail-blocked because gear 0/idle is always reachable (downshifts are
  // not hold-gated). This also lets a stale positive bias unwind at gear 1 (e<0 is not frozen),
  // fixing the prior sat_low trap, and stops windup behind a held upshift from cascading gears.
  bool upshift_held = (cool_gear_ < 5) && (time_in_gear < HOLD_MS[cool_gear_ + 1]);
  // Rate gate: the room is "warming" (gear-raising allowed) only when dT/dt clears the threshold.
  // Unknown drift (NaN, e.g. just after boot / a sensor gap) is treated as warming → legacy behavior.
  bool warming = isnan(room_drift_cpm_) || room_drift_cpm_ > ADAPT_UPSHIFT_DRIFT_MIN_CPM;
  // Freeze positive accumulation whenever the gear can't rise right now — at gear 5, while an
  // upshift is hold-blocked, OR while the rate gate is suppressing the upshift (!warming). The
  // last clause is load-bearing: without it the integral would keep winding behind a drift-gated
  // upshift and then over-leap the moment the room ticked warm again (re-introducing the windup).
  bool block_up = (e > 0.0f) && (cool_gear_ >= 5 || upshift_held || !warming);
  bool freeze = kickstart_active_() || fan_edge_freeze || block_up;

  if (idle) {
    // Idle: forget a stale equilibrium so a re-engage doesn't inherit a wrong load.
    if (dt_min > 0.0f) bias_c_ *= expf(-dt_min / ADAPT_DECAY_TAU_MIN);
  } else if (!freeze && dt_min > 0.0f) {
    bias_c_ += ADAPT_KI * e * dt_min;
    if (bias_c_ > ADAPT_BIAS_C_MAX) bias_c_ = ADAPT_BIAS_C_MAX;
    if (bias_c_ < -ADAPT_BIAS_C_MAX) bias_c_ = -ADAPT_BIAS_C_MAX;
  }

  float ff_c = vent_fan_on_() ? (fan_feedforward_gears_ * GEAR_STEP_C) : 0.0f;
  float eff = real_diff + bias_c_ + ff_c;
  // Upshift decisions see the learned bias only while warming. When not warming, fall back to the
  // unbiased diff (+ the fast fan feedforward, which is anticipatory and must still act). fminf
  // guarantees the gate can only ever SUPPRESS an upshift, never enable one: a stale NEGATIVE bias
  // (eff < real_diff) keeps eff, so the gate never makes an upshift easier than the static ladder.
  cool_eff_up_diff_ = warming ? eff : fminf(eff, real_diff + ff_c);
  return eff;
}

int FurrionChillCube::compute_setpoint_c_(bool is_heat) {
  float target_c = is_heat ? get_heat_target_() : get_cool_target_();
  if (isnan(target_c)) return 22;  // safe default
  int rounded = (int)roundf(target_c);
  return std::max(FURRION_MIN_TEMP_C, std::min(FURRION_MAX_TEMP_C, rounded));
}

void FurrionChillCube::update_furrion_setpoint_(bool is_heat) {
  // NaN target → hold the last good setpoint. A NaN target can briefly arrive
  // from an unavailable HA sensor; never let compute_setpoint_c_()'s safe
  // default silently clobber furrion_setpoint_c_.
  float target_c = is_heat ? get_heat_target_() : get_cool_target_();
  if (isnan(target_c)) return;

  int new_sp = compute_setpoint_c_(is_heat);
  int gear = is_heat ? heat_gear_ : cool_gear_;

  // Update the °C anchor IMMEDIATELY — even mid-debounce. furrion_setpoint_c_ is the anchor
  // compute_gear_cs_() reads; deferring it would let a gear pass during the settle window emit
  // a CS frame whose gear was chosen for the new target but whose CS value is still anchored to
  // the OLD setpoint (a transient new-gear/old-anchor inconsistency). Updating it here is cheap
  // and IR-free — only the setpoint-display TRANSMIT is debounced (below). Because the anchor
  // now tracks the live target, change detection diffs against last_tx_setpoint_c_ (the last °C
  // actually put on the wire), NOT furrion_setpoint_c_.
  if (new_sp != furrion_setpoint_c_) {
    ESP_LOGI(TAG, "Furrion setpoint %d°C → %d°C (%s)",
             furrion_setpoint_c_, new_sp, is_heat ? "heat" : "cool");
    furrion_setpoint_c_ = new_sp;
    // Re-anchor current_cs_ to the new setpoint for the current gear so CS frames stay
    // consistent with it. Skip during a kickstart: current_cs_ is then the kickstart CS and the
    // kickstart state machine must keep owning it (end_kickstart_ re-derives the gear CS).
    if (gear >= 0 && !kickstart_active_()) {
      current_cs_ = compute_gear_cs_(is_heat, gear);
      if (cs_value_sensor_) cs_value_sensor_->publish_state(current_cs_);
    }
  }

  // Debounce: defer ONLY the IR transmit while a setpoint change is still settling. The anchor
  // above already tracks the live target; loop() clears setpoint_pending_ + sets user_changed_
  // when the settle window elapses, and this runs again to fire the single coalesced transmit.
  if (setpoint_pending_) return;

  // sp_changed = whole-°C change vs the last transmitted setpoint. f_changed = sub-°C (1°F)
  // change (e.g. 73→74°F both round to 23°C) so the F-protocol panel display stays in sync.
  bool sp_changed = (furrion_setpoint_c_ != last_tx_setpoint_c_);
  bool f_changed = false;
  if (use_fahrenheit_ && active_ir_mode_ != climate::CLIMATE_MODE_OFF) {
    int target_f = (int) roundf(target_c * 1.8f + 32.0f);
    target_f = std::max(FURRION_MIN_TEMP_F, std::min(FURRION_MAX_TEMP_F, target_f));
    f_changed = (target_f != last_tx_target_f_);
  }
  // Retransmit so the unit gets the new setpoint / panel target — but only when
  // there is an active gear to anchor the CS to; otherwise the gear controller's
  // own HVAC-on path handles the transmit a few lines later. transmit_mode_with_
  // cs_() brackets the mode command with real CS frames so the new setpoint never
  // lands against a stale CS, and a CS always follows a mode change.
  if ((sp_changed || f_changed) && gear >= 0 &&
      active_ir_mode_ != climate::CLIMATE_MODE_OFF) {
    // Transmit even during a kickstart. The old `!kickstart_active_()` guard here silently
    // dropped setpoint changes for the whole 90s/5:30 wake window — a confirmed cause of
    // "the setpoint won't take". We do NOT re-anchor current_cs_: it is already the kickstart
    // CS, and transmit_mode_with_cs_() brackets the new MODE/setpoint frame with that SAME
    // kickstart CS — so the new setpoint reaches the panel while the compressor-wake stimulus
    // is preserved exactly. (Re-deriving the kickstart CS from the new setpoint could WEAKEN
    // it below the compressor's restart threshold and stall the wake — the failure the
    // kickstart exists to prevent. end_kickstart_ recomputes the correct gear CS for the new
    // setpoint at release.) furrion_setpoint_c_ was already updated above, so the C-mode temp
    // byte and the post-kickstart CS both pick up the new anchor.
    transmit_mode_with_cs_();
  }
}

climate::ClimateFanMode FurrionChillCube::get_effective_fan_mode_() {
  if (clamp_phase_ == ClampPhase::CLAMPED) {
    return climate::CLIMATE_FAN_LOW;
  }
  return this->fan_mode.value_or(climate::CLIMATE_FAN_AUTO);
}

int FurrionChillCube::compute_gear_cs_(bool is_heat, int gear) {
  // Active-gear (1-5 cool / 1-3 heat) CS values use the offset logic to
  // stay within the Furrion's 15-30 CS protocol range — keeps active
  // gears distinguishable at boundary setpoints.
  //
  // Idle-gear (gear 0) is an unambiguous "stop compressor" signal set at
  // setpoint ± 5°C with no offset or clamping applied. At typical setpoints
  // this is well within 15-30; at boundary setpoints (16°C cool, 30°C heat)
  // gear 0 reaches CS=11 or CS=35 respectively. The CS byte is transmitted
  // raw — no clamping in the IR encode path — on the assumption that the
  // Furrion will treat out-of-range CS as a stronger version of the nearest
  // in-range value (verified on a Midea unit at 55-95°F extremes; not
  // explicitly tested on this Furrion but low-risk given symmetric behavior
  // patterns across similar HVAC protocols).
  if (is_heat) {
    int hi = furrion_setpoint_c_ + 2;
    int offset = (hi > 30) ? (30 - hi) : 0;
    switch (gear) {
      case 3:  return furrion_setpoint_c_ - 1 + offset;
      case 2:  return furrion_setpoint_c_     + offset;
      case 1:  return furrion_setpoint_c_ + 1 + offset;
      default: return furrion_setpoint_c_ + 5;   // gear 0: raw, unclamped
    }
  } else {
    int lo = furrion_setpoint_c_ - 3;
    int hi = furrion_setpoint_c_ + 2;
    int offset = (lo < 15) ? (15 - lo) : (hi > 30) ? (30 - hi) : 0;
    switch (gear) {
      case 5:  return furrion_setpoint_c_ + 2 + offset;
      case 4:  return furrion_setpoint_c_ + 1 + offset;
      case 3:  return furrion_setpoint_c_     + offset;
      case 2:  return furrion_setpoint_c_ - 1 + offset;
      case 1:  return furrion_setpoint_c_ - 2 + offset;
      default: return furrion_setpoint_c_ - 5;   // gear 0: raw, unclamped
    }
  }
}

void FurrionChillCube::set_cs_value_(int cs, uint32_t now) {
  current_cs_ = cs;
  if (boot_ready_ && !failsafe_active_ && active_ir_mode_ != climate::CLIMATE_MODE_OFF) {
    transmit_cs_update_();
    last_cs_heartbeat_ = now;
  }
  if (cs_value_sensor_) cs_value_sensor_->publish_state(cs);
}

void FurrionChillCube::update_action_() {
  climate::ClimateAction action;
  if (this->mode == climate::CLIMATE_MODE_OFF) {
    action = climate::CLIMATE_ACTION_OFF;
  } else if (heat_gear_ >= 1) {
    action = climate::CLIMATE_ACTION_HEATING;
  } else if (cool_gear_ >= 1) {
    action = climate::CLIMATE_ACTION_COOLING;
  } else {
    action = climate::CLIMATE_ACTION_IDLE;
  }
  if (this->action != action) {
    this->action = action;
    this->publish_state();
  }
}

void FurrionChillCube::send_swing_state_() {
  if (active_ir_mode_ == climate::CLIMATE_MODE_OFF) return;
  if (this->swing_mode == climate::CLIMATE_SWING_VERTICAL) {
    send_swing_on();
  } else {
    send_swing_off();
  }
}

// ============================================================
// Kickstart State Machine
// ============================================================

void FurrionChillCube::start_clamped_kickstart_(bool is_heat, uint32_t now) {
  // Defensive: clear any prior kickstart state
  quick_kick_active_ = false;
  clamp_is_heat_ = is_heat;
  // Kickstart CS: heat=gear2 CS, cool=gear4 CS (includes boundary offset).
  // No clamping — active-gear CS is already in 15-30 range via the offset.
  clamp_kickstart_cs_ = compute_gear_cs_(is_heat, is_heat ? 2 : 4);

  // PRE_CS: set kickstart CS before mode-on (500ms lead)
  current_cs_ = clamp_kickstart_cs_;
  if (boot_ready_ && !failsafe_active_) {
    transmit_cs_update_();
    last_cs_heartbeat_ = now;
  }
  if (cs_value_sensor_) cs_value_sensor_->publish_state(clamp_kickstart_cs_);

  clamp_phase_ = ClampPhase::PRE_CS;
  clamp_phase_start_ = now;
  ESP_LOGI(TAG, "Clamped kickstart: PRE_CS %s cs=%d", is_heat ? "HEAT" : "COOL", clamp_kickstart_cs_);
}

void FurrionChillCube::start_quick_kickstart_(bool is_heat, int kickstart_cs, uint32_t now, uint32_t hold_ms) {
  // Defensive: clear any prior clamped kickstart state
  clamp_phase_ = ClampPhase::IDLE;
  quick_kick_active_ = true;
  quick_kick_start_ = now;
  quick_kick_cs_ = kickstart_cs;
  quick_kick_hold_ms_ = hold_ms;
  quick_kick_is_heat_ = is_heat;
  quick_kick_reinforced_ = false;

  // Set and transmit kickstart CS immediately
  current_cs_ = quick_kick_cs_;
  if (boot_ready_ && !failsafe_active_ && active_ir_mode_ != climate::CLIMATE_MODE_OFF) {
    transmit_cs_update_();
    last_cs_heartbeat_ = now;
  }
  if (cs_value_sensor_) cs_value_sensor_->publish_state(quick_kick_cs_);

  // If unit is OFF, turn it on now (no fan clamp, fan=AUTO)
  if (active_ir_mode_ == climate::CLIMATE_MODE_OFF) {
    set_active_ir_mode_(is_heat ? climate::CLIMATE_MODE_HEAT : climate::CLIMATE_MODE_COOL);
    transmit_mode_command_();
  }

  ESP_LOGI(TAG, "Quick kickstart: %s cs=%d hold=%lus", is_heat ? "HEAT" : "COOL",
           quick_kick_cs_, (unsigned long) (hold_ms / 1000));
}

void FurrionChillCube::advance_kickstart_(uint32_t now) {
  // === Clamped kickstart state machine ===
  if (clamp_phase_ == ClampPhase::PRE_CS) {
    if ((now - clamp_phase_start_) >= 500) {
      // Mode ON + fan=LOW, then send CS with CS_DATA.
      // NOTE: clamp_phase_ MUST be set to CLAMPED BEFORE transmit_mode_command_()
      // so get_effective_fan_mode_() returns LOW when building the IR frame.
      // (Previous ordering transmitted with fan=AUTO, defeating the clamp entirely.)
      set_active_ir_mode_(clamp_is_heat_ ? climate::CLIMATE_MODE_HEAT : climate::CLIMATE_MODE_COOL);
      clamp_phase_ = ClampPhase::CLAMPED;
      clamp_start_ = now;
      transmit_mode_command_();
      transmit_cs_update_();
      last_cs_heartbeat_ = now;
      ESP_LOGI(TAG, "Clamped kickstart: MODE ON + fan=LOW, clamp=5:30");
    }
  } else if (clamp_phase_ == ClampPhase::CLAMPED) {
    // Check gear threshold: cool 3+, heat 2+
    int drop_gear = clamp_is_heat_ ? 2 : 3;
    int current_gear = clamp_is_heat_ ? heat_gear_ : cool_gear_;
    if (current_gear >= drop_gear) {
      ESP_LOGI(TAG, "Clamped kickstart: ended — gear %d >= %d", current_gear, drop_gear);
      end_kickstart_(now);
      return;
    }
    // Check 5:30 timeout
    if ((now - clamp_start_) >= CLAMP_DURATION_MS) {
      ESP_LOGI(TAG, "Clamped kickstart: ended — 5:30 timeout");
      end_kickstart_(now);
      return;
    }
  }

  // === Quick kickstart ===
  if (quick_kick_active_) {
    uint32_t elapsed = now - quick_kick_start_;
    if (elapsed >= 5000 && !quick_kick_reinforced_) {
      // 5s: reinforce kickstart CS (one-shot). Past this, the 30s CS heartbeat keeps
      // re-asserting quick_kick_cs_, so the CS stays continuously asserted for long holds.
      quick_kick_reinforced_ = true;
      transmit_cs_update_();
      last_cs_heartbeat_ = now;
      ESP_LOGI(TAG, "Quick kickstart: reinforce cs=%d", quick_kick_cs_);
    }
    if (elapsed >= quick_kick_hold_ms_) {
      // hold window elapsed: release — transmit current gear's CS
      ESP_LOGI(TAG, "Quick kickstart: ended — %lus", (unsigned long) (quick_kick_hold_ms_ / 1000));
      end_kickstart_(now);
    }
  }
}

void FurrionChillCube::end_kickstart_(uint32_t now) {
  // Determine heat/cool BEFORE clearing state (active_ir_mode_ may be OFF during PRE_CS)
  bool is_heat = (clamp_phase_ != ClampPhase::IDLE) ? clamp_is_heat_ :
                 quick_kick_active_ ? quick_kick_is_heat_ :
                 (active_ir_mode_ == climate::CLIMATE_MODE_HEAT);

  // Clear all kickstart state
  clamp_phase_ = ClampPhase::IDLE;
  clamp_start_ = 0;
  clamp_phase_start_ = 0;
  quick_kick_active_ = false;
  quick_kick_start_ = 0;
  int gear = is_heat ? heat_gear_ : cool_gear_;
  if (gear >= 0) {
    int cs = compute_gear_cs_(is_heat, gear);
    set_cs_value_(cs, now);
  }

  // Always re-send mode command when unit is ON — ensures Furrion display
  // setpoint is current (fixes stale display after quick kickstart) and
  // restores fan=AUTO after clamped kickstart
  if (active_ir_mode_ != climate::CLIMATE_MODE_OFF) {
    transmit_mode_command_();
  }

  // Reset keep-alive window so the next pulse fires 5 min from kickstart-end,
  // not immediately. The clamped kickstart lasts 5:30 (>KEEPALIVE_INTERVAL_MS),
  // which leaves keepalive_last_ stale by end-of-clamp — without this reset,
  // step 4 in loop() fires a keep-alive pulse on the same iteration that
  // released the clamp. The pulse's step2 CS (anchor±1 = gear 3 CS for heat,
  // gear 3+ CS for cool) over-stimulates the compressor harder than the clamp
  // did, undoing the careful ramp-up.
  bool keepalive_eligible = is_heat ? (heat_gear_ == 1)
                                    : (cool_gear_ >= 1 && cool_gear_ <= 2);
  if (keepalive_eligible) {
    keepalive_last_ = now;
  }

  ESP_LOGI(TAG, "Kickstart released: cs=%d mode_resent=%d ka_reset=%d",
           current_cs_, active_ir_mode_ != climate::CLIMATE_MODE_OFF,
           keepalive_eligible);
}

// ============================================================
// Keep-Alive Pulse (sustain compressor at low CS gears)
// ============================================================

void FurrionChillCube::start_keepalive_(bool is_heat, uint32_t now) {
  keepalive_restore_cs_ = current_cs_;
  int anchor = furrion_setpoint_c_;
  keepalive_step2_cs_ = is_heat ? (anchor - 1) : (anchor + 1);
  set_cs_value_(anchor, now);  // Step 1: setpoint CS
  keepalive_phase_ = KeepAlivePhase::STEP1;
  keepalive_phase_start_ = now;
  ESP_LOGI(TAG, "Keep-alive: start %s pulse, anchor=%d step2=%d restore=%d",
           is_heat ? "HEAT" : "COOL", anchor, keepalive_step2_cs_, keepalive_restore_cs_);
}

void FurrionChillCube::advance_keepalive_(uint32_t now) {
  uint32_t elapsed = now - keepalive_phase_start_;

  switch (keepalive_phase_) {
    case KeepAlivePhase::STEP1:
      if (elapsed >= KEEPALIVE_STEP_MS) {
        set_cs_value_(keepalive_step2_cs_, now);  // Step 2: over-anchor (heat=19, cool=26)
        keepalive_phase_ = KeepAlivePhase::STEP2;
        keepalive_phase_start_ = now;
        ESP_LOGI(TAG, "Keep-alive: cs=%d", keepalive_step2_cs_);
      }
      break;

    case KeepAlivePhase::STEP2:
      if (elapsed >= KEEPALIVE_STEP_MS) {
        set_cs_value_(keepalive_restore_cs_, now);  // Step 3: restore (1/2)
        keepalive_phase_ = KeepAlivePhase::STEP_RESTORE1;
        keepalive_phase_start_ = now;
        ESP_LOGI(TAG, "Keep-alive: restore cs=%d (1/2)", keepalive_restore_cs_);
      }
      break;

    case KeepAlivePhase::STEP_RESTORE1:
      if (elapsed >= KEEPALIVE_STEP_MS) {
        set_cs_value_(keepalive_restore_cs_, now);  // Step 4: restore (2/2) — done
        keepalive_phase_ = KeepAlivePhase::IDLE;
        keepalive_last_ = now;
        ESP_LOGI(TAG, "Keep-alive: restore cs=%d (2/2), done", keepalive_restore_cs_);
      }
      break;

    default:
      keepalive_phase_ = KeepAlivePhase::IDLE;
      break;
  }
}

void FurrionChillCube::abort_keepalive_() {
  if (keepalive_phase_ != KeepAlivePhase::IDLE) {
    ESP_LOGI(TAG, "Keep-alive: aborted (phase=%d)", (int)keepalive_phase_);
    keepalive_phase_ = KeepAlivePhase::IDLE;
    keepalive_last_ = millis();  // fresh 5-min window (not 0, which could cause immediate retrigger)
  }
}

// ============================================================
// Timed Vane Positioning
// ============================================================

// Called on an OFF→active transition (from set_active_ir_mode_). Starts the timed
// homing only when the mode's (delay, interval) are BOTH configured AND the user has
// the vane switch OFF (swing_mode == OFF). When swing is ON the user wants oscillation,
// so there is no fixed position to seek and we leave the vane alone.
void FurrionChillCube::maybe_start_vent_positioning_(bool is_heat) {
  uint32_t delay_ms = is_heat ? heat_vent_move_delay_ms_ : cool_vent_move_delay_ms_;
  uint32_t interval_ms = is_heat ? heat_vent_interval_ms_ : cool_vent_interval_ms_;
  if (delay_ms == 0 || interval_ms == 0) return;                     // feature unset for this mode
  if (this->swing_mode != climate::CLIMATE_SWING_OFF) return;        // user wants oscillation
  vent_active_delay_ms_ = delay_ms;
  vent_active_interval_ms_ = interval_ms;
  vent_phase_ = VentPhase::WAIT_DELAY;
  vent_phase_start_ = millis();
  ESP_LOGI(TAG, "Vane: %s positioning armed — wait %lus then move %lus",
           is_heat ? "HEAT" : "COOL",
           (unsigned long)(delay_ms / 1000), (unsigned long)(interval_ms / 1000));
}

// Non-blocking state machine. Sends ONLY raw swing IR and never touches this->swing_mode,
// so the HA swing switch stays "off" the whole time (the homing is invisible to the user).
void FurrionChillCube::advance_vent_positioning_() {
  // Self-clock on millis(): vent_phase_start_ is set (in maybe_start_/below) to a millis()
  // value that is newer than loop()'s cached `now`, so using the cached `now` here would
  // underflow `elapsed` and fire the move instantly. See the header note.
  uint32_t now = millis();
  uint32_t elapsed = now - vent_phase_start_;
  switch (vent_phase_) {
    case VentPhase::WAIT_DELAY:
      if (elapsed >= vent_active_delay_ms_) {
        send_swing_on();                       // raw SWING_ON — start the vane moving
        vent_phase_ = VentPhase::MOVING;
        vent_phase_start_ = now;
        ESP_LOGI(TAG, "Vane: move ON (hold %lus)", (unsigned long)(vent_active_interval_ms_ / 1000));
      }
      break;
    case VentPhase::MOVING:
      if (elapsed >= vent_active_interval_ms_) {
        send_swing_off();                      // raw SWING_OFF — stop at the target position
        vent_phase_ = VentPhase::IDLE;
        ESP_LOGI(TAG, "Vane: move OFF — positioned");
      }
      break;
    default:
      vent_phase_ = VentPhase::IDLE;
      break;
  }
}

// Pure state reset (no IR). Every abort trigger — unit OFF (the OFF command's own swing
// frame stops the vane) or a user swing toggle (send_swing_state_() sets the requested
// state) — already emits the correct swing frame, so emitting one here would only fight it.
void FurrionChillCube::abort_vent_positioning_() {
  if (vent_phase_ != VentPhase::IDLE) {
    ESP_LOGI(TAG, "Vane: positioning aborted (phase=%d)", (int)vent_phase_);
    vent_phase_ = VentPhase::IDLE;
  }
}

// Force a real OFF + off-dwell on a heat↔cool transition. Turns the unit OFF, stamps
// off_since_ so the fresh-start off_long_enough gate then holds the new mode off for
// mode_switch_off_ms_, and tears down any kickstart/keepalive. The OFF→ON that follows
// the dwell re-homes the vane. set_active_ir_mode_(OFF) also aborts any vane sequence.
void FurrionChillCube::force_off_for_mode_switch_(uint32_t now) {
  set_active_ir_mode_(climate::CLIMATE_MODE_OFF);
  transmit_mode_command_();   // OFF on the wire; its swing frame parks the vane
  heat_gear_ = -1;
  cool_gear_ = -1;
  off_since_ = now;
  bias_c_ = 0.0f;
  clamp_phase_ = ClampPhase::IDLE;
  quick_kick_active_ = false;
  abort_keepalive_();
  if (heat_gear_sensor_) heat_gear_sensor_->publish_state(-1);
  if (cool_gear_sensor_) cool_gear_sensor_->publish_state(-1);
  if (compressor_output_sensor_) compressor_output_sensor_->publish_state(0.0f);
  update_action_();
}

// ============================================================
// Gear Controller
// ============================================================

// Hold time in the current gear. The 999999999 sentinel ("no gear change yet
// this boot session") reads as effectively-infinite so any can_upshift_to()
// check passes on the first computation.
uint32_t FurrionChillCube::time_in_gear_(uint32_t now) {
  return (last_gear_change_ == 0) ? 999999999 : (now - last_gear_change_);
}

void FurrionChillCube::run_gear_controller_() {
  float room = inside_temp_c_;
  uint32_t now = millis();

  // Failsafe scenarios + NaN-room grace (also clears a stale failsafe). Returns
  // true if a failsafe/grace path was taken — stop this pass.
  if (check_failsafe_(now, room)) return;

  // Fix: seed idle_since_ if gear is 0 but timer was never set (post-boot recovery)
  if ((heat_gear_ == 0 || cool_gear_ == 0) && idle_since_ == 0) {
    idle_since_ = now;
  }

  bool user_input = user_changed_;
  if (user_input) user_changed_ = false;

  // Captured before dispatch so the periodic state log reports the hold time the
  // gear logic actually saw (a gear change this pass resets last_gear_change_).
  uint32_t time_in_gear = time_in_gear_(now);

  update_outside_lockout_();

  bool do_heat = false, do_cool = false;
  arbitrate_mode_(room, do_heat, do_cool);

  // Dispatch to the active mode. run_heat/cool_mode_ return true if they took an
  // early (NaN-target) hold-return — propagate it so this pass stops cleanly.
  float gear_diff = NAN;  // active diff, for debug publishing
  if (do_heat) {
    if (run_heat_mode_(room, now, user_input, gear_diff)) return;
  } else if (do_cool) {
    if (run_cool_mode_(room, now, user_input, gear_diff)) return;
  } else {
    run_idle_mode_(now);
  }

  // Phase 2: the adaptive bias only exists during active cooling. Clear it whenever cooling
  // is not the active mode so a heat→cool switch (or a return from full-off) starts fresh
  // rather than inheriting a stale equilibrium. (Within-cool gear-0 idle is handled by the
  // decay in adaptive_cool_eff_diff_, which still runs because do_cool stays true at gear 0.)
  if (!do_cool) bias_c_ = 0.0f;

  // Boot gate: first successful gear computation enables IR
  if (!boot_ready_) {
    boot_ready_ = true;
    ESP_LOGI(TAG, "Boot ready — first gear computation complete, IR enabled");
  }

  // Persist gear + bias for the warm-reboot restore (no-op unless one changed)
  save_gear_pref_();

  // Debug sensor publishing
  publish_debug_state_(gear_diff);

  // Periodic state log
  ESP_LOGD(TAG, "state: heat=%d cool=%d room=%.2f cs=%d hold=%lus idle=%lum mode=%d bias=%.2f drift=%.3f",
           heat_gear_, cool_gear_, room, current_cs_,
           time_in_gear / 1000,
           idle_since_ > 0 ? (now - idle_since_) / 60000 : 0,
           (int)last_active_mode_, bias_c_, room_drift_cpm_);
}

// Failsafe detection + NaN-room grace. Returns true if the caller should stop
// this pass (a failsafe or NaN-grace path was taken). On the normal path it
// also clears a stale failsafe_active_ flag.
bool FurrionChillCube::check_failsafe_(uint32_t now, float room) {
  // === Failsafe scenario 1: Boot, HA never connects (5 min) ===
  bool never_got_update = (last_temp_update_ == 0 && (now - boot_time_) > 300000);

  // === Failsafe scenario 2: HA API disconnected (15 min) ===
#ifdef USE_API
  bool api_connected = (api::global_api_server != nullptr &&
                        api::global_api_server->is_connected());
  if (api_connected) {
    ha_disconnect_time_ = 0;
  } else if (ha_disconnect_time_ == 0 && last_temp_update_ > 0) {
    ha_disconnect_time_ = now;
    ESP_LOGW(TAG, "HA API disconnected — 15-min failsafe timer started");
  }
  bool ha_disconnected = (ha_disconnect_time_ > 0 &&
                          (now - ha_disconnect_time_) > 900000);
#else
  bool ha_disconnected = false;
#endif

  // === Failsafe scenario 3: Room temp NaN (5 min grace) ===
  if (isnan(room)) {
    if (temp_nan_since_ == 0) {
      temp_nan_since_ = now;
      ESP_LOGW(TAG, "Room temp NaN — 5-min grace period started");
    }
  } else {
    temp_nan_since_ = 0;
  }
  bool temp_unavailable = (temp_nan_since_ > 0 && (now - temp_nan_since_) > 300000);

  // === Failsafe trigger ===
  if (never_got_update || ha_disconnected || temp_unavailable) {
    ESP_LOGW(TAG, "FAILSAFE — CS stopped, unit will revert to internal sensor (boot=%d ha_dc=%d nan=%d)",
             never_got_update, ha_disconnected, temp_unavailable);
    heat_gear_ = -1;
    cool_gear_ = -1;
    bias_c_ = 0.0f;   // drop the adaptive equilibrium on failsafe — re-engage learns fresh
    idle_since_ = 0;
    last_active_mode_ = MODE_NONE;
    last_mode_event_at_ = 0;
    off_since_ = 0;
    heater_locked_out_ = false;
    setpoint_pending_ = false;  // drop any in-flight debounce — no deferred commit after failsafe
    clamp_phase_ = ClampPhase::IDLE;
    quick_kick_active_ = false;
    abort_keepalive_();
    if (heat_gear_sensor_) heat_gear_sensor_->publish_state(-1);
    if (cool_gear_sensor_) cool_gear_sensor_->publish_state(-1);
    if (compressor_output_sensor_) compressor_output_sensor_->publish_state(0.0f);
    // Sync active_ir_mode_ (and mode_pref_) to OFF to match gear state.
    // No IR is transmitted — set_active_ir_mode_() only updates state + saves flash.
    // Without this, mode_pref_ stays at HEAT/COOL and a reboot during failsafe
    // causes setup() to restore gear=0, producing a spurious MODE_ON IR command
    // on the next controller run (6A compressor spike).
    if (active_ir_mode_ != climate::CLIMATE_MODE_OFF) {
      set_active_ir_mode_(climate::CLIMATE_MODE_OFF);
    }
    // Stop all IR transmission — failsafe_active_ gates CS heartbeat.
    // Unit's setpoint is already near the desired target (dynamic setpoint).
    // After ~7 min with no CS, unit reverts to its own internal sensor.
    failsafe_active_ = true;
    boot_ready_ = true;
    update_action_();
    return true;
  }

  // NaN grace: hold current state
  if (isnan(room)) {
    ESP_LOGD(TAG, "Room temp NaN — holding (%lus left)",
             (300000 - (now - temp_nan_since_)) / 1000);
    publish_debug_state_(NAN);
    return true;
  }

  // Clear failsafe if normal operation
  if (failsafe_active_) {
    failsafe_active_ = false;
    ESP_LOGI(TAG, "Failsafe cleared — normal operation resumed");
  }
  return false;
}

// Outside-temperature lockout for heating (1°F hysteresis; fail-open if the
// outside sensor is unavailable).
void FurrionChillCube::update_outside_lockout_() {
  if (!isnan(outside_temp_c_)) {
    float lockout_low = outside_lockout_temp_c_ - 0.556f;  // 1°F hysteresis
    if (outside_temp_c_ < lockout_low && !heater_locked_out_) {
      heater_locked_out_ = true;
      ESP_LOGW(TAG, "Outside %.1f°C < lockout — heating locked out", outside_temp_c_);
    } else if (outside_temp_c_ >= outside_lockout_temp_c_ && heater_locked_out_) {
      heater_locked_out_ = false;
      ESP_LOGI(TAG, "Outside %.1f°C >= lockout — heating re-enabled", outside_temp_c_);
    }
  } else if (heater_locked_out_) {
    // Outside sensor unavailable — fail-open to allow heating
    heater_locked_out_ = false;
    ESP_LOGW(TAG, "Outside temp unavailable — heating lockout cleared (fail-open)");
  }
}

// Picks the active mode for this pass. At most one of do_heat / do_cool is true.
void FurrionChillCube::arbitrate_mode_(float room, bool &do_heat, bool &do_cool) {
  // Determine active modes from user's climate mode
  bool heater_on = !heater_locked_out_ &&
                   (this->mode == climate::CLIMATE_MODE_HEAT ||
                    this->mode == climate::CLIMATE_MODE_HEAT_COOL);
  bool cooler_on = (this->mode == climate::CLIMATE_MODE_COOL ||
                    this->mode == climate::CLIMATE_MODE_HEAT_COOL);

  // When both modes enabled, decide based on room temp vs targets
  do_heat = heater_on;
  do_cool = cooler_on;
  if (heater_on && cooler_on) {
    float h_target = get_heat_target_();
    float c_target = get_cool_target_();
    // Guard: if targets are inverted (low >= high), swap to prevent stuck mode
    if (!isnan(h_target) && !isnan(c_target) && h_target > c_target) {
      float tmp = h_target; h_target = c_target; c_target = tmp;
    }

    // Mode change protection now lives in the 0→-1 gate (idle/event/temp lockouts)
    // and -1→active gate (1-min off lockout). Here we just pick the active mode
    // based on current gear state and temperature.
    if (heat_gear_ >= 0) {
      do_cool = false;  // heat already engaged — stay in heat
    } else if (cool_gear_ >= 0) {
      do_heat = false;  // cool already engaged — stay in cool
    } else {
      // Both gears -1 (fully off). Pick based on temperature.
      if (room <= h_target) { do_cool = false; }
      else if (room >= c_target) { do_heat = false; }
      else {
        // Room in deadband — neither mode should engage. Stay off.
        do_heat = false;
        do_cool = false;
      }
    }
  }
}

// HEATING mode pass. Returns true if it took an early (NaN-target) hold-return.
bool FurrionChillCube::run_heat_mode_(float room, uint32_t now, bool user_input,
                                     float &gear_diff) {
  // The cool-mode adaptive bias is meaningless in heat; clear it here (not just in the
  // post-dispatch !do_cool clear) so it's cleared even if this pass early-returns on a NaN
  // heat target — guaranteeing a later heat→cool switch always starts from bias_c_ = 0.
  bias_c_ = 0.0f;

  uint32_t time_in_gear = time_in_gear_(now);
  auto can_upshift_to = [&](int target_gear) -> bool {
    return user_input || (time_in_gear >= HOLD_MS[target_gear]);
  };

  // Cool→Heat MUST route through a real OFF held for the off-dwell — same rationale as
  // run_cool_mode_ (compressor safety + vane OFF→ON anchor). Catches a DIRECT switch while
  // the unit is still actively COOLing; the natural HEAT_COOL handoff already turned it OFF.
  if (active_ir_mode_ == climate::CLIMATE_MODE_COOL) {
    ESP_LOGI(TAG, "Cool→Heat: force OFF + %lus dwell before heating",
             (unsigned long)(mode_switch_off_ms_ / 1000));
    force_off_for_mode_switch_(now);
    return true;  // hold; heat re-engages from -1 after the dwell (OFF→ON homes the vane)
  }
  // Defensive: clear a stale cool gear (unit already OFF via the natural path).
  if (cool_gear_ != -1) {
    cool_gear_ = -1;
    if (cool_gear_sensor_) cool_gear_sensor_->publish_state(-1);
  }

  update_furrion_setpoint_(true);
  float target = get_heat_target_();
  if (isnan(target)) {
    ESP_LOGW(TAG, "Heat target NaN — holding gear/CS");
    user_changed_ = user_input;  // don't consume a user event during a NaN hold
    publish_debug_state_(NAN);
    return true;
  }
  float diff = room - target;
  gear_diff = diff;
  int gear = heat_gear_;
  int new_gear = gear;

  if (gear == -1 || user_input) {
    // Fresh start from -1 requires 1-min off lockout (hardware wind-down).
    // No bypass — applies even to user_input.
    bool off_long_enough = (off_since_ == 0) || (now - off_since_ >= mode_switch_off_ms_);
    if (gear == -1 && !off_long_enough) {
      new_gear = -1;  // still in 1-min wind-down period
    } else if (user_input && gear >= 0 && gear_in_band_heat_(gear, diff)) {
      // User event (setpoint/fan tweak) but current gear is still valid for
      // the current diff — preserve hunting state instead of recomputing.
      new_gear = gear;
    } else {
      // From -1, only go to 1+ (never 0 — gear 0 is only reachable by downshift from 1)
      if (diff < H_UP_23)         new_gear = 3;
      else if (diff < H_UP_12)    new_gear = 2;
      else if (diff < H_UP_01)    new_gear = 1;
      else if (gear == -1)        new_gear = -1;  // stays off
      // user_input at gear >=0 with room past setpoint → go to -1 (bypass restrictions)
      else if (user_input && diff > H_IDLE) new_gear = -1;
      else                        new_gear = 0;
    }
  } else {
    switch (gear) {
      case 0: {
        // First compute post-restore: skip the 0→1→2→3 HOLD_MS ladder
        // (~6 min) and jump straight to the correct gear.
        if (last_gear_change_ == 0) {
          if (diff < H_UP_23)         new_gear = 3;
          else if (diff < H_UP_12)    new_gear = 2;
          else if (diff < H_UP_01)    new_gear = 1;
        } else {
          if (can_upshift_to(1) && diff < H_UP_01) new_gear = 1;
        }
        // 0→-1 gate (natural path only — user_input handled in fresh-start above)
        bool imm_off = !boot_ready_;
        bool idle_enough = (idle_since_ > 0) && (now - idle_since_ >= mode_switch_idle_ms_);
        bool event_ok = (last_mode_event_at_ == 0) || (now - last_mode_event_at_ >= mode_switch_event_ms_);
        bool past_setpoint = diff > mode_switch_temp_offset_c_;
        bool natural_off = idle_enough && event_ok && past_setpoint;
        if ((imm_off || natural_off) && diff > H_IDLE) new_gear = -1;
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

  // Track idle_since
  if (new_gear == 0 && gear != 0) {
    idle_since_ = now;
  } else if (new_gear != 0) {
    idle_since_ = 0;
  }

  // Track off_since_ for the 1-min off lockout
  if (new_gear == -1 && gear != -1) {
    off_since_ = now;
  }

  // Publish gear and compressor on change
  if (new_gear != gear) {
    heat_gear_ = new_gear;
    last_gear_change_ = now;
    if (heat_gear_sensor_) heat_gear_sensor_->publish_state(new_gear);
    if (new_gear >= 1) last_active_mode_ = MODE_HEAT;
    if (compressor_output_sensor_)
      compressor_output_sensor_->publish_state(gear_output_pct(true, new_gear));
    ESP_LOGI(TAG, "HEAT %d -> %d (room=%.2f target=%.2f diff=%.2f)",
             gear, new_gear, room, target, diff);

    // Keep-alive: reset timer on entering eligible range, abort on leaving
    if (new_gear == 1) {
      keepalive_last_ = now;  // fresh 5-min window
    } else {
      abort_keepalive_();
    }
  }

  // CS + kickstart logic
  if (new_gear >= 0) {
    int cs = compute_gear_cs_(true, new_gear);
    if (gear == -1 && new_gear == 1) {
      // OFF→gear 1: clamped kickstart (fan=LOW 5:30, CS=gear2 level)
      last_mode_event_at_ = now;
      start_clamped_kickstart_(true, now);
    } else if (gear == -1 && new_gear >= 2) {
      // OFF→gear 2+: direct start, no kickstart needed
      last_mode_event_at_ = now;
      if (!kickstart_active_() && current_cs_ != cs) {
        set_cs_value_(cs, now);
      }
    } else if (!kickstart_active_() && current_cs_ != cs) {
      set_cs_value_(cs, now);
    }
  }

  // HVAC on/off
  if (new_gear >= 0 && active_ir_mode_ != climate::CLIMATE_MODE_HEAT && !kickstart_active_()) {
    set_active_ir_mode_(climate::CLIMATE_MODE_HEAT);
    transmit_mode_with_cs_();
  }
  if (new_gear == -1 && active_ir_mode_ != climate::CLIMATE_MODE_OFF) {
    set_active_ir_mode_(climate::CLIMATE_MODE_OFF);
    transmit_mode_command_();
    if (kickstart_active_()) end_kickstart_(now);
  }

  update_action_();
  return false;
}

// COOLING mode pass. Returns true if it took an early (NaN-target) hold-return.
bool FurrionChillCube::run_cool_mode_(float room, uint32_t now, bool user_input,
                                     float &gear_diff) {
  uint32_t time_in_gear = time_in_gear_(now);
  auto can_upshift_to = [&](int target_gear) -> bool {
    return user_input || (time_in_gear >= HOLD_MS[target_gear]);
  };

  // Heat→Cool MUST route through a real OFF held for the off-dwell: compressor safety
  // AND the vane's known OFF→ON re-home anchor. This catches a DIRECT switch — the unit
  // still actively HEATing when COOL is selected (user flip, or sync_mode handoff). The
  // natural HEAT_COOL drift already turns the unit OFF via the heat 0→-1 path, so by the
  // time it reaches here active_ir_mode_ is already OFF and this is skipped.
  if (active_ir_mode_ == climate::CLIMATE_MODE_HEAT) {
    ESP_LOGI(TAG, "Heat→Cool: force OFF + %lus dwell before cooling",
             (unsigned long)(mode_switch_off_ms_ / 1000));
    force_off_for_mode_switch_(now);
    return true;  // hold; cool re-engages from -1 after the dwell (OFF→ON homes the vane)
  }
  // Defensive: clear a stale heat gear (unit already OFF via the natural path).
  if (heat_gear_ != -1) {
    heat_gear_ = -1;
    if (heat_gear_sensor_) heat_gear_sensor_->publish_state(-1);
  }

  update_furrion_setpoint_(false);
  float target = get_cool_target_();
  if (isnan(target)) {
    ESP_LOGW(TAG, "Cool target NaN — holding gear/CS");
    user_changed_ = user_input;  // don't consume a user event during a NaN hold
    publish_debug_state_(NAN);
    return true;
  }
  float diff = room - target;
  gear_diff = diff;  // debug always reports REAL (unbiased) diff
  // Phase 2 adaptive: advance the integral and get the effective diff used ONLY for the
  // active-gear switch cases (1-5). Re-engage/idle/mode-switch decisions stay on real diff.
  // Called every cool pass so the integral advances (or decays while idle) consistently.
  float eff_diff = adaptive_cool_eff_diff_(diff, now, time_in_gear);
  // Upshift comparisons use the rate-gated diff (bias removed while the room isn't warming);
  // downshifts and re-engage/idle decisions keep using eff_diff / real diff respectively.
  float up_diff = cool_eff_up_diff_;
  int gear = cool_gear_;
  int new_gear = gear;

  if (gear == -1 || user_input) {
    // Fresh start from -1 requires 1-min off lockout (hardware wind-down).
    // No bypass — applies even to user_input.
    bool off_long_enough = (off_since_ == 0) || (now - off_since_ >= mode_switch_off_ms_);
    if (gear == -1 && !off_long_enough) {
      new_gear = -1;  // still in 1-min wind-down period
    } else if (user_input && gear >= 0 && gear_in_band_cool_(gear, diff)) {
      // User event (setpoint/fan tweak) but the current gear is still valid for the current
      // diff — preserve hunting state instead of recomputing.
      new_gear = gear;
    } else {
      // From -1: minimum gear 2 (gear 1 can't cold-start the compressor),
      // and never gear 0 (only reachable by downshift from 1)
      if (diff > C_UP_45)         new_gear = 5;
      else if (diff > C_UP_34)    new_gear = 4;
      else if (diff > C_UP_23)    new_gear = 3;
      else if (diff > C_UP_12)    new_gear = 2;
      else if (diff > C_UP_01)    new_gear = (gear == -1) ? 2 : 1;
      else if (gear == -1)        new_gear = -1;  // stays off
      // user_input at gear >=0 with room past setpoint → go to -1 (bypass restrictions)
      else if (user_input && diff < C_IDLE) new_gear = -1;
      else                        new_gear = 0;
    }
    // NOTE: this whole user_input/fresh-start block deliberately uses REAL diff, NOT eff_diff.
    // The adaptive bias governs only the STEADY-STATE switch cases (1-5) below. A user action is
    // a transient: respond conservatively to the actual room temperature and let the bias
    // re-apply over the next few normal passes via the switch. This keeps the path bit-identical
    // to the non-adaptive ladder and avoids the bias ever forcing an overcool/over-collapse on a
    // user tap. (Round-1/2 attempts to make this path bias-aware introduced regressions; reverted
    // Round-3.)
  } else {
    switch (gear) {
      case 0: {
        // First compute post-restore: skip the 0→1→2→3 HOLD_MS ladder
        // (~6 min) and jump straight to the correct gear.
        if (last_gear_change_ == 0) {
          if (diff > C_UP_45)         new_gear = 5;
          else if (diff > C_UP_34)    new_gear = 4;
          else if (diff > C_UP_23)    new_gear = 3;
          else if (diff > C_UP_12)    new_gear = 2;
          else if (diff > C_UP_01)    new_gear = 1;
        } else {
          if (can_upshift_to(1) && diff > C_UP_01) new_gear = 1;
        }
        // 0→-1 gate (natural path only — user_input handled in fresh-start above)
        bool imm_off = !boot_ready_;
        bool idle_enough = (idle_since_ > 0) && (now - idle_since_ >= mode_switch_idle_ms_);
        bool event_ok = (last_mode_event_at_ == 0) || (now - last_mode_event_at_ >= mode_switch_event_ms_);
        bool past_setpoint = diff < -mode_switch_temp_offset_c_;
        bool natural_off = idle_enough && event_ok && past_setpoint;
        if ((imm_off || natural_off) && diff < C_IDLE) new_gear = -1;
        break;
      }
      // Active gears (1-5) select on eff_diff (= diff + adaptive bias + fan feedforward),
      // floating the ladder's center to today's sustaining gear. eff_diff == diff when
      // adaptive is disabled, so this is bit-identical to the static ladder in that case.
      case 1:
        if (can_upshift_to(2) && up_diff > C_UP_12)   new_gear = 2;
        else if (eff_diff < C_DN_10)                   new_gear = 0;
        break;
      case 2:
        if (can_upshift_to(3) && up_diff > C_UP_23)   new_gear = 3;
        else if (eff_diff < C_DN_21)                   new_gear = 1;
        break;
      case 3:
        if (can_upshift_to(4) && up_diff > C_UP_34)   new_gear = 4;
        else if (eff_diff < C_DN_32)                   new_gear = 2;
        break;
      case 4:
        if (can_upshift_to(5) && up_diff > C_UP_45)   new_gear = 5;
        else if (eff_diff < C_DN_43)                   new_gear = 3;
        break;
      case 5:
        if (eff_diff < C_DN_54)                        new_gear = 4;
        break;
    }
  }

  // Track idle_since
  if (new_gear == 0 && gear != 0) {
    idle_since_ = now;
  } else if (new_gear != 0) {
    idle_since_ = 0;
  }

  // Track off_since_ for the 1-min off lockout
  if (new_gear == -1 && gear != -1) {
    off_since_ = now;
  }

  // Publish gear and compressor on change
  if (new_gear != gear) {
    cool_gear_ = new_gear;
    last_gear_change_ = now;
    if (cool_gear_sensor_) cool_gear_sensor_->publish_state(new_gear);
    if (new_gear >= 1) last_active_mode_ = MODE_COOL;
    if (compressor_output_sensor_)
      compressor_output_sensor_->publish_state(gear_output_pct(false, new_gear));
    ESP_LOGI(TAG, "COOL %d -> %d (room=%.2f target=%.2f diff=%.2f)",
             gear, new_gear, room, target, diff);

    // Keep-alive: reset timer on entering eligible range, abort on leaving
    if (new_gear >= 1 && new_gear <= 2) {
      keepalive_last_ = now;  // fresh 5-min window
    } else {
      abort_keepalive_();
    }
  }

  // CS + kickstart logic
  if (new_gear >= 0) {
    int cs = compute_gear_cs_(false, new_gear);
    if (gear == -1 && new_gear == 2) {
      // OFF→gear 2: clamped kickstart (fan=LOW 5:30, CS=gear4 level)
      last_mode_event_at_ = now;
      start_clamped_kickstart_(false, now);
    } else if (gear == -1 && new_gear == 3) {
      // OFF→gear 3: quick kickstart (CS=gear4 briefly, then gear 3). A short hold is
      // sufficient here — start_quick_kickstart_ also sends a MODE-ON command from OFF,
      // which does the heavy lifting of waking the compressor.
      last_mode_event_at_ = now;
      start_quick_kickstart_(false, compute_gear_cs_(false, 4), now, QUICK_KICK_HOLD_MS);
    } else if (gear == -1 && new_gear >= 4) {
      // OFF→gear 4+: direct, no kickstart
      last_mode_event_at_ = now;
      if (!kickstart_active_() && current_cs_ != cs) {
        set_cs_value_(cs, now);
      }
    } else if (gear == 0 && new_gear >= 1 && new_gear <= 2) {
      // Idle→gear 1-2: quick kickstart at gear-4 CS (setpoint+1°C demand), held for
      // IDLE_KICK_HOLD_MS (90s). This path is CS-only — the unit is already in COOL
      // mode, so there is NO MODE-ON command behind it. A 10s hold failed on
      // 2026-05-20: the CS=21 pulse fired inside the compressor's anti-short-cycle
      // restart lockout and was gone before the lockout cleared, leaving the unit
      // stopped ~11 min. The 90s hold keeps CS=21 asserted across that lockout.
      // gear-4 CS (anchor+1) is the documented restart threshold; gear-3 CS (anchor)
      // is sustain-only — see session_log 2026-05-20.
      start_quick_kickstart_(false, compute_gear_cs_(false, 4), now, IDLE_KICK_HOLD_MS);
    } else if (!kickstart_active_() && current_cs_ != cs) {
      set_cs_value_(cs, now);
    }
  }

  // HVAC on/off
  if (new_gear >= 0 && active_ir_mode_ != climate::CLIMATE_MODE_COOL && !kickstart_active_()) {
    set_active_ir_mode_(climate::CLIMATE_MODE_COOL);
    transmit_mode_with_cs_();
  }
  if (new_gear == -1 && active_ir_mode_ != climate::CLIMATE_MODE_OFF) {
    set_active_ir_mode_(climate::CLIMATE_MODE_OFF);
    transmit_mode_command_();
    if (kickstart_active_()) end_kickstart_(now);
  }

  update_action_();
  return false;
}

// NEITHER-active pass: ensure the unit is OFF and both gears are -1.
void FurrionChillCube::run_idle_mode_(uint32_t now) {
  // Ensure HVAC is OFF
  if (active_ir_mode_ != climate::CLIMATE_MODE_OFF) {
    set_active_ir_mode_(climate::CLIMATE_MODE_OFF);
    transmit_mode_command_();
    clamp_phase_ = ClampPhase::IDLE;
    quick_kick_active_ = false;
    abort_keepalive_();
    if (compressor_output_sensor_) compressor_output_sensor_->publish_state(0.0f);
    // Start the absolute 60s off-lockout. This is the user-OFF (or both-off) path that
    // turns the unit off WITHOUT going through run_heat/cool_mode_'s -1 transition, so it
    // is the one active→OFF route that didn't stamp off_since_. Without this, heat→OFF→cool
    // (or any mode→OFF→mode) re-engaged immediately. The off_long_enough gate in the
    // fresh-start paths then holds ANY re-engage for mode_switch_off_ms_. Reboot is exempt:
    // setup() restores to gear 0 in the saved mode and never reaches here.
    off_since_ = now;
    ESP_LOGI(TAG, "NEITHER ACTIVE — HVAC OFF (60s lockout armed)");
  }

  // Enforce neutral CS (at setpoint — unit is OFF so doesn't matter much)
  if (current_cs_ != furrion_setpoint_c_) {
    set_cs_value_(furrion_setpoint_c_, now);
  }

  // Force gears to -1
  if (heat_gear_ != -1) {
    ESP_LOGI(TAG, "HEAT %d -> -1 (neither active)", heat_gear_);
    heat_gear_ = -1;
    if (heat_gear_sensor_) heat_gear_sensor_->publish_state(-1);
  }
  if (cool_gear_ != -1) {
    ESP_LOGI(TAG, "COOL %d -> -1 (neither active)", cool_gear_);
    cool_gear_ = -1;
    if (cool_gear_sensor_) cool_gear_sensor_->publish_state(-1);
  }
  idle_since_ = 0;

  update_action_();
}

// ============================================================
// Debug State Publishing
// ============================================================

void FurrionChillCube::publish_debug_state_(float diff) {
  // Skip if no debug sensors registered
  if (!debug_active_ir_mode_sensor_) return;

  uint32_t now = millis();

  // Helper: only publish if value changed (reduces HA state churn)
  auto pub = [](sensor::Sensor *s, float val) {
    if (s && (isnan(val) != isnan(s->state) || (!isnan(val) && val != s->state))) {
      s->publish_state(val);
    }
  };

  // Active IR mode: 0=OFF, 1=HEAT, 2=COOL
  float ir_mode = (active_ir_mode_ == climate::CLIMATE_MODE_HEAT) ? 1.0f :
                  (active_ir_mode_ == climate::CLIMATE_MODE_COOL) ? 2.0f : 0.0f;
  pub(debug_active_ir_mode_sensor_, ir_mode);
  pub(debug_last_active_mode_sensor_, (float)last_active_mode_);

  float phase = (float)(uint8_t)clamp_phase_;
  if (quick_kick_active_) phase = 10.0f;
  pub(debug_kick_phase_sensor_, phase);

  pub(debug_gear_diff_sensor_, diff);

  float tig = (last_gear_change_ == 0) ? -1.0f : (float)((now - last_gear_change_) / 1000);
  pub(debug_time_in_gear_sensor_, tig);

  float idle = (idle_since_ == 0) ? -1.0f : (float)((now - idle_since_) / 1000);
  pub(debug_idle_duration_sensor_, idle);

  // Debug "mode_switch_cooldown" sensor now reflects off_since_ 1-min lockout
  float cd = 0.0f;
  if (off_since_ > 0) {
    uint32_t elapsed = now - off_since_;
    cd = (elapsed < mode_switch_off_ms_) ? (float)((mode_switch_off_ms_ - elapsed) / 1000) : 0.0f;
  }
  pub(debug_mode_switch_cooldown_sensor_, cd);

  float clamp = 0.0f;
  if (clamp_phase_ == ClampPhase::CLAMPED && clamp_start_ > 0) {
    uint32_t elapsed = now - clamp_start_;
    clamp = (elapsed < CLAMP_DURATION_MS) ? (float)((CLAMP_DURATION_MS - elapsed) / 1000) : 0.0f;
  }
  pub(debug_fan_clamp_remaining_sensor_, clamp);

  pub(debug_heater_locked_out_sensor_, heater_locked_out_ ? 1.0f : 0.0f);
  pub(debug_failsafe_active_sensor_, failsafe_active_ ? 1.0f : 0.0f);
  pub(debug_boot_ready_sensor_, boot_ready_ ? 1.0f : 0.0f);

  // Phase 2 adaptive observability
  pub(debug_adaptive_bias_c_sensor_, bias_c_);
  pub(debug_room_drift_sensor_, room_drift_cpm_);
  pub(debug_fan_feedforward_sensor_,
      (adaptive_enable_ && vent_fan_on_()) ? (float)fan_feedforward_gears_ : 0.0f);
}

// ============================================================
// Diagnostics
// ============================================================

void FurrionChillCube::dump_config() {
  ESP_LOGCONFIG(TAG, "Furrion Chill Cube:");
  ESP_LOGCONFIG(TAG, "  Outside Lockout: %.1f°F (%.1f°C)",
                outside_lockout_temp_c_ * 9.0f / 5.0f + 32.0f, outside_lockout_temp_c_);
  ESP_LOGCONFIG(TAG, "  Inside Temp Unit: %s", inside_temp_fahrenheit_ ? "°F" : "°C");
  if (outside_temp_sensor_) {
    ESP_LOGCONFIG(TAG, "  Outside Temp Unit: %s", outside_temp_fahrenheit_ ? "°F" : "°C");
  }
  const char *mode_str = (heat_gear_ >= 0) ? "HEAT (restored from prior session)" :
                         (cool_gear_ >= 0) ? "COOL (restored from prior session)" :
                         "none (fresh boot)";
  ESP_LOGCONFIG(TAG, "  Prior Mode: %s", mode_str);
  ESP_LOGCONFIG(TAG, "  Mode-switch off-dwell: %lus", (unsigned long)(mode_switch_off_ms_ / 1000));
  if (heat_vent_move_delay_ms_ && heat_vent_interval_ms_) {
    ESP_LOGCONFIG(TAG, "  Vane HEAT positioning: wait %lus, move %lus",
                  (unsigned long)(heat_vent_move_delay_ms_ / 1000),
                  (unsigned long)(heat_vent_interval_ms_ / 1000));
  }
  if (cool_vent_move_delay_ms_ && cool_vent_interval_ms_) {
    ESP_LOGCONFIG(TAG, "  Vane COOL positioning: wait %lus, move %lus",
                  (unsigned long)(cool_vent_move_delay_ms_ / 1000),
                  (unsigned long)(cool_vent_interval_ms_ / 1000));
  }
}

}  // namespace furrion_chill_cube
}  // namespace esphome
