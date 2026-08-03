#pragma once

#include "esphome/core/component.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/remote_transmitter/remote_transmitter.h"
#include "esphome/components/remote_base/remote_base.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/button/button.h"
#include "esphome/core/preferences.h"

namespace esphome {
namespace furrion_chill_cube {

class FurrionChillCube : public climate::Climate, public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

  // Configuration setters (called by code generation)
  void set_transmitter(remote_transmitter::RemoteTransmitterComponent *tx) { transmitter_ = tx; }
  void set_inside_temperature_sensor(sensor::Sensor *s) { inside_temp_sensor_ = s; }
  void set_inside_temperature_fahrenheit(bool f) { inside_temp_fahrenheit_ = f; }
  void set_outside_temperature_sensor(sensor::Sensor *s) { outside_temp_sensor_ = s; }
  void set_outside_temperature_fahrenheit(bool f) { outside_temp_fahrenheit_ = f; }
  void set_outside_lockout_temp(float temp_f);
  void set_mode_switch_idle_min(int min);
  void set_mode_switch_event_min(int min);
  void set_mode_switch_temp_offset(float offset_c);
  // Off-dwell between active states (ms). Enforced on EVERY heat<->cool transition —
  // direct user mode change AND natural HEAT_COOL handoff — and on any re-engage from -1.
  void set_mode_switch_off_ms(uint32_t ms) { mode_switch_off_ms_ = ms; }
  void set_use_fahrenheit(bool enable) { use_fahrenheit_ = enable; }

  // Configurable gear tables (gear number → °C CS offset from setpoint anchor + optional
  // commanded fan; fan -1 = unset → falls through to the HA fan-mode entity).
  void add_cool_gear(int gear, int cs_offset, int fan) { set_gear_offset_(false, gear, cs_offset, fan); }
  void add_heat_gear(int gear, int cs_offset, int fan) { set_gear_offset_(true, gear, cs_offset, fan); }
  // Ladder params (spacing S, hysteresis h, pinned start/stop/idle). The modulation
  // rungs are auto-built from S in setup() (build_ladders_); pins stored as-is.
  void set_cool_ladder(float spacing, float hyst, float start, float stop, float idle) {
    cool_spacing_ = spacing; cool_hyst_ = hyst; cool_start_ = start; cool_stop_ = stop; cool_idle_ = idle;
  }
  void set_heat_ladder(float spacing, float hyst, float start, float stop, float idle) {
    heat_spacing_ = spacing; heat_hyst_ = hyst; heat_start_ = start; heat_stop_ = stop; heat_idle_ = idle;
  }
  // Register a transition quirk (path-dependent maneuver). from_gear -1 = OFF→gear clamped start.
  // via_fan -1 = leave fan; escape_up = release early if a higher gear is demanded. duration 0 = global.
  void add_quirk(bool is_heat, int from_gear, int to_gear, int via_offset, int via_fan,
                 bool escape_up, uint32_t duration_ms);
  void set_quirk_duration_ms(uint32_t ms) { quirk_duration_ms_ = ms; }
  void set_cs_transmit_interval_ms(uint32_t ms) { cs_transmit_interval_ms_ = ms; }
  void set_quirk_transmit_interval_ms(uint32_t ms) { quirk_transmit_interval_ms_ = ms; }
  void set_mode_resend_delay_ms(uint32_t ms) { mode_resend_delay_ms_ = ms; }
  void set_gear_step_c(float c) { gear_step_c_ = c; }

  // Timed vane positioning (optional). Per-mode (delay, interval) in ms; 0 = unset.
  // A mode's positioning runs only when BOTH its values are set. See project_vane_control.
  void set_heat_vent_move_delay_ms(uint32_t ms) { heat_vent_move_delay_ms_ = ms; }
  void set_heat_vent_interval_ms(uint32_t ms) { heat_vent_interval_ms_ = ms; }
  void set_cool_vent_move_delay_ms(uint32_t ms) { cool_vent_move_delay_ms_ = ms; }
  void set_cool_vent_interval_ms(uint32_t ms) { cool_vent_interval_ms_ = ms; }
  void set_vane_step_duration_ms(uint32_t ms) { vane_step_duration_ms_ = ms; }

  // Phase 2 adaptive equilibrium-gear controller (cool mode)
  void set_adaptive_enable(bool enable) { adaptive_enable_ = enable; }
  void set_vent_fan_sensor(binary_sensor::BinarySensor *s) { vent_fan_sensor_ = s; }
  void set_fan_feedforward_gears(int g) { fan_feedforward_gears_ = g; }
  // Setpoint-transition preload (fix-setpoint-transition-integral, 2026-08-02). Default 0 = OFF
  // (bit-identical). Fraction of a committed user setpoint change (°C, demand direction) added to
  // the active mode's integral bias, gated on real demand at the new target. (A companion
  // "pulldown windup" option was cut in bug-check — see the note in adaptive_cool_eff_diff_.)
  void set_sp_preload_factor(float f) { sp_preload_factor_ = f; }

  // Diagnostic sensor setters
  void set_heat_gear_sensor(sensor::Sensor *s) { heat_gear_sensor_ = s; }
  void set_cool_gear_sensor(sensor::Sensor *s) { cool_gear_sensor_ = s; }
  void set_compressor_output_sensor(sensor::Sensor *s) { compressor_output_sensor_ = s; }
  void set_comfort_sense_sensor(sensor::Sensor *s) { cs_value_sensor_ = s; }

  // Debug sensor setters
  void set_debug_active_ir_mode_sensor(sensor::Sensor *s) { debug_active_ir_mode_sensor_ = s; }
  void set_debug_last_active_mode_sensor(sensor::Sensor *s) { debug_last_active_mode_sensor_ = s; }
  void set_debug_kick_phase_sensor(sensor::Sensor *s) { debug_kick_phase_sensor_ = s; }
  void set_debug_gear_diff_sensor(sensor::Sensor *s) { debug_gear_diff_sensor_ = s; }
  void set_debug_time_in_gear_sensor(sensor::Sensor *s) { debug_time_in_gear_sensor_ = s; }
  void set_debug_idle_duration_sensor(sensor::Sensor *s) { debug_idle_duration_sensor_ = s; }
  void set_debug_mode_switch_cooldown_sensor(sensor::Sensor *s) { debug_mode_switch_cooldown_sensor_ = s; }
  void set_debug_fan_clamp_remaining_sensor(sensor::Sensor *s) { debug_fan_clamp_remaining_sensor_ = s; }
  void set_debug_heater_locked_out_sensor(sensor::Sensor *s) { debug_heater_locked_out_sensor_ = s; }
  void set_debug_failsafe_active_sensor(sensor::Sensor *s) { debug_failsafe_active_sensor_ = s; }
  void set_debug_boot_ready_sensor(sensor::Sensor *s) { debug_boot_ready_sensor_ = s; }
  void set_debug_adaptive_bias_c_sensor(sensor::Sensor *s) { debug_adaptive_bias_c_sensor_ = s; }
  void set_debug_room_drift_sensor(sensor::Sensor *s) { debug_room_drift_sensor_ = s; }
  void set_debug_fan_feedforward_sensor(sensor::Sensor *s) { debug_fan_feedforward_sensor_ = s; }
  void set_debug_effective_fan_sensor(sensor::Sensor *s) { debug_effective_fan_sensor_ = s; }

  // IR commands (public for button access)
  void send_display_toggle();
  void send_turbo_on();
  void send_turbo_off();
  void send_swing_on();
  void send_swing_off();
  // Manual uniform vane nudge: pulse the physical swing ON, then OFF after
  // vane_step_duration_ms_ (non-blocking). Each press = one fixed-size step.
  void send_vane_step();

  // ── Bench test harness hooks (active only when test_mode is set) ─────────────
  // While test_mode_ is set, loop() is INERT (no gear controller / kickstart / maneuver /
  // heartbeat) and the transmit sensor-availability gates are bypassed — the unit is driven
  // ENTIRELY by these hooks from the YAML sequencer. Deliberately overrides the failover
  // anchor (see project_failover_invariant): this is an operator-present bench test. On
  // set_test_mode(false) the next production gear pass re-anchors the unit's setpoint to the
  // real HA target, so failover is restored on exit.
  void set_test_mode(bool t);
  bool is_test_mode() const { return test_mode_; }   // single source of truth for the sequencer
  // Send one full test frame: mode (0=OFF,1=COOL,2=HEAT), °C setpoint, raw CS byte, fan
  // (0=AUTO,1=LOW,2=MED,3=HIGH). Sets state + transmits the CS→MODE→CS bracket (or OFF).
  void test_frame(int mode, int setpoint_c, int cs, int fan);
  // Re-assert the current CS only (keep-alive tick; no-op when the held mode is OFF).
  void test_resend_cs();
  void test_off();

 protected:
  // Climate interface
  climate::ClimateTraits traits() override;
  void control(const climate::ClimateCall &call) override;

  // IR protocol
  void encode_(remote_base::RemoteTransmitData *data, const uint8_t *msg, uint8_t len, uint8_t repeat);
  bool transmit_mode_command_();  // false = suppressed by the no-valid-setpoint gate (nothing sent)
  void transmit_mode_with_cs_();
  void transmit_cs_update_();
  void transmit_raw_6byte_(const uint8_t *msg);

  // Gear controller
  void run_gear_controller_();
  bool check_failsafe_(uint32_t now, float room);
  void update_outside_lockout_();
  void arbitrate_mode_(float room, bool &do_heat, bool &do_cool);
  bool run_heat_mode_(float room, uint32_t now, bool user_input, bool from_test, float &gear_diff);
  bool run_cool_mode_(float room, uint32_t now, bool user_input, bool from_test, float &gear_diff);
  void run_idle_mode_(uint32_t now);
  uint32_t time_in_gear_(uint32_t now);
  float get_heat_target_();
  float get_cool_target_();
  float get_active_ir_target_();
  void seed_last_tx_target_f_();   // sync last_tx_target_f_ to the active target (boot restore)
  int compute_gear_cs_(bool is_heat, int gear);   // CS for a gear from its configured offset
  int gear_cs_with_clamp_(bool is_heat, int offset);  // SP+offset with the 15-30 group boundary shift
  void set_gear_offset_(bool is_heat, int gear, int cs_offset, int fan);  // codegen: register a gear
  void build_ladders_();                          // build modulation up/dn trips from spacing (setup)
  void compute_cold_start_floors_();              // derive OFF-start floor from the -1 quirks (setup)
  bool gear_in_band_heat_(int gear, float diff);
  bool gear_in_band_cool_(int gear, float diff);

  // Phase 2 adaptive (cool): advance the integral bias with anti-windup, then return
  // the effective diff (real_diff + bias_c + fan feedforward) the cool ladder selects on.
  // Returns real_diff unchanged when adaptive is disabled. time_in_gear gates the
  // conditional-integration anti-windup (freeze positive accumulation behind a hold-blocked
  // upshift). Must be called once per cool pass so the integral advances/decays.
  float adaptive_cool_eff_diff_(float real_diff, uint32_t now, uint32_t time_in_gear);
  // Phase 2 adaptive (heat): mirror of the cool integral with inverted sign (heat demand = -diff).
  // Advances bias_h_ with anti-windup and returns eff_diff (real_diff - bias_h_); more-negative
  // eff selects a higher heat gear. Returns real_diff unchanged when adaptive is disabled. Must be
  // called once per heat pass so the integral advances/decays.
  float adaptive_heat_eff_diff_(float real_diff, uint32_t now, uint32_t time_in_gear);
  float update_room_drift_(uint32_t now);  // push sample + return 3-min windowed slope (°C/min)
  bool vent_fan_on_();

  // "IR override active" — a transition maneuver (which now also runs the OFF→gear clamped start)
  // currently owns the CS/mode/fan, so the normal gear-pass CS writes and OFF→ON mode command must
  // stand down. (Name kept for the many existing call sites.) maneuver_phase_ / ManeuverPhase are
  // declared in the config-members section below; an inline body may reference them (the class is
  // fully parsed before inline bodies are compiled).
  bool kickstart_active_() { return maneuver_phase_ != ManeuverPhase::IDLE; }

  // Unified transition maneuver engine (path-dependent quirk). Absorbs the old clamped kickstart:
  // a quirk with from_gear == -1 runs the OFF→gear start (PRE_CS lead → mode-on with via_fan);
  // from_gear >= 0 is a running-unit maneuver. In all cases: hold via CS (and, if set, via_fan) for
  // the window, re-asserting at the quirk interval, then release + re-evaluate the gear. escape_up
  // releases early if the demanded gear rises above to_gear. See design_gear_engine_v2.
  // Nested QuirkDef is fully defined in the config-members section below; forward-declare it HERE so
  // the pointer parameter / find_quirk_ return type bind to FurrionChillCube::QuirkDef.
  struct QuirkDef;
  void start_maneuver_(const QuirkDef *q, uint32_t now);
  void enter_maneuver_hold_(uint32_t now);   // PRE_CS→HOLD (or immediate HOLD for a running unit)
  void advance_maneuver_(uint32_t now);
  void end_maneuver_(uint32_t now);
  // Look up a configured quirk for a (mode, from_gear→to_gear) transition; nullptr if none.
  const QuirkDef *find_quirk_(bool is_heat, int from_gear, int to_gear);
  climate::ClimateFanMode fan_int_to_mode_(int f);   // 0 AUTO, 1 LOW, 2 MED, 3 HIGH (else AUTO)
  void maybe_apply_gear_fan_(uint32_t now);           // emit a mode frame if the current gear's fan changed

  // Timed vane positioning (open-loop IR homing off the power-on anchor).
  // On an OFF->active start the unit re-homes the vane to a fixed default; we then
  // wait <delay>, pulse the physical swing ON for <interval>, and stop it — landing
  // the vane at a known mode-specific position (heat=down, cool=ceiling). Runtime-only:
  // it sends RAW swing IR and NEVER touches this->swing_mode, so the HA swing switch
  // stays "off" throughout (the homing is invisible to the user). Any user swing toggle,
  // unit-off, or mode change exits it cleanly.
  enum class VentPhase : uint8_t {
    IDLE,
    WAIT_DELAY,   // unit just started; waiting <delay> for the vane to reach its anchor
    MOVING,       // swing pulsed ON; waiting <interval> before stopping at target
  };
  void maybe_start_vent_positioning_(bool is_heat);
  // Self-clocks on millis() — NOT the loop's cached `now`. Unlike kickstart/keepalive
  // (armed inside the gear pass where `now` is valid), the vane is armed from
  // set_active_ir_mode_(), so vent_phase_start_ is a millis() value NEWER than the loop's
  // cached `now`; comparing against the stale `now` underflows the unsigned elapsed and
  // fires the move instantly. Keep it parameter-less so the wrong clock can't be passed.
  void advance_vent_positioning_();
  void abort_vent_positioning_();
  bool vent_positioning_active_() { return vent_phase_ != VentPhase::IDLE; }

  // Force a real OFF + off-dwell on a heat<->cool transition (compressor safety +
  // the vane's known OFF->ON anchor). Stamps off_since_ so the dwell gate then holds
  // the new mode off until mode_switch_off_ms_ elapses.
  void force_off_for_mode_switch_(uint32_t now);

  // Fan mode
  climate::ClimateFanMode get_effective_fan_mode_();

  // Dynamic setpoint
  int compute_setpoint_c_(bool is_heat);
  void update_furrion_setpoint_(bool is_heat);

  // Helpers
  void set_cs_value_(int cs, uint32_t now);
  void update_action_();
  void send_swing_state_();
  void set_active_ir_mode_(climate::ClimateMode mode);
  void publish_debug_state_(float diff);
  void save_gear_pref_();

  // Hardware
  remote_transmitter::RemoteTransmitterComponent *transmitter_{nullptr};
  sensor::Sensor *inside_temp_sensor_{nullptr};
  sensor::Sensor *outside_temp_sensor_{nullptr};

  // Diagnostic output sensors
  sensor::Sensor *heat_gear_sensor_{nullptr};
  sensor::Sensor *cool_gear_sensor_{nullptr};
  sensor::Sensor *compressor_output_sensor_{nullptr};
  sensor::Sensor *cs_value_sensor_{nullptr};

  // Debug sensors (registered when debug: true)
  sensor::Sensor *debug_active_ir_mode_sensor_{nullptr};
  sensor::Sensor *debug_last_active_mode_sensor_{nullptr};
  sensor::Sensor *debug_kick_phase_sensor_{nullptr};
  sensor::Sensor *debug_gear_diff_sensor_{nullptr};
  sensor::Sensor *debug_time_in_gear_sensor_{nullptr};
  sensor::Sensor *debug_idle_duration_sensor_{nullptr};
  sensor::Sensor *debug_mode_switch_cooldown_sensor_{nullptr};
  sensor::Sensor *debug_fan_clamp_remaining_sensor_{nullptr};
  sensor::Sensor *debug_heater_locked_out_sensor_{nullptr};
  sensor::Sensor *debug_failsafe_active_sensor_{nullptr};
  sensor::Sensor *debug_boot_ready_sensor_{nullptr};
  sensor::Sensor *debug_adaptive_bias_c_sensor_{nullptr};
  sensor::Sensor *debug_room_drift_sensor_{nullptr};
  sensor::Sensor *debug_fan_feedforward_sensor_{nullptr};
  sensor::Sensor *debug_effective_fan_sensor_{nullptr};   // last-transmitted fan (0 auto/1 low/2 med/3 high, -1 off)

  // Phase 2 adaptive input
  binary_sensor::BinarySensor *vent_fan_sensor_{nullptr};

  // Configuration
  float outside_lockout_temp_c_{1.67f};   // 35°F default
  bool inside_temp_fahrenheit_{false};
  bool outside_temp_fahrenheit_{false};

  // Gear state
  enum ActiveMode : uint8_t { MODE_NONE = 0, MODE_HEAT = 1, MODE_COOL = 2 };
  int heat_gear_{-1};
  int cool_gear_{-1};
  ActiveMode last_active_mode_{MODE_NONE};
  int current_cs_{22};
  int furrion_setpoint_c_{22}; // Dynamic Furrion setpoint in °C anchor (16-30). Updated
                               // IMMEDIATELY on a target change (even mid-debounce) so
                               // compute_gear_cs_() always anchors to the live target.
  int last_tx_setpoint_c_{22}; // Last °C setpoint actually transmitted. Baseline for sp_changed
                               // detection (furrion_setpoint_c_ tracks live, so can't be it).
  int last_tx_target_f_{0};   // Last °F target byte actually transmitted (F-protocol);
                              // lets update_furrion_setpoint_() catch sub-°C changes
  int last_tx_fan_{-1};       // Last fan enum put on the wire via a mode frame (-1 = none/OFF).
                              // Set inside transmit_mode_command_(); gates the per-gear fan resend.
  climate::ClimateMode active_ir_mode_{climate::CLIMATE_MODE_OFF};

  // Timing (all uint32_t for millis())
  uint32_t boot_time_{0};
  uint32_t last_temp_update_{0};
  uint32_t last_gear_change_{0};
  uint32_t idle_since_{0};
  uint32_t last_cs_heartbeat_{0};
  uint32_t last_gear_run_{0};
  uint32_t off_since_{0};               // timestamp when gear transitioned to -1 (for 1-min off lockout)

  // Mode switch tunables (configurable via YAML; defaults = 10min/20min/1°F/1min)
  uint32_t mode_switch_idle_ms_{600000};    // 10 min at gear 0 before mode switch allowed
  uint32_t mode_switch_event_ms_{1200000};  // 20 min since last fresh start
  float mode_switch_temp_offset_c_{0.556f}; // room must be this far past setpoint (°C delta)
  uint32_t mode_switch_off_ms_{60000};      // 1 min minimum in -1 before fresh start
  uint32_t last_mode_event_at_{0};  // last mode switch or fresh start (time-based lockout)
  uint32_t ha_disconnect_time_{0};
  uint32_t temp_nan_since_{0};

  // Unified transition maneuver (path-dependent quirk) — also runs the OFF→gear clamped start.
  // from_gear == -1: PRE_CS (set via CS ~500ms before mode-on) → HOLD (unit on, hold via_cs and,
  // if set, via_fan; re-assert every quirk_transmit_interval_ms_; release + re-evaluate the gear at
  // duration, or when the demanded gear escapes above to_gear). from_gear >= 0 enters HOLD directly.
  enum class ManeuverPhase : uint8_t { IDLE, PRE_CS, HOLD };
  ManeuverPhase maneuver_phase_{ManeuverPhase::IDLE};
  uint32_t maneuver_start_{0};         // HOLD start (times the duration window)
  uint32_t maneuver_phase_start_{0};   // current phase start (PRE_CS 500ms lead)
  uint32_t maneuver_last_tx_{0};       // last via-CS re-assert (for quirk_transmit_interval_ms_)
  int maneuver_via_cs_{0};             // CS held during the maneuver
  int8_t maneuver_via_fan_{-1};        // fan held during HOLD (-1 = leave fan as-is)
  uint32_t maneuver_duration_ms_{0};   // this maneuver's HOLD window
  bool maneuver_is_heat_{false};
  int8_t maneuver_from_gear_{0};       // -1 = OFF→gear entry (PRE_CS + mode-on path)
  int8_t maneuver_to_gear_{0};
  bool maneuver_escape_up_{false};     // release early if the demanded gear > to_gear

  // Timed vane positioning config (ms; 0 = unset → feature off for that mode)
  uint32_t heat_vent_move_delay_ms_{0};
  uint32_t heat_vent_interval_ms_{0};
  uint32_t cool_vent_move_delay_ms_{0};
  uint32_t cool_vent_interval_ms_{0};
  // Timed vane positioning runtime state
  VentPhase vent_phase_{VentPhase::IDLE};
  uint32_t vent_phase_start_{0};         // when the current phase began
  uint32_t vent_active_delay_ms_{0};     // delay for the in-progress run (heat or cool)
  uint32_t vent_active_interval_ms_{0};  // interval for the in-progress run

  // Manual vane step (uniform nudge). Self-clocks on millis() (started from a button
  // press, outside the loop-now context — see reference_furrion_millis_now_footgun).
  uint32_t vane_step_duration_ms_{500};  // swing-ON hold per press (YAML, default 0.5s)
  bool vane_step_active_{false};
  uint32_t vane_step_start_{0};

  // Target encoding (F vs C) — configurable via YAML, default Fahrenheit.
  // Selects whether transmit_mode_command_() encodes the target temperature
  // byte using the Toshiba RAC-PT1411HWRU F-protocol (60–86 °F) or
  // C-protocol (16–30 °C). Does NOT affect gear-CS math, which still
  // anchors on furrion_setpoint_c_ (the °C-rounded HA target).
  bool use_fahrenheit_{true};

  // ── Configurable gear ladders + transition quirks (populated by codegen) ─────
  // Gear COUNT is dynamic (v2, 2026-07-18): the YAML gear list drives cool_max_gear_/heat_max_gear_
  // and the run_*_mode_ selection loops + ladder auto-derive from it. MAX_GEARS is a compile ceiling
  // only. Each gear also carries an optional commanded fan; each quirk an optional via_fan + escape_up
  // and may start from OFF (from_gear -1). Defaults below reproduce the shipped 2026-07-08 3-gear
  // ladder bit-identically. See design_gear_engine_v2.
  static constexpr int MAX_GEARS = 8;    // compile ceiling; actual count = *_max_gear_ (indices 0..7)
  static constexpr int MAX_QUIRKS = 16;
  struct QuirkDef {
    bool is_heat;
    int8_t from_gear;     // -1 = OFF, 0 = idle, 1.. = active
    int8_t to_gear;
    int8_t via_offset;    // CS offset (°C) from setpoint anchor, held during the maneuver
    int8_t via_fan;       // fan held during the maneuver: -1 unset, 0 AUTO, 1 LOW, 2 MED, 3 HIGH
    bool escape_up;       // release early if the demanded gear rises above to_gear
    uint32_t duration_ms; // 0 = use quirk_duration_ms_
  };
  int cool_gear_offset_[MAX_GEARS] = {-5, -2, 0, 3, 0, 0, 0, 0};    // idle, LOW, MED, MAX (defaults)
  int heat_gear_offset_[MAX_GEARS] = {5, 1, 0, -1, 0, 0, 0, 0};     // idle, g1, g2, g3
  int cool_gear_fan_[MAX_GEARS] = {-1, -1, -1, -1, -1, -1, -1, -1}; // per-gear commanded fan (-1 = HA fan)
  int heat_gear_fan_[MAX_GEARS] = {-1, -1, -1, -1, -1, -1, -1, -1};
  int cool_max_gear_{3};
  int heat_max_gear_{3};
  int cool_cold_start_floor_{1};   // lowest gear pickable from OFF; derived from OFF quirks in setup()
  int heat_cold_start_floor_{1};
  QuirkDef quirks_[MAX_QUIRKS];
  int quirk_count_{0};
  // Ladder params (defaults = shipped values). Modulation rungs auto-built in build_ladders_().
  float cool_spacing_{0.55f}, cool_hyst_{0.0f};
  float cool_start_{0.35f}, cool_stop_{0.15f}, cool_idle_{-0.30f};
  float heat_spacing_{0.55f}, heat_hyst_{0.0f};
  float heat_start_{-0.35f}, heat_stop_{-0.15f}, heat_idle_{0.30f};
  // Modulation trips built from spacing (index n = boundary between gear n and n+1)
  float cool_up_[MAX_GEARS] = {0};     // cool_up_[n]: upshift n→n+1  (= +n·S)
  float cool_dn_[MAX_GEARS] = {0};     // cool_dn_[n]: downshift n+1→n (= +n·S − h)
  float heat_up_[MAX_GEARS] = {0};     // heat_up_[n]: upshift n→n+1  (= −n·S)
  float heat_dn_[MAX_GEARS] = {0};     // heat_dn_[n]: downshift n+1→n (= −(n·S − h))
  // CS transmit cadence + quirk timing (all YAML-configurable)
  uint32_t cs_transmit_interval_ms_{10000};    // normal heartbeat (was fixed 30s)
  uint32_t quirk_transmit_interval_ms_{5000};  // denser re-assert during a maneuver
  uint32_t quirk_duration_ms_{60000};          // default maneuver hold (per-quirk override)
  float gear_step_c_{0.25f};                   // fan feedforward: °C eff_diff per fan-gear

  // One-shot mode-frame reinforcement: every mode frame re-sends itself once after this delay
  // (0 = disabled, the default — preserves bit-identical default IR traffic). Mode/fan frames
  // have no heartbeat, unlike CS frames; a single missed frame on a fan-only gear shift sticks
  // the unit at the old operating point (2026-07-27 overcool).
  uint32_t mode_resend_delay_ms_{0};
  uint32_t mode_resend_armed_at_{0};   // millis() stamp of the arming frame (self-clocked timer)
  bool mode_resend_pending_{false};
  bool mode_resending_{false};         // reinforcement in flight — suppresses re-arm in transmit_mode_command_

  // Flags
  bool boot_ready_{false};
  bool failsafe_active_{false};
  bool test_mode_{false};   // bench test harness: loop() inert, unit driven only by test_* hooks
  int test_fan_{-1};        // fan override for test frames (-1 = none; 0=AUTO,1=LOW,2=MED,3=HIGH)
  bool user_changed_{false};
  bool resume_from_test_{false};  // set on test-exit → gear re-pick uses eff_diff (bias-aware), not real diff
  bool temp_dirty_{false};
  bool heater_locked_out_{false};
  // Setpoint debounce (see SETPOINT_SETTLE_MS): a temp change arms this; loop() commits it
  // (sets user_changed_) once SETPOINT_SETTLE_MS of steady state has elapsed.
  bool setpoint_pending_{false};
  uint32_t setpoint_pending_since_{0};

  // Phase 2 adaptive equilibrium-gear controller (cool mode) — see PHASE2_ADAPTIVE_DESIGN.md
  bool adaptive_enable_{false};          // master switch (set true via YAML on the phase2 branch)
  int fan_feedforward_gears_{1};         // gear-equivalents of feedforward while vent fan runs
  float bias_c_{0.0f};                   // integral bias (°C) added to the diff the cool ladder sees
  uint32_t adaptive_last_advance_{0};    // last integral advance (for dt)
  uint32_t vent_fan_changed_at_{0};      // last vent-fan edge (for the integral freeze window)
  float room_drift_cpm_{NAN};            // inside dT/dt (°C/min): 3-min windowed slope — observability + upshift gate
  float cool_eff_up_diff_{NAN};          // eff_diff for UPSHIFT decisions (bias gated out while not warming)
  // Phase 2 adaptive (heat mode) — mirror of the cool state above. bias_h_ is a SIGNED heat-demand
  // integral (+ = persistently cold = push toward a higher heat gear). Only one of bias_c_/bias_h_ is
  // ever live at a time (the inactive mode's bias is zeroed each pass), so they never interfere.
  float bias_h_{0.0f};                   // integral bias (°C): eff_diff = real_diff - bias_h_ for heat
  uint32_t heat_adaptive_last_advance_{0}; // last heat integral advance (for dt) — separate from cool's
  float heat_eff_up_diff_{NAN};          // eff_diff for heat UPSHIFT decisions (bias gated out while not cooling)
  // Setpoint-transition preload (fix-setpoint-transition-integral). last_committed_* track the
  // last COMMITTED (post-debounce) target seen by the active mode's pass; NAN = no baseline (boot,
  // mode re-entry, failsafe, test session) → next pass records only, never preloads. Not persisted
  // to NVS (a reboot just re-records; the preloaded bias itself rides the existing bias save).
  float sp_preload_factor_{0.0f};        // 0 = preload disabled (default; bit-identical)
  float last_committed_cool_target_c_{NAN};
  float last_committed_heat_target_c_{NAN};
  // Room-drift estimator: ring buffer of recent (timestamp ms, inside °C) samples for the
  // trailing-window slope (see DRIFT_WINDOW_MS in the .cpp). Sized to hold ~6 min at normal cadence.
  static constexpr uint8_t DRIFT_BUF_N = 48;
  uint32_t drift_buf_at_[DRIFT_BUF_N] = {0};
  float    drift_buf_temp_[DRIFT_BUF_N] = {0};
  uint8_t  drift_buf_head_{0};           // next write slot
  uint8_t  drift_buf_count_{0};          // valid samples currently in the ring

  // Cached temperatures (Celsius)
  float inside_temp_c_{NAN};
  float outside_temp_c_{NAN};

  // Persisted mode (survives reboot)
  ESPPreferenceObject mode_pref_;

  // Persisted gear + adaptive bias (restored only on a WARM reset — ESP-only reboot
  // such as OTA/crash, where the Furrion kept power and is still running the last
  // commanded gear CS. A cold boot means the unit power-cycled too and resumed at
  // the user target, so the pre-outage gear is NOT the unit's state — gear 0 is.)
  struct GearPrefData {
    int8_t gear;    // gear of the active IR mode at save time (-1..5)
    float bias;     // adaptive integral bias of the ACTIVE mode (bias_c_ if cool, bias_h_ if heat;
                    // quantized before save). One field: only one mode is active at reboot. Field
                    // order/type unchanged from the old `bias_c` → NVS layout stable, no re-init.
  };
  ESPPreferenceObject gear_pref_;
  int8_t last_saved_gear_{-128};   // -128 = nothing saved this boot (forces first save)
  float last_saved_bias_c_{0.0f};  // last saved bias value (any mode), for the no-op write guard
};

// Button sub-entities
class DisplayToggleButton : public button::Button, public Parented<FurrionChillCube> {
 protected:
  void press_action() override { this->parent_->send_display_toggle(); }
};

class TurboOnButton : public button::Button, public Parented<FurrionChillCube> {
 protected:
  void press_action() override { this->parent_->send_turbo_on(); }
};

class TurboOffButton : public button::Button, public Parented<FurrionChillCube> {
 protected:
  void press_action() override { this->parent_->send_turbo_off(); }
};

class SwingOnButton : public button::Button, public Parented<FurrionChillCube> {
 protected:
  void press_action() override { this->parent_->send_swing_on(); }
};

class SwingOffButton : public button::Button, public Parented<FurrionChillCube> {
 protected:
  void press_action() override { this->parent_->send_swing_off(); }
};

class VaneStepButton : public button::Button, public Parented<FurrionChillCube> {
 protected:
  void press_action() override { this->parent_->send_vane_step(); }
};

}  // namespace furrion_chill_cube
}  // namespace esphome
