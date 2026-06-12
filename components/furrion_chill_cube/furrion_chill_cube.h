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
  void set_keepalive_enable(bool enable) { keepalive_enable_ = enable; }
  void set_use_fahrenheit(bool enable) { use_fahrenheit_ = enable; }

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

  // IR commands (public for button access)
  void send_display_toggle();
  void send_turbo_on();
  void send_turbo_off();
  void send_swing_on();
  void send_swing_off();
  // Manual uniform vane nudge: pulse the physical swing ON, then OFF after
  // vane_step_duration_ms_ (non-blocking). Each press = one fixed-size step.
  void send_vane_step();

 protected:
  // Climate interface
  climate::ClimateTraits traits() override;
  void control(const climate::ClimateCall &call) override;

  // IR protocol
  void encode_(remote_base::RemoteTransmitData *data, const uint8_t *msg, uint8_t len, uint8_t repeat);
  void transmit_mode_command_();
  void transmit_mode_with_cs_();
  void transmit_cs_update_();
  void transmit_raw_6byte_(const uint8_t *msg);

  // Gear controller
  void run_gear_controller_();
  bool check_failsafe_(uint32_t now, float room);
  void update_outside_lockout_();
  void arbitrate_mode_(float room, bool &do_heat, bool &do_cool);
  bool run_heat_mode_(float room, uint32_t now, bool user_input, float &gear_diff);
  bool run_cool_mode_(float room, uint32_t now, bool user_input, float &gear_diff);
  void run_idle_mode_(uint32_t now);
  uint32_t time_in_gear_(uint32_t now);
  float get_heat_target_();
  float get_cool_target_();
  float get_active_ir_target_();
  void seed_last_tx_target_f_();   // sync last_tx_target_f_ to the active target (boot restore)
  int compute_gear_cs_(bool is_heat, int gear);
  bool gear_in_band_heat_(int gear, float diff);
  bool gear_in_band_cool_(int gear, float diff);

  // Phase 2 adaptive (cool): advance the integral bias with anti-windup, then return
  // the effective diff (real_diff + bias_c + fan feedforward) the cool ladder selects on.
  // Returns real_diff unchanged when adaptive is disabled. time_in_gear gates the
  // conditional-integration anti-windup (freeze positive accumulation behind a hold-blocked
  // upshift). Must be called once per cool pass so the integral advances/decays.
  float adaptive_cool_eff_diff_(float real_diff, uint32_t now, uint32_t time_in_gear);
  bool vent_fan_on_();

  // Kickstart system
  // Clamped kickstart: OFF→low gear, fan=LOW for 5:30, gear controller runs but CS overridden
  // Quick kickstart: borderline restart, CS override for a per-call hold window
  //   (QUICK_KICK_HOLD_MS for OFF→gear-3, IDLE_KICK_HOLD_MS for idle→gear-1/2), no fan clamp
  enum class ClampPhase : uint8_t {
    IDLE,
    PRE_CS,     // 500ms: kickstart CS pre-set before mode-on
    CLAMPED,    // 5:30: fan=LOW, kickstart CS retransmitted, gear controller monitored
  };
  void start_clamped_kickstart_(bool is_heat, uint32_t now);
  void start_quick_kickstart_(bool is_heat, int kickstart_cs, uint32_t now, uint32_t hold_ms);
  void advance_kickstart_(uint32_t now);
  void end_kickstart_(uint32_t now);
  bool kickstart_active_() { return clamp_phase_ != ClampPhase::IDLE || quick_kick_active_; }

  // Keep-alive pulse (sustain compressor at low CS gears)
  enum class KeepAlivePhase : uint8_t {
    IDLE,
    STEP1,          // Anchor CS sent, waiting 5s
    STEP2,          // Over-anchor CS sent, waiting 5s
    STEP_RESTORE1,  // Target CS sent (1/2), waiting 5s
    // Final step (restore 2/2) transitions directly to IDLE
  };
  void start_keepalive_(bool is_heat, uint32_t now);
  void advance_keepalive_(uint32_t now);
  void abort_keepalive_();

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
  int furrion_setpoint_c_{22}; // Dynamic Furrion setpoint in °C (16-30)
  int last_tx_target_f_{0};   // Last °F target byte actually transmitted (F-protocol);
                              // lets update_furrion_setpoint_() catch sub-°C changes
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

  // Clamped kickstart (OFF→low gear: fan=LOW + CS override for 5:30)
  ClampPhase clamp_phase_{ClampPhase::IDLE};
  uint32_t clamp_start_{0};           // when CLAMPED phase began (for 5:30 timeout)
  uint32_t clamp_phase_start_{0};     // when current phase began (for PRE_CS 500ms)
  int clamp_kickstart_cs_{0};         // CS to hold during clamp
  bool clamp_is_heat_{false};         // which mode the clamp is for

  // Quick kickstart (borderline restart: CS override for quick_kick_hold_ms_, no fan clamp)
  bool quick_kick_active_{false};
  uint32_t quick_kick_start_{0};
  int quick_kick_cs_{0};              // kickstart CS to hold
  uint32_t quick_kick_hold_ms_{10000}; // hold window; set per-call by start_quick_kickstart_
  bool quick_kick_is_heat_{false};
  bool quick_kick_reinforced_{false}; // one-shot guard for 5s reinforce

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

  // Keep-alive
  bool keepalive_enable_{true};      // configurable via YAML; disables pulse trigger
  KeepAlivePhase keepalive_phase_{KeepAlivePhase::IDLE};
  uint32_t keepalive_phase_start_{0};
  uint32_t keepalive_last_{0};       // last completion (or gear entered eligible range)
  int keepalive_restore_cs_{22};     // CS to restore after pulse
  int keepalive_step2_cs_{22};       // CS for step 2 (anchor±1 depending on mode)

  // Flags
  bool boot_ready_{false};
  bool failsafe_active_{false};
  bool user_changed_{false};
  bool temp_dirty_{false};
  bool heater_locked_out_{false};

  // Phase 2 adaptive equilibrium-gear controller (cool mode) — see PHASE2_ADAPTIVE_DESIGN.md
  bool adaptive_enable_{false};          // master switch (set true via YAML on the phase2 branch)
  int fan_feedforward_gears_{1};         // gear-equivalents of feedforward while vent fan runs
  float bias_c_{0.0f};                   // integral bias (°C) added to the diff the cool ladder sees
  uint32_t adaptive_last_advance_{0};    // last integral advance (for dt)
  uint32_t vent_fan_changed_at_{0};      // last vent-fan edge (for the integral freeze window)
  float room_drift_cpm_{NAN};            // EMA of inside dT/dt (°C/min) — observability + future use
  float prev_inside_temp_c_{NAN};        // previous inside temp for drift computation
  uint32_t prev_inside_temp_at_{0};      // timestamp of prev_inside_temp_c_

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
    float bias_c;   // adaptive integral bias (cool only; quantized before save)
  };
  ESPPreferenceObject gear_pref_;
  int8_t last_saved_gear_{-128};   // -128 = nothing saved this boot (forces first save)
  float last_saved_bias_c_{0.0f};
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
