#pragma once

#include "esphome/core/component.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/remote_transmitter/remote_transmitter.h"
#include "esphome/components/remote_base/remote_base.h"
#include "esphome/components/sensor/sensor.h"
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

  // IR commands (public for button access)
  void send_display_toggle();
  void send_turbo_on();
  void send_turbo_off();
  void send_swing_on();
  void send_swing_off();

 protected:
  // Climate interface
  climate::ClimateTraits traits() override;
  void control(const climate::ClimateCall &call) override;

  // IR protocol
  void encode_(remote_base::RemoteTransmitData *data, const uint8_t *msg, uint8_t len, uint8_t repeat);
  void transmit_mode_command_();
  void transmit_cs_update_(bool send_data);
  void transmit_raw_6byte_(const uint8_t *msg);

  // Gear controller
  void run_gear_controller_();
  float get_heat_target_();
  float get_cool_target_();
  int compute_gear_cs_(bool is_heat, int gear);

  // Kickstart system
  // Clamped kickstart: OFF→low gear, fan=LOW for 5:30, gear controller runs but CS overridden
  // Quick kickstart: borderline restart, CS override for 10s, no fan clamp
  enum class ClampPhase : uint8_t {
    IDLE,
    PRE_CS,     // 500ms: kickstart CS pre-set before mode-on
    CLAMPED,    // 5:30: fan=LOW, kickstart CS retransmitted, gear controller monitored
  };
  void start_clamped_kickstart_(bool is_heat, uint32_t now);
  void start_quick_kickstart_(bool is_heat, int kickstart_cs, uint32_t now);
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

  // Fan mode
  climate::ClimateFanMode get_effective_fan_mode_();

  // Dynamic setpoint
  int compute_setpoint_c_(bool is_heat);
  void update_furrion_setpoint_(bool is_heat);

  // Helpers
  void set_cs_value_(int cs);
  void update_action_();
  void send_swing_state_();
  void set_active_ir_mode_(climate::ClimateMode mode);
  void publish_debug_state_(float diff);

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

  // Configuration
  float outside_lockout_temp_c_{1.67f};   // 35°F default
  bool inside_temp_fahrenheit_{false};
  bool outside_temp_fahrenheit_{false};

  // Gear state
  int heat_gear_{-1};
  int cool_gear_{-1};
  int last_active_mode_{0};    // 0=none, 1=heat, 2=cool
  int current_cs_{22};
  int furrion_setpoint_c_{22}; // Dynamic Furrion setpoint in °C (16-30)
  climate::ClimateMode active_ir_mode_{climate::CLIMATE_MODE_OFF};

  // Timing (all uint32_t for millis())
  uint32_t last_temp_update_{0};
  uint32_t last_gear_change_{0};
  uint32_t idle_since_{0};
  uint32_t last_cs_heartbeat_{0};
  uint32_t last_gear_run_{0};
  uint32_t mode_switch_off_at_{0};
  uint32_t last_mode_event_at_{0};  // last mode switch or fresh start (time-based lockout)
  uint32_t ha_disconnect_time_{0};
  uint32_t temp_nan_since_{0};

  // Clamped kickstart (OFF→low gear: fan=LOW + CS override for 5:30)
  ClampPhase clamp_phase_{ClampPhase::IDLE};
  uint32_t clamp_start_{0};           // when CLAMPED phase began (for 5:30 timeout)
  uint32_t clamp_phase_start_{0};     // when current phase began (for PRE_CS 500ms)
  int clamp_kickstart_cs_{0};         // CS to hold during clamp
  bool clamp_is_heat_{false};         // which mode the clamp is for

  // Quick kickstart (borderline restart: CS override for 10s, no fan clamp)
  bool quick_kick_active_{false};
  uint32_t quick_kick_start_{0};
  int quick_kick_cs_{0};              // kickstart CS to hold for 10s
  bool quick_kick_is_heat_{false};
  bool quick_kick_reinforced_{false}; // one-shot guard for 5s reinforce

  // Keep-alive
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

  // Cached temperatures (Celsius)
  float inside_temp_c_{NAN};
  float outside_temp_c_{NAN};

  // Persisted mode (survives reboot)
  ESPPreferenceObject mode_pref_;
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

}  // namespace furrion_chill_cube
}  // namespace esphome
