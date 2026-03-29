#include "furrion_chill_cube.h"
#include "esphome/core/log.h"

#ifdef USE_API
#include "esphome/components/api/api_server.h"
#endif

namespace esphome {
namespace furrion_chill_cube {

static const char *const TAG = "furrion_chill_cube";

// ============================================================
// IR Protocol Constants — RAC-PT1411HWRU-F (Fahrenheit model)
// ============================================================

static const uint16_t IR_HEADER_MARK = 4380;
static const uint16_t IR_HEADER_SPACE = 4370;
static const uint16_t IR_BIT_MARK = 540;
static const uint16_t IR_ZERO_SPACE = 540;
static const uint16_t IR_ONE_SPACE = 1620;
static const uint16_t IR_GAP_SPACE = 5480;
static const uint16_t IR_PACKET_SPACE = 10500;
static const uint16_t IR_CARRIER_FREQ = 38000;

// Fahrenheit temperature table (index = temp_F - 60, range 60-86°F)
static const uint8_t TEMP_F_TABLE[] = {
    0x10, 0x30, 0x00, 0x20, 0x01, 0x21, 0x03, 0x23, 0x02,
    0x22, 0x06, 0x26, 0x07, 0x05, 0x25, 0x04, 0x24, 0x0C,
    0x2C, 0x0D, 0x2D, 0x09, 0x08, 0x28, 0x0A, 0x2A, 0x0B};

// Gear controller constants
static const int HEAT_ANCHOR = 20;   // 20°C = 68°F
static const int COOL_ANCHOR = 25;   // 25°C = 77°F
static const int HEAT_ANCHOR_F = 68;
static const int COOL_ANCHOR_F = 77;

// Per-gear upshift hold times (ms) — index = target gear
//                        0      1       2       3        4        5
static const uint32_t HOLD_MS[] = {0, 180000, 180000, 300000, 300000, 600000};
static const uint32_t IDLE_BEFORE_OFF_MS = 900000;  // 15 min
static const uint32_t FAN_CLAMP_DURATION_MS = 330000;  // 5 min + 30s buffer
static const uint32_t CS_HEARTBEAT_MS = 30000;  // 30s
static const uint32_t GEAR_INTERVAL_MS = 60000;  // 60s fallback

// Heating deadbands (diff = room - target, negative = cold)
static const float H_UP_01 = -0.3f;   // 0->1: room 0.5F below target
static const float H_UP_12 = -0.8f;   // 1->2
static const float H_UP_23 = -1.5f;   // 2->3
static const float H_DN_32 = -0.8f;   // 3->2
static const float H_DN_21 = -0.3f;   // 2->1
static const float H_DN_10 =  0.3f;   // 1->0: room 0.5F above target
static const float H_IDLE  =  0.3f;   // 0->-1 threshold

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

// Mode-switching hysteresis
static const float MODE_SWITCH_EXTRA = 1.11f;  // ~2.0F

// ============================================================
// Configuration Setters
// ============================================================

void FurrionChillCube::set_heat_cool_gap(float gap_f) {
  // Convert Fahrenheit delta to Celsius delta
  heat_cool_gap_c_ = gap_f * (5.0f / 9.0f);
}

void FurrionChillCube::set_outside_lockout_temp(float temp_f) {
  // Convert Fahrenheit to Celsius
  outside_lockout_temp_c_ = (temp_f - 32.0f) * (5.0f / 9.0f);
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
  auto transmit = this->transmitter_->transmit();
  auto *data = transmit.get_data();

  uint8_t message[12] = {0};

  // Packet 1 header
  message[0] = 0xB2;
  message[1] = ~message[0];

  // Temperature code (always use fixed anchor for this model)
  uint8_t temp_code;
  if (active_ir_mode_ == climate::CLIMATE_MODE_OFF) {
    temp_code = 0x0E;  // OFF special value
  } else {
    int temp_f = (active_ir_mode_ == climate::CLIMATE_MODE_HEAT) ? HEAT_ANCHOR_F : COOL_ANCHOR_F;
    temp_code = TEMP_F_TABLE[temp_f - 60];
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
    message[9] |= 0x01;  // FAH flag (always Fahrenheit model)
    message[10] = 0x00;
    message[11] = 0;
    for (int i = 6; i <= 10; i++) message[11] += message[i];
  }

  // Encode packet 1 (repeated once = sent twice)
  this->encode_(data, &message[0], 6, 1);

  // Encode packet 2 (sent once) if not OFF
  if (message[6] != 0) {
    this->encode_(data, &message[6], 6, 0);
  }

  transmit.perform();

  ESP_LOGD(TAG, "IR mode=%d fan=%d swing=%d",
           (int)active_ir_mode_, (int)fan, (int)this->swing_mode);

  // Follow-up CS update (enabled flag only, no data)
  if (active_ir_mode_ != climate::CLIMATE_MODE_OFF && !failsafe_active_) {
    this->transmit_cs_update_(false);
  }
}

void FurrionChillCube::transmit_cs_update_(bool send_data) {
  if (active_ir_mode_ == climate::CLIMATE_MODE_OFF) return;

  auto transmit = this->transmitter_->transmit();
  auto *data = transmit.get_data();

  uint8_t message[6] = {0};
  message[0] = 0xBA;
  message[1] = ~message[0];  // 0x45

  // CS temperature (clamped 0-30°C)
  message[2] = (uint8_t)std::max(0, std::min(30, current_cs_));

  // Feature flags
  if (send_data) {
    message[2] |= 0xC0;  // CS_ENABLED + CS_DATA
  } else {
    message[2] |= 0x40;  // CS_ENABLED only
  }
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

void FurrionChillCube::transmit_raw_6byte_(const uint8_t *msg) {
  auto transmit = this->transmitter_->transmit();
  auto *data = transmit.get_data();
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

  // Register temperature sensor callbacks
  if (inside_temp_sensor_) {
    inside_temp_sensor_->add_on_state_callback([this](float value) {
      if (isnan(value)) {
        inside_temp_c_ = NAN;
      } else {
        inside_temp_c_ = inside_temp_fahrenheit_ ? (value - 32.0f) * (5.0f / 9.0f) : value;
      }
      this->current_temperature = inside_temp_c_;
      this->last_temp_update_ = millis();
      this->temp_dirty_ = true;
      this->publish_state();
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

  // Initialize diagnostic sensors
  if (heat_gear_sensor_) heat_gear_sensor_->publish_state(-1);
  if (cool_gear_sensor_) cool_gear_sensor_->publish_state(-1);
  if (compressor_output_sensor_) compressor_output_sensor_->publish_state(0.0f);
}

void FurrionChillCube::loop() {
  uint32_t now = millis();

  // 1. Advance kickstart state machine
  if (kick_phase_ != KickPhase::IDLE) {
    advance_kickstart_();
  }

  // 2. Run gear controller if triggered
  bool should_run = false;
  if (temp_dirty_) { should_run = true; temp_dirty_ = false; }
  if (last_gear_run_ == 0 || (now - last_gear_run_) >= GEAR_INTERVAL_MS) should_run = true;
  if (user_changed_) should_run = true;

  if (should_run && kick_phase_ == KickPhase::IDLE) {
    run_gear_controller_();
    last_gear_run_ = now;
  }

  // 3. Post-kickstart CS reinforcement (re-transmit gear controller's CS at 5s intervals)
  if (cs_reinforce_count_ > 0 && (now - cs_reinforce_at_) >= 5000) {
    transmit_cs_update_(true);
    last_cs_heartbeat_ = now;
    cs_reinforce_count_--;
    cs_reinforce_at_ = now;
    ESP_LOGI(TAG, "CS reinforce cs=%d (%d left)", current_cs_, cs_reinforce_count_);
  }

  // 4. CS heartbeat every 30s
  if (boot_ready_ && !failsafe_active_ &&
      active_ir_mode_ != climate::CLIMATE_MODE_OFF &&
      (now - last_cs_heartbeat_) >= CS_HEARTBEAT_MS) {
    transmit_cs_update_(true);
    last_cs_heartbeat_ = now;
  }

  // 5. Fan clamp management (release clamp when expired)
  if (fan_clamp_start_ > 0 && (now - fan_clamp_start_) >= FAN_CLAMP_DURATION_MS) {
    fan_clamp_start_ = 0;
    // Re-send mode with AUTO fan if unit is on
    if (active_ir_mode_ != climate::CLIMATE_MODE_OFF && kick_phase_ == KickPhase::IDLE) {
      transmit_mode_command_();
      mode_resend_at_ = now;
      ESP_LOGI(TAG, "Fan clamp expired — fan → AUTO (re-send in 5s)");
    }
  }

  // 6. Mode command re-send (one-shot, 5s after fan recovery)
  if (mode_resend_at_ > 0 && (now - mode_resend_at_) >= 5000) {
    mode_resend_at_ = 0;
    if (active_ir_mode_ != climate::CLIMATE_MODE_OFF && kick_phase_ == KickPhase::IDLE) {
      transmit_mode_command_();
      ESP_LOGI(TAG, "Mode re-send (fan recovery)");
    }
  }
}

climate::ClimateTraits FurrionChillCube::traits() {
  auto traits = climate::ClimateTraits();
  traits.set_supports_current_temperature(true);
  traits.set_supports_two_point_target_temperature(true);
  traits.set_supports_action(true);
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
  if (call.get_mode().has_value()) {
    auto new_mode = *call.get_mode();
    auto old_mode = this->mode;

    // Handle setpoint transitions between single/dual modes
    if (new_mode == climate::CLIMATE_MODE_HEAT_COOL && old_mode != climate::CLIMATE_MODE_HEAT_COOL) {
      if (old_mode == climate::CLIMATE_MODE_HEAT && !isnan(this->target_temperature)) {
        this->target_temperature_low = this->target_temperature;
        this->target_temperature_high = this->target_temperature + heat_cool_gap_c_;
      } else if (old_mode == climate::CLIMATE_MODE_COOL && !isnan(this->target_temperature)) {
        this->target_temperature_high = this->target_temperature;
        this->target_temperature_low = this->target_temperature - heat_cool_gap_c_;
      } else {
        if (isnan(this->target_temperature_low)) this->target_temperature_low = 20.0f;
        if (isnan(this->target_temperature_high)) this->target_temperature_high = 25.0f;
      }
    } else if (new_mode == climate::CLIMATE_MODE_HEAT && old_mode == climate::CLIMATE_MODE_HEAT_COOL) {
      this->target_temperature = this->target_temperature_low;
    } else if (new_mode == climate::CLIMATE_MODE_COOL && old_mode == climate::CLIMATE_MODE_HEAT_COOL) {
      this->target_temperature = this->target_temperature_high;
    }

    this->mode = new_mode;

    // Abort active kickstart on mode change — user override takes priority
    if (kick_phase_ != KickPhase::IDLE) {
      ESP_LOGI(TAG, "Kickstart aborted — user mode change to %d", (int)new_mode);
      kick_phase_ = KickPhase::IDLE;
      mode_resend_at_ = 0;
      fan_clamp_start_ = 0;
      cs_reinforce_count_ = 0;
    }
  }

  // Target temperature changes — abort kickstart for immediate gear re-evaluation
  bool temp_changed = false;
  if (call.get_target_temperature().has_value()) {
    this->target_temperature = *call.get_target_temperature();
    temp_changed = true;
  }
  if (call.get_target_temperature_low().has_value()) {
    this->target_temperature_low = *call.get_target_temperature_low();
    if (this->target_temperature_high - this->target_temperature_low < heat_cool_gap_c_) {
      this->target_temperature_high = this->target_temperature_low + heat_cool_gap_c_;
    }
    temp_changed = true;
  }
  if (call.get_target_temperature_high().has_value()) {
    this->target_temperature_high = *call.get_target_temperature_high();
    if (this->target_temperature_high - this->target_temperature_low < heat_cool_gap_c_) {
      this->target_temperature_low = this->target_temperature_high - heat_cool_gap_c_;
    }
    temp_changed = true;
  }
  if (temp_changed && kick_phase_ != KickPhase::IDLE) {
    ESP_LOGI(TAG, "Kickstart aborted — user target temp change");
    kick_phase_ = KickPhase::IDLE;
    mode_resend_at_ = 0;
    cs_reinforce_count_ = 0;
  }

  // Fan mode change — works during kickstart, explicit non-AUTO terminates clamp
  if (call.get_fan_mode().has_value()) {
    auto new_fan = *call.get_fan_mode();
    this->fan_mode = new_fan;
    // Explicit non-AUTO fan selection overrides the 5-min LOW clamp
    if (new_fan != climate::CLIMATE_FAN_AUTO && fan_clamp_start_ > 0) {
      fan_clamp_start_ = 0;
      ESP_LOGI(TAG, "Fan clamp terminated — user set fan=%d", (int)new_fan);
    }
    if (active_ir_mode_ != climate::CLIMATE_MODE_OFF) {
      transmit_mode_command_();
      ESP_LOGI(TAG, "User fan change → %d, mode command sent", (int)new_fan);
    }
  }

  // Swing mode change — standalone swing frame, works during kickstart
  if (call.get_swing_mode().has_value()) {
    this->swing_mode = *call.get_swing_mode();
    send_swing_state_();
    ESP_LOGI(TAG, "User swing change → %d", (int)*call.get_swing_mode());
  }

  this->user_changed_ = true;
  this->publish_state();
}

// ============================================================
// Helpers
// ============================================================

float FurrionChillCube::get_heat_target_() {
  if (this->mode == climate::CLIMATE_MODE_HEAT_COOL)
    return this->target_temperature_low;
  return this->target_temperature;
}

float FurrionChillCube::get_cool_target_() {
  if (this->mode == climate::CLIMATE_MODE_HEAT_COOL)
    return this->target_temperature_high;
  return this->target_temperature;
}

climate::ClimateFanMode FurrionChillCube::get_effective_fan_mode_() {
  bool clamp_active = fan_clamp_start_ > 0 && (millis() - fan_clamp_start_) < FAN_CLAMP_DURATION_MS;
  if (clamp_active) {
    return climate::CLIMATE_FAN_LOW;
  }
  return this->fan_mode.value_or(climate::CLIMATE_FAN_AUTO);
}

void FurrionChillCube::set_cs_value_(int cs) {
  cs = std::max(15, std::min(30, cs));
  current_cs_ = cs;
  if (boot_ready_ && !failsafe_active_ && active_ir_mode_ != climate::CLIMATE_MODE_OFF) {
    transmit_cs_update_(true);
    last_cs_heartbeat_ = millis();
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

void FurrionChillCube::start_fresh_kickstart_(int mode, int target_cs) {
  kick_mode_ = mode;
  kick_target_cs_ = target_cs;

  // Step 1: Set kickstart CS BEFORE mode change
  int restart_cs = (mode == 1) ? 20 : 26;
  current_cs_ = restart_cs;
  if (boot_ready_ && !failsafe_active_) {
    transmit_cs_update_(true);
    last_cs_heartbeat_ = millis();
  }
  if (cs_value_sensor_) cs_value_sensor_->publish_state(restart_cs);

  kick_phase_ = KickPhase::FRESH_PRE_CS;
  kick_phase_start_ = millis();
  mode_resend_at_ = 0;
  ESP_LOGI(TAG, "Kickstart: PRE-SET cs=%d, target_cs=%d", restart_cs, target_cs);
}

void FurrionChillCube::start_idle_kickstart_(int target_cs) {
  kick_target_cs_ = target_cs;
  kick_mode_ = 2;  // cool only

  current_cs_ = 25;  // cool idle restart threshold
  if (boot_ready_ && !failsafe_active_) {
    transmit_cs_update_(true);
    last_cs_heartbeat_ = millis();
  }
  if (cs_value_sensor_) cs_value_sensor_->publish_state(25);

  kick_phase_ = KickPhase::IDLE_KICK_CS;
  kick_phase_start_ = millis();
  mode_resend_at_ = 0;
  ESP_LOGI(TAG, "Idle kickstart: cs=25 target_cs=%d", target_cs);
}

void FurrionChillCube::advance_kickstart_() {
  uint32_t elapsed = millis() - kick_phase_start_;

  switch (kick_phase_) {
    case KickPhase::FRESH_PRE_CS:
      if (elapsed >= 500) {
        // Step 2: Turn on mode — first IR frame has correct CS
        active_ir_mode_ = (kick_mode_ == 1) ? climate::CLIMATE_MODE_HEAT : climate::CLIMATE_MODE_COOL;
        fan_clamp_start_ = millis();
        transmit_mode_command_();
        send_swing_state_();
        kick_phase_ = KickPhase::FRESH_MODE_ON;
        kick_phase_start_ = millis();
        ESP_LOGI(TAG, "Kickstart: MODE ON + fan=LOW, clamp=310s");
      }
      break;

    case KickPhase::FRESH_MODE_ON:
      if (elapsed >= 60000) {
        // Step 3: Release to gear controller + reinforce drop
        cs_reinforce_count_ = 2;
        cs_reinforce_at_ = millis();
        kick_phase_ = KickPhase::IDLE;
        uint32_t clamp_elapsed = millis() - fan_clamp_start_;
        int remaining = (clamp_elapsed < FAN_CLAMP_DURATION_MS) ?
                        (int)(FAN_CLAMP_DURATION_MS - clamp_elapsed) : 0;
        ESP_LOGI(TAG, "Kickstart: released, clamp %dms left", remaining);
      }
      break;

    case KickPhase::IDLE_KICK_CS:
      if (elapsed >= 5000) {
        // t=5s: reinforce CS=25
        transmit_cs_update_(true);
        kick_phase_ = KickPhase::IDLE_KICK_CS2;
        kick_phase_start_ = millis();
        ESP_LOGI(TAG, "Idle kickstart: reinforce cs=25");
      }
      break;

    case KickPhase::IDLE_KICK_CS2:
      if (elapsed >= 5000) {
        // t=10s: release to gear controller, start CS reinforcement
        cs_reinforce_count_ = 2;
        cs_reinforce_at_ = millis();
        kick_phase_ = KickPhase::IDLE;
        ESP_LOGI(TAG, "Idle kickstart: released, gear controller will set CS");
      }
      break;

    default:
      kick_phase_ = KickPhase::IDLE;
      break;
  }
}

// ============================================================
// Gear Controller
// ============================================================

void FurrionChillCube::run_gear_controller_() {
  float room = inside_temp_c_;
  uint32_t now = millis();

  // === Failsafe scenario 1: Boot, HA never connects (5 min) ===
  bool never_got_update = (last_temp_update_ == 0 && now > 300000);

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
    ESP_LOGW(TAG, "FAILSAFE COOL 76F (boot=%d ha_dc=%d nan=%d)",
             never_got_update, ha_disconnected, temp_unavailable);
    heat_gear_ = -1;
    cool_gear_ = -1;
    idle_since_ = 0;
    last_active_mode_ = 0;
    fan_clamp_start_ = 0;
    mode_resend_at_ = 0;
    cs_reinforce_count_ = 0;
    if (heat_gear_sensor_) heat_gear_sensor_->publish_state(-1);
    if (cool_gear_sensor_) cool_gear_sensor_->publish_state(-1);
    if (compressor_output_sensor_) compressor_output_sensor_->publish_state(0.0f);
    // CS feed stops (failsafe gate), unit reverts to own sensor
    failsafe_active_ = true;
    active_ir_mode_ = climate::CLIMATE_MODE_COOL;
    transmit_mode_command_();
    send_swing_state_();
    boot_ready_ = true;
    update_action_();
    return;
  }

  // NaN grace: hold current state
  if (isnan(room)) {
    ESP_LOGD(TAG, "Room temp NaN — holding (%lus left)",
             (300000 - (now - temp_nan_since_)) / 1000);
    return;
  }

  // Clear failsafe if normal operation
  if (failsafe_active_) {
    failsafe_active_ = false;
    ESP_LOGI(TAG, "Failsafe cleared — normal operation resumed");
  }

  // === Mode switch guard: 60s OFF between heat↔cool ===
  if (mode_switch_off_at_ > 0) {
    if ((now - mode_switch_off_at_) < 60000) {
      ESP_LOGD(TAG, "Mode switch cooldown — %lus remaining",
               (60000 - (now - mode_switch_off_at_)) / 1000);
      return;
    }
    mode_switch_off_at_ = 0;
  }

  bool user_input = user_changed_;
  if (user_input) user_changed_ = false;

  // Per-gear upshift check
  uint32_t time_in_gear = (last_gear_change_ == 0) ? 999999999 :
                          (now - last_gear_change_);
  auto can_upshift_to = [&](int target_gear) -> bool {
    return user_input || (time_in_gear >= HOLD_MS[target_gear]);
  };

  bool idle_long_enough = (idle_since_ > 0) &&
                          (now - idle_since_ >= IDLE_BEFORE_OFF_MS);

  // Outside temperature lockout for heating
  if (!isnan(outside_temp_c_)) {
    float lockout_low = outside_lockout_temp_c_ - 0.556f;  // 1°F hysteresis
    if (outside_temp_c_ < lockout_low && !heater_locked_out_) {
      heater_locked_out_ = true;
      ESP_LOGW(TAG, "Outside %.1f°C < lockout — heating locked out", outside_temp_c_);
    } else if (outside_temp_c_ >= outside_lockout_temp_c_ && heater_locked_out_) {
      heater_locked_out_ = false;
      ESP_LOGI(TAG, "Outside %.1f°C >= lockout — heating re-enabled", outside_temp_c_);
    }
  }

  // Determine active modes from user's climate mode
  bool heater_on = !heater_locked_out_ &&
                   (this->mode == climate::CLIMATE_MODE_HEAT ||
                    this->mode == climate::CLIMATE_MODE_HEAT_COOL);
  bool cooler_on = (this->mode == climate::CLIMATE_MODE_COOL ||
                    this->mode == climate::CLIMATE_MODE_HEAT_COOL);

  // When both modes enabled, decide based on room temp vs targets
  bool do_heat = heater_on;
  bool do_cool = cooler_on;
  if (heater_on && cooler_on) {
    float h_target = get_heat_target_();
    float c_target = get_cool_target_();
    int last_mode = last_active_mode_;

    float heat_engage = h_target;
    float cool_engage = c_target;
    if (last_mode == 1) {
      cool_engage = c_target + MODE_SWITCH_EXTRA;
    } else if (last_mode == 2) {
      heat_engage = h_target - MODE_SWITCH_EXTRA;
    }

    if (room <= heat_engage) {
      do_cool = false;
    } else if (room >= cool_engage) {
      do_heat = false;
    } else {
      // Hysteresis deadband — keep whichever was last active
      if (heat_gear_ >= 0) { do_cool = false; }
      else if (cool_gear_ >= 0) { do_heat = false; }
      else {
        float h_dist = h_target - room;
        float c_dist = room - c_target;
        if (h_dist >= c_dist) { do_cool = false; }
        else { do_heat = false; }
      }
    }
  }

  // ==========================
  // HEATING
  // ==========================
  if (do_heat) {
    // Mode switch: if HVAC is actively in cool mode, force 60s OFF first
    if (active_ir_mode_ == climate::CLIMATE_MODE_COOL) {
      cool_gear_ = -1;
      heat_gear_ = -1;
      fan_clamp_start_ = 0;
      mode_resend_at_ = 0;
      cs_reinforce_count_ = 0;
      if (cool_gear_sensor_) cool_gear_sensor_->publish_state(-1);
      if (heat_gear_sensor_) heat_gear_sensor_->publish_state(-1);
      if (compressor_output_sensor_) compressor_output_sensor_->publish_state(0.0f);
      active_ir_mode_ = climate::CLIMATE_MODE_OFF;
      transmit_mode_command_();
      mode_switch_off_at_ = now;
      ESP_LOGI(TAG, "MODE SWITCH cool→heat — OFF for 60s");
      update_action_();
      return;
    }
    if (cool_gear_ != -1) {
      cool_gear_ = -1;
      if (cool_gear_sensor_) cool_gear_sensor_->publish_state(-1);
    }

    float target = get_heat_target_();
    float diff = room - target;
    int gear = heat_gear_;
    int new_gear = gear;

    if (gear == -1) {
      if (diff < H_UP_23)         new_gear = 3;
      else if (diff < H_UP_12)    new_gear = 2;
      else if (diff < H_UP_01)    new_gear = 1;
    } else {
      switch (gear) {
        case 0: {
          if (can_upshift_to(1) && diff < H_UP_01) new_gear = 1;
          bool imm_off = !boot_ready_ || user_input;
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

    // Track idle_since
    if (new_gear == 0 && gear != 0) {
      idle_since_ = now;
    } else if (new_gear != 0) {
      idle_since_ = 0;
    }

    // Publish gear and compressor on change
    if (new_gear != gear) {
      heat_gear_ = new_gear;
      last_gear_change_ = now;
      if (heat_gear_sensor_) heat_gear_sensor_->publish_state(new_gear);
      if (new_gear >= 1) last_active_mode_ = 1;
      float pct;
      switch (new_gear) {
        case 3:  pct = 100.0f; break;
        case 2:  pct = 66.6f;  break;
        case 1:  pct = 33.3f;  break;
        default: pct = 0.0f;   break;
      }
      if (compressor_output_sensor_) compressor_output_sensor_->publish_state(pct);
      ESP_LOGI(TAG, "HEAT %d -> %d (room=%.2f target=%.2f diff=%.2f)",
               gear, new_gear, room, target, diff);
    }

    // CS + kickstart logic
    bool heat_kickstart_pending = false;
    if (new_gear >= 0 && kick_phase_ == KickPhase::IDLE) {
      int cs;
      switch (new_gear) {
        case 3:  cs = HEAT_ANCHOR - 1; break;
        case 2:  cs = HEAT_ANCHOR;     break;
        case 1:  cs = HEAT_ANCHOR + 1; break;
        default: cs = HEAT_ANCHOR + 2; break;
      }
      bool needs_fresh_start = (gear == -1 && new_gear == 1 && new_gear != gear);
      if (needs_fresh_start) {
        start_fresh_kickstart_(1, cs);
        heat_kickstart_pending = true;
      } else if (current_cs_ != cs) {
        set_cs_value_(cs);
      }
    }

    // HVAC on/off
    if (new_gear >= 0 && active_ir_mode_ != climate::CLIMATE_MODE_HEAT && !heat_kickstart_pending) {
      active_ir_mode_ = climate::CLIMATE_MODE_HEAT;
      transmit_mode_command_();
      send_swing_state_();
    }
    if (new_gear == -1 && active_ir_mode_ != climate::CLIMATE_MODE_OFF) {
      active_ir_mode_ = climate::CLIMATE_MODE_OFF;
      transmit_mode_command_();
      fan_clamp_start_ = 0;
      mode_resend_at_ = 0;
      cs_reinforce_count_ = 0;
    }

    // Fan clamp: drop if heat gear 2+
    if (new_gear >= 0 && kick_phase_ == KickPhase::IDLE) {
      bool clamp_active = fan_clamp_start_ > 0 && (now - fan_clamp_start_) < FAN_CLAMP_DURATION_MS;
      if (clamp_active && new_gear >= 2) {
        fan_clamp_start_ = 0;
        transmit_mode_command_();
        mode_resend_at_ = now;
        ESP_LOGI(TAG, "HEAT clamp dropped — gear %d (re-send in 5s)", new_gear);
      }
    }

    update_action_();

  // ==========================
  // COOLING
  // ==========================
  } else if (do_cool) {
    // Mode switch: if HVAC is actively in heat mode, force 60s OFF first
    if (active_ir_mode_ == climate::CLIMATE_MODE_HEAT) {
      heat_gear_ = -1;
      cool_gear_ = -1;
      fan_clamp_start_ = 0;
      mode_resend_at_ = 0;
      cs_reinforce_count_ = 0;
      if (heat_gear_sensor_) heat_gear_sensor_->publish_state(-1);
      if (cool_gear_sensor_) cool_gear_sensor_->publish_state(-1);
      if (compressor_output_sensor_) compressor_output_sensor_->publish_state(0.0f);
      active_ir_mode_ = climate::CLIMATE_MODE_OFF;
      transmit_mode_command_();
      mode_switch_off_at_ = now;
      ESP_LOGI(TAG, "MODE SWITCH heat→cool — OFF for 60s");
      update_action_();
      return;
    }
    if (heat_gear_ != -1) {
      heat_gear_ = -1;
      if (heat_gear_sensor_) heat_gear_sensor_->publish_state(-1);
    }

    float target = get_cool_target_();
    float diff = room - target;
    int gear = cool_gear_;
    int new_gear = gear;

    if (gear == -1) {
      if (diff > C_UP_45)         new_gear = 5;
      else if (diff > C_UP_34)    new_gear = 4;
      else if (diff > C_UP_23)    new_gear = 3;
      else if (diff > C_UP_12)    new_gear = 2;
      else if (diff > C_UP_01)    new_gear = 1;
    } else {
      switch (gear) {
        case 0: {
          if (can_upshift_to(1) && diff > C_UP_01) new_gear = 1;
          bool imm_off = !boot_ready_ || user_input;
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

    // Track idle_since
    if (new_gear == 0 && gear != 0) {
      idle_since_ = now;
    } else if (new_gear != 0) {
      idle_since_ = 0;
    }

    // Publish gear and compressor on change
    if (new_gear != gear) {
      cool_gear_ = new_gear;
      last_gear_change_ = now;
      if (cool_gear_sensor_) cool_gear_sensor_->publish_state(new_gear);
      if (new_gear >= 1) last_active_mode_ = 2;
      float pct;
      switch (new_gear) {
        case 5:  pct = 100.0f; break;
        case 4:  pct = 80.0f;  break;
        case 3:  pct = 60.0f;  break;
        case 2:  pct = 40.0f;  break;
        case 1:  pct = 20.0f;  break;
        default: pct = 0.0f;   break;
      }
      if (compressor_output_sensor_) compressor_output_sensor_->publish_state(pct);
      ESP_LOGI(TAG, "COOL %d -> %d (room=%.2f target=%.2f diff=%.2f)",
               gear, new_gear, room, target, diff);
    }

    // CS + kickstart logic
    bool cool_kickstart_pending = false;
    if (new_gear >= 0 && kick_phase_ == KickPhase::IDLE) {
      int cs;
      switch (new_gear) {
        case 5:  cs = COOL_ANCHOR + 2; break;
        case 4:  cs = COOL_ANCHOR + 1; break;
        case 3:  cs = COOL_ANCHOR;     break;
        case 2:  cs = COOL_ANCHOR - 1; break;
        case 1:  cs = COOL_ANCHOR - 2; break;
        default: cs = COOL_ANCHOR - 3; break;
      }
      bool needs_fresh_start = (gear == -1 && new_gear >= 1 && new_gear <= 3);
      bool needs_idle_kick = (gear == 0 && new_gear >= 1 && new_gear <= 2);
      if (needs_fresh_start) {
        start_fresh_kickstart_(2, cs);
        cool_kickstart_pending = true;
      } else if (needs_idle_kick) {
        start_idle_kickstart_(cs);
      } else if (current_cs_ != cs) {
        set_cs_value_(cs);
      }
    }

    // HVAC on/off
    if (new_gear >= 0 && active_ir_mode_ != climate::CLIMATE_MODE_COOL && !cool_kickstart_pending) {
      active_ir_mode_ = climate::CLIMATE_MODE_COOL;
      transmit_mode_command_();
      send_swing_state_();
    }
    if (new_gear == -1 && active_ir_mode_ != climate::CLIMATE_MODE_OFF) {
      active_ir_mode_ = climate::CLIMATE_MODE_OFF;
      transmit_mode_command_();
      fan_clamp_start_ = 0;
      mode_resend_at_ = 0;
      cs_reinforce_count_ = 0;
    }

    // Fan clamp: drop if cool gear 4+
    if (new_gear >= 0 && kick_phase_ == KickPhase::IDLE) {
      bool clamp_active = fan_clamp_start_ > 0 && (now - fan_clamp_start_) < FAN_CLAMP_DURATION_MS;
      if (clamp_active && new_gear >= 4) {
        fan_clamp_start_ = 0;
        transmit_mode_command_();
        mode_resend_at_ = now;
        ESP_LOGI(TAG, "COOL clamp dropped — gear %d (re-send in 5s)", new_gear);
      }
    }

    update_action_();

  // ==========================
  // NEITHER ACTIVE
  // ==========================
  } else {
    // Wind down any active gears to 0 (intermediate state for logging)
    if (heat_gear_ > 0) {
      ESP_LOGI(TAG, "HEAT %d -> -1 (neither active)", heat_gear_);
      heat_gear_ = 0;
    }
    if (cool_gear_ > 0) {
      ESP_LOGI(TAG, "COOL %d -> -1 (neither active)", cool_gear_);
      cool_gear_ = 0;
    }

    // Ensure HVAC is OFF
    if (active_ir_mode_ != climate::CLIMATE_MODE_OFF) {
      active_ir_mode_ = climate::CLIMATE_MODE_OFF;
      transmit_mode_command_();
      fan_clamp_start_ = 0;
      mode_resend_at_ = 0;
      cs_reinforce_count_ = 0;
      if (compressor_output_sensor_) compressor_output_sensor_->publish_state(0.0f);
      ESP_LOGI(TAG, "NEITHER ACTIVE — HVAC OFF");
    }

    // Enforce CS=22
    if (current_cs_ != 22) {
      set_cs_value_(22);
    }

    // HVAC is OFF — force gears to -1
    if (heat_gear_ != -1) {
      heat_gear_ = -1;
      if (heat_gear_sensor_) heat_gear_sensor_->publish_state(-1);
    }
    if (cool_gear_ != -1) {
      cool_gear_ = -1;
      if (cool_gear_sensor_) cool_gear_sensor_->publish_state(-1);
    }
    idle_since_ = 0;

    update_action_();
  }

  // Boot gate: first successful gear computation enables IR
  if (!boot_ready_) {
    boot_ready_ = true;
    ESP_LOGI(TAG, "Boot ready — first gear computation complete, IR enabled");
  }

  // Periodic state log
  ESP_LOGD(TAG, "state: heat=%d cool=%d room=%.2f cs=%d hold=%lus idle=%lum mode=%d",
           heat_gear_, cool_gear_, room, current_cs_,
           time_in_gear / 1000,
           idle_since_ > 0 ? (now - idle_since_) / 60000 : 0,
           last_active_mode_);
}

// ============================================================
// Diagnostics
// ============================================================

void FurrionChillCube::dump_config() {
  ESP_LOGCONFIG(TAG, "Furrion Chill Cube:");
  ESP_LOGCONFIG(TAG, "  Heat/Cool Gap: %.1f°F (%.2f°C)", heat_cool_gap_c_ * 9.0f / 5.0f, heat_cool_gap_c_);
  ESP_LOGCONFIG(TAG, "  Outside Lockout: %.1f°F (%.1f°C)",
                outside_lockout_temp_c_ * 9.0f / 5.0f + 32.0f, outside_lockout_temp_c_);
  ESP_LOGCONFIG(TAG, "  Inside Temp Unit: %s", inside_temp_fahrenheit_ ? "°F" : "°C");
  if (outside_temp_sensor_) {
    ESP_LOGCONFIG(TAG, "  Outside Temp Unit: %s", outside_temp_fahrenheit_ ? "°F" : "°C");
  }
}

}  // namespace furrion_chill_cube
}  // namespace esphome
