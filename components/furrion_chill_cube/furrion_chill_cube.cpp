#include "furrion_chill_cube.h"
#include "esphome/core/log.h"

#ifdef USE_API
#include "esphome/components/api/api_server.h"
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

// Temperature encoding: non-linear Gray code lookup
// RAC-PT1411HWRU Celsius table (index 0=16°C, 14=30°C)
// Bits: [5]=FRAC, [4]=NEG, [3:0]=temp nibble
static const uint8_t TEMP_C_TABLE[] = {
    0x10, 0x00, 0x01, 0x03, 0x02, 0x06, 0x07, 0x05,
    0x04, 0x0C, 0x0D, 0x09, 0x08, 0x0A, 0x0B};

// Celsius setpoint range (Furrion accepts 16-30°C)
static const int FURRION_MIN_TEMP_C = 16;
static const int FURRION_MAX_TEMP_C = 30;

// Gear controller constants — no fixed anchors; setpoint is dynamic

// Per-gear upshift hold times (ms) — index = target gear
//                        0      1       2       3        4        5
static const uint32_t HOLD_MS[] = {0, 180000, 180000, 300000, 300000, 600000};
static const uint32_t IDLE_BEFORE_OFF_MS = 900000;  // 15 min
static const uint32_t CLAMP_DURATION_MS = 330000;  // 5 min 30s
static const uint32_t CS_HEARTBEAT_MS = 30000;  // 30s
static const uint32_t GEAR_INTERVAL_MS = 60000;  // 60s fallback
static const uint32_t KEEPALIVE_INTERVAL_MS = 300000;  // 5 min
static const uint32_t KEEPALIVE_STEP_MS = 5000;  // 5s between steps
static const uint32_t MODE_SWITCH_IDLE_MS = 600000;   // 10 min idle before mode switch allowed
static const uint32_t MODE_SWITCH_EVENT_MS = 1200000;  // 20 min lockout after mode switch or fresh start

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

// Mode-switching protection is time-based (MODE_SWITCH_IDLE_MS / MODE_SWITCH_EVENT_MS)

// ============================================================
// Configuration Setters
// ============================================================

void FurrionChillCube::set_outside_lockout_temp(float temp_f) {
  // Convert Fahrenheit to Celsius
  outside_lockout_temp_c_ = (temp_f - 32.0f) * (5.0f / 9.0f);
}

// ============================================================
// Active IR Mode Persistence
// ============================================================

void FurrionChillCube::set_active_ir_mode_(climate::ClimateMode mode) {
  if (active_ir_mode_ == mode) return;
  active_ir_mode_ = mode;
  uint8_t m = (mode == climate::CLIMATE_MODE_HEAT) ? 1 :
              (mode == climate::CLIMATE_MODE_COOL) ? 2 : 0;
  mode_pref_.save(&m);
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

  // Temperature code (dynamic Celsius setpoint → Gray code lookup)
  uint8_t temp_code;
  if (active_ir_mode_ == climate::CLIMATE_MODE_OFF) {
    temp_code = 0x0E;  // OFF special value
  } else {
    int temp_c = std::max(FURRION_MIN_TEMP_C, std::min(FURRION_MAX_TEMP_C, furrion_setpoint_c_));
    temp_code = TEMP_C_TABLE[temp_c - FURRION_MIN_TEMP_C];
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
    // FAH flag NOT set — Celsius mode
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

  // === Transmission 2: Swing (B9) — 10.5ms gaps before and after ===
  // Matches Toshiba component: reuse same transmit object with reset
  data->reset();
  data->space(IR_PACKET_SPACE);
  if (this->swing_mode == climate::CLIMATE_SWING_VERTICAL) {
    static const uint8_t SWING_ON[] = {0xB9, 0x46, 0xF5, 0x0A, 0x04, 0xFB};
    this->encode_(data, SWING_ON, 6, 1);
  } else {
    static const uint8_t SWING_OFF[] = {0xB9, 0x46, 0xF5, 0x0A, 0x05, 0xFA};
    this->encode_(data, SWING_OFF, 6, 1);
  }
  data->space(IR_PACKET_SPACE);
  transmit.perform();

  ESP_LOGD(TAG, "IR mode=%d fan=%d swing=%d",
           (int)active_ir_mode_, (int)fan, (int)this->swing_mode);

  // === Transmission 3: CS follow-up (BA) — separate transmit object ===
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

  // Restore active IR mode from flash (survives reboot)
  mode_pref_ = global_preferences->make_preference<uint8_t>(this->get_object_id_hash() ^ 0x4D4F4445);
  uint8_t saved_mode = 0;
  if (mode_pref_.load(&saved_mode)) {
    if (saved_mode == 1) {
      heat_gear_ = 0;
      last_active_mode_ = 1;
      ESP_LOGI(TAG, "Restored prior mode: HEAT (gear → idle, skip kickstart)");
    } else if (saved_mode == 2) {
      cool_gear_ = 0;
      last_active_mode_ = 2;
      ESP_LOGI(TAG, "Restored prior mode: COOL (gear → idle, skip kickstart)");
    }
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

  // Initialize diagnostic sensors (reflect restored gear state)
  if (heat_gear_sensor_) heat_gear_sensor_->publish_state(heat_gear_);
  if (cool_gear_sensor_) cool_gear_sensor_->publish_state(cool_gear_);
  if (compressor_output_sensor_) compressor_output_sensor_->publish_state(0.0f);
}

void FurrionChillCube::loop() {
  uint32_t now = millis();

  // 1. Advance keep-alive state machine
  if (keepalive_phase_ != KeepAlivePhase::IDLE) {
    advance_keepalive_(now);
  }

  // 2. Run gear controller if triggered (not blocked by kickstart — kickstart suppresses CS writes)
  bool should_run = false;
  if (temp_dirty_) { should_run = true; temp_dirty_ = false; }
  if (last_gear_run_ == 0 || (now - last_gear_run_) >= GEAR_INTERVAL_MS) should_run = true;
  if (user_changed_) should_run = true;

  if (should_run && keepalive_phase_ == KeepAlivePhase::IDLE) {
    run_gear_controller_();
    last_gear_run_ = now;
  }

  // 3. Advance kickstart (after gear controller so it can check gear thresholds)
  if (kickstart_active_()) {
    advance_kickstart_(now);
  }

  // 4. Keep-alive trigger for low-CS gears (cool 1-2, heat 1)
  if (!kickstart_active_() && keepalive_phase_ == KeepAlivePhase::IDLE &&
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
    transmit_cs_update_(true);
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
  if (call.get_mode().has_value()) {
    auto new_mode = *call.get_mode();
    auto old_mode = this->mode;

    // Sync target_temperature for HA single-slider display in HEAT/COOL modes
    if (new_mode == climate::CLIMATE_MODE_HEAT) {
      this->target_temperature = this->target_temperature_low;
    } else if (new_mode == climate::CLIMATE_MODE_COOL) {
      this->target_temperature = this->target_temperature_high;
    }
    // Ensure two-point values valid for first boot
    if (isnan(this->target_temperature_low)) this->target_temperature_low = 20.0f;
    if (isnan(this->target_temperature_high)) this->target_temperature_high = 25.0f;

    this->mode = new_mode;

    // Abort kickstart only if new mode is incompatible with the active IR mode.
    if (kickstart_active_()) {
      bool compatible = false;
      if (active_ir_mode_ == climate::CLIMATE_MODE_COOL) {
        compatible = (new_mode == climate::CLIMATE_MODE_COOL || new_mode == climate::CLIMATE_MODE_HEAT_COOL);
      } else if (active_ir_mode_ == climate::CLIMATE_MODE_HEAT) {
        compatible = (new_mode == climate::CLIMATE_MODE_HEAT || new_mode == climate::CLIMATE_MODE_HEAT_COOL);
      }
      if (!compatible) {
        ESP_LOGI(TAG, "Kickstart aborted — mode %d incompatible with IR mode %d",
                 (int)new_mode, (int)active_ir_mode_);
        end_kickstart_(millis());
        // If switching COOL↔HEAT, trigger 60s cooldown
        if ((active_ir_mode_ == climate::CLIMATE_MODE_COOL &&
             (new_mode == climate::CLIMATE_MODE_HEAT)) ||
            (active_ir_mode_ == climate::CLIMATE_MODE_HEAT &&
             (new_mode == climate::CLIMATE_MODE_COOL))) {
          set_active_ir_mode_(climate::CLIMATE_MODE_OFF);
          transmit_mode_command_();
          mode_switch_off_at_ = millis();
          last_mode_event_at_ = millis();
        }
      }
    }
  }

  // Target temperature changes — setpoint changes don't affect kickstart
  bool temp_changed = false;
  if (call.get_target_temperature().has_value()) {
    this->target_temperature = *call.get_target_temperature();
    temp_changed = true;
  }
  if (call.get_target_temperature_low().has_value()) {
    this->target_temperature_low = *call.get_target_temperature_low();
    temp_changed = true;
  }
  if (call.get_target_temperature_high().has_value()) {
    this->target_temperature_high = *call.get_target_temperature_high();
    temp_changed = true;
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

  // Fan mode change
  if (call.get_fan_mode().has_value()) {
    auto new_fan = *call.get_fan_mode();
    this->fan_mode = new_fan;
    if (active_ir_mode_ != climate::CLIMATE_MODE_OFF && !kickstart_active_()) {
      transmit_mode_command_();
      ESP_LOGI(TAG, "User fan change → %d, mode command sent", (int)new_fan);
    }
  }

  // Swing mode change — standalone swing frame, works during kickstart
  // Does NOT set user_changed_: vent direction is cosmetic and must never
  // trigger gear recalculation, timer resets, or immediate-off logic.
  if (call.get_swing_mode().has_value()) {
    this->swing_mode = *call.get_swing_mode();
    send_swing_state_();
    ESP_LOGI(TAG, "User swing change → %d", (int)*call.get_swing_mode());
  }

  // Only flag gear recalculation for changes that affect cooling/heating
  // (mode, setpoint, fan). Swing is purely cosmetic.
  bool gear_relevant = call.get_mode().has_value() ||
                       temp_changed ||
                       call.get_fan_mode().has_value();
  if (gear_relevant) {
    this->user_changed_ = true;
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

int FurrionChillCube::compute_setpoint_c_(bool is_heat) {
  float target_c = is_heat ? get_heat_target_() : get_cool_target_();
  if (isnan(target_c)) return 22;  // safe default
  int rounded = (int)roundf(target_c);
  return std::max(FURRION_MIN_TEMP_C, std::min(FURRION_MAX_TEMP_C, rounded));
}

void FurrionChillCube::update_furrion_setpoint_(bool is_heat) {
  int new_sp = compute_setpoint_c_(is_heat);
  if (new_sp != furrion_setpoint_c_) {
    ESP_LOGI(TAG, "Furrion setpoint %d°C → %d°C (%s)",
             furrion_setpoint_c_, new_sp, is_heat ? "heat" : "cool");
    furrion_setpoint_c_ = new_sp;
    // Retransmit mode command so unit gets the new setpoint
    if (active_ir_mode_ != climate::CLIMATE_MODE_OFF && !kickstart_active_()) {
      transmit_mode_command_();
    }
  }
}

climate::ClimateFanMode FurrionChillCube::get_effective_fan_mode_() {
  if (clamp_phase_ == ClampPhase::CLAMPED) {
    return climate::CLIMATE_FAN_LOW;
  }
  return this->fan_mode.value_or(climate::CLIMATE_FAN_AUTO);
}

int FurrionChillCube::compute_gear_cs_(bool is_heat, int gear) {
  if (is_heat) {
    int hi = furrion_setpoint_c_ + 2;
    int offset = (hi > 30) ? (30 - hi) : 0;
    switch (gear) {
      case 3:  return furrion_setpoint_c_ - 1 + offset;
      case 2:  return furrion_setpoint_c_     + offset;
      case 1:  return furrion_setpoint_c_ + 1 + offset;
      default: return furrion_setpoint_c_ + 2 + offset;
    }
  } else {
    int lo = furrion_setpoint_c_ - 3;
    int offset = (lo < 15) ? (15 - lo) : 0;
    switch (gear) {
      case 5:  return furrion_setpoint_c_ + 2 + offset;
      case 4:  return furrion_setpoint_c_ + 1 + offset;
      case 3:  return furrion_setpoint_c_     + offset;
      case 2:  return furrion_setpoint_c_ - 1 + offset;
      case 1:  return furrion_setpoint_c_ - 2 + offset;
      default: return furrion_setpoint_c_ - 3 + offset;
    }
  }
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

void FurrionChillCube::start_clamped_kickstart_(bool is_heat, uint32_t now) {
  clamp_is_heat_ = is_heat;
  // Kickstart CS: heat=gear2 (setpoint), cool=gear4 (setpoint+1)
  clamp_kickstart_cs_ = is_heat ? furrion_setpoint_c_ : (furrion_setpoint_c_ + 1);
  clamp_kickstart_cs_ = std::max(15, std::min(30, clamp_kickstart_cs_));

  // PRE_CS: set kickstart CS before mode-on (500ms lead)
  current_cs_ = clamp_kickstart_cs_;
  if (boot_ready_ && !failsafe_active_) {
    transmit_cs_update_(true);
    last_cs_heartbeat_ = now;
  }
  if (cs_value_sensor_) cs_value_sensor_->publish_state(clamp_kickstart_cs_);

  clamp_phase_ = ClampPhase::PRE_CS;
  clamp_phase_start_ = now;
  ESP_LOGI(TAG, "Clamped kickstart: PRE_CS %s cs=%d", is_heat ? "HEAT" : "COOL", clamp_kickstart_cs_);
}

void FurrionChillCube::start_quick_kickstart_(bool is_heat, int kickstart_cs, uint32_t now) {
  quick_kick_active_ = true;
  quick_kick_start_ = now;
  quick_kick_cs_ = std::max(15, std::min(30, kickstart_cs));
  quick_kick_is_heat_ = is_heat;
  quick_kick_reinforced_ = false;

  // Set and transmit kickstart CS immediately
  current_cs_ = quick_kick_cs_;
  if (boot_ready_ && !failsafe_active_ && active_ir_mode_ != climate::CLIMATE_MODE_OFF) {
    transmit_cs_update_(true);
    last_cs_heartbeat_ = now;
  }
  if (cs_value_sensor_) cs_value_sensor_->publish_state(quick_kick_cs_);

  // If unit is OFF, turn it on now (no fan clamp, fan=AUTO)
  if (active_ir_mode_ == climate::CLIMATE_MODE_OFF) {
    set_active_ir_mode_(is_heat ? climate::CLIMATE_MODE_HEAT : climate::CLIMATE_MODE_COOL);
    transmit_mode_command_();
  }

  ESP_LOGI(TAG, "Quick kickstart: %s cs=%d", is_heat ? "HEAT" : "COOL", quick_kick_cs_);
}

void FurrionChillCube::advance_kickstart_(uint32_t now) {
  // === Clamped kickstart state machine ===
  if (clamp_phase_ == ClampPhase::PRE_CS) {
    if ((now - clamp_phase_start_) >= 500) {
      // Mode ON + fan=LOW
      set_active_ir_mode_(clamp_is_heat_ ? climate::CLIMATE_MODE_HEAT : climate::CLIMATE_MODE_COOL);
      transmit_mode_command_();
      clamp_phase_ = ClampPhase::CLAMPED;
      clamp_start_ = now;
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
      // 5s: reinforce kickstart CS (one-shot)
      quick_kick_reinforced_ = true;
      transmit_cs_update_(true);
      last_cs_heartbeat_ = now;
      ESP_LOGI(TAG, "Quick kickstart: reinforce cs=%d", quick_kick_cs_);
    }
    if (elapsed >= 10000) {
      // 10s: release — transmit current gear's CS
      ESP_LOGI(TAG, "Quick kickstart: ended — 10s");
      end_kickstart_(now);
    }
  }
}

void FurrionChillCube::end_kickstart_(uint32_t now) {
  bool was_clamped = (clamp_phase_ == ClampPhase::CLAMPED);

  // Clear all kickstart state
  clamp_phase_ = ClampPhase::IDLE;
  clamp_start_ = 0;
  clamp_phase_start_ = 0;
  quick_kick_active_ = false;
  quick_kick_start_ = 0;

  // Determine current gear and compute the correct CS
  bool is_heat = (active_ir_mode_ == climate::CLIMATE_MODE_HEAT);
  int gear = is_heat ? heat_gear_ : cool_gear_;
  if (gear >= 0) {
    int cs = compute_gear_cs_(is_heat, gear);
    set_cs_value_(cs);
  }

  // If we were clamped (fan=LOW), re-send mode with fan=AUTO
  if (was_clamped && active_ir_mode_ != climate::CLIMATE_MODE_OFF) {
    transmit_mode_command_();
  }

  ESP_LOGI(TAG, "Kickstart released: cs=%d fan=%s", current_cs_, was_clamped ? "AUTO (was LOW)" : "unchanged");
}

// ============================================================
// Keep-Alive Pulse (sustain compressor at low CS gears)
// ============================================================

void FurrionChillCube::start_keepalive_(bool is_heat, uint32_t now) {
  keepalive_restore_cs_ = current_cs_;
  int anchor = furrion_setpoint_c_;
  int step2_raw = is_heat ? (anchor - 1) : (anchor + 1);
  keepalive_step2_cs_ = std::max(15, std::min(30, step2_raw));
  // At boundary setpoints (cool@30, heat@16), step2 clamps to anchor — no swing possible
  if (keepalive_step2_cs_ == anchor) {
    keepalive_last_ = now;  // reset timer without pulsing
    ESP_LOGD(TAG, "Keep-alive: skipped (boundary setpoint, no CS swing)");
    return;
  }
  set_cs_value_(anchor);  // Step 1: setpoint CS
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
        set_cs_value_(keepalive_step2_cs_);  // Step 2: over-anchor (heat=19, cool=26)
        keepalive_phase_ = KeepAlivePhase::STEP2;
        keepalive_phase_start_ = now;
        ESP_LOGI(TAG, "Keep-alive: cs=%d", keepalive_step2_cs_);
      }
      break;

    case KeepAlivePhase::STEP2:
      if (elapsed >= KEEPALIVE_STEP_MS) {
        set_cs_value_(keepalive_restore_cs_);  // Step 3: restore (1/2)
        keepalive_phase_ = KeepAlivePhase::STEP_RESTORE1;
        keepalive_phase_start_ = now;
        ESP_LOGI(TAG, "Keep-alive: restore cs=%d (1/2)", keepalive_restore_cs_);
      }
      break;

    case KeepAlivePhase::STEP_RESTORE1:
      if (elapsed >= KEEPALIVE_STEP_MS) {
        set_cs_value_(keepalive_restore_cs_);  // Step 4: restore (2/2) — done
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
    keepalive_last_ = 0;
  }
}

// ============================================================
// Gear Controller
// ============================================================

void FurrionChillCube::run_gear_controller_() {
  float room = inside_temp_c_;
  uint32_t now = millis();
  float gear_diff = NAN;  // Tracks active diff for debug publishing

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
    ESP_LOGW(TAG, "FAILSAFE — CS stopped, unit will revert to internal sensor (boot=%d ha_dc=%d nan=%d)",
             never_got_update, ha_disconnected, temp_unavailable);
    heat_gear_ = -1;
    cool_gear_ = -1;
    idle_since_ = 0;
    last_active_mode_ = 0;
    last_mode_event_at_ = 0;
    clamp_phase_ = ClampPhase::IDLE;
    quick_kick_active_ = false;
    abort_keepalive_();
    if (heat_gear_sensor_) heat_gear_sensor_->publish_state(-1);
    if (cool_gear_sensor_) cool_gear_sensor_->publish_state(-1);
    if (compressor_output_sensor_) compressor_output_sensor_->publish_state(0.0f);
    // Stop all IR transmission — failsafe_active_ gates CS heartbeat.
    // Unit's setpoint is already near the desired target (dynamic setpoint).
    // After ~7 min with no CS, unit reverts to its own internal sensor.
    failsafe_active_ = true;
    boot_ready_ = true;
    update_action_();
    return;
  }

  // NaN grace: hold current state
  if (isnan(room)) {
    ESP_LOGD(TAG, "Room temp NaN — holding (%lus left)",
             (300000 - (now - temp_nan_since_)) / 1000);
    publish_debug_state_(NAN);
    return;
  }

  // Clear failsafe if normal operation
  if (failsafe_active_) {
    failsafe_active_ = false;
    ESP_LOGI(TAG, "Failsafe cleared — normal operation resumed");
  }

  // Fix: seed idle_since_ if gear is 0 but timer was never set (post-boot recovery)
  if ((heat_gear_ == 0 || cool_gear_ == 0) && idle_since_ == 0) {
    idle_since_ = now;
  }

  // === Mode switch guard: 60s OFF between heat↔cool ===
  if (mode_switch_off_at_ > 0) {
    if ((now - mode_switch_off_at_) < 60000) {
      ESP_LOGD(TAG, "Mode switch cooldown — %lus remaining",
               (60000 - (now - mode_switch_off_at_)) / 1000);
      publish_debug_state_(NAN);
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

    // Time-based mode switch lockout (replaces temperature-based MODE_SWITCH_EXTRA)
    bool mode_switch_allowed = user_input;  // user override cancels all lockouts
    if (!mode_switch_allowed && last_mode != 0) {
      bool idle_enough = idle_since_ > 0 && (now - idle_since_) >= MODE_SWITCH_IDLE_MS;
      bool no_recent_event = last_mode_event_at_ == 0 || (now - last_mode_event_at_) >= MODE_SWITCH_EVENT_MS;
      mode_switch_allowed = idle_enough && no_recent_event;
    }

    if (!mode_switch_allowed && last_mode == 1) {
      do_cool = false;    // locked: stay in heat path
    } else if (!mode_switch_allowed && last_mode == 2) {
      do_heat = false;    // locked: stay in cool path
    } else if (room <= h_target) {
      do_cool = false;
    } else if (room >= c_target) {
      do_heat = false;
    } else {
      // Deadband — keep whichever was last active
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
      clamp_phase_ = ClampPhase::IDLE;
      quick_kick_active_ = false;
      abort_keepalive_();
      if (cool_gear_sensor_) cool_gear_sensor_->publish_state(-1);
      if (heat_gear_sensor_) heat_gear_sensor_->publish_state(-1);
      if (compressor_output_sensor_) compressor_output_sensor_->publish_state(0.0f);
      set_active_ir_mode_(climate::CLIMATE_MODE_OFF);
      transmit_mode_command_();
      mode_switch_off_at_ = now;
      last_mode_event_at_ = now;
      ESP_LOGI(TAG, "MODE SWITCH cool→heat — OFF for 60s");
      update_action_();
      publish_debug_state_(NAN);
      return;
    }
    if (cool_gear_ != -1) {
      cool_gear_ = -1;
      if (cool_gear_sensor_) cool_gear_sensor_->publish_state(-1);
    }

    update_furrion_setpoint_(true);
    float target = get_heat_target_();
    float diff = room - target;
    gear_diff = diff;
    int gear = heat_gear_;
    int new_gear = gear;

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
          bool imm_off = !boot_ready_;
          if ((imm_off || idle_long_enough) && diff > H_IDLE) new_gear = -1;
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
        if (!kickstart_active_() && current_cs_ != cs) {
          set_cs_value_(cs);
        }
      } else if (!kickstart_active_() && current_cs_ != cs) {
        set_cs_value_(cs);
      }
    }

    // HVAC on/off
    if (new_gear >= 0 && active_ir_mode_ != climate::CLIMATE_MODE_HEAT && !kickstart_active_()) {
      set_active_ir_mode_(climate::CLIMATE_MODE_HEAT);
      transmit_mode_command_();
      transmit_cs_update_(true);
    }
    if (new_gear == -1 && active_ir_mode_ != climate::CLIMATE_MODE_OFF) {
      set_active_ir_mode_(climate::CLIMATE_MODE_OFF);
      transmit_mode_command_();
      if (kickstart_active_()) end_kickstart_(now);
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
      clamp_phase_ = ClampPhase::IDLE;
      quick_kick_active_ = false;
      abort_keepalive_();
      if (heat_gear_sensor_) heat_gear_sensor_->publish_state(-1);
      if (cool_gear_sensor_) cool_gear_sensor_->publish_state(-1);
      if (compressor_output_sensor_) compressor_output_sensor_->publish_state(0.0f);
      set_active_ir_mode_(climate::CLIMATE_MODE_OFF);
      transmit_mode_command_();
      mode_switch_off_at_ = now;
      last_mode_event_at_ = now;
      ESP_LOGI(TAG, "MODE SWITCH heat→cool — OFF for 60s");
      update_action_();
      publish_debug_state_(NAN);
      return;
    }
    if (heat_gear_ != -1) {
      heat_gear_ = -1;
      if (heat_gear_sensor_) heat_gear_sensor_->publish_state(-1);
    }

    update_furrion_setpoint_(false);
    float target = get_cool_target_();
    float diff = room - target;
    gear_diff = diff;
    int gear = cool_gear_;
    int new_gear = gear;

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
          bool imm_off = !boot_ready_;
          if ((imm_off || idle_long_enough) && diff < C_IDLE) new_gear = -1;
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

    // Block -1→1 for cooling (can't cold-start at gear 1)
    if (gear == -1 && new_gear == 1) new_gear = 2;

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
        // OFF→gear 3: quick kickstart (CS=gear4 for 10s, then gear 3)
        last_mode_event_at_ = now;
        int kick_cs = std::max(15, std::min(30, furrion_setpoint_c_ + 1));
        start_quick_kickstart_(false, kick_cs, now);
      } else if (gear == -1 && new_gear >= 4) {
        // OFF→gear 4+: direct, no kickstart
        if (!kickstart_active_() && current_cs_ != cs) {
          set_cs_value_(cs);
        }
      } else if (gear == 0 && new_gear >= 1 && new_gear <= 2) {
        // Idle→gear 1-2: quick kickstart (CS=setpoint for 10s)
        int kick_cs = std::max(15, std::min(30, furrion_setpoint_c_));
        start_quick_kickstart_(false, kick_cs, now);
      } else if (!kickstart_active_() && current_cs_ != cs) {
        set_cs_value_(cs);
      }
    }

    // HVAC on/off
    if (new_gear >= 0 && active_ir_mode_ != climate::CLIMATE_MODE_COOL && !kickstart_active_()) {
      set_active_ir_mode_(climate::CLIMATE_MODE_COOL);
      transmit_mode_command_();
      transmit_cs_update_(true);
    }
    if (new_gear == -1 && active_ir_mode_ != climate::CLIMATE_MODE_OFF) {
      set_active_ir_mode_(climate::CLIMATE_MODE_OFF);
      transmit_mode_command_();
      if (kickstart_active_()) end_kickstart_(now);
    }

    update_action_();

  // ==========================
  // NEITHER ACTIVE
  // ==========================
  } else {
    // Ensure HVAC is OFF
    if (active_ir_mode_ != climate::CLIMATE_MODE_OFF) {
      set_active_ir_mode_(climate::CLIMATE_MODE_OFF);
      transmit_mode_command_();
      clamp_phase_ = ClampPhase::IDLE;
      quick_kick_active_ = false;
      abort_keepalive_();
      if (compressor_output_sensor_) compressor_output_sensor_->publish_state(0.0f);
      ESP_LOGI(TAG, "NEITHER ACTIVE — HVAC OFF");
    }

    // Enforce neutral CS (at setpoint — unit is OFF so doesn't matter much)
    if (current_cs_ != furrion_setpoint_c_) {
      set_cs_value_(furrion_setpoint_c_);
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

  // Boot gate: first successful gear computation enables IR
  if (!boot_ready_) {
    boot_ready_ = true;
    ESP_LOGI(TAG, "Boot ready — first gear computation complete, IR enabled");
  }

  // Debug sensor publishing
  publish_debug_state_(gear_diff);

  // Periodic state log
  ESP_LOGD(TAG, "state: heat=%d cool=%d room=%.2f cs=%d hold=%lus idle=%lum mode=%d",
           heat_gear_, cool_gear_, room, current_cs_,
           time_in_gear / 1000,
           idle_since_ > 0 ? (now - idle_since_) / 60000 : 0,
           last_active_mode_);
}

// ============================================================
// Debug State Publishing
// ============================================================

void FurrionChillCube::publish_debug_state_(float diff) {
  // Skip if no debug sensors registered
  if (!debug_active_ir_mode_sensor_) return;

  uint32_t now = millis();

  // Active IR mode: 0=OFF, 1=HEAT, 2=COOL
  float ir_mode = (active_ir_mode_ == climate::CLIMATE_MODE_HEAT) ? 1.0f :
                  (active_ir_mode_ == climate::CLIMATE_MODE_COOL) ? 2.0f : 0.0f;
  debug_active_ir_mode_sensor_->publish_state(ir_mode);

  if (debug_last_active_mode_sensor_)
    debug_last_active_mode_sensor_->publish_state((float)last_active_mode_);

  if (debug_kick_phase_sensor_) {
    float phase = (float)(uint8_t)clamp_phase_;
    if (quick_kick_active_) phase = 10.0f;  // distinct value for quick kick
    debug_kick_phase_sensor_->publish_state(phase);
  }

  if (debug_gear_diff_sensor_)
    debug_gear_diff_sensor_->publish_state(isnan(diff) ? NAN : diff);

  if (debug_time_in_gear_sensor_) {
    float tig = (last_gear_change_ == 0) ? -1.0f : (float)((now - last_gear_change_) / 1000);
    debug_time_in_gear_sensor_->publish_state(tig);
  }

  if (debug_idle_duration_sensor_) {
    float idle = (idle_since_ == 0) ? -1.0f : (float)((now - idle_since_) / 1000);
    debug_idle_duration_sensor_->publish_state(idle);
  }

  if (debug_mode_switch_cooldown_sensor_) {
    float cd = 0.0f;
    if (mode_switch_off_at_ > 0) {
      uint32_t elapsed = now - mode_switch_off_at_;
      cd = (elapsed < 60000) ? (float)((60000 - elapsed) / 1000) : 0.0f;
    }
    debug_mode_switch_cooldown_sensor_->publish_state(cd);
  }

  if (debug_fan_clamp_remaining_sensor_) {
    float clamp = 0.0f;
    if (clamp_phase_ == ClampPhase::CLAMPED && clamp_start_ > 0) {
      uint32_t elapsed = now - clamp_start_;
      clamp = (elapsed < CLAMP_DURATION_MS) ? (float)((CLAMP_DURATION_MS - elapsed) / 1000) : 0.0f;
    }
    debug_fan_clamp_remaining_sensor_->publish_state(clamp);
  }

  if (debug_heater_locked_out_sensor_)
    debug_heater_locked_out_sensor_->publish_state(heater_locked_out_ ? 1.0f : 0.0f);

  if (debug_failsafe_active_sensor_)
    debug_failsafe_active_sensor_->publish_state(failsafe_active_ ? 1.0f : 0.0f);

  if (debug_boot_ready_sensor_)
    debug_boot_ready_sensor_->publish_state(boot_ready_ ? 1.0f : 0.0f);
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
  const char *mode_str = (heat_gear_ == 0) ? "HEAT (idle)" :
                         (cool_gear_ == 0) ? "COOL (idle)" : "OFF";
  ESP_LOGCONFIG(TAG, "  Restored Mode: %s", mode_str);
}

}  // namespace furrion_chill_cube
}  // namespace esphome
