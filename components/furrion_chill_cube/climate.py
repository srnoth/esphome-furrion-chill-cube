import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate, sensor, binary_sensor, button, remote_transmitter
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    ENTITY_CATEGORY_DIAGNOSTIC,
    ENTITY_CATEGORY_CONFIG,
    UNIT_CELSIUS,
    UNIT_PERCENT,
    UNIT_SECOND,
)

DEPENDENCIES = ["remote_transmitter"]
AUTO_LOAD = ["sensor", "binary_sensor", "button"]
CODEOWNERS = ["@srnoth"]

fcc_ns = cg.esphome_ns.namespace("furrion_chill_cube")
FurrionChillCube = fcc_ns.class_("FurrionChillCube", climate.Climate, cg.Component)

# Button classes
DisplayToggleButton = fcc_ns.class_("DisplayToggleButton", button.Button)
TurboOnButton = fcc_ns.class_("TurboOnButton", button.Button)
TurboOffButton = fcc_ns.class_("TurboOffButton", button.Button)
SwingOnButton = fcc_ns.class_("SwingOnButton", button.Button)
SwingOffButton = fcc_ns.class_("SwingOffButton", button.Button)
VaneStepButton = fcc_ns.class_("VaneStepButton", button.Button)

# Config keys
CONF_TRANSMITTER_ID = "transmitter_id"
CONF_INSIDE_TEMPERATURE = "inside_temperature"
CONF_INSIDE_TEMPERATURE_IS_FAHRENHEIT = "inside_temperature_is_fahrenheit"
CONF_OUTSIDE_TEMPERATURE = "outside_temperature"
CONF_OUTSIDE_TEMPERATURE_IS_FAHRENHEIT = "outside_temperature_is_fahrenheit"
CONF_OUTSIDE_LOCKOUT_TEMP = "outside_lockout_temp"
CONF_MODE_SWITCH_IDLE_MIN = "mode_switch_idle_min"
CONF_MODE_SWITCH_EVENT_MIN = "mode_switch_event_min"
CONF_MODE_SWITCH_TEMP_OFFSET = "mode_switch_temp_offset"
CONF_MODE_SWITCH_OFF = "mode_switch_off_time"
# Configurable gear ladders + transition quirks + CS transmit cadence.
# Gears are (gear_number → CS offset from the °C setpoint anchor). Quirks are
# path-dependent transition maneuvers: on a matching (mode, from→to) gear change,
# the controller holds `via_offset` CS for `duration`, re-asserting every
# `quirk_transmit_interval`, then re-evaluates the correct gear. See PHASE2 / session_log.
CONF_COOL_GEARS = "cool_gears"
CONF_HEAT_GEARS = "heat_gears"
CONF_GEAR = "gear"
CONF_CS_OFFSET = "cs_offset"
CONF_COOL_LADDER = "cool_ladder"
CONF_HEAT_LADDER = "heat_ladder"
CONF_MODULATION_SPACING = "modulation_spacing"
CONF_MODULATION_HYSTERESIS = "modulation_hysteresis"
CONF_LADDER_START = "start"
CONF_LADDER_STOP = "stop"
CONF_LADDER_IDLE = "idle"
CONF_QUIRKS = "quirks"
CONF_QUIRK_MODE = "mode"
CONF_QUIRK_FROM = "from_gear"
CONF_QUIRK_TO = "to_gear"
CONF_QUIRK_VIA_OFFSET = "via_offset"
CONF_QUIRK_DURATION = "duration"
CONF_QUIRK_DURATION_DEFAULT = "quirk_duration"
CONF_CS_TRANSMIT_INTERVAL = "cs_transmit_interval"
CONF_QUIRK_TRANSMIT_INTERVAL = "quirk_transmit_interval"
CONF_GEAR_STEP_C = "gear_step_c"
# Timed vane positioning (per-mode delay + interval; ESPHome time format, no defaults)
CONF_HEAT_VENT_MOVE_DELAY = "heat_vent_move_delay"
CONF_HEAT_VENT_INTERVAL = "heat_final_position_interval"
CONF_COOL_VENT_MOVE_DELAY = "cool_vent_move_delay"
CONF_COOL_VENT_INTERVAL = "cool_final_position_interval"
CONF_USE_FAHRENHEIT = "use_fahrenheit"
CONF_TEST_MODE = "test_mode"
# Phase 2 adaptive equilibrium-gear controller (cool mode)
CONF_ADAPTIVE_ENABLE = "adaptive_enable"
CONF_VENT_FAN = "vent_fan"
CONF_FAN_FEEDFORWARD_GEARS = "fan_feedforward_gears"


def validate_mode_switch_temp_offset(value):
    """Parse '1F', '0.56C', or bare number (= Celsius). Returns float in °C."""
    if isinstance(value, (int, float)):
        return float(value)
    s = str(value).strip().upper()
    if s.endswith("F"):
        num = float(s[:-1].strip())
        return num * (5.0 / 9.0)  # °F delta → °C delta
    elif s.endswith("C"):
        return float(s[:-1].strip())
    else:
        return float(s)
CONF_HEAT_GEAR = "heat_gear"
CONF_COOL_GEAR = "cool_gear"
CONF_COMPRESSOR_OUTPUT = "compressor_output"
CONF_COMFORT_SENSE_VALUE = "comfort_sense_value"
CONF_DISPLAY_TOGGLE = "display_toggle"
CONF_TURBO_ON = "turbo_on"
CONF_TURBO_OFF = "turbo_off"
CONF_SWING_ON = "swing_on"
CONF_SWING_OFF = "swing_off"
CONF_VANE_STEP = "vane_step"
CONF_VANE_STEP_DURATION = "vane_step_duration"
CONF_DEBUG = "debug"
CONF_DEBUG_ACTIVE_IR_MODE = "debug_active_ir_mode"
CONF_DEBUG_LAST_ACTIVE_MODE = "debug_last_active_mode"
CONF_DEBUG_KICK_PHASE = "debug_kick_phase"
CONF_DEBUG_GEAR_DIFF = "debug_gear_diff"
CONF_DEBUG_TIME_IN_GEAR = "debug_time_in_gear"
CONF_DEBUG_IDLE_DURATION = "debug_idle_duration"
CONF_DEBUG_MODE_SWITCH_COOLDOWN = "debug_mode_switch_cooldown"
CONF_DEBUG_FAN_CLAMP_REMAINING = "debug_fan_clamp_remaining"
CONF_DEBUG_HEATER_LOCKED_OUT = "debug_heater_locked_out"
CONF_DEBUG_FAILSAFE_ACTIVE = "debug_failsafe_active"
CONF_DEBUG_BOOT_READY = "debug_boot_ready"
CONF_DEBUG_ADAPTIVE_BIAS_C = "debug_adaptive_bias_c"
CONF_DEBUG_ROOM_DRIFT = "debug_room_drift"
CONF_DEBUG_FAN_FEEDFORWARD = "debug_fan_feedforward"

# (config_key, setter_name)
DEBUG_SENSOR_MAP = [
    (CONF_DEBUG_ACTIVE_IR_MODE, "set_debug_active_ir_mode_sensor"),
    (CONF_DEBUG_LAST_ACTIVE_MODE, "set_debug_last_active_mode_sensor"),
    (CONF_DEBUG_KICK_PHASE, "set_debug_kick_phase_sensor"),
    (CONF_DEBUG_GEAR_DIFF, "set_debug_gear_diff_sensor"),
    (CONF_DEBUG_TIME_IN_GEAR, "set_debug_time_in_gear_sensor"),
    (CONF_DEBUG_IDLE_DURATION, "set_debug_idle_duration_sensor"),
    (CONF_DEBUG_MODE_SWITCH_COOLDOWN, "set_debug_mode_switch_cooldown_sensor"),
    (CONF_DEBUG_FAN_CLAMP_REMAINING, "set_debug_fan_clamp_remaining_sensor"),
    (CONF_DEBUG_HEATER_LOCKED_OUT, "set_debug_heater_locked_out_sensor"),
    (CONF_DEBUG_FAILSAFE_ACTIVE, "set_debug_failsafe_active_sensor"),
    (CONF_DEBUG_BOOT_READY, "set_debug_boot_ready_sensor"),
    (CONF_DEBUG_ADAPTIVE_BIAS_C, "set_debug_adaptive_bias_c_sensor"),
    (CONF_DEBUG_ROOM_DRIFT, "set_debug_room_drift_sensor"),
    (CONF_DEBUG_FAN_FEEDFORWARD, "set_debug_fan_feedforward_sensor"),
]

_DEBUG_SENSOR = sensor.sensor_schema(
    accuracy_decimals=0,
    entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
)
_DEBUG_SENSOR_C = sensor.sensor_schema(
    accuracy_decimals=2,
    unit_of_measurement=UNIT_CELSIUS,
    entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
)
_DEBUG_SENSOR_S = sensor.sensor_schema(
    accuracy_decimals=0,
    unit_of_measurement=UNIT_SECOND,
    entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
)
_DEBUG_SENSOR_DRIFT = sensor.sensor_schema(
    accuracy_decimals=3,
    unit_of_measurement="°C/min",
    entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
)

_DEBUG_SCHEMAS = {
    CONF_DEBUG_ACTIVE_IR_MODE: _DEBUG_SENSOR,
    CONF_DEBUG_LAST_ACTIVE_MODE: _DEBUG_SENSOR,
    CONF_DEBUG_KICK_PHASE: _DEBUG_SENSOR,
    CONF_DEBUG_GEAR_DIFF: _DEBUG_SENSOR_C,
    CONF_DEBUG_TIME_IN_GEAR: _DEBUG_SENSOR_S,
    CONF_DEBUG_IDLE_DURATION: _DEBUG_SENSOR_S,
    CONF_DEBUG_MODE_SWITCH_COOLDOWN: _DEBUG_SENSOR_S,
    CONF_DEBUG_FAN_CLAMP_REMAINING: _DEBUG_SENSOR_S,
    CONF_DEBUG_HEATER_LOCKED_OUT: _DEBUG_SENSOR,
    CONF_DEBUG_FAILSAFE_ACTIVE: _DEBUG_SENSOR,
    CONF_DEBUG_BOOT_READY: _DEBUG_SENSOR,
    CONF_DEBUG_ADAPTIVE_BIAS_C: _DEBUG_SENSOR_C,
    CONF_DEBUG_ROOM_DRIFT: _DEBUG_SENSOR_DRIFT,
    CONF_DEBUG_FAN_FEEDFORWARD: _DEBUG_SENSOR,
}


def _auto_debug_sensors(config):
    """When debug: true, auto-populate any missing debug sensor configs."""
    if config.get(CONF_DEBUG):
        for key, _ in DEBUG_SENSOR_MAP:
            if key not in config:
                human_name = key.replace("debug_", "Debug ").replace("_", " ").title()
                schema = _DEBUG_SCHEMAS[key]
                # Build a complete config dict with all required fields,
                # then validate through the schema for defaults/normalization
                base = {
                    "name": human_name,
                    "disabled_by_default": False,
                }
                config[key] = schema(base)
    return config


def _validate_vent_pairs(config):
    """A mode's timed vane positioning needs BOTH delay and interval, or neither;
    and a configured value must be > 0 (the C++ treats 0 as 'unset', which would
    silently disable the mode and surprise the user)."""
    for delay, interval, mode in [
        (CONF_HEAT_VENT_MOVE_DELAY, CONF_HEAT_VENT_INTERVAL, "heat"),
        (CONF_COOL_VENT_MOVE_DELAY, CONF_COOL_VENT_INTERVAL, "cool"),
    ]:
        if (delay in config) != (interval in config):
            raise cv.Invalid(
                f"Timed vane positioning for {mode} needs BOTH '{delay}' and "
                f"'{interval}' set (or neither)."
            )
        for key in (delay, interval):
            if key in config and config[key].total_milliseconds == 0:
                raise cv.Invalid(f"'{key}' must be greater than 0.")
    return config


# ── Configurable ladders + quirks ───────────────────────────────────────────
# One gear row: which gear number, and its CS offset (°C) from the setpoint anchor.
GEAR_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_GEAR): cv.int_range(min=0, max=3),
        cv.Required(CONF_CS_OFFSET): cv.int_range(min=-15, max=15),
    }
)

# One quirk: a path-dependent transition maneuver. On a `mode` gear change from
# `from_gear`→`to_gear`, hold `via_offset` CS for `duration` (or the global
# quirk_duration if unset), then re-evaluate. from_gear -1 = OFF, 0 = idle.
QUIRK_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_QUIRK_MODE): cv.one_of("cool", "heat", lower=True),
        cv.Required(CONF_QUIRK_FROM): cv.int_range(min=-1, max=3),
        cv.Required(CONF_QUIRK_TO): cv.int_range(min=0, max=3),
        cv.Required(CONF_QUIRK_VIA_OFFSET): cv.int_range(min=-15, max=15),
        cv.Optional(CONF_QUIRK_DURATION): cv.positive_time_period_milliseconds,
    }
)

# Ladder = gear-selection thresholds (diff = room − target, °C). Modulation rungs
# (1↔2, 2↔3, …) are auto-built from spacing S: up-trip(n→n+1) = n·S, down-trip =
# n·S − hysteresis. start (0→1), stop (1→0), idle (0→−1) are pinned off-grid.
# Signs: cool positive-is-hot, heat negative-is-cold (caller supplies signed pins).
LADDER_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_MODULATION_SPACING, default=0.55): cv.float_,
        cv.Optional(CONF_MODULATION_HYSTERESIS, default=0.0): cv.float_,
        cv.Required(CONF_LADDER_START): cv.float_,
        cv.Required(CONF_LADDER_STOP): cv.float_,
        cv.Required(CONF_LADDER_IDLE): cv.float_,
    }
)

# Defaults reproduce the shipped behavior bit-identically.
_DEFAULT_COOL_GEARS = [
    {CONF_GEAR: 0, CONF_CS_OFFSET: -5},  # idle (raw, unclamped)
    {CONF_GEAR: 1, CONF_CS_OFFSET: -2},  # LOW
    {CONF_GEAR: 2, CONF_CS_OFFSET: 0},   # MED
    {CONF_GEAR: 3, CONF_CS_OFFSET: 3},   # MAX
]
_DEFAULT_HEAT_GEARS = [
    {CONF_GEAR: 0, CONF_CS_OFFSET: 5},   # idle (raw, unclamped)
    {CONF_GEAR: 1, CONF_CS_OFFSET: 1},
    {CONF_GEAR: 2, CONF_CS_OFFSET: 0},
    {CONF_GEAR: 3, CONF_CS_OFFSET: -1},
]
_DEFAULT_COOL_LADDER = {
    CONF_MODULATION_SPACING: 0.55,
    CONF_MODULATION_HYSTERESIS: 0.0,
    CONF_LADDER_START: 0.35,
    CONF_LADDER_STOP: 0.15,
    CONF_LADDER_IDLE: -0.30,
}
_DEFAULT_HEAT_LADDER = {
    CONF_MODULATION_SPACING: 0.55,
    CONF_MODULATION_HYSTERESIS: 0.0,
    CONF_LADDER_START: -0.35,
    CONF_LADDER_STOP: -0.15,
    CONF_LADDER_IDLE: 0.30,
}
# Cool-only live quirks:
#   idle→LOW (0→1) via SP+0, held 90s: the from-idle restart threshold is SP+0; the 90s hold
#     (explicit per-quirk override, preserves the pre-refactor IDLE_KICK_HOLD_MS behavior) keeps it
#     asserted across the compressor's ~3-min anti-short-cycle lockout, then settles to LOW's SP−2.
#   MAX→MED (3→2) via SP−1, held the global quirk_duration (60s): MED is path-dependent (~9A from
#     MAX vs ~6A from LOW), so dip below MED to re-seat it from underneath (2026-07-09).
_DEFAULT_QUIRKS = [
    {CONF_QUIRK_MODE: "cool", CONF_QUIRK_FROM: 0, CONF_QUIRK_TO: 1, CONF_QUIRK_VIA_OFFSET: 0,
     CONF_QUIRK_DURATION: "90s"},
    {CONF_QUIRK_MODE: "cool", CONF_QUIRK_FROM: 3, CONF_QUIRK_TO: 2, CONF_QUIRK_VIA_OFFSET: -1},
]


def _validate_gears(key):
    """Each gear number appears once; gear 0 (idle) is required."""
    def validator(value):
        seen = set()
        for row in value:
            g = row[CONF_GEAR]
            if g in seen:
                raise cv.Invalid(f"{key}: gear {g} listed more than once")
            seen.add(g)
        if 0 not in seen:
            raise cv.Invalid(f"{key}: must define gear 0 (idle) offset")
        return value
    return validator


CONFIG_SCHEMA = cv.All(
    climate.climate_schema(FurrionChillCube)
    .extend(
        {
            cv.Required(CONF_TRANSMITTER_ID): cv.use_id(
                remote_transmitter.RemoteTransmitterComponent
            ),
            cv.Required(CONF_INSIDE_TEMPERATURE): cv.use_id(sensor.Sensor),
            cv.Optional(CONF_INSIDE_TEMPERATURE_IS_FAHRENHEIT, default=False): cv.boolean,
            cv.Optional(CONF_OUTSIDE_TEMPERATURE): cv.use_id(sensor.Sensor),
            cv.Optional(CONF_OUTSIDE_TEMPERATURE_IS_FAHRENHEIT, default=False): cv.boolean,
            # Below this outside temp (°F), heating is locked out. Default 35°F ≈ 1.7°C
            cv.Optional(CONF_OUTSIDE_LOCKOUT_TEMP, default=35.0): cv.float_range(
                min=-40.0, max=120.0
            ),
            # Mode switch tunables (all optional with sensible defaults)
            # Minutes the compressor must be at idle (gear 0) before allowing mode switch
            cv.Optional(CONF_MODE_SWITCH_IDLE_MIN, default=10): cv.int_range(min=1, max=60),
            # Minutes since last fresh start before another mode change is allowed
            cv.Optional(CONF_MODE_SWITCH_EVENT_MIN, default=20): cv.int_range(min=1, max=120),
            # Room must be this far past the setpoint before 0→-1. Accepts "1F", "0.56C", or bare = °C
            cv.Optional(CONF_MODE_SWITCH_TEMP_OFFSET, default="1F"): validate_mode_switch_temp_offset,
            # Off-dwell enforced on EVERY heat↔cool transition (direct user flip AND
            # natural HEAT_COOL handoff) and any re-engage from -1. ESPHome time format
            # (e.g. "60s", "90s", "1min"). Default 60s.
            cv.Optional(CONF_MODE_SWITCH_OFF, default="60s"): cv.positive_time_period_milliseconds,
            # Timed vane positioning (optional, no defaults → feature off if unset). On an
            # OFF→active start, wait <move_delay> then run the vane <interval> and stop, landing
            # it at a fixed mode-specific position. A mode runs only if BOTH its values are set.
            cv.Optional(CONF_HEAT_VENT_MOVE_DELAY): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_HEAT_VENT_INTERVAL): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_COOL_VENT_MOVE_DELAY): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_COOL_VENT_INTERVAL): cv.positive_time_period_milliseconds,
            # Manual vane-step nudge: each press of the vane_step button pulses the swing
            # ON then OFF after this duration, for a uniform incremental move. Default 0.5s.
            cv.Optional(CONF_VANE_STEP_DURATION, default="0.5s"): cv.positive_time_period_milliseconds,
            # Configurable gear CS-offset tables (gear → °C offset from setpoint).
            cv.Optional(CONF_COOL_GEARS, default=_DEFAULT_COOL_GEARS): cv.All(
                cv.ensure_list(GEAR_SCHEMA), _validate_gears("cool_gears")
            ),
            cv.Optional(CONF_HEAT_GEARS, default=_DEFAULT_HEAT_GEARS): cv.All(
                cv.ensure_list(GEAR_SCHEMA), _validate_gears("heat_gears")
            ),
            # Gear-selection ladders (auto-built from spacing + pinned start/stop/idle).
            cv.Optional(CONF_COOL_LADDER, default=_DEFAULT_COOL_LADDER): LADDER_SCHEMA,
            cv.Optional(CONF_HEAT_LADDER, default=_DEFAULT_HEAT_LADDER): LADDER_SCHEMA,
            # Path-dependent transition quirks (see QUIRK_SCHEMA).
            cv.Optional(CONF_QUIRKS, default=_DEFAULT_QUIRKS): cv.ensure_list(QUIRK_SCHEMA),
            # Uniform quirk hold (per-quirk `duration` overrides). Default 60s.
            cv.Optional(CONF_QUIRK_DURATION_DEFAULT, default="60s"): cv.positive_time_period_milliseconds,
            # CS re-transmit cadence: normal heartbeat + denser re-assert during a quirk.
            cv.Optional(CONF_CS_TRANSMIT_INTERVAL, default="10s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_QUIRK_TRANSMIT_INTERVAL, default="5s"): cv.positive_time_period_milliseconds,
            # Fan feedforward scale: °C of eff_diff per fan-gear (adaptive cool). NOTE:
            # historically tied to modulation spacing; left at 0.25 for behavior-parity.
            cv.Optional(CONF_GEAR_STEP_C, default=0.25): cv.float_range(min=0.0, max=2.0),
            # Target temperature encoding sent to the unit via Toshiba IR.
            # true (default) = Fahrenheit (RAC-PT1411HWRU F-protocol, 60-86°F,
            #   FAH flag set, unit panel displays °F).
            # false          = Celsius (RAC-PT1411HWRU C-protocol, 16-30°C,
            #   unit panel displays °C).
            # NOTE: This only affects the on-the-wire target byte and what the
            # unit's own display shows — gear-CS math is unchanged because it
            # anchors on the °C-rounded HA target (furrion_setpoint_c_), which
            # tracks the unit's internal target regardless of the F/C protocol
            # used to transmit it.
            cv.Optional(CONF_USE_FAHRENHEIT, default=True): cv.boolean,
            # Bench test harness: when true, loop() is inert and the unit is driven only by the
            # test_* hooks (from a YAML sequencer). Production controller suspended. Default false.
            cv.Optional(CONF_TEST_MODE, default=False): cv.boolean,
            # Phase 2 adaptive equilibrium-gear controller (cool mode).
            # When enabled, a slow integral floats the cool ladder's operating point to the
            # gear that sustains the current (unobservable-from-outside) load, so the room
            # holds setpoint with a gentle ±1-gear cycle on any day instead of swinging.
            # false (default) = identical to the static ladder. See PHASE2_ADAPTIVE_DESIGN.md.
            cv.Optional(CONF_ADAPTIVE_ENABLE, default=False): cv.boolean,
            # Optional observed-disturbance input: the CO2/fresh-air vent fan. While ON, a
            # fixed feedforward biases cooling up and the integral is frozen across the edge.
            cv.Optional(CONF_VENT_FAN): cv.use_id(binary_sensor.BinarySensor),
            # Gear-equivalents of feedforward applied while the vent fan runs (field-tuned).
            cv.Optional(CONF_FAN_FEEDFORWARD_GEARS, default=1): cv.int_range(min=0, max=3),
            # Diagnostic sensors (optional)
            cv.Optional(CONF_HEAT_GEAR): sensor.sensor_schema(
                accuracy_decimals=0,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_COOL_GEAR): sensor.sensor_schema(
                accuracy_decimals=0,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_COMPRESSOR_OUTPUT): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT,
                accuracy_decimals=1,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_COMFORT_SENSE_VALUE): sensor.sensor_schema(
                accuracy_decimals=0,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            # Debug mode: auto-creates diagnostic sensors for internal state
            cv.Optional(CONF_DEBUG, default=False): cv.boolean,
            # Debug sensors (auto-populated when debug: true, or add individually)
            cv.Optional(CONF_DEBUG_ACTIVE_IR_MODE): _DEBUG_SENSOR,
            cv.Optional(CONF_DEBUG_LAST_ACTIVE_MODE): _DEBUG_SENSOR,
            cv.Optional(CONF_DEBUG_KICK_PHASE): _DEBUG_SENSOR,
            cv.Optional(CONF_DEBUG_GEAR_DIFF): _DEBUG_SENSOR_C,
            cv.Optional(CONF_DEBUG_TIME_IN_GEAR): _DEBUG_SENSOR_S,
            cv.Optional(CONF_DEBUG_IDLE_DURATION): _DEBUG_SENSOR_S,
            cv.Optional(CONF_DEBUG_MODE_SWITCH_COOLDOWN): _DEBUG_SENSOR_S,
            cv.Optional(CONF_DEBUG_FAN_CLAMP_REMAINING): _DEBUG_SENSOR_S,
            cv.Optional(CONF_DEBUG_HEATER_LOCKED_OUT): _DEBUG_SENSOR,
            cv.Optional(CONF_DEBUG_FAILSAFE_ACTIVE): _DEBUG_SENSOR,
            cv.Optional(CONF_DEBUG_BOOT_READY): _DEBUG_SENSOR,
            cv.Optional(CONF_DEBUG_ADAPTIVE_BIAS_C): _DEBUG_SENSOR_C,
            cv.Optional(CONF_DEBUG_ROOM_DRIFT): _DEBUG_SENSOR_DRIFT,
            cv.Optional(CONF_DEBUG_FAN_FEEDFORWARD): _DEBUG_SENSOR,
            # Buttons (optional)
            cv.Optional(CONF_DISPLAY_TOGGLE): button.button_schema(
                DisplayToggleButton,
                entity_category=ENTITY_CATEGORY_CONFIG,
            ),
            cv.Optional(CONF_TURBO_ON): button.button_schema(
                TurboOnButton,
                entity_category=ENTITY_CATEGORY_CONFIG,
            ),
            cv.Optional(CONF_TURBO_OFF): button.button_schema(
                TurboOffButton,
                entity_category=ENTITY_CATEGORY_CONFIG,
            ),
            cv.Optional(CONF_SWING_ON): button.button_schema(
                SwingOnButton,
                entity_category=ENTITY_CATEGORY_CONFIG,
            ),
            cv.Optional(CONF_SWING_OFF): button.button_schema(
                SwingOffButton,
                entity_category=ENTITY_CATEGORY_CONFIG,
            ),
            cv.Optional(CONF_VANE_STEP): button.button_schema(
                VaneStepButton,
                entity_category=ENTITY_CATEGORY_CONFIG,
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA),
    _auto_debug_sensors,
    _validate_vent_pairs,
)


async def to_code(config):
    var = await climate.new_climate(config)
    await cg.register_component(var, config)

    # Remote transmitter
    transmitter = await cg.get_variable(config[CONF_TRANSMITTER_ID])
    cg.add(var.set_transmitter(transmitter))

    # Inside temperature sensor (required)
    sens = await cg.get_variable(config[CONF_INSIDE_TEMPERATURE])
    cg.add(var.set_inside_temperature_sensor(sens))
    cg.add(var.set_inside_temperature_fahrenheit(config[CONF_INSIDE_TEMPERATURE_IS_FAHRENHEIT]))

    # Outside temperature sensor (optional)
    if CONF_OUTSIDE_TEMPERATURE in config:
        sens = await cg.get_variable(config[CONF_OUTSIDE_TEMPERATURE])
        cg.add(var.set_outside_temperature_sensor(sens))
        cg.add(
            var.set_outside_temperature_fahrenheit(config[CONF_OUTSIDE_TEMPERATURE_IS_FAHRENHEIT])
        )

    # Configuration values (in Fahrenheit, converted to Celsius internally)
    cg.add(var.set_outside_lockout_temp(config[CONF_OUTSIDE_LOCKOUT_TEMP]))
    cg.add(var.set_mode_switch_idle_min(config[CONF_MODE_SWITCH_IDLE_MIN]))
    cg.add(var.set_mode_switch_event_min(config[CONF_MODE_SWITCH_EVENT_MIN]))
    cg.add(var.set_mode_switch_temp_offset(config[CONF_MODE_SWITCH_TEMP_OFFSET]))
    cg.add(var.set_mode_switch_off_ms(config[CONF_MODE_SWITCH_OFF].total_milliseconds))
    cg.add(var.set_use_fahrenheit(config[CONF_USE_FAHRENHEIT]))
    cg.add(var.set_test_mode(config[CONF_TEST_MODE]))

    # Configurable gear CS-offset tables
    for row in config[CONF_COOL_GEARS]:
        cg.add(var.add_cool_gear(row[CONF_GEAR], row[CONF_CS_OFFSET]))
    for row in config[CONF_HEAT_GEARS]:
        cg.add(var.add_heat_gear(row[CONF_GEAR], row[CONF_CS_OFFSET]))

    # Gear-selection ladders (spacing + pinned start/stop/idle)
    cool_l = config[CONF_COOL_LADDER]
    cg.add(
        var.set_cool_ladder(
            cool_l[CONF_MODULATION_SPACING],
            cool_l[CONF_MODULATION_HYSTERESIS],
            cool_l[CONF_LADDER_START],
            cool_l[CONF_LADDER_STOP],
            cool_l[CONF_LADDER_IDLE],
        )
    )
    heat_l = config[CONF_HEAT_LADDER]
    cg.add(
        var.set_heat_ladder(
            heat_l[CONF_MODULATION_SPACING],
            heat_l[CONF_MODULATION_HYSTERESIS],
            heat_l[CONF_LADDER_START],
            heat_l[CONF_LADDER_STOP],
            heat_l[CONF_LADDER_IDLE],
        )
    )

    # Transition quirks (per-quirk duration 0 = use global default)
    default_quirk_ms = config[CONF_QUIRK_DURATION_DEFAULT].total_milliseconds
    for q in config[CONF_QUIRKS]:
        dur = q[CONF_QUIRK_DURATION].total_milliseconds if CONF_QUIRK_DURATION in q else 0
        cg.add(
            var.add_quirk(
                q[CONF_QUIRK_MODE] == "heat",
                q[CONF_QUIRK_FROM],
                q[CONF_QUIRK_TO],
                q[CONF_QUIRK_VIA_OFFSET],
                dur,
            )
        )
    cg.add(var.set_quirk_duration_ms(default_quirk_ms))
    cg.add(var.set_cs_transmit_interval_ms(config[CONF_CS_TRANSMIT_INTERVAL].total_milliseconds))
    cg.add(var.set_quirk_transmit_interval_ms(config[CONF_QUIRK_TRANSMIT_INTERVAL].total_milliseconds))
    cg.add(var.set_gear_step_c(config[CONF_GEAR_STEP_C]))

    # Timed vane positioning (optional; only plumbed when set)
    for key, setter in [
        (CONF_HEAT_VENT_MOVE_DELAY, "set_heat_vent_move_delay_ms"),
        (CONF_HEAT_VENT_INTERVAL, "set_heat_vent_interval_ms"),
        (CONF_COOL_VENT_MOVE_DELAY, "set_cool_vent_move_delay_ms"),
        (CONF_COOL_VENT_INTERVAL, "set_cool_vent_interval_ms"),
    ]:
        if key in config:
            cg.add(getattr(var, setter)(config[key].total_milliseconds))
    cg.add(var.set_vane_step_duration_ms(config[CONF_VANE_STEP_DURATION].total_milliseconds))

    # Phase 2 adaptive equilibrium-gear controller (cool mode)
    cg.add(var.set_adaptive_enable(config[CONF_ADAPTIVE_ENABLE]))
    cg.add(var.set_fan_feedforward_gears(config[CONF_FAN_FEEDFORWARD_GEARS]))
    if CONF_VENT_FAN in config:
        fan = await cg.get_variable(config[CONF_VENT_FAN])
        cg.add(var.set_vent_fan_sensor(fan))

    # Diagnostic sensors
    for key, setter in [
        (CONF_HEAT_GEAR, "set_heat_gear_sensor"),
        (CONF_COOL_GEAR, "set_cool_gear_sensor"),
        (CONF_COMPRESSOR_OUTPUT, "set_compressor_output_sensor"),
        (CONF_COMFORT_SENSE_VALUE, "set_comfort_sense_sensor"),
    ]:
        if key in config:
            s = await sensor.new_sensor(config[key])
            cg.add(getattr(var, setter)(s))

    # Buttons
    for key in [
        CONF_DISPLAY_TOGGLE,
        CONF_TURBO_ON,
        CONF_TURBO_OFF,
        CONF_SWING_ON,
        CONF_SWING_OFF,
        CONF_VANE_STEP,
    ]:
        if key in config:
            btn = await button.new_button(config[key])
            await cg.register_parented(btn, config[CONF_ID])

    # Debug sensors (auto-populated by _auto_debug_sensors when debug: true)
    for key, setter in DEBUG_SENSOR_MAP:
        if key in config:
            s = await sensor.new_sensor(config[key])
            cg.add(getattr(var, setter)(s))
