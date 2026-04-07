import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate, sensor, button, remote_transmitter
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
AUTO_LOAD = ["sensor", "button"]
CODEOWNERS = ["@srnoth"]

fcc_ns = cg.esphome_ns.namespace("furrion_chill_cube")
FurrionChillCube = fcc_ns.class_("FurrionChillCube", climate.Climate, cg.Component)

# Button classes
DisplayToggleButton = fcc_ns.class_("DisplayToggleButton", button.Button)
TurboOnButton = fcc_ns.class_("TurboOnButton", button.Button)
TurboOffButton = fcc_ns.class_("TurboOffButton", button.Button)
SwingOnButton = fcc_ns.class_("SwingOnButton", button.Button)
SwingOffButton = fcc_ns.class_("SwingOffButton", button.Button)

# Config keys
CONF_TRANSMITTER_ID = "transmitter_id"
CONF_INSIDE_TEMPERATURE = "inside_temperature"
CONF_INSIDE_TEMPERATURE_IS_FAHRENHEIT = "inside_temperature_is_fahrenheit"
CONF_OUTSIDE_TEMPERATURE = "outside_temperature"
CONF_OUTSIDE_TEMPERATURE_IS_FAHRENHEIT = "outside_temperature_is_fahrenheit"
CONF_HEAT_COOL_GAP = "heat_cool_gap"
CONF_OUTSIDE_LOCKOUT_TEMP = "outside_lockout_temp"
CONF_HEAT_GEAR = "heat_gear"
CONF_COOL_GEAR = "cool_gear"
CONF_COMPRESSOR_OUTPUT = "compressor_output"
CONF_COMFORT_SENSE_VALUE = "comfort_sense_value"
CONF_DISPLAY_TOGGLE = "display_toggle"
CONF_TURBO_ON = "turbo_on"
CONF_TURBO_OFF = "turbo_off"
CONF_SWING_ON = "swing_on"
CONF_SWING_OFF = "swing_off"
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
]


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
}


def _auto_debug_sensors(config):
    """When debug: true, auto-populate any missing debug sensor configs."""
    if config.get(CONF_DEBUG):
        for key, _ in DEBUG_SENSOR_MAP:
            if key not in config:
                # Validate empty dict through the sensor schema to generate ID + defaults
                config[key] = _DEBUG_SCHEMAS[key]({})
    return config

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
            # Gap between heat/cool setpoints, in °F (converted to °C internally)
            cv.Optional(CONF_HEAT_COOL_GAP, default=0.0): cv.float_range(
                min=0.0, max=20.0
            ),
            # Below this outside temp (°F), heating is locked out. Default 35°F ≈ 1.7°C
            cv.Optional(CONF_OUTSIDE_LOCKOUT_TEMP, default=35.0): cv.float_,
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
        }
    )
    .extend(cv.COMPONENT_SCHEMA),
    _auto_debug_sensors,
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
    cg.add(var.set_heat_cool_gap(config[CONF_HEAT_COOL_GAP]))
    cg.add(var.set_outside_lockout_temp(config[CONF_OUTSIDE_LOCKOUT_TEMP]))

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
    ]:
        if key in config:
            btn = await button.new_button(config[key])
            await cg.register_parented(btn, config[CONF_ID])

    # Debug sensors (auto-populated by _auto_debug_sensors when debug: true)
    for key, setter in DEBUG_SENSOR_MAP:
        if key in config:
            s = await sensor.new_sensor(config[key])
            cg.add(getattr(var, setter)(s))
