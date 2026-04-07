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

# Debug sensor definitions: (suffix, setter_name, unit, accuracy_decimals, icon)
DEBUG_SENSORS = [
    ("active_ir_mode", "set_debug_active_ir_mode_sensor", "", 0, "mdi:hvac"),
    ("last_active_mode", "set_debug_last_active_mode_sensor", "", 0, "mdi:swap-horizontal"),
    ("kick_phase", "set_debug_kick_phase_sensor", "", 0, "mdi:rocket-launch"),
    ("gear_diff", "set_debug_gear_diff_sensor", UNIT_CELSIUS, 2, "mdi:thermometer-lines"),
    ("time_in_gear", "set_debug_time_in_gear_sensor", UNIT_SECOND, 0, "mdi:timer"),
    ("idle_duration", "set_debug_idle_duration_sensor", UNIT_SECOND, 0, "mdi:timer-sand"),
    ("mode_switch_cooldown", "set_debug_mode_switch_cooldown_sensor", UNIT_SECOND, 0, "mdi:timer-lock"),
    ("fan_clamp_remaining", "set_debug_fan_clamp_remaining_sensor", UNIT_SECOND, 0, "mdi:fan-clock"),
    ("heater_locked_out", "set_debug_heater_locked_out_sensor", "", 0, "mdi:lock"),
    ("failsafe_active", "set_debug_failsafe_active_sensor", "", 0, "mdi:shield-alert"),
    ("boot_ready", "set_debug_boot_ready_sensor", "", 0, "mdi:check-circle"),
]

CONFIG_SCHEMA = (
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
    .extend(cv.COMPONENT_SCHEMA)
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

    # Debug sensors
    if config[CONF_DEBUG]:
        from esphome.core import ID

        parent_id = str(config[CONF_ID].id)
        for suffix, setter, unit, accuracy, icon in DEBUG_SENSORS:
            sid = ID(
                f"{parent_id}_debug_{suffix}",
                is_declaration=True,
                type=sensor.Sensor,
            )
            s = cg.new_Pvariable(sid)
            human_name = suffix.replace("_", " ").title()
            cg.add(s.set_name(f"Debug {human_name}"))
            cg.add(s.set_object_id(f"{parent_id}_debug_{suffix}"))
            if unit:
                cg.add(s.set_unit_of_measurement(unit))
            cg.add(s.set_accuracy_decimals(accuracy))
            cg.add(s.set_icon(icon))
            cg.add(
                s.set_entity_category(
                    cg.global_ns.ENTITY_CATEGORY_DIAGNOSTIC
                )
            )
            cg.add(cg.App.register_sensor(s))
            cg.add(getattr(var, setter)(s))
