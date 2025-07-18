from esphome import automation, pins
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate, uart
from esphome.const import (
    CONF_ID,
    CONF_LEVEL,
    CONF_BEEPER,
    CONF_VISUAL,
    CONF_MAX_TEMPERATURE,
    CONF_MIN_TEMPERATURE,
    CONF_SUPPORTED_MODES,
    CONF_TEMPERATURE_STEP,
    CONF_SUPPORTED_PRESETS,
    CONF_TARGET_TEMPERATURE,
    CONF_SUPPORTED_FAN_MODES,
    CONF_SUPPORTED_SWING_MODES,
)

from esphome.components.climate import (
    ClimateMode,
    ClimatePreset,
    ClimateSwingMode,
)

AUTO_LOAD = ["climate"]
CODEOWNERS = ["sav94rus"]
DEPENDENCIES = ["climate", "uart"]

TCLAC_MIN_TEMPERATURE = 16.0
TCLAC_MAX_TEMPERATURE = 31.0
TCLAC_TARGET_TEMPERATURE_STEP = 1.0

CONF_RX_LED = "rx_led"
CONF_TX_LED = "tx_led"
CONF_DISPLAY = "show_display"
CONF_FORCE_MODE = "force_mode"
CONF_VERTICAL_AIRFLOW = "vertical_airflow"
CONF_MODULE_DISPLAY = "show_module_display"
CONF_HORIZONTAL_AIRFLOW = "horizontal_airflow"
CONF_VERTICAL_SWING_MODE = "vertical_swing_mode"
CONF_HORIZONTAL_SWING_MODE = "horizontal_swing_mode"

tclac_ns = cg.esphome_ns.namespace("tclac")
tclacClimate = tclac_ns.class_("tclacClimate", uart.UARTDevice, climate.Climate, cg.PollingComponent)

SUPPORTED_FAN_MODES_OPTIONS = {
    "AUTO": ClimateMode.CLIMATE_FAN_AUTO,
    "QUIET": ClimateMode.CLIMATE_FAN_QUIET,
    "LOW": ClimateMode.CLIMATE_FAN_LOW,
    "MIDDLE": ClimateMode.CLIMATE_FAN_MIDDLE,
    "MEDIUM": ClimateMode.CLIMATE_FAN_MEDIUM,
    "HIGH": ClimateMode.CLIMATE_FAN_HIGH,
    "FOCUS": ClimateMode.CLIMATE_FAN_FOCUS,
    "DIFFUSE": ClimateMode.CLIMATE_FAN_DIFFUSE,
}

SUPPORTED_SWING_MODES_OPTIONS = {
    "OFF": ClimateSwingMode.CLIMATE_SWING_OFF,
    "VERTICAL": ClimateSwingMode.CLIMATE_SWING_VERTICAL,
    "HORIZONTAL": ClimateSwingMode.CLIMATE_SWING_HORIZONTAL,
    "BOTH": ClimateSwingMode.CLIMATE_SWING_BOTH,
}

SUPPORTED_CLIMATE_MODES_OPTIONS = {
    "OFF": ClimateMode.CLIMATE_MODE_OFF,
    "AUTO": ClimateMode.CLIMATE_MODE_AUTO,
    "COOL": ClimateMode.CLIMATE_MODE_COOL,
    "HEAT": ClimateMode.CLIMATE_MODE_HEAT,
    "DRY": ClimateMode.CLIMATE_MODE_DRY,
    "FAN_ONLY": ClimateMode.CLIMATE_MODE_FAN_ONLY,
}

SUPPORTED_CLIMATE_PRESETS_OPTIONS = {
    "NONE": ClimatePreset.CLIMATE_PRESET_NONE,
    "ECO": ClimatePreset.CLIMATE_PRESET_ECO,
    "SLEEP": ClimatePreset.CLIMATE_PRESET_SLEEP,
    "COMFORT": ClimatePreset.CLIMATE_PRESET_COMFORT,
}

VerticalSwingDirection = tclac_ns.enum("VerticalSwingDirection", True)
VERTICAL_SWING_DIRECTION_OPTIONS = {
    "UP_DOWN": VerticalSwingDirection.UPDOWN,
    "UPSIDE": VerticalSwingDirection.UPSIDE,
    "DOWNSIDE": VerticalSwingDirection.DOWNSIDE,
}

HorizontalSwingDirection = tclac_ns.enum("HorizontalSwingDirection", True)
HORIZONTAL_SWING_DIRECTION_OPTIONS = {
    "LEFT_RIGHT": HorizontalSwingDirection.LEFT_RIGHT,
    "LEFTSIDE": HorizontalSwingDirection.LEFTSIDE,
    "CENTER": HorizontalSwingDirection.CENTER,
    "RIGHTSIDE": HorizontalSwingDirection.RIGHTSIDE,
}

AirflowVerticalDirection = tclac_ns.enum("AirflowVerticalDirection", True)
AIRFLOW_VERTICAL_DIRECTION_OPTIONS = {
    "LAST": AirflowVerticalDirection.LAST,
    "MAX_UP": AirflowVerticalDirection.MAX_UP,
    "UP": AirflowVerticalDirection.UP,
    "CENTER": AirflowVerticalDirection.CENTER,
    "DOWN": AirflowVerticalDirection.DOWN,
    "MAX_DOWN": AirflowVerticalDirection.MAX_DOWN,
}

AirflowHorizontalDirection = tclac_ns.enum("AirflowHorizontalDirection", True)
AIRFLOW_HORIZONTAL_DIRECTION_OPTIONS = {
    "LAST": AirflowHorizontalDirection.LAST,
    "MAX_LEFT": AirflowHorizontalDirection.MAX_LEFT,
    "LEFT": AirflowHorizontalDirection.LEFT,
    "CENTER": AirflowHorizontalDirection.CENTER,
    "RIGHT": AirflowHorizontalDirection.RIGHT,
    "MAX_RIGHT": AirflowHorizontalDirection.MAX_RIGHT,
}

def validate_visual(config):
    if CONF_VISUAL in config:
        visual_config = config[CONF_VISUAL]
        if CONF_MIN_TEMPERATURE in visual_config:
            min_temp = visual_config[CONF_MIN_TEMPERATURE]
            if min_temp < TCLAC_MIN_TEMPERATURE:
                raise cv.Invalid(f"Минимальная температура {min_temp} ниже допустимой {TCLAC_MIN_TEMPERATURE}")
        else:
            config[CONF_VISUAL][CONF_MIN_TEMPERATURE] = TCLAC_MIN_TEMPERATURE
        if CONF_MAX_TEMPERATURE in visual_config:
            max_temp = visual_config[CONF_MAX_TEMPERATURE]
            if max_temp > TCLAC_MAX_TEMPERATURE:
                raise cv.Invalid(f"Максимальная температура {max_temp} выше допустимой {TCLAC_MAX_TEMPERATURE}")
        else:
            config[CONF_VISUAL][CONF_MAX_TEMPERATURE] = TCLAC_MAX_TEMPERATURE
        if CONF_TEMPERATURE_STEP in visual_config:
            temp_step = config[CONF_VISUAL][CONF_TEMPERATURE_STEP].get(CONF_TARGET_TEMPERATURE, TCLAC_TARGET_TEMPERATURE_STEP)
            if ((int)(temp_step * 2)) / 2 != temp_step:
                raise cv.Invalid(f"Шаг температуры {temp_step} некорректен, должен быть кратен 1")
        else:
            config[CONF_VISUAL][CONF_TEMPERATURE_STEP] = {
                CONF_TARGET_TEMPERATURE: TCLAC_TARGET_TEMPERATURE_STEP,
            }
    else:
        config[CONF_VISUAL] = {
            CONF_MIN_TEMPERATURE: TCLAC_MIN_TEMPERATURE,
            CONF_MAX_TEMPERATURE: TCLAC_MAX_TEMPERATURE,
            CONF_TEMPERATURE_STEP: {
                CONF_TARGET_TEMPERATURE: TCLAC_TARGET_TEMPERATURE_STEP,
            },
        }
    return config

CONFIG_SCHEMA = cv.All(
    climate.climate_schema({
        cv.GenerateID(): cv.declare_id(tclacClimate),
        cv.Optional(CONF_BEEPER, default=True): cv.boolean,
        cv.Optional(CONF_DISPLAY, default=True): cv.boolean,
        cv.Optional(CONF_RX_LED): pins.gpio_output_pin_schema,
        cv.Optional(CONF_TX_LED): pins.gpio_output_pin_schema,
        cv.Optional(CONF_FORCE_MODE, default=True): cv.boolean,
        cv.Optional(CONF_MODULE_DISPLAY, default=True): cv.boolean,
        cv.Optional(CONF_VERTICAL_AIRFLOW, default="CENTER"): cv.ensure_list(cv.enum(AIRFLOW_VERTICAL_DIRECTION_OPTIONS, upper=True)),
        cv.Optional(CONF_VERTICAL_SWING_MODE, default="UP_DOWN"): cv.ensure_list(cv.enum(VERTICAL_SWING_DIRECTION_OPTIONS, upper=True)),
        cv.Optional(CONF_HORIZONTAL_AIRFLOW, default="CENTER"): cv.ensure_list(cv.enum(AIRFLOW_HORIZONTAL_DIRECTION_OPTIONS, upper=True)),
        cv.Optional(CONF_HORIZONTAL_SWING_MODE, default="LEFT_RIGHT"): cv.ensure_list(cv.enum(HORIZONTAL_SWING_DIRECTION_OPTIONS, upper=True)),
        cv.Optional(CONF_SUPPORTED_PRESETS, default=["NONE", "ECO", "SLEEP", "COMFORT"]): cv.ensure_list(cv.enum(SUPPORTED_CLIMATE_PRESETS_OPTIONS, upper=True)),
        cv.Optional(CONF_SUPPORTED_SWING_MODES, default=["OFF", "VERTICAL", "HORIZONTAL", "BOTH"]): cv.ensure_list(cv.enum(SUPPORTED_SWING_MODES_OPTIONS, upper=True)),
        cv.Optional(CONF_SUPPORTED_MODES, default=["OFF", "AUTO", "COOL", "HEAT", "DRY", "FAN_ONLY"]): cv.ensure_list(cv.enum(SUPPORTED_CLIMATE_MODES_OPTIONS, upper=True)),
        cv.Optional(CONF_SUPPORTED_FAN_MODES, default=["AUTO", "QUIET", "LOW", "MIDDLE", "MEDIUM", "HIGH", "FOCUS", "DIFFUSE"]): cv.ensure_list(cv.enum(SUPPORTED_FAN_MODES_OPTIONS, upper=True)),
    }).extend(uart.UART_DEVICE_SCHEMA).extend(cv.COMPONENT_SCHEMA),
    validate_visual,
)
