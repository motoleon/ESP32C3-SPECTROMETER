import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    CONF_UPDATE_INTERVAL,
    UNIT_EMPTY,
    STATE_CLASS_MEASUREMENT,
)
from esphome import pins

from . import FFTSensor

CONF_SAMPLE_RATE = "sample_rate"
CONF_WS_PIN = "ws_pin"
CONF_SCK_PIN = "sck_pin"
CONF_SD_PIN = "sd_pin"
CONF_BAND_FREQUENCIES = "band_frequencies"
CONF_FFT_SIZE = "fft_size"
CONF_GANANCIA = "Ganancia"

VALID_FFT_SIZES = [128, 256, 512, 1024]

CONFIG_SCHEMA = sensor.sensor_schema(
    FFTSensor,
    unit_of_measurement=UNIT_EMPTY,
    icon="mdi:chart-bar",
    accuracy_decimals=2,
    state_class=STATE_CLASS_MEASUREMENT,
).extend(
    {
        cv.GenerateID(): cv.declare_id(FFTSensor),
        cv.Optional(CONF_SAMPLE_RATE, default=22050): cv.int_range(min=8000, max=48000),
        cv.Optional(CONF_FFT_SIZE, default=256): cv.one_of(*VALID_FFT_SIZES, int=True),
        cv.Optional(CONF_GANANCIA, default=200): cv.int_range(min=1, max=999999999),
        cv.Required(CONF_WS_PIN): pins.internal_gpio_output_pin_number,
        cv.Required(CONF_SCK_PIN): pins.internal_gpio_output_pin_number,
        cv.Required(CONF_SD_PIN): pins.internal_gpio_input_pin_number,
        cv.Optional(CONF_BAND_FREQUENCIES, default=[100, 1000, 5000, 10000]): cv.All(
            cv.ensure_list(cv.positive_float),
            cv.Length(min=1, max=32),
        ),
        cv.Optional(CONF_UPDATE_INTERVAL, default="120ms"): cv.update_interval,
    }
).extend(cv.polling_component_schema("120ms"))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await sensor.register_sensor(var, config)

    cg.add(var.set_sample_rate(config[CONF_SAMPLE_RATE]))
    cg.add(var.set_fft_size(config[CONF_FFT_SIZE]))
    cg.add(var.set_Ganancia(config[CONF_GANANCIA]))
    cg.add(var.set_ws_pin(config[CONF_WS_PIN]))
    cg.add(var.set_sck_pin(config[CONF_SCK_PIN]))
    cg.add(var.set_sd_pin(config[CONF_SD_PIN]))

    freqs = cg.ArrayInitializer(*config[CONF_BAND_FREQUENCIES])
    cg.add(var.set_band_frequencies(freqs))
