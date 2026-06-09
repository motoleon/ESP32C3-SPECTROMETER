import esphome.codegen as cg
from esphome.components import sensor

fft3bands_ns = cg.esphome_ns.namespace("fft3bands")
FFTSensor = fft3bands_ns.class_("FFTSensor", sensor.Sensor, cg.PollingComponent)