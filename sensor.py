import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
)

piscinetemp_ns = cg.esphome_ns.namespace("piscine_temp")

PiscineTempSensor = piscinetemp_ns.class_(
    "PiscineTempSensor",
    cg.Component,
    sensor.Sensor,
)

CONFIG_SCHEMA = (
    sensor.sensor_schema(
        PiscineTempSensor,
        unit_of_measurement=UNIT_CELSIUS,
        accuracy_decimals=1,
        device_class=DEVICE_CLASS_TEMPERATURE,
        state_class=STATE_CLASS_MEASUREMENT,
    )
)

async def to_code(config):
    var = await sensor.new_sensor(config)
    await cg.register_component(var, config)
    cg.add_library("dernasherbrezon/sx127x", '4.0.1')
