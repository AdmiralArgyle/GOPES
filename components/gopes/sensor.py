"""GOPES sensor platform implementation."""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    CONF_TRIGGER_ID,
    CONF_NAME,
    DEVICE_CLASS_DISTANCE,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_HUMIDITY,
    DEVICE_CLASS_PRESSURE,
    DEVICE_CLASS_ILLUMINANCE,
)
from . import gopes_ns, GOPESSensor
from .const import (
    CONF_MOVING_DISTANCE,
    CONF_STILL_DISTANCE,
    CONF_DETECTION_DISTANCE,
    CONF_MOVING_ENERGY,
    CONF_STILL_ENERGY,
)

DEPENDENCIES = ['gopes']

# Sensor platform schema
CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(GOPESSensor),
    cv.Optional(CONF_MOVING_DISTANCE): sensor.sensor_schema(
        unit_of_measurement="m",
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_DISTANCE
    ),
    cv.Optional(CONF_STILL_DISTANCE): sensor.sensor_schema(
        unit_of_measurement="m",
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_DISTANCE
    ),
    cv.Optional(CONF_DETECTION_DISTANCE): sensor.sensor_schema(
        unit_of_measurement="m",
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_DISTANCE
    ),
    cv.Optional(CONF_MOVING_ENERGY): sensor.sensor_schema(
        unit_of_measurement="dB",
        accuracy_decimals=0,
        device_class=DEVICE_CLASS_ENERGY
    ),
    cv.Optional(CONF_STILL_ENERGY): sensor.sensor_schema(
        unit_of_measurement="dB",
        accuracy_decimals=0,
        device_class=DEVICE_CLASS_ENERGY
    ),
    cv.Optional("temperature"): sensor.sensor_schema(
        unit_of_measurement="°C",
        accuracy_decimals=1,
        device_class=DEVICE_CLASS_TEMPERATURE
    ),
    cv.Optional("humidity"): sensor.sensor_schema(
        unit_of_measurement="%",
        accuracy_decimals=1,
        device_class=DEVICE_CLASS_HUMIDITY
    ),
    cv.Optional("pressure"): sensor.sensor_schema(
        unit_of_measurement="hPa",
        accuracy_decimals=1,
        device_class=DEVICE_CLASS_PRESSURE
    ),
    cv.Optional("illuminance"): sensor.sensor_schema(
        unit_of_measurement="lx",
        accuracy_decimals=1,
        device_class=DEVICE_CLASS_ILLUMINANCE
    ),
})

async def to_code(config):
    """Register the sensor platform."""
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    if CONF_MOVING_DISTANCE in config:
        sens = await sensor.new_sensor(config[CONF_MOVING_DISTANCE])
        cg.add(var.set_moving_distance_sensor(sens))
        
    if CONF_STILL_DISTANCE in config:
        sens = await sensor.new_sensor(config[CONF_STILL_DISTANCE])
        cg.add(var.set_still_distance_sensor(sens))
        
    if CONF_DETECTION_DISTANCE in config:
        sens = await sensor.new_sensor(config[CONF_DETECTION_DISTANCE])
        cg.add(var.set_detection_distance_sensor(sens))
        
    if CONF_MOVING_ENERGY in config:
        sens = await sensor.new_sensor(config[CONF_MOVING_ENERGY])
        cg.add(var.set_moving_energy_sensor(sens))
        
    if CONF_STILL_ENERGY in config:
        sens = await sensor.new_sensor(config[CONF_STILL_ENERGY])
        cg.add(var.set_still_energy_sensor(sens))

    if "temperature" in config:
        sens = await sensor.new_sensor(config["temperature"])
        cg.add(var.set_temperature_sensor(sens))

    if "humidity" in config:
        sens = await sensor.new_sensor(config["humidity"])
        cg.add(var.set_humidity_sensor(sens))

    if "pressure" in config:
        sens = await sensor.new_sensor(config["pressure"])
        cg.add(var.set_pressure_sensor(sens))

    if "illuminance" in config:
        sens = await sensor.new_sensor(config["illuminance"])
        cg.add(var.set_illuminance_sensor(sens))