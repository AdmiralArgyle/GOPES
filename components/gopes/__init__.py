"""GOPES component for ESPHome."""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, binary_sensor, uart
from esphome.const import (
    CONF_ID,
    CONF_TRIGGER_ID,
    CONF_NAME,
    DEVICE_CLASS_OCCUPANCY,
    DEVICE_CLASS_MOTION,
)

DEPENDENCIES = ['uart']
AUTO_LOAD = ['sensor', 'binary_sensor']

# Component namespace
gopes_ns = cg.esphome_ns.namespace('gopes')
GOPESComponent = gopes_ns.class_('GOPESComponent', cg.Component)
GOPESBinarySensor = gopes_ns.class_('GOPESBinarySensor', binary_sensor.BinarySensor)
GOPESSensor = gopes_ns.class_('GOPESSensor', sensor.Sensor)

# Configuration keys
CONF_LD2410 = "ld2410"
CONF_VL53L1X = "vl53l1x"
CONF_FUSION = "fusion"
CONF_MOVING_DISTANCE = "moving_distance"
CONF_STILL_DISTANCE = "still_distance"
CONF_DETECTION_DISTANCE = "detection_distance"
CONF_MOVING_ENERGY = "moving_energy"
CONF_STILL_ENERGY = "still_energy"
CONF_LOCATION_X = "location_x"
CONF_LOCATION_Y = "location_y"

# Extend existing configuration
CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(GOPESComponent),
    cv.Optional(CONF_LD2410): sensor.sensor_schema(
        GOPESSensor,
        unit_of_measurement="m",
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_OCCUPANCY,
    ),
    cv.Optional(CONF_VL53L1X): sensor.sensor_schema(
        GOPESSensor,
        unit_of_measurement="mm",
        accuracy_decimals=0,
        device_class=DEVICE_CLASS_MOTION,
    ),
    cv.Optional(CONF_FUSION): cv.Schema({
        cv.Optional(CONF_LOCATION_X): sensor.sensor_schema(),
        cv.Optional(CONF_LOCATION_Y): sensor.sensor_schema(),
    }),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    if CONF_LD2410 in config:
        sens = await sensor.new_sensor(config[CONF_LD2410])
        cg.add(var.set_ld2410_sensor(sens))

    if CONF_VL53L1X in config:
        sens = await sensor.new_sensor(config[CONF_VL53L1X])
        cg.add(var.set_vl53l1x_sensor(sens))

    if CONF_FUSION in config:
        fusion_config = config[CONF_FUSION]
        if CONF_LOCATION_X in fusion_config:
            sens = await sensor.new_sensor(fusion_config[CONF_LOCATION_X])
            cg.add(var.set_location_x_sensor(sens))
        if CONF_LOCATION_Y in fusion_config:
            sens = await sensor.new_sensor(fusion_config[CONF_LOCATION_Y])
            cg.add(var.set_location_y_sensor(sens))