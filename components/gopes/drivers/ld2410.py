# components/gopes/drivers/ld2410.py
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

DEPENDENCIES = ['uart']

# Configuration constants
CONF_SENSITIVITY = "sensitivity"
CONF_DETECTION_RANGE = "detection_range"

# Default values
DEFAULT_SENSITIVITY = 5
DEFAULT_DETECTION_RANGE = 6.0

# Register component
gopes_ns = cg.esphome_ns.namespace('gopes')
LD2410Sensor = gopes_ns.class_('LD2410Sensor', cg.Component, uart.UARTDevice)

# Simple configuration schema
CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(LD2410Sensor),
    cv.Optional(CONF_SENSITIVITY, default=DEFAULT_SENSITIVITY): 
        cv.int_range(min=0, max=9),
    cv.Optional(CONF_DETECTION_RANGE, default=DEFAULT_DETECTION_RANGE):
        cv.float_range(min=0.0, max=8.0),
}).extend(uart.UART_DEVICE_SCHEMA)