"""Configuration flow and template handling for GOPES component."""
import esphome.codegen as cg
import esphome.config_validation as cv
import voluptuous as vol
import jinja2
from typing import Any, Dict
from .const import (
    DOMAIN,
    CONF_LD2410_CONFIG,
    CONF_VL53L1X_CONFIG,
    CONF_SENSOR_FUSION,
    CONF_LOCATION_TRACKING,
    CONF_BLUETOOTH_CONFIG,
    DEFAULT_SENSITIVITY,
    DEFAULT_DETECTION_RANGE,
    DEFAULT_STATIONARY_RANGE,
    DEFAULT_MOTION_RANGE,
    DEFAULT_TIMEOUT,
)

# Original config flow schema
CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(DOMAIN),
    cv.Optional(CONF_LD2410_CONFIG): cv.Schema({
        cv.Optional("sensitivity", default=DEFAULT_SENSITIVITY): cv.int_range(min=0, max=9),
        cv.Optional("detection_range", default=DEFAULT_DETECTION_RANGE): cv.float_range(min=0, max=8.0),
        cv.Optional("stationary_range", default=DEFAULT_STATIONARY_RANGE): cv.float_range(min=0, max=6.0),
        cv.Optional("motion_range", default=DEFAULT_MOTION_RANGE): cv.float_range(min=0, max=8.0),
        cv.Optional("timeout", default=DEFAULT_TIMEOUT): cv.positive_int,
    }),
    cv.Optional(CONF_VL53L1X_CONFIG): cv.Schema({
        cv.Optional("timing_budget"): cv.positive_int,
        cv.Optional("inter_measurement"): cv.positive_int,
        cv.Optional("distance_mode"): cv.enum({"SHORT": 1, "MEDIUM": 2, "LONG": 3}),
    }),
    cv.Optional(CONF_SENSOR_FUSION): cv.Schema({
        cv.Optional("algorithm"): cv.enum({"KALMAN": 1, "COMPLEMENTARY": 2}),
        cv.Optional("confidence_threshold"): cv.percentage,
    }),
    cv.Optional(CONF_LOCATION_TRACKING): cv.Schema({
        cv.Optional("mode"): cv.enum({"2D": 1, "3D": 2}),
        cv.Optional("history_size"): cv.positive_int,
        cv.Optional("smoothing_factor"): cv.percentage,
    }),
    cv.Optional(CONF_BLUETOOTH_CONFIG): cv.Schema({
        cv.Optional("enabled"): cv.boolean,
        cv.Optional("sync_interval"): cv.positive_time_period_seconds,
    }),
})

class GOPESConfigFlow:
    """Handle configuration flow for GOPES component."""
    
    def __init__(self):
        self._config = {}
        self.template_generator = GOPESConfigTemplate()
        
    async def async_step_user(self, user_input=None):
        """Handle a flow initialized by the user."""
        errors = {}
        
        if user_input is not None:
            try:
                self._config.update(user_input)
                return await self.async_step_sensors()
            except vol.Invalid as err:
                errors["base"] = "invalid_config"
                
        return self.async_show_form(
            step_id="user",
            data_schema=vol.Schema({
                vol.Required("name"): str,
                vol.Optional("location"): str,
            }),
            errors=errors
        )
        
    async def async_step_sensors(self, user_input=None):
        """Configure sensor settings."""
        if user_input is not None:
            try:
                self._config.update(user_input)
                return await self.async_step_fusion()
            except vol.Invalid as err:
                errors["base"] = "invalid_sensor_config"
                
        return self.async_show_form(
            step_id="sensors",
            data_schema=vol.Schema({
                vol.Required("update_interval", default=0.1): cv.positive_float,
                **CONFIG_SCHEMA.schema[CONF_LD2410_CONFIG].schema,
                **CONFIG_SCHEMA.schema[CONF_VL53L1X_CONFIG].schema,
            })
        )
        
    async def async_step_fusion(self, user_input=None):
        """Configure sensor fusion settings."""
        if user_input is not None:
            try:
                self._config.update(user_input)
                config = await self._generate_config()
                return self.async_create_entry(
                    title="GOPES Configuration",
                    data=config
                )
            except vol.Invalid as err:
                errors["base"] = "invalid_fusion_config"
                
        return self.async_show_form(
            step_id="fusion",
            data_schema=vol.Schema({
                **CONFIG_SCHEMA.schema[CONF_SENSOR_FUSION].schema,
                **CONFIG_SCHEMA.schema[CONF_LOCATION_TRACKING].schema,
            })
        )
        
    async def _generate_config(self):
        """Generate final configuration using template."""
        try:
            # Validate complete configuration
            self.template_generator.validate_config(self._config)
            # Generate configuration from template
            return self.template_generator.generate_config(self._config)
        except ValueError as err:
            raise vol.Invalid(str(err))

class GOPESConfigTemplate:
    """Generate GOPES configuration from template."""
    
    def __init__(self):
        self.template = """
# Generated GOPES Configuration
# Do not edit directly - use configuration interface

# Core configuration from initial setup
substitutions:
  name: "{{ name }}"
  friendly_name: "{{ friendly_name }}"

# Extend core ESPHome configuration
esphome:
  name: "${name}"
  includes:
    - gopes_components.h
  libraries:
    - Wire
    - SPI
    - adafruit/Adafruit BME280 Library
    - claws/BH1750
    - pololu/VL53L1X

# Sensor configurations  
sensor:
  # LD2410 mmWave Configuration
  - platform: gopes
    id: ld2410_sensor
    moving_distance:
      name: "${friendly_name} Moving Distance"
    still_distance:
      name: "${friendly_name} Still Distance"
    detection_distance:
      name: "${friendly_name} Detection Distance"
    update_interval: {{ update_interval }}s
    configuration:
      sensitivity: {{ ld2410_config.sensitivity }}
      detection_range: {{ ld2410_config.detection_range }}
      stationary_range: {{ ld2410_config.stationary_range }}
      motion_range: {{ ld2410_config.motion_range }}
      timeout: {{ ld2410_config.timeout }}

  # VL53L1X Time of Flight Configuration  
  - platform: vl53l1x
    id: vl53l1x_sensor
    name: "${friendly_name} ToF Distance"
    update_interval: {{ update_interval }}s
    address: 0x29
    configuration:
      timing_budget: {{ vl53l1x_config.timing_budget }}
      inter_measurement: {{ vl53l1x_config.inter_measurement }}
      distance_mode: {{ vl53l1x_config.distance_mode }}

# Sensor Fusion Configuration
sensor_fusion:
  algorithm: {{ fusion_config.algorithm }}
  confidence_threshold: {{ fusion_config.confidence_threshold }}
  smoothing_factor: {{ fusion_config.smoothing_factor }}

# Location Tracking Configuration  
location_tracking:
  mode: {{ location_config.mode }}
  history_size: {{ location_config.history_size }}
  coordinate_system: {{ location_config.coordinate_system }}

# MQTT Configuration
mqtt:
  topic_prefix: "{{ mqtt_config.topic_prefix }}"
  discovery_prefix: "{{ mqtt_config.discovery_prefix }}"

# Bluetooth Configuration  
bluetooth:
  id: ld2410_ble
  enabled: {{ bluetooth_config.enabled }}
  sync_interval: {{ bluetooth_config.sync_interval }}s
"""
    
    def generate_config(self, config: Dict[str, Any]) -> str:
        """Generate configuration from template and parameters."""
        template = jinja2.Template(self.template)
        return template.render(**config)
        
    def validate_config(self, config: Dict[str, Any]) -> bool:
        """Validate configuration parameters."""
        required_keys = [
            'name',
            'friendly_name',
            'update_interval',
            'ld2410_config',
            'vl53l1x_config',
            'fusion_config',
            'location_config',
            'mqtt_config',
            'bluetooth_config'
        ]
        
        for key in required_keys:
            if key not in config:
                raise ValueError(f"Missing required configuration key: {key}")
                
        # Validate specific configuration sections
        self._validate_ld2410_config(config['ld2410_config'])
        self._validate_vl53l1x_config(config['vl53l1x_config'])
        self._validate_fusion_config(config['fusion_config'])
        self._validate_location_config(config['location_config'])
        
        return True
        
    def _validate_ld2410_config(self, config: Dict[str, Any]):
        """Validate LD2410 configuration parameters."""
        required = {
            'sensitivity': (0, 9),
            'detection_range': (0.0, 8.0),
            'stationary_range': (0.0, 6.0),
            'motion_range': (0.0, 8.0),
            'timeout': (0, 3600)
        }
        
        for key, (min_val, max_val) in required.items():
            if key not in config:
                raise ValueError(f"Missing LD2410 configuration key: {key}")
            value = config[key]
            if not isinstance(value, (int, float)):
                raise ValueError(f"Invalid type for {key}: {type(value)}")
            if value < min_val or value > max_val:
                raise ValueError(f"{key} must be between {min_val} and {max_val}")
                
    def _validate_vl53l1x_config(self, config: Dict[str, Any]):
        """Validate VL53L1X configuration parameters."""
        required = {
            'timing_budget': (20, 1000),
            'inter_measurement': (20, 1000),
            'distance_mode': ['SHORT', 'MEDIUM', 'LONG']
        }
        
        for key, valid_values in required.items():
            if key not in config:
                raise ValueError(f"Missing VL53L1X configuration key: {key}")
            value = config[key]
            
            if key == 'distance_mode':
                if value not in valid_values:
                    raise ValueError(f"Invalid {key}: {value}. Must be one of {valid_values}")
            else:
                if not isinstance(value, int):
                    raise ValueError(f"Invalid type for {key}: {type(value)}")
                min_val, max_val = valid_values
                if value < min_val or value > max_val:
                    raise ValueError(f"{key} must be between {min_val} and {max_val}")
                    
    def _validate_fusion_config(self, config: Dict[str, Any]):
        """Validate sensor fusion configuration parameters."""
        required = {
            'algorithm': ['KALMAN', 'COMPLEMENTARY'],
            'confidence_threshold': (0.0, 1.0),
            'smoothing_factor': (0.0, 1.0)
        }
        
        for key, valid_values in required.items():
            if key not in config:
                raise ValueError(f"Missing fusion configuration key: {key}")
            value = config[key]
            
            if key == 'algorithm':
                if value not in valid_values:
                    raise ValueError(f"Invalid {key}: {value}. Must be one of {valid_values}")
            else:
                if not isinstance(value, float):
                    raise ValueError(f"Invalid type for {key}: {type(value)}")
                min_val, max_val = valid_values
                if value < min_val or value > max_val:
                    raise ValueError(f"{key} must be between {min_val} and {max_val}")
                    
    def _validate_location_config(self, config: Dict[str, Any]):
        """Validate location tracking configuration parameters."""
        required = {
            'mode': ['2D', '3D'],
            'history_size': (10, 1000),
            'coordinate_system': ['CARTESIAN', 'POLAR']
        }
        
        for key, valid_values in required.items():
            if key not in config:
                raise ValueError(f"Missing location configuration key: {key}")
            value = config[key]
            
            if key in ['mode', 'coordinate_system']:
                if value not in valid_values:
                    raise ValueError(f"Invalid {key}: {value}. Must be one of {valid_values}")
            else:
                if not isinstance(value, int):
                    raise ValueError(f"Invalid type for {key}: {type(value)}")
                min_val, max_val = valid_values
                if value < min_val or value > max_val:
                    raise ValueError(f"{key} must be between {min_val} and {max_val}")