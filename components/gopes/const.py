"""Constants for GOPES component."""

# Component constants
DOMAIN = "gopes"
VERSION = "1.0.0"

# Configuration keys
CONF_PRESENCE_DETECTION = "presence_detection"
CONF_MOVEMENT_DETECTION = "movement_detection"
CONF_SENSOR_FUSION = "sensor_fusion"
CONF_LOCATION_TRACKING = "location_tracking"
CONF_BLUETOOTH_CONFIG = "bluetooth_config"

# LD2410 Configuration
CONF_LD2410_CONFIG = "ld2410_config"
CONF_SENSITIVITY = "sensitivity"
CONF_DETECTION_RANGE = "detection_range"
CONF_STATIONARY_RANGE = "stationary_range"
CONF_MOTION_RANGE = "motion_range"
CONF_TIMEOUT = "timeout"

# VL53L1X Configuration
CONF_VL53L1X_CONFIG = "vl53l1x_config"
CONF_TIMING_BUDGET = "timing_budget"
CONF_INTER_MEASUREMENT = "inter_measurement"
CONF_DISTANCE_MODE = "distance_mode"

# Sensor Fusion Configuration
CONF_FUSION_ALGORITHM = "fusion_algorithm"
CONF_KALMAN_PARAMS = "kalman_parameters"
CONF_CONFIDENCE_THRESHOLD = "confidence_threshold"

# Location Tracking Configuration
CONF_TRACKING_MODE = "tracking_mode"
CONF_HISTORY_SIZE = "history_size"
CONF_SMOOTHING_FACTOR = "smoothing_factor"
CONF_COORDINATE_SYSTEM = "coordinate_system"

# MQTT Topics
MQTT_TOPIC_STATE = "state"
MQTT_TOPIC_CONFIG = "config"
MQTT_TOPIC_COMMAND = "command"
MQTT_TOPIC_PRESENCE = "presence"
MQTT_TOPIC_LOCATION = "location"

# Default Values
DEFAULT_SENSITIVITY = 5
DEFAULT_DETECTION_RANGE = 6.0
DEFAULT_STATIONARY_RANGE = 4.0
DEFAULT_MOTION_RANGE = 6.0
DEFAULT_TIMEOUT = 30
DEFAULT_HISTORY_SIZE = 100
DEFAULT_SMOOTHING_FACTOR = 0.8
DEFAULT_CONFIDENCE_THRESHOLD = 0.75

# State Values
STATE_OCCUPIED = "occupied"
STATE_CLEAR = "clear"
STATE_MOVING = "moving"
STATE_STATIONARY = "stationary"