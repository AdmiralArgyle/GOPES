"""GOPES binary sensor implementation."""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_MOTION,
    DEVICE_CLASS_OCCUPANCY,
)
from . import gopes_ns, GOPESBinarySensor
from .const import (
    CONF_PRESENCE_DETECTION,
    CONF_MOVEMENT_DETECTION,
)

DEPENDENCIES = ['gopes']

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(GOPESBinarySensor),
    cv.Optional(CONF_PRESENCE_DETECTION): binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_OCCUPANCY
    ),
    cv.Optional(CONF_MOVEMENT_DETECTION): binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_MOTION
    ),
})

class GOPESPresenceDetector:
    """Enhanced presence detection using sensor fusion."""
    
    def __init__(self):
        self._presence_threshold = 0.75
        self._movement_threshold = 0.5
        self._presence_history = []
        self._movement_history = []
        
    def update_presence(self, fusion_data, location_data):
        """Update presence detection state."""
        presence_confidence = self._calculate_presence_confidence(
            fusion_data, location_data
        )
        movement_confidence = self._calculate_movement_confidence(location_data)
        
        self._presence_history.append(presence_confidence)
        self._movement_history.append(movement_confidence)
        
        # Maintain history window
        if len(self._presence_history) > 10:
            self._presence_history.pop(0)
        if len(self._movement_history) > 10:
            self._movement_history.pop(0)
            
        return {
            'presence': sum(self._presence_history) / len(self._presence_history) > self._presence_threshold,
            'movement': sum(self._movement_history) / len(self._movement_history) > self._movement_threshold
        }
        
    def _calculate_presence_confidence(self, fusion_data, location_data):
        """Calculate confidence level for presence detection."""
        if fusion_data is None or location_data is None:
            return 0.0
            
        distance_factor = 1.0 - min(fusion_data['distance'] / 10.0, 1.0)
        energy_factor = min(fusion_data['energy'] / 100.0, 1.0)
        stability_factor = self._calculate_stability(location_data)
        
        return (distance_factor * 0.4 + 
                energy_factor * 0.4 + 
                stability_factor * 0.2)
        
    def _calculate_movement_confidence(self, location_data):
        """Calculate confidence level for movement detection."""
        if not location_data or len(location_data) < 2:
            return 0.0
            
        # Calculate movement based on position changes
        x_diff = location_data[-1][0] - location_data[-2][0]
        y_diff = location_data[-1][1] - location_data[-2][1]
        movement = (x_diff ** 2 + y_diff ** 2) ** 0.5
        
        return min(movement / 0.5, 1.0)  # 0.5m as maximum movement threshold
        
    def _calculate_stability(self, location_data):
        """Calculate stability factor from location history."""
        if not location_data or len(location_data) < 5:
            return 0.0
            
        # Calculate variance in positions
        x_var = sum((x - sum(x for x, _ in location_data) / len(location_data)) ** 2 
                   for x, _ in location_data) / len(location_data)
        y_var = sum((y - sum(y for _, y in location_data) / len(location_data)) ** 2 
                   for _, y in location_data) / len(location_data)
                   
        stability = 1.0 - min((x_var + y_var) / 2.0, 1.0)
        return stability