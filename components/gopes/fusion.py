"""GOPES sensor fusion implementation."""
import numpy as np
from typing import Optional, Tuple, List, Dict
from dataclasses import dataclass
from .const import (
    DEFAULT_CONFIDENCE_THRESHOLD,
    DEFAULT_SMOOTHING_FACTOR,
)

@dataclass
class SensorReading:
    """Class for storing sensor readings."""
    distance: float
    confidence: float
    timestamp: float

class KalmanFilter:
    """Kalman filter implementation for sensor fusion."""
    
    def __init__(self):
        # State vector [distance, velocity]
        self.x = np.array([[0.0], [0.0]])
        # State covariance matrix
        self.P = np.array([[1.0, 0.0], [0.0, 1.0]])
        # State transition matrix
        self.F = np.array([[1.0, 0.1], [0.0, 1.0]])
        # Measurement matrix
        self.H = np.array([[1.0, 0.0]])
        # Process noise covariance
        self.Q = np.array([[0.1, 0.0], [0.0, 0.1]])
        # Measurement noise covariance
        self.R = np.array([[1.0]])
        
    def predict(self) -> np.ndarray:
        """Predict next state."""
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x
        
    def update(self, measurement: float, confidence: float) -> np.ndarray:
        """Update state with new measurement."""
        # Adjust R based on confidence
        self.R = np.array([[1.0 / confidence]])
        
        # Kalman gain
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        
        # Update state
        y = measurement - np.dot(self.H, self.x)
        self.x = self.x + np.dot(K, y)
        self.P = self.P - np.dot(np.dot(K, self.H), self.P)
        
        return self.x

class ComplementaryFilter:
    """Complementary filter implementation for sensor fusion."""
    
    def __init__(self, alpha: float = 0.8):
        self.alpha = alpha
        self.last_estimate = None
        
    def update(self, measurement: float, confidence: float) -> float:
        """Update filter with new measurement."""
        if self.last_estimate is None:
            self.last_estimate = measurement
            return measurement
            
        # Adjust alpha based on confidence
        adjusted_alpha = self.alpha * confidence
        
        # Update estimate
        estimate = (adjusted_alpha * measurement + 
                   (1 - adjusted_alpha) * self.last_estimate)
        self.last_estimate = estimate
        
        return estimate

class GOPESSensorFusion:
    """Main sensor fusion class combining multiple algorithms."""
    
    def __init__(self, config: Dict):
        self.config = config
        self.kalman_filter = KalmanFilter()
        self.complementary_filter = ComplementaryFilter()
        self.confidence_threshold = config.get(
            'confidence_threshold', 
            DEFAULT_CONFIDENCE_THRESHOLD
        )
        self.smoothing_factor = config.get(
            'smoothing_factor',
            DEFAULT_SMOOTHING_FACTOR
        )
        self.ld2410_history: List[SensorReading] = []
        self.vl53l1x_history: List[SensorReading] = []
        
    def update(
        self,
        ld2410_data: Optional[SensorReading] = None,
        vl53l1x_data: Optional[SensorReading] = None
    ) -> Dict:
        """Update sensor fusion with new readings."""
        if ld2410_data:
            self.ld2410_history.append(ld2410_data)
            if len(self.ld2410_history) > 10:
                self.ld2410_history.pop(0)
                
        if vl53l1x_data:
            self.vl53l1x_history.append(vl53l1x_data)
            if len(self.vl53l1x_history) > 10:
                self.vl53l1x_history.pop(0)
                
        # Get fused distance using Kalman filter
        kalman_distance = self._kalman_fusion()
        
        # Get fused distance using complementary filter
        comp_distance = self._complementary_fusion()
        
        # Combine results based on confidence
        kalman_confidence = self._calculate_confidence(kalman_distance)
        comp_confidence = self._calculate_confidence(comp_distance)
        
        total_confidence = kalman_confidence + comp_confidence
        if total_confidence > 0:
            fused_distance = (
                (kalman_distance * kalman_confidence +
                 comp_distance * comp_confidence) / total_confidence
            )
        else:
            fused_distance = kalman_distance
            
        return {
            'distance': fused_distance,
            'confidence': max(kalman_confidence, comp_confidence),
            'kalman_distance': kalman_distance,
            'complementary_distance': comp_distance
        }
        
    def _kalman_fusion(self) -> float:
        """Perform Kalman filter fusion."""
        self.kalman_filter.predict()
        
        if self.ld2410_history and self.vl53l1x_history:
            ld2410 = self.ld2410_history[-1]
            vl53l1x = self.vl53l1x_history[-1]
            
            # Use most recent reading with highest confidence
            if ld2410.confidence > vl53l1x.confidence:
                measurement = ld2410.distance
                confidence = ld2410.confidence
            else:
                measurement = vl53l1x.distance
                confidence = vl53l1x.confidence
                
            state = self.kalman_filter.update(measurement, confidence)
            return state[0, 0]
            
        return 0.0
        
    def _complementary_fusion(self) -> float:
        """Perform complementary filter fusion."""
        if self.ld2410_history and self.vl53l1x_history:
            ld2410 = self.ld2410_history[-1]
            vl53l1x = self.vl53l1x_history[-1]
            
            # Weighted average based on confidence
            total_confidence = ld2410.confidence + vl53l1x.confidence
            if total_confidence > 0:
                weight_ld2410 = ld2410.confidence / total_confidence
                weight_vl53l1x = vl53l1x.confidence / total_confidence
                
                measurement = (ld2410.distance * weight_ld2410 +
                             vl53l1x.distance * weight_vl53l1x)
                confidence = max(ld2410.confidence, vl53l1x.confidence)
                
                return self.complementary_filter.update(measurement, confidence)
                
        return 0.0
        
    def _calculate_confidence(self, distance: float) -> float:
        """Calculate confidence score for a distance measurement."""
        if distance <= 0:
            return 0.0
            
        # Basic confidence calculation based on distance
        # Confidence decreases with distance
        base_confidence = max(0.0, 1.0 - (distance / 8.0))  # 8m max range
        
        # Adjust confidence based on sensor history stability
        stability_factor = self._calculate_stability()
        
        # Combine factors
        confidence = base_confidence * stability_factor
        
        return min(1.0, max(0.0, confidence))
        
    def _calculate_stability(self) -> float:
        """Calculate stability factor from sensor history."""
        if not self.ld2410_history or not self.vl53l1x_history:
            return 0.5
            
        # Calculate variance in recent readings
        ld2410_distances = [reading.distance for reading in self.ld2410_history]
        vl53l1x_distances = [reading.distance for reading in self.vl53l1x_history]
        
        ld2410_var = np.var(ld2410_distances) if len(ld2410_distances) > 1 else 0
        vl53l1x_var = np.var(vl53l1x_distances) if len(vl53l1x_distances) > 1 else 0
        
        # Lower variance indicates more stable readings
        stability = 1.0 / (1.0 + ld2410_var + vl53l1x_var)
        
        return min(1.0, max(0.1, stability))
        
    def reset(self):
        """Reset sensor fusion state."""
        self.ld2410_history.clear()
        self.vl53l1x_history.clear()
        self.kalman_filter = KalmanFilter()
        self.complementary_filter = ComplementaryFilter()