/**
 * GOPES Component Declarations
 * 
 * This file contains the main component classes for the GOPES system,
 * including sensor interfaces, fusion logic, and state management.
 */

#pragma once

#include "esphome.h"
#include "gopes_utils.h"
#include <memory>
#include <queue>

namespace gopes {

using namespace esphome;

// Forward declarations
class GOPESComponent;
class LD2410Interface;
class VL53L1XInterface;
class SensorFusion;
class LocationTracker;

/**
 * @brief Main GOPES component class
 */
class GOPESComponent : public Component {
public:
    GOPESComponent();
    
    void setup() override;
    void loop() override;
    float get_setup_priority() const override { return setup_priority::LATE; }
    
    // Configuration methods
    void set_ld2410_sensor(sensor::Sensor *sensor) { ld2410_sensor_ = sensor; }
    void set_vl53l1x_sensor(sensor::Sensor *sensor) { vl53l1x_sensor_ = sensor; }
    void set_presence_binary_sensor(binary_sensor::BinarySensor *sensor) { presence_sensor_ = sensor; }
    void set_movement_binary_sensor(binary_sensor::BinarySensor *sensor) { movement_sensor_ = sensor; }
    
    // Service methods
    void calibrate(bool force = false);
    void sync_bluetooth(const std::string& direction);
    void reset_tracking(bool clear_history);
    
    // State accessors
    bool is_presence_detected() const { return presence_detected_; }
    bool is_movement_detected() const { return movement_detected_; }
    const Point2D& get_current_location() const { return current_location_; }
    
protected:
    // Sensor interfaces
    std::unique_ptr<LD2410Interface> ld2410_;
    std::unique_ptr<VL53L1XInterface> vl53l1x_;
    
    // Sensor fusion and tracking
    std::unique_ptr<SensorFusion> fusion_;
    std::unique_ptr<LocationTracker> tracker_;
    
    // ESPHome sensors
    sensor::Sensor *ld2410_sensor_{nullptr};
    sensor::Sensor *vl53l1x_sensor_{nullptr};
    binary_sensor::BinarySensor *presence_sensor_{nullptr};
    binary_sensor::BinarySensor *movement_sensor_{nullptr};
    
    // State variables
    bool presence_detected_{false};
    bool movement_detected_{false};
    Point2D current_location_;
    
    // Configuration
    float update_interval_{0.1f};  // 100ms default
    bool calibration_required_{true};
    
    // Internal methods
    void process_sensor_data();
    void update_state();
    void publish_state();
};

/**
 * @brief Interface for LD2410 mmWave sensor
 */
class LD2410Interface {
public:
    struct Reading {
        float moving_distance;
        float still_distance;
        float detection_distance;
        float moving_energy;
        float still_energy;
        uint32_t timestamp;
    };
    
    LD2410Interface();
    
    bool init();
    void update();
    Reading get_last_reading() const { return last_reading_; }
    
    // Configuration
    void set_sensitivity(uint8_t value);
    void set_detection_range(float value);
    void set_timeout(uint32_t value);
    
    // Bluetooth configuration sync
    bool sync_config(const std::string& direction);
    
protected:
    Reading last_reading_{};
    CircularBuffer<Reading, 10> history_;
    
    // Configuration parameters
    uint8_t sensitivity_{5};
    float detection_range_{6.0f};
    float stationary_range_{4.0f};
    float motion_range_{6.0f};
    uint32_t timeout_{30};
    
    // Processing methods
    void process_uart_data();
    void filter_readings();
};

/**
 * @brief Interface for VL53L1X Time-of-Flight sensor
 */
class VL53L1XInterface {
public:
    struct Reading {
        float distance;
        uint8_t status;
        uint32_t timestamp;
    };
    
    VL53L1XInterface();
    
    bool init();
    void update();
    Reading get_last_reading() const { return last_reading_; }
    
    // Configuration
    void set_timing_budget(uint16_t value);
    void set_inter_measurement(uint16_t value);
    void set_distance_mode(uint8_t value);
    
protected:
    Reading last_reading_{};
    CircularBuffer<Reading, 10> history_;
    
    // Configuration parameters
    uint16_t timing_budget_{50};
    uint16_t inter_measurement_{50};
    uint8_t distance_mode_{2};  // MEDIUM
    
    // Processing methods
    void process_i2c_data();
    void filter_readings();
};

/**
 * @brief Sensor fusion implementation
 */
class SensorFusion {
public:
    struct FusedData {
        float distance;
        float confidence;
        Point2D location;
        bool movement_detected;
        uint32_t timestamp;
    };
    
    SensorFusion();
    
    void update(const LD2410Interface::Reading& ld2410_data,
                const VL53L1XInterface::Reading& vl53l1x_data);
    FusedData get_last_fusion() const { return last_fusion_; }
    
    // Configuration
    void set_algorithm(const std::string& algo);
    void set_confidence_threshold(float value);
    void set_smoothing_factor(float value);
    
    void reset();
    
protected:
    FusedData last_fusion_{};
    KalmanFilter1D kalman_filter_;
    EMAFilter ema_filter_;
    
    // Configuration parameters
    std::string algorithm_{"KALMAN"};
    float confidence_threshold_{0.75f};
    float smoothing_factor_{0.8f};
    
    // Processing methods
    float fuse_distances(float ld2410_dist, float vl53l1x_dist);
    float calculate_confidence(float fused_dist);
    Point2D estimate_location(float distance);
};

/**
 * @brief Location tracking implementation
 */
class LocationTracker {
public:
    struct TrackingData {
        Point2D location;
        float velocity;
        bool movement_detected;
        uint32_t timestamp;
    };
    
    LocationTracker();
    
    void update(const Point2D& location, uint32_t timestamp);
    TrackingData get_last_tracking() const { return last_tracking_; }
    const std::vector<Point2D>& get_history() const { return location_history_; }
    
    void set_mode(const std::string& mode);
    void set_history_size(size_t size);
    
    void reset();
    
protected:
    TrackingData last_tracking_{};
    std::vector<Point2D> location_history_;
    
    // Configuration parameters
    std::string mode_{"2D"};
    size_t history_size_{100};
    float movement_threshold_{0.1f};
    
    // Processing methods
    void update_history(const Point2D& location);
    float calculate_velocity();
    bool detect_movement();
};

} // namespace gopes