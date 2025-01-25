/**
 * GOPES Utility Functions and Helper Classes
 * 
 * This file contains utility functions and helper classes for the GOPES component,
 * including sensor data processing, coordinate transformations, and common calculations.
 */

#pragma once

#include "esphome.h"
#include <array>
#include <vector>
#include <cmath>
#include <algorithm>

namespace gopes {

/**
 * @brief Structure to hold 2D coordinates
 */
struct Point2D {
    float x;
    float y;
    
    Point2D(float x = 0.0f, float y = 0.0f) : x(x), y(y) {}
    
    float distance_to(const Point2D& other) const {
        float dx = x - other.x;
        float dy = y - other.y;
        return std::sqrt(dx * dx + dy * dy);
    }
};

/**
 * @brief Structure to hold 3D coordinates
 */
struct Point3D {
    float x;
    float y;
    float z;
    
    Point3D(float x = 0.0f, float y = 0.0f, float z = 0.0f) 
        : x(x), y(y), z(z) {}
    
    float distance_to(const Point3D& other) const {
        float dx = x - other.x;
        float dy = y - other.y;
        float dz = z - other.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }
};

/**
 * @brief Structure to hold polar coordinates
 */
struct PolarCoord {
    float distance;  // Distance from origin
    float angle;     // Angle in radians
    
    PolarCoord(float d = 0.0f, float a = 0.0f) 
        : distance(d), angle(a) {}
    
    Point2D to_cartesian() const {
        return Point2D(
            distance * std::cos(angle),
            distance * std::sin(angle)
        );
    }
    
    static PolarCoord from_cartesian(const Point2D& p) {
        float distance = std::sqrt(p.x * p.x + p.y * p.y);
        float angle = std::atan2(p.y, p.x);
        return PolarCoord(distance, angle);
    }
};

/**
 * @brief Circular buffer for storing sensor readings
 */
template<typename T, size_t N>
class CircularBuffer {
public:
    void push(const T& value) {
        buffer_[write_pos_] = value;
        write_pos_ = (write_pos_ + 1) % N;
        if (size_ < N) size_++;
    }
    
    T average() const {
        if (size_ == 0) return T();
        T sum = std::accumulate(begin(), end(), T());
        return sum / static_cast<float>(size_);
    }
    
    T median() const {
        if (size_ == 0) return T();
        std::vector<T> sorted(begin(), end());
        std::sort(sorted.begin(), sorted.end());
        return sorted[size_ / 2];
    }
    
    size_t size() const { return size_; }
    bool empty() const { return size_ == 0; }
    bool full() const { return size_ == N; }
    
    auto begin() const { return buffer_.begin(); }
    auto end() const { return buffer_.begin() + size_; }

private:
    std::array<T, N> buffer_;
    size_t write_pos_ = 0;
    size_t size_ = 0;
};

/**
 * @brief Kalman filter for 1D measurements
 */
class KalmanFilter1D {
public:
    KalmanFilter1D(float process_noise = 0.1f, float measurement_noise = 1.0f)
        : q_(process_noise)
        , r_(measurement_noise)
        , p_(1.0f)
        , initialized_(false) {}
    
    float update(float measurement) {
        if (!initialized_) {
            x_ = measurement;
            initialized_ = true;
            return x_;
        }
        
        // Prediction
        p_ = p_ + q_;
        
        // Update
        float k = p_ / (p_ + r_);
        x_ = x_ + k * (measurement - x_);
        p_ = (1.0f - k) * p_;
        
        return x_;
    }
    
    void reset() {
        initialized_ = false;
        p_ = 1.0f;
    }
    
    float get_state() const { return x_; }

private:
    float x_;       // State estimate
    float p_;       // Estimation error covariance
    float q_;       // Process noise covariance
    float r_;       // Measurement noise covariance
    bool initialized_;
};

/**
 * @brief Exponential moving average filter
 */
class EMAFilter {
public:
    explicit EMAFilter(float alpha = 0.2f)
        : alpha_(alpha)
        , initialized_(false) {}
    
    float update(float measurement) {
        if (!initialized_) {
            value_ = measurement;
            initialized_ = true;
            return value_;
        }
        
        value_ = alpha_ * measurement + (1.0f - alpha_) * value_;
        return value_;
    }
    
    void reset() {
        initialized_ = false;
    }
    
    float get_value() const { return value_; }
    
    void set_alpha(float alpha) {
        alpha_ = std::max(0.0f, std::min(1.0f, alpha));
    }

private:
    float value_;
    float alpha_;
    bool initialized_;
};

/**
 * @brief Signal strength to distance conversion utilities
 */
class SignalUtils {
public:
    static float rssi_to_distance(float rssi, float reference_rssi = -59.0f, float path_loss = 2.0f) {
        return std::pow(10.0f, (reference_rssi - rssi) / (10.0f * path_loss));
    }
    
    static float distance_to_rssi(float distance, float reference_rssi = -59.0f, float path_loss = 2.0f) {
        return reference_rssi - 10.0f * path_loss * std::log10(distance);
    }
};

/**
 * @brief Movement detection utilities
 */
class MovementUtils {
public:
    static bool detect_movement(const std::vector<Point2D>& history, 
                              float movement_threshold = 0.1f,
                              size_t window_size = 5) {
        if (history.size() < window_size) return false;
        
        float total_movement = 0.0f;
        for (size_t i = 1; i < std::min(history.size(), window_size); ++i) {
            total_movement += history[i].distance_to(history[i-1]);
        }
        
        return (total_movement / static_cast<float>(window_size - 1)) > movement_threshold;
    }
    
    static float calculate_velocity(const Point2D& p1, const Point2D& p2, float time_delta) {
        if (time_delta <= 0.0f) return 0.0f;
        return p1.distance_to(p2) / time_delta;
    }
};

} // namespace gopes