# GOPES - GUPPI Occupancy and Presence Enhancement System

GOPES is an advanced occupancy and human presence detection system that combines mmWave and Time-of-Flight sensors for enhanced accuracy. It's designed to work with ESPHome and Home Assistant, providing robust presence detection and tracking capabilities.

## Prerequisites

### Hardware Requirements
- ESP32 development board (recommended: ESP32-C3)
- LD2410C 24GHz mmWave sensor
- VL53L1X Time-of-Flight sensor
- BME280 environmental sensor
- BH1750 light sensor
- Connecting wires
- (Optional) Custom PCB for sensor integration

### Software Requirements
- ESPHome 2024.12.4 or later
- Home Assistant 2025.1.4 or later
- Python 3.9 or later (for development)

## Installation

### Quick Installation (End Users)

1. In Home Assistant, navigate to Settings -> Add-ons -> Add-on Store
2. Add the ESPHome repository if you haven't already
3. Install ESPHome if not already installed
4. Create a new ESPHome device and paste the following basic configuration:

```yaml
substitutions:
  name: gopes-occupancy
  friendly_name: "GOPES Occupancy"

external_components:
  - source:
      type: git
      url: https://github.com/AdmiralArgyle/GOPES
    components: [gopes]
    refresh: 1d

# Copy the rest from gopes.yaml provided in the repository
```

5. Flash the initial configuration to your ESP32
6. Once the device is set up, it will automatically pull the full configuration

### Manual Installation (Developers)

1. Clone the repository:
```bash
git clone https://github.com/AdmiralArgyle/GOPES.git
```

2. Install development dependencies:
```bash
pip install -r requirements-dev.txt
```

3. Copy the component files to your ESPHome configuration directory:
```bash
cp -r components/gopes ~/.esphome/custom_components/
```

4. Add the component to your ESPHome configuration:
```yaml
external_components:
  - source:
      type: local
      path: custom_components
    components: [gopes]
```

## Hardware Setup

### Wiring Diagram

| ESP32 Pin | Sensor Pin | Description |
|-----------|------------|-------------|
| 3.3V      | VCC       | Power       |
| GND       | GND       | Ground      |
| GPIO21    | SDA       | I2C Data    |
| GPIO22    | SCL       | I2C Clock   |
| GPIO16    | LD2410 RX | UART RX     |
| GPIO17    | LD2410 TX | UART TX     |

### Sensor Placement

For optimal performance:
- Mount the sensors at a height of 2.1-2.4m (7-8ft)
- Ensure clear line of sight to the monitored area
- Avoid placing near moving objects (fans, curtains)
- Keep away from heat sources

## Configuration

### Basic Configuration

The component supports automatic configuration through Home Assistant. However, you can customize various parameters:

```yaml
# Example custom configuration
gopes:
  ld2410_config:
    sensitivity: 7
    detection_range: 6.0
    stationary_range: 4.0
    motion_range: 6.0
    timeout: 30

  vl53l1x_config:
    timing_budget: 50
    inter_measurement: 50
    distance_mode: MEDIUM

  sensor_fusion:
    algorithm: KALMAN
    confidence_threshold: 0.75
    smoothing_factor: 0.8

  location_tracking:
    mode: "2D"
    history_size: 100
    coordinate_system: CARTESIAN
```

### MQTT Integration

GOPES automatically integrates with MQTT when configured in Home Assistant. The default topic structure is:

```
GUPPI/GOPES/<device_name>/
  ├── status
  ├── presence
  ├── movement
  ├── location
  └── config/
      ├── sensitivity
      ├── range
      └── timeout
```

## Calibration and Tuning

### Initial Calibration

1. After installation, trigger initial calibration:
   ```yaml
   # In Home Assistant:
   service: gopes.calibrate_sensors
   data:
     force: true
   ```

2. Wait for the calibration to complete (approximately 30 seconds)

### Fine-tuning

1. Monitor sensor readings through Home Assistant
2. Adjust sensitivity and range based on your space:
   - Smaller rooms: Reduce range, increase sensitivity
   - Larger rooms: Increase range, decrease sensitivity
3. Use the built-in bluetooth interface for LD2410 configuration

## Troubleshooting

### Common Issues

1. **No Presence Detection**
   - Check sensor wiring
   - Verify sensor addresses
   - Ensure clear line of sight
   - Try increasing sensitivity

2. **False Positives**
   - Reduce sensitivity
   - Check for moving objects in range
   - Verify sensor mounting stability

3. **Configuration Issues**
   - Check ESPHome logs
   - Verify component version compatibility
   - Ensure correct MQTT configuration

### Debug Logging

Enable debug logging by adding to your configuration:

```yaml
logger:
  level: DEBUG
  logs:
    gopes: DEBUG
```

## Contributing

We welcome contributions! Please see CONTRIBUTING.md for guidelines.

## License

This project is licensed under the MIT License - see LICENSE.md for details.

## Support

For support:
1. Check the [FAQ](docs/FAQ.md)
2. Open an issue on GitHub
3. Join our Discord community

## Credits

GOPES is maintained by AdmiralArgyle and contributors.
Special thanks to the ESPHome and Home Assistant communities.