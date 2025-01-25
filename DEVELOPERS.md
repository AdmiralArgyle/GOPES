# GOPES Developer Guide

This guide covers the technical details of the GOPES component for developers who want to contribute to or modify the component.

## Development Environment Setup

### Prerequisites

1. Development tools:
```bash
sudo apt-get install build-essential git python3-dev python3-venv
```

2. Create a Python virtual environment:
```bash
python3 -m venv gopes-dev
source gopes-dev/bin/activate
```

3. Install development dependencies:
```bash
pip install esphome
pip install -r requirements-dev.txt
```

### Repository Structure

```
gopes/
├── components/
│   └── gopes/
│       ├── __init__.py           # Component registration
│       ├── binary_sensor.py      # Binary sensor implementation
│       ├── const.py             # Constants
│       ├── config_flow.py       # Configuration handling
│       ├── fusion.py            # Sensor fusion implementation
│       ├── manifest.json        # Component manifest
│       ├── sensor.py            # Sensor platform implementation
│       ├── services.yaml        # Service definitions
│       ├── include/
│       │   ├── gopes_utils.h    # Utility functions
│       │   └── gopes_components.h # Core components
│       └── translations/
│           └── en.json          # English translations
├── examples/
│   └── gopes.yaml              # Example configuration
└── tests/
    └── test_gopes.py           # Unit tests
```

### Key Component Files

- `__init__.py`: Core component registration and initialization
- `binary_sensor.py`: Implementation of presence and movement detection
- `const.py`: Shared constants and configuration keys
- `config_flow.py`: Configuration validation and template handling
- `fusion.py`: Sensor fusion algorithms and data processing
- `manifest.json`: Component metadata and dependencies
- `sensor.py`: Implementation of sensor platforms (LD2410, VL53L1X)
- `services.yaml`: Service definitions for Home Assistant
- `gopes_utils.h`: C++ utility functions and helper classes
- `gopes_components.h`: Main C++ component implementations
- `en.json`: English translations for UI elements

[Rest of developer guide content remains the same...]