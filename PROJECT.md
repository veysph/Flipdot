## Project Overview

This is a Python-based matrix display controller for flipdot displays. It supports 56x56 matrix displays with dual-controller architecture (left and right columns) connected via serial communication. The system features differential updates, camera integration, hand gesture controls, and various display modes.

## Key Commands

### Running the Application

```bash
# Basic display with configuration
python3 flipdot.py --config config/lcd.json

# Run all configurations in directory with interval
python3 flipdot.py --config-dir config/ --interval 1200

# Rain simulation
python3 flipdot.py --rain-simulator --rain-density 0.1 --rain-speed 1

# Matrix test patterns
python3 flipdot.py --black  # All dots off
python3 flipdot.py --white  # All dots on
```

## Architecture Overview

### Core Components

- **Main Controller** (`flipdot.py`): Central application managing display modes, configuration loading, and coordination between components
- **Display System**: Dual-controller architecture with left (Column 1) and right (Column 2) controllers managing 16 total panels (8 per controller)
- **Serial Communication**: Multi-threaded serial connection pool with health monitoring and automatic reconnection
- **Differential Updates**: Smart transmission system that only sends changed panel data to reduce bandwidth
- **Text Rendering** (`lib/text_renderer.py`): Custom bitmap font rendering with CP437 encoding support
- **Jeedom Integration** (`lib/pyJeedom.py`): Home automation API client for temperature sensor data integration

### Configuration System

JSON-based configuration files in `config/` directory define:
- Base images for display backgrounds
- Text positioning and font settings (time, temperature data)
- Display parameters (inverted colors, trail effects)
- Font selection (4x6.otb or 6x8.otb bitmap fonts)

### Hardware Configuration

- Matrix size: 56x56 pixels
- Panel count: 16 panels (7x28 pixels each)
- Serial ports: `/dev/ttyUSB5_` (left), `/dev/ttyUSB6_` (right)
- Baudrate: 57600
- Protocol: Custom binary with start/end markers

### Dependencies

Key Python packages required:
- `numpy`: Matrix operations and image processing
- `opencv-python` (cv2): Camera and image processing
- `pyserial`: Serial communication
- `PIL`: Image manipulation
- `freetype-py`: Font rendering
- `zmq`: Hand gesture communication
- `jetcam`: NVIDIA Jetson camera support

### Development Notes

- The system is optimized for NVIDIA Jetson hardware with CSI camera support
- Configuration files use specific positioning for 56x56 matrix layout