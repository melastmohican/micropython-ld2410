# micropython-ld2410

A MicroPython driver for the HLK-LD2410 human presence radar sensor.

## Features

- **Robust Protocol Parsing**: Implements the frame header/footer checks and checksum validation from the Rust crate.
- **MicroPython Optimized**: Uses `readinto` and `memoryview` where appropriate to minimize memory fragmentation.
- **Full Control**: Supports reading presence data, engineering mode, firmware version, and configuration commands.
- **Zero Dependencies**: Pure MicroPython implementation.

## Installation

### Using mip (Recommended)

If your device is connected to the internet:

```python
import mip
# Replace with your repo URL once published
mip.install("github:your-username/micropython-ld2410")
```

### Manual Installation

Copy the `src/ld2410` directory to the `lib` folder on your MicroPython device.

## Usage

```python
import time
from machine import UART, Pin
from ld2410 import LD2410

# Initialize UART
# Note: LD2410 default baudrate is 256000
# Raspberry Pi Pico UART1 default: TX=Pin(8), RX=Pin(9)
uart = UART(1, baudrate=256000, tx=Pin(8), rx=Pin(9))

# Create driver instance
sensor = LD2410(uart)

print("Starting sensor monitoring...")

# Optional: Read firmware version
try:
    sensor.enable_config_mode()
    version = sensor.read_firmware_version()
    print("Firmware Version:", version)
    sensor.exit_config_mode()
except Exception as e:
    print("Failed to read firmware:", e)

# Main loop
while True:
    try:
        # Blocks until a valid presence frame is received
        data = sensor.read_presence()
        
        if data:
            print(f"Target: {data.target_state}, Dist: {data.detection_distance_cm}cm")
            print(f"  Moving: {data.moving_distance_cm}cm (Energy: {data.moving_energy})")
            print(f"  Still:  {data.still_distance_cm}cm (Energy: {data.still_energy})")
            
    except Exception as e:
        print("Error reading sensor:", e)
        time.sleep(1)
```

## API Reference

### `class LD2410(uart)`

Initialize with a MicroPython `machine.UART` object.

- `read_presence()`: Reads the next valid presence report. Skips engineering frames. Returns `PresenceData`.
- `enter_config_mode()`: Sends command to enter configuration mode.
- `exit_config_mode()`: Sends command to exit configuration mode.
- `enable_engineering_mode()`: Enables engineering mode output (more detailed data).
- `disable_engineering_mode()`: Disables engineering mode.
- `read_firmware_version()`: Reads the firmware version (requires config mode).
- `restart()`: Restarts the module.

### `class PresenceData`

Data structure for sensor reports.

- `target_state`: One of `TargetState.NO_TARGET`, `MOVING`, `STATIONARY`, `BOTH`.
- `moving_distance_cm`: Distance to moving target (cm).
- `moving_energy`: Energy of moving target (0-100).
- `still_distance_cm`: Distance to stationary target (cm).
- `still_energy`: Energy of stationary target (0-100).
- `detection_distance_cm`: Overall detection distance (cm).

### `class TargetState`

Constants for target status:
- `NO_TARGET` (0x00)
- `MOVING` (0x01)
- `STATIONARY` (0x02)
- `BOTH` (0x03)

## References

- [Rust `ld2410` crate](https://github.com/lomagno2003/ld2410)
