# Drone GCS - Ground Control Station

A PyQt6-based ground control station for drone firmware with Mavlink protocol support.

## Installation

```bash
cd DroneGCS
uv sync
```

## Running

```bash
uv run python -m drone_gcs
```

## Requirements

- Python 3.10+
- PyQt6
- pymavlink
- pyserial
- numpy
- pyqtgraph

## Features

### Connection Manager

Supports multiple connection types:

- **USB Serial**: Direct USB connection to flight controller
- **UDP**: Network connection (default port 14550)
- **Relay**: Intermediate microcontroller forwarding

Auto-detection finds "DroneFlightController" devices by:
- USB product name containing "drone", "fc", "flight", etc.
- VID/PID matching known flight controller devices

### Mavlink Protocol

Implements standard Mavlink messages plus custom PID tuning extensions:

| Message | Description |
|---------|-------------|
| HEARTBEAT | System heartbeat |
| ATTITUDE | Drone attitude (roll, pitch, yaw) |
| GLOBAL_POSITION_INT | GPS position and altitude |
| RC_CHANNELS_OVERRIDE | Manual control input |
| VFR_HUD | Flight data display |

### Custom PID Tuning Messages (ID 200-202)

#### Message 200: PID Tuning Command
```
Byte 0: Command
  0x01 = Start tuning
  0x02 = Stop tuning
  0x03 = Abort tuning
  0x07 = Get status
Byte 1: Tuning method (for start)
Byte 2: Axis (for start)
Bytes 3-6: Amplitude (float)
```

#### Message 201: PID Tuning Status
```
Byte 0: State (0-6)
Byte 1: Method
Byte 2: Axis
Bytes 3-4: Progress (0-100%)
Bytes 5-8: Timestamp
```

#### Message 202: PID Gains
```
Single axis (13 bytes):
  Byte 0: Axis (0-7)
  Bytes 1-4: Kp (float)
  Bytes 5-8: Ki (float)
  Bytes 9-12: Kd (float)

Full profile (120 bytes):
  27 floats for all PIDs
  1 uint32 version
  1 uint32 timestamp
```

### Tuning Methods

1. **Ziegler-Nichols**: Ultimate gain oscillation method
2. **Relay (Åström-Hägglund)**: Relay feedback for ultimate gain
3. **Step Response**: First-order system identification
4. **Frequency Sweep**: Bode plot analysis

### PID Axes

| Index | Axis |
|-------|------|
| 0 | Roll Rate |
| 1 | Pitch Rate |
| 2 | Yaw Rate |
| 3 | Roll Attitude |
| 4 | Pitch Attitude |
| 5 | Yaw Attitude |
| 6 | Altitude |
| 7 | Position |

### API Reference

#### ConnectionManager

```python
from drone_gcs.connection import ConnectionManager

conn = ConnectionManager()

# List available ports
ports = conn.list_serial_ports()

# Auto-detect drone device
device = conn.detect_drone_device()

# Connect
conn.connect_usb("/dev/ttyUSB0", 115200)
# or
conn.auto_connect(115200)

# Send RC channels
conn.send_rc_channels(roll=1500, pitch=1500, throttle=1200, yaw=1500)

# PID tuning
conn.send_pid_tune_start(method=0, axis=0, amplitude=50.0)
conn.send_pid_gains(axis=0, kp=0.8, ki=0.02, kd=0.05)
conn.request_all_gains()

# Disconnect
conn.disconnect()
```

#### MavlinkProtocol

```python
from drone_gcs.protocol import MavlinkProtocol, TuneMethod, TuneAxis

protocol = MavlinkProtocol()

# Connect
protocol.connect_serial("/dev/ttyUSB0", 115200)

# Receive messages
msg = protocol.receive_message(timeout=1.0)
if msg:
    print(f"{msg.name}: {msg.data}")

# Send commands
protocol.send_rc_channels(1500, 1500, 1000, 1500)
protocol.send_pid_tune_start(TuneMethod.RELAY, TuneAxis.ROLL_RATE, 50.0)
protocol.save_pid_profile(0)
protocol.load_pid_profile(0)

protocol.disconnect()
```

### UI Components

#### Main Window Tabs

1. **Flight Control**
   - Mode selector (Stabilize, Acro, AltHold, etc.)
   - Arm/Disarm buttons
   - Throttle slider
   - Real-time attitude display
   - GPS and battery status

2. **PID Tuning**
   - Auto-tune with progress bar
   - Method and axis selection
   - 4 profile slots (save/load)
   - Manual gain adjustment with Apply button

3. **Motor Test**
   - Individual motor PWM control
   - Test All Motors button
   - Emergency STOP button

4. **Sensors**
   - IMU accelerometer/gyroscope values
   - Barometer temperature/pressure

5. **Messages**
   - Mavlink message log
   - Debug output

### Troubleshooting

**GCS doesn't detect drone:**
- Check USB connection
- Verify firmware is uploaded
- Try manual port selection

**No telemetry data:**
- Check baudrate matches (115200)
- Verify Mavlink protocol is enabled
- Check antenna connection for radio links

**PID tuning fails:**
- Ensure drone is armed
- Maintain stable hover during tuning
- Reduce amplitude if oscillations are too aggressive

## File Structure

```
drone_gcs/
├── __init__.py
├── __main__.py           # Entry point
├── protocol/
│   ├── __init__.py
│   └── mavlink_protocol.py  # Mavlink handler
├── connection/
│   ├── __init__.py
│   └── connection_manager.py  # USB/UDP connections
└── ui/
    ├── __init__.py
    └── main_window.py     # PyQt6 main window
```

## License

MIT
