# Drone Flight Controller - Complete Interface Specification

## Overview

This document defines the complete interface specification for:
1. **Flight Controller** - Raspberry Pi Pico (RP2040)
2. **RC Transmitter/Receiver** - ESP32-C3 with ESP-NOW
3. **Companion Computer** - Raspberry Pi Zero 2W
4. **Ground Control Station** - PyQt6 desktop application

**Firmware Device Name**: `DroneFlightController`  
**USB VID/PID**: `0x2E8A / 0x00C0` (standard Pico CDC)  
**Serial Baudrate**: `2000000` (2 Mbps)  
**Protocol**: Binary Serial + Mavlink + SPI + ESP-NOW

---

## 1. Physical Connection

### 1.1 USB Direct Connection
- Connect companion USB to Pico USB connector
- Serial: `/dev/ttyACM0` (Linux), `COMx` (Windows)
- Baudrate: 115200, 8N1

### 1.2 SPI High-Speed Connection
- **Speed**: Up to 125 MHz (125,000,000 Hz)
- **Mode**: SPI Mode 0 (CPOL=0, CPHA=0)
- **Bit Order**: MSB First
- **Pins** (Drone side):
  | Pin | Function |
  |-----|---------|
  | GPIO 8 | MISO (Master In) |
  | GPIO 9 | CS (Chip Select) |
  | GPIO 10 | SCK (Clock) |
  | GPIO 11 | MOSI (Master Out) |

### 1.3 NRF24L01+ Wireless
- **Speed**: 2 Mbps / 1 Mbps / 250 Kbps
- **Channel**: 76 (default, 0-125)
- **Mode**: Single, Half-Duplex, Full-Duplex
- **Pins** (configurable per board):
  | Pin | Function |
  |-----|---------|
  | GPIO 17 | CE (Chip Enable) |
  | GPIO 9 | CSN (SPI Select) |

### 1.4 UDP Network (Future)
- Port: 14550 (standard Mavlink UDP)
- Broadcast or unicast to Pico IP

### 1.5 Relay Mode
- Intermediate microcontroller forwards between companion and drone
- Same serial protocol as direct connection

---

## 2. Protocol Stack

```
┌─────────────────────────────────────────┐
│         Companion Computer              │
├─────────────────────────────────────────┤
│  Mavlink 2.0 (Standard Messages)        │  ← Message ID 0-255
│  Mavlink 2.0 (Custom Messages)          │  ← Message ID 200-204
│  SPI Protocol (High-Speed)               │  ← Custom Binary Protocol
│  NRF24 Protocol (Wireless)               │  ← Custom Binary Protocol
├─────────────────────────────────────────┤
│  Serial / SPI / NRF24 / UDP Transport   │
└─────────────────────────────────────────┘
```

---

## 3. Communication Specifications

### 3.1 SPI Protocol Specification

#### Physical Layer
| Parameter | Value |
|-----------|-------|
| Max Speed | 125 MHz |
| Typical Speed | 125 MHz |
| Mode | SPI_MODE0 |
| Bit Order | MSB First |
| Voltage | 3.3V |

#### Protocol Frame Structure
```
┌────────┬──────┬────────┬──────┬────────┬────────┐
│ SYNC   │ TYPE │ LENGTH │ DATA │  CRC   │ TRAILER │
│ 1 byte │ 1    │  1     │ N    │ 2 bytes│ 1 byte │
│ 0xAA   │      │        │      │        │  0x55   │
└────────┴──────┴────────┴──────┴────────┴────────┘
```

#### Packet Header (5 bytes)
| Offset | Field | Type | Description |
|--------|-------|------|-------------|
| 0 | sync | uint8 | Sync byte (0xAA) |
| 1 | type | uint8 | Packet type (see below) |
| 2 | length | uint8 | Total packet length including header and CRC |
| 3 | crc_l | uint8 | CRC16 low byte |
| 4 | crc_h | uint8 | CRC16 high byte |

#### CRC16 Calculation
```
Polynomial: 0xA001
Initial: 0xFFFF
XOR Out: 0x0000

CRC covers bytes from 'type' through end of 'data'
```

#### Packet Types (SPI)

| Type | Name | Direction | Description |
|------|------|-----------|-------------|
| 0x01 | HEARTBEAT | ↔ | Connection heartbeat |
| 0x02 | TELEMETRY | Drone→Comp | Flight telemetry data |
| 0x03 | COMMAND | Comp→Drone | Control commands |
| 0x04 | CONFIG | ↔ | Configuration read/write |
| 0x05 | RAW_SENSOR | Drone→Comp | Raw sensor data |
| 0x10 | RC_CHANNELS | Comp→Drone | RC override |
| 0x11 | MOTOR_OUTPUT | Drone→Comp | Motor PWM values |
| 0x20 | ERROR_STATUS | Drone→Comp | Error flags |
| 0xFF | NOP | ↔ | No operation (keepalive) |

#### HEARTBEAT Packet (Type 0x01)
**Length**: 10 bytes
```
Byte 0: sync (0xAA)
Byte 1: type (0x01)
Byte 2: length (0x0A)
Byte 3-4: crc16
Byte 5-8: timestamp (uint32, ms since boot)
Byte 9: status flags
  - bit 0: armed (1) / disarmed (0)
  - bit 1: connected (1) / disconnected (0)
  - bit 2: error present (1)
  - bit 3-7: reserved
Byte 10: trailer (0x55)
```

#### TELEMETRY Packet (Type 0x02)
**Length**: 34 bytes
```
Byte 0: sync (0xAA)
Byte 1: type (0x02)
Byte 2: length (0x22)
Byte 3-4: crc16
Byte 5-8: timestamp (uint32)
Byte 9: armed (1) / mode (bits 0-4)
Byte 10-13: roll (float, radians)
Byte 14-17: pitch (float, radians)
Byte 18-21: yaw (float, radians)
Byte 22-25: altitude (float, meters)
Byte 26-29: battery_voltage (float, V)
Byte 30-33: reserved
Byte 34: trailer (0x55)
```

#### COMMAND Packet (Type 0x03)
**Length**: 25 bytes
```
Byte 0: sync (0xAA)
Byte 1: type (0x03)
Byte 2: length (0x19)
Byte 3-4: crc16
Byte 5-6: roll (int16, 1000-2000)
Byte 7-8: pitch (int16, 1000-2000)
Byte 9-10: throttle (int16, 1000-2000)
Byte 11-12: yaw (int16, 1000-2000)
Byte 13-14: aux1 (int16)
Byte 15-16: aux2 (int16)
Byte 17-18: aux3 (int16)
Byte 19-20: aux4 (int16)
Byte 21: command_id
  - 0x01: ARM
  - 0x02: DISARM
  - 0x03: SET_MODE
  - 0x04: CALIBRATE_IMU
Byte 22-28: reserved
Byte 29: trailer (0x55)
```

#### RAW_SENSOR Packet (Type 0x05)
**Length**: 32 bytes
```
Byte 0: sync (0xAA)
Byte 1: type (0x05)
Byte 2: length (0x20)
Byte 3-4: crc16
Byte 5-8: timestamp (uint32)
Byte 9-10: accel_x (int16, raw)
Byte 11-12: accel_y (int16, raw)
Byte 13-14: accel_z (int16, raw)
Byte 15-16: gyro_x (int16, raw)
Byte 17-18: gyro_y (int16, raw)
Byte 19-20: gyro_z (int16, raw)
Byte 21-22: mag_x (int16, raw) [optional]
Byte 23-24: mag_y (int16, raw) [optional]
Byte 25-26: mag_z (int16, raw) [optional]
Byte 27-28: baro_press (int32, raw) [optional]
Byte 29-30: baro_temp (int16, raw) [optional]
Byte 31: sensor_status
  - bit 0: IMU OK
  - bit 1: Baro OK
  - bit 2: Mag OK
  - bit 3: GPS OK
Byte 32: trailer (0x55)
```

### 3.2 NRF24L01+ Protocol Specification

#### Physical Layer
| Parameter | Value |
|-----------|-------|
| Max Speed | 2 Mbps |
| Channels | 0-125 (2400-2525 MHz) |
| Default Channel | 76 |
| Address Width | 5 bytes |
| Payload Width | 32 bytes max |

#### Default Addresses
| Direction | Address |
|-----------|---------|
| Drone TX → Companion RX | 0x52, 0x46, 0x32, 0x34, 0x01 |
| Companion TX → Drone RX | 0x52, 0x46, 0x32, 0x34, 0x02 |

#### NRF24 Frame Structure
```
┌────────┬──────┬────────┬───────┬────────┬────────┐
│ PREAM  │ ADDR │  TYPE  │ SEQ   │ DATA   │  CRC   │
│ 5 byte │ 5    │  1     │  1     │ N      │ 2 bytes│
│ auto   │      │        │       │        │        │
└────────┴──────┴────────┴───────┴────────┴────────┘
```

#### NRF24 Packet Types

| Type | Name | Direction | Description |
|------|------|-----------|-------------|
| 0x01 | HEARTBEAT | ↔ | Link heartbeat |
| 0x02 | TELEMETRY | Drone→Comp | Flight data |
| 0x03 | COMMAND | Comp→Drone | Control input |
| 0x04 | ACK | Drone→Comp | Acknowledge |
| 0x05 | NACK | Drone→Comp | Negative ack |
| 0x06 | CONFIG | ↔ | Link configuration |
| 0x07 | PAIRING | ↔ | Device pairing |

#### NRF24 Packet Header (4 bytes)
```
Byte 0: sync (0xAA)
Byte 1: type
Byte 2: length (payload only)
Byte 3: sequence (for ACK/NACK)
```

#### CRC16 for NRF24
Same polynomial as SPI (0xA001)

### 3.3 NRF24 Transmission Modes

#### Single Radio Mode
- One NRF24 module
- Half-duplex operation
- TX/RX alternation required
- Max throughput: ~4 kbps at 2 Mbps

#### Half-Duplex Dual Radio Mode
- Two NRF24 modules (TX dedicated, RX dedicated)
- Simultaneous TX and RX possible
- Reduces latency
- Max throughput: ~8 kbps

#### Full-Duplex Dual Radio Mode
- Two NRF24 modules
- Continuous bidirectional communication
- Lowest latency
- Requires careful timing

---

### 3.4 ESP-NOW RC Protocol Specification

ESP-NOW is the primary wireless link between the RC transmitter (handheld) and receiver (drone).

#### Physical Layer
| Parameter | Value |
|-----------|-------|
| Protocol | ESP-NOW |
| Frequency | 2.4 GHz (WiFi channels 1-11) |
| Max Range | ~100m (line of sight) |
| Max Payload | 250 bytes |
| Encryption | None (default) |

#### RC Data Packet (ESP-NOW)
```
Type: 0x01
Size: ~37 bytes

Byte 0: type (0x01)
Byte 1: channel_count (1-16)
Bytes 2-3: ch[0] (roll)
Bytes 4-5: ch[1] (pitch)
Bytes 6-7: ch[2] (throttle)
Bytes 8-9: ch[3] (yaw)
Bytes 10-11: ch[4] (aux1)
... up to 16 channels
Bytes N-2: timestamp (4 bytes)
Bytes N-1: rssi (1 byte)
```

#### RC Status Packet (Serial to PC/FC)
Sent every 100ms from RC receiver.

```
Sync: 0x66
Type: 0x03
Size: 48 bytes

Byte 0-3: uptime (uint32, ms)
Byte 4-7: batteryVoltage (float)
Byte 8: batteryPercent (int8)
Byte 9: txRssi (int8)
Byte 10: txSignalQuality (int8, 0-100)
Byte 11-12: txPacketLoss (uint16)
Byte 13-16: txLatency (uint32, us)
Byte 17-20: temperature (float)
Byte 21-24: freeMemory (uint32)
Byte 25: cpuLoad (uint8)
Byte 26: espnowChannel (uint8)
Byte 27: connectionState (uint8)
Byte 28: errorFlags (uint8)
Bytes 29-47: reserved
```

#### FC Telemetry Packet (Serial from FC)
Forwarded from FC through RC receiver to PC.

```
Sync: 0x55
Type: 0x02
Size: 80 bytes

See FC Telemetry Structure in Section 3.5
```

---

## 4. Standard Mavlink Messages

### 4.1 PID Tuning Command (ID: 200)
**Direction**: Companion → Drone  
**Usage**: Control PID auto-tuning process

#### Packet Structure (13 bytes payload)

```
Byte 0: Command
  0x01 = Start tuning
  0x02 = Stop tuning
  0x03 = Abort tuning
  0x04 = Set method
  0x05 = Set axis
  0x06 = Set amplitude
  0x07 = Get status
  0x08 = Get result

Bytes 1-2 (for Start 0x01):
  Byte 1: Method (0-3)
  Byte 2: Axis (0-7)

Bytes 3-6 (optional):
  Float: Amplitude

Byte 1 (for Set Method 0x04):
  Byte 1: Method (0-3)

Byte 1 (for Set Axis 0x05):
  Byte 1: Axis (0-7)
```

#### Tuning Methods

| Value | Method | Description |
|-------|--------|-------------|
| 0 | ZIEGLER_NICHOLS | Ultimate gain oscillation method |
| 1 | RELAY | Åström-Hägglund relay feedback |
| 2 | STEP_RESPONSE | Cohen-Coon first-order analysis |
| 3 | FREQUENCY_SWEEP | Bode plot frequency response |

#### Tuning Axes

| Value | Axis | Description |
|-------|------|-------------|
| 0 | ROLL_RATE | Roll rate PID |
| 1 | PITCH_RATE | Pitch rate PID |
| 2 | YAW_RATE | Yaw rate PID |
| 3 | ROLL_ATTITUDE | Roll attitude PID |
| 4 | PITCH_ATTITUDE | Pitch attitude PID |
| 5 | YAW_ATTITUDE | Yaw attitude PID |
| 6 | ALTITUDE | Altitude hold PID |
| 7 | POSITION | Position control PID |

### 4.2 PID Tuning Status (ID: 201)
**Direction**: Drone → Companion  
**Frequency**: On request or 5 Hz during tuning

#### Packet Structure (16 bytes payload)

```
Byte 0: State (0-6)
  0 = IDLE
  1 = WAITING_FOR_EXCITATION
  2 = COLLECTING_DATA
  3 = ANALYZING
  4 = APPLYING
  5 = COMPLETE
  6 = FAILED

Byte 1: Method
Byte 2: Axis
Bytes 3-4: Progress (uint16, 0-10000 = 0.00-100.00%)
Bytes 5-8: Timestamp (uint32, ms since tune start)
Byte 9: Active (1=actively tuning, 0=idle)
Bytes 10-15: Reserved
```

### 4.3 PID Tuning Result (ID: 202)
**Direction**: Drone → Companion  
**Trigger**: After tuning completes

#### Packet Structure (32 bytes payload)

```
Byte 0: Success (1=success, 0=failed)

Bytes 1-4: Kp (float)
Bytes 5-8: Ki (float)
Bytes 9-12: Kd (float)

Bytes 13-16: Overshoot (float)
Bytes 17-20: Settling time (float, seconds)
Bytes 21-24: Rise time (float, seconds)

Bytes 25-28: Tune time (uint32, milliseconds)
Bytes 29-31: Reserved
```

### 4.4 PID Gains (ID: 203)
**Direction**: Companion ↔ Drone  
**Usage**: Read/write single axis or full profile

#### Single Axis (13 bytes)
```
Byte 0: Axis (0-7)
Bytes 1-4: Kp (float)
Bytes 5-8: Ki (float)
Bytes 9-12: Kd (float)
```

#### Full Profile (120 bytes)
```
Bytes 0-3: roll_rate_kp
Bytes 4-7: roll_rate_ki
Bytes 8-11: roll_rate_kd
Bytes 12-15: pitch_rate_kp
Bytes 16-19: pitch_rate_ki
Bytes 20-23: pitch_rate_kd
Bytes 24-27: yaw_rate_kp
Bytes 28-31: yaw_rate_ki
Bytes 32-35: yaw_rate_kd
Bytes 36-39: roll_att_kp
Bytes 40-43: roll_att_ki
Bytes 44-47: roll_att_kd
Bytes 48-51: pitch_att_kp
Bytes 52-55: pitch_att_ki
Bytes 56-59: pitch_att_kd
Bytes 60-63: yaw_att_kp
Bytes 64-67: yaw_att_ki
Bytes 68-71: yaw_att_kd
Bytes 72-75: alt_kp
Bytes 76-79: alt_ki
Bytes 80-83: alt_kd
Bytes 84-87: pos_kp
Bytes 88-91: pos_ki
Bytes 92-95: pos_kd
Bytes 96-99: version (uint32)
Bytes 100-103: timestamp (uint32)
Bytes 104-119: Reserved
```

### 4.5 PID Profile Commands (ID: 204)
**Direction**: Companion → Drone

```
Byte 0: Command
  0x10 = Save profile to slot
  0x11 = Load profile from slot
  0x12 = Reset to defaults
  0x21 = Request all gains

Byte 1: Profile slot (0-3, for save/load)
```

---

## 5. Flight Modes

### 5.1 Mode Enumeration

| Mode | Value | Description |
|------|-------|-------------|
| MANUAL | 0 | Direct RC passthrough |
| ANGLE_MODE | 1 | Self-leveling attitude control |
| HORIZON_MODE | 2 | Angle mode with rate limiting |
| ALTHOLD_MODE | 3 | Altitude hold using baro |
| LOITER_MODE | 4 | GPS position hold |
| RTL_MODE | 5 | Return to launch |
| AUTO_MODE | 6 | Autonomous mission |
| AUTOTUNE_MODE | 7 | PID auto-tuning |
| FPV_MODE | 8 | First-person view mode |
| CINEMATIC_MODE | 9 | Smooth cinematic movements |

### 5.2 Mode Selection
- Sent via `custom_mode` in HEARTBEAT message
- Companion must send HEARTBEAT with desired mode
- Drone ACKs with its current mode

---

## 6. Telemetry Stream

### 6.1 Default Rates

| Message | Rate | Direction |
|---------|------|-----------|
| HEARTBEAT | 1 Hz | Drone |
| ATTITUDE | 100 Hz | Drone |
| GLOBAL_POSITION_INT | 10 Hz | Drone |
| VFR_HUD | 10 Hz | Drone |
| BATTERY_STATUS | 1 Hz | Drone |
| RC_CHANNELS | 50 Hz | Drone |
| PID Tuning Status | 5 Hz | Drone (during tuning) |

### 6.2 Requesting Message Streams

Use MAV_CMD_SET_MESSAGE_INTERVAL (command_long, ID 511):
```
param1: Message ID to request
param2: Interval in microseconds (1000000/frequency)
```

Example: Request ATTITUDE at 200 Hz
```
param1 = 30 (ATTITUDE ID)
param2 = 5000 (microseconds)
```

---

## 7. RC Channel Mapping

### 7.1 Input Channels (Companion → Drone)

| Channel | Input | Range | Neutral |
|---------|-------|-------|---------|
| 1 | Roll | 1000-2000 | 1500 |
| 2 | Pitch | 1000-2000 | 1500 |
| 3 | Throttle | 1000-2000 | 1000 |
| 4 | Yaw | 1000-2000 | 1500 |
| 5 | Aux1 | 1000/2000 | Mode switch |
| 6 | Aux2 | 1000/2000 | Flight mode modifier |
| 7 | Aux3 | 1000/2000 | Safety overrides |
| 8 | Aux4 | 1000/2000 | Custom functions |

### 7.2 Arming Sequence

1. Set throttle to 1000 (minimum)
2. Set Aux1 to 2000 for >1 second
3. Drone sends HEARTBEAT with armed flag set

### 7.3 Disarm Sequence

1. Set throttle to 1000 (minimum)
2. Set Aux1 to 1000 for >2 seconds
3. Drone sends HEARTBEAT with disarmed flag

---

## 8. Protocol Timing

### 8.1 Timing Requirements

| Parameter | Value | Description |
|-----------|-------|-------------|
| Heartbeat timeout | 3000 ms | Drone disarms if no heartbeat |
| RC override timeout | 500 ms | Drone enters failsafe if no RC |
| Command timeout | 1000 ms | General command acknowledgment |

### 8.2 Latency Budget

```
Companion → Serial TX → Serial RX → Parse → Process → Output
     0ms      1ms      1ms       0.5ms    0.5ms     -
Total: ~3ms one-way latency
```

### 8.3 Recommended Update Rates

| Function | Rate | Period |
|----------|------|--------|
| RC Override | 50 Hz | 20 ms |
| Heartbeat | 1 Hz | 1000 ms |
| Telemetry Receive | 100 Hz | 10 ms |

---

## 9. Error Handling

### 9.1 Failsafe States

| State | Trigger | Action |
|-------|---------|--------|
| RC Loss | No RC for 500ms | Hover or RTL |
| Heartbeat Loss | No heartbeat for 3s | Land or RTL |
| Low Battery | <20% capacity | RTL |
| GPS Loss | No GPS fix for 10s | AltHold mode |

### 9.2 Error Codes

| Code | Name | Description |
|------|------|-------------|
| 0 | NONE | No error |
| 1 | IMU_ERROR | IMU failure/drift |
| 2 | MOTOR_ERROR | Motor failure detected |
| 3 | GPS_ERROR | GPS malfunction |
| 4 | RC_ERROR | RC receiver lost |
| 5 | BATTERY_CRITICAL | Battery below 10% |

---

## 10. Data Types Reference

### 10.1 Primitive Types

| Type | Size | Range |
|------|------|-------|
| int8 | 1 byte | -128 to 127 |
| uint8 | 1 byte | 0 to 255 |
| int16 | 2 bytes | -32768 to 32767 |
| uint16 | 2 bytes | 0 to 65535 |
| int32 | 4 bytes | ±2.1×10⁹ |
| uint32 | 4 bytes | 0 to 4.3×10⁹ |
| float | 4 bytes | IEEE 754 single |
| double | 8 bytes | IEEE 754 double |

### 10.2 Custom Types

#### PIDGains (13 bytes)
```python
struct PIDGains:
    axis: uint8      # 0-7
    kp: float        # Proportional gain
    ki: float        # Integral gain
    kd: float        # Derivative gain
```

#### PIDProfile (120 bytes)
```python
struct PIDProfile:
    roll_rate_kp/ki/kd: float
    pitch_rate_kp/ki/kd: float
    yaw_rate_kp/ki/kd: float
    roll_att_kp/ki/kd: float
    pitch_att_kp/ki/kd: float
    yaw_att_kp/ki/kd: float
    alt_kp/ki/kd: float
    pos_kp/ki/kd: float
    version: uint32
    timestamp: uint32
```

---

## 11. Companion Computer Reference

### 11.1 Minimum Requirements

| Component | Specification |
|-----------|---------------|
| CPU | ARM Cortex-A or x86 |
| RAM | 256 MB |
| Storage | 4 GB |
| OS | Linux (Raspbian, Ubuntu) or similar |
| USB | USB 2.0+ |
| Network | WiFi/Ethernet (optional) |

### 11.2 Recommended Platforms

- Raspberry Pi 4B
- NVIDIA Jetson Nano/Xavier NX
- BeagleBone Black
- Any Linux SBC with USB host

### 11.3 Software Dependencies

```
Python 3.8+:
  - pymavlink >= 2.4.0
  - pyserial >= 3.5
  - numpy >= 1.20

Optional:
  - OpenCV (for computer vision)
  - ROS2 (robotics framework)
```

---

## 12. Example Implementation

### 12.1 Python - Basic Connection

```python
from pymavlink import mavutil

# Connect to drone
master = mavutil.mavserial_connection('/dev/ttyACM0', baud=115200)

# Wait for heartbeat
master.wait_heartbeat()

# Arm the drone
master.mav.rc_channels_override_send(
    master.target_system, master.target_component,
    1500, 1500, 1000, 1500,  # Roll, Pitch, Throttle, Yaw
    2000, 1000, 1000, 1000  # Aux1=ARM, others default
)

# Set to Angle Mode
master.set_mode('STABILIZE')

# Control loop
while True:
    # Receive messages
    msg = master.recv_match(blocking=True)
    
    if msg.get_type() == 'ATTITUDE':
        print(f"Roll: {msg.roll}, Pitch: {msg.pitch}, Yaw: {msg.yaw}")
    
    # Send RC (50 Hz)
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        1500, 1500, 1200, 1500,  # Add some throttle
        2000, 1000, 1000, 1000
    )
```

### 12.2 Python - PID Tuning

```python
from pymavlink import mavutil
import struct

def send_pid_command(master, cmd, data=b''):
    payload = bytes([cmd]) + data
    master.mav.payload16_send(payload)

# Start tuning (Relay method, Roll Rate axis)
send_pid_command(master, 0x01, bytes([1, 0]) + struct.pack('<f', 50.0))

# Poll for status
while True:
    msg = master.recv_match(type='ATTITUDE', blocking=True)
    
    if tuning_complete:
        # Get result
        result = receive_pid_result(master)
        print(f"Kp={result.kp}, Ki={result.ki}, Kd={result.kd}")

# Apply gains
def apply_gains(master, axis, kp, ki, kd):
    data = struct.pack('<Bfff', axis, kp, ki, kd)
    send_pid_command(master, 0x20, data)  # CMD_SET_SINGLE_GAIN
```

### 12.3 C++ - Minimal Implementation

```cpp
#include <mavlink.h>
#include <serialport.h>

class DroneInterface {
    SerialPort serial;
    uint8_t system_id = 255;  // Companion
    uint8_t component_id = 0;
    
public:
    bool connect(const char* port, int baudrate) {
        return serial.open(port, baudrate);
    }
    
    void send_rc(int roll, int pitch, int throttle, int yaw) {
        mavlink_message_t msg;
        mavlink_msg_rc_channels_override_encode(
            system_id, component_id, &msg,
            target_system, target_component,
            roll, pitch, throttle, yaw,
            1500, 1500, 1500, 1500
        );
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        serial.write(buf, len);
    }
    
    mavlink_attitude_t receive_attitude() {
        mavlink_message_t msg;
        if (serial.read(&msg, sizeof(msg))) {
            if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE) {
                mavlink_attitude_t att;
                mavlink_msg_attitude_decode(&msg, &att);
                return att;
            }
        }
        return {0};
    }
};
```

---

## 13. Appendix: Message ID Reference

### Standard Mavlink Messages

| ID | Name | Direction | Rate |
|----|------|-----------|------|
| 0 | HEARTBEAT | ↔ | 1 Hz |
| 1 | SYS_STATUS | → | 1 Hz |
| 2 | GPS_RAW_INT | → | 10 Hz |
| 6 | SET_MODE | ↔ | On change |
| 30 | ATTITUDE | → | 100 Hz |
| 33 | GLOBAL_POSITION_INT | → | 10 Hz |
| 35 | RC_CHANNELS_RAW | → | 50 Hz |
| 36 | SERVO_OUTPUT_RAW | → | 50 Hz |
| 69 | RC_CHANNELS_OVERRIDE | ← | 50 Hz |
| 74 | VFR_HUD | → | 10 Hz |
| 360 | BATTERY_STATUS | → | 1 Hz |
| 511 | COMMAND_LONG | ↔ | On request |

### Custom Messages

| ID | Name | Direction | Description |
|----|------|-----------|-------------|
| 200 | PID_TUNE_CMD | ← | Tuning control |
| 201 | PID_TUNE_STATUS | → | Tuning progress |
| 202 | PID_TUNE_RESULT | → | Tuning output |
| 203 | PID_GAINS | ↔ | Gain read/write |
| 204 | PID_PROFILE_CMD | ← | Profile management |

---

## 14. Revision History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2026-04-14 | Initial specification |
