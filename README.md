# DroneFlightController

A modular drone flight control system with companion computer support, ESP-NOW RC transmitter/receiver, and Ground Control Station.

## Table of Contents

- [System Architecture](#system-architecture)
- [Hardware](#hardware)
- [Building](#building)
- [Flashing Firmware](#flashing-firmware)
- [Pin Configuration](#pin-configuration)
- [Communication Protocols](#communication-protocols)
- [RC System](#rc-system)
- [Audio Cues](#audio-cues)
- [Ground Control Station](#ground-control-station)
- [Serial Commands](#serial-commands)
- [Performance](#performance)
- [Safety](#safety)

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           GROUND STATION                                 │
│                        DroneGCS (PyQt6)                                  │
│                   Joystick / Keyboard / Mavlink                          │
└─────────────────────────────────────────────────────────────────────────┘
                     │                                    ▲
                     │ USB/UDP                             │ Mavlink
                     ▼                                    │
┌─────────────────────────────────────────────────────────────────────────┐
│                      ESP32-C3 RC SYSTEM                                  │
│                                                                          │
│   ┌─────────────────┐           ESP-NOW          ┌─────────────────┐  │
│   │   TRANSMITTER   │  ◄─────────────────────►   │    RECEIVER     │  │
│   │   (Handheld)    │                             │    (Drone)      │  │
│   │                 │   RC Data + Telemetry       │                 │  │
│   │  PC USB Input   │                             │  SBUS → FC      │  │
│   │  Joystick/Stick │                             │  Serial → FC    │  │
│   └─────────────────┘                             └─────────────────┘  │
└─────────────────────────────────────────────────────────────────────────┘
                     │                                    ▲
                     │ Serial 2Mbps                       │ Telemetry
                     ▼                                    │
┌─────────────────────────────────────────────────────────────────────────┐
│                      FLIGHT CONTROLLER                                   │
│                   (Raspberry Pi Pico RP2040)                             │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  ┌────────────┐ │
│  │  IMU MPU6050 │  │   Motors     │  │   Baro/GPS   │  │   Audio    │ │
│  │  (Required)  │  │   4-8ch      │  │  (Optional)  │  │  Buzzer    │ │
│  └──────────────┘  └──────────────┘  └──────────────┘  └────────────┘ │
└─────────────────────────────────────────────────────────────────────────┘
                     │
                     │ SPI 125MHz
                     ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                    COMPANION COMPUTER                                     │
│                  (Raspberry Pi Zero 2W)                                   │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐               │
│  │  Optical Flow │  │  Autopilot    │  │   Vision      │               │
│  │   PMW3901    │  │  Waypoints    │  │   Processing  │               │
│  └──────────────┘  └──────────────┘  └──────────────┘               │
└─────────────────────────────────────────────────────────────────────────┘
```

### Design Principles

1. **IMU Required**: Flight controller requires only MPU6050 IMU to fly
2. **Hot-Pluggable Sensors**: GPS, barometer, optical flow auto-detected
3. **Graceful Degradation**: Can fly with failed motors or sensors
4. **Standalone FC**: Companion computer optional - FC flies alone
5. **Auto-Pair RC**: No manual pairing needed for ESP-NOW RC

---

## Hardware

### Required Components

| Component | Model | Purpose |
|-----------|-------|---------|
| Flight Controller | Raspberry Pi Pico | Main FC |
| IMU | MPU6050 | Attitude sensing |
| RC Receiver | ESP32-C3 | Wireless RC |
| Motors | 4x Brushed/Brushless | Propulsion |
| ESC/BEC | 5V regulator | Power |

### Optional Components

| Component | Model | Purpose |
|-----------|-------|---------|
| RC Transmitter | ESP32-C3 | Handheld controller |
| GPS | u-blox NEO-M8N | Position |
| Barometer | BMP280 | Altitude |
| Optical Flow | PMW3901 | Position estimation |
| Companion Computer | Pi Zero 2W | Autopilot/Vision |

### RC Transmitter Options

| Mode | Description | Input Source |
|------|-------------|--------------|
| RECEIVER | RC→SBUS→FC | ESP-NOW from TX |
| TRANSMITTER | PC→ESP-NOW→RX | USB from PC/GCS |
| BRIDGE | PC↔FC passthrough | USB serial |

---

## Building

### Prerequisites

```bash
# Install PlatformIO
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py | python

# Or via pip
pip install platformio

# Python dependencies for ESP32 builds
uv venv .venv
uv pip install intelhex esptool
```

### Build All Firmware

```bash
# Single command builds everything
./build.sh
```

Output:
```
firmware/
├── firmware.uf2                # Pico FC (copy to Pico via USB)
├── rc_receiver_esp32c3.bin     # RC Receiver for ESP32-C3
├── rc_receiver_esp32s3.bin     # RC Receiver for ESP32-S3
├── rc_transmitter_esp32c3.bin  # RC Transmitter for ESP32-C3
└── rc_transmitter_esp32s3.bin  # RC Transmitter for ESP32-S3
```

### Build Individual Components

```bash
# Flight Controller (Pico)
pio run

# RC Receiver ESP32-C3
pio run -d esp32-rc -e esp32c3-rc

# RC Transmitter ESP32-C3
pio run -d esp32-rc -e esp32c3-tx
```

---

## Flashing Firmware

### Flight Controller (Pico)

**Method 1: Copy UF2 file**
```bash
# Pico appears as USB drive when BOOTSEL pressed
cp firmware/firmware.uf2 /media/$USER/RPI-RP2/
```

**Method 2: PlatformIO**
```bash
pio run --target upload
```

### ESP32 RC Module

```bash
# ESP32-C3 Receiver
esptool.py --chip esp32c3 --port /dev/ttyUSB0 write_flash \
    0x0000 firmware/rc_receiver_esp32c3.bin

# ESP32-S3 Receiver
esptool.py --chip esp32s3 --port /dev/ttyUSB0 write_flash \
    0x0000 firmware/rc_receiver_esp32s3.bin
```

### Flash via PlatformIO

```bash
# RC Receiver
pio run -d esp32-rc -e esp32c3-rc --target upload

# RC Transmitter
pio run -d esp32-rc -e esp32c3-tx --target upload --upload-port /dev/ttyUSB0
```

---

## Pin Configuration

### Flight Controller (Pico)

```
┌─────────────────────────────────────────────────────┐
│                    Raspberry Pi Pico                │
│                                                     │
│  Motors:                                            │
│    GPIO 12  ──► Motor 0 (Front Right CW)            │
│    GPIO 13  ──► Motor 1 (Front Left CCW)           │
│    GPIO 14  ──► Motor 2 (Back Left CW)             │
│    GPIO 15  ──► Motor 3 (Back Right CCW)           │
│                                                     │
│  I2C (IMU):                                         │
│    GPIO 4   ──► SDA (IMU)                         │
│    GPIO 5   ──► SCL (IMU)                         │
│                                                     │
│  RC Input (optional):                               │
│    GPIO 16  ──► RC Channel 1                       │
│    GPIO 17  ──► RC Channel 2                       │
│    ...                                              │
│    GPIO 23  ──► RC Channel 8                       │
│                                                     │
│  SPI (Companion):                                  │
│    GPIO 8   ──► MISO                               │
│    GPIO 9   ──► CS                                 │
│    GPIO 10  ──► SCK                                │
│    GPIO 11  ──► MOSI                               │
│                                                     │
│  UART (RC Receiver):                               │
│    GPIO 0   ──► RX (from RC RX)                   │
│    GPIO 1   ──► TX (to RC RX)                     │
│                                                     │
└─────────────────────────────────────────────────────┘
```

### ESP32-C3 RC Receiver

```
┌─────────────────────────────┐
│        ESP32-C3             │
│                             │
│  GPIO 0  ──► ADC Joystick X │
│  GPIO 1  ──► ADC Joystick Y │
│  GPIO 2  ──► ADC Throttle   │
│  GPIO 3  ──► ADC Rudder     │
│                             │
│  GPIO 4  ──► Switch 1      │
│  GPIO 5  ──► Switch 2      │
│  GPIO 6  ──► Switch 3      │
│  GPIO 7  ──► Switch 4      │
│                             │
│  GPIO 8  ──► LED (status)   │
│                             │
│  UART1:                     │
│    GPIO 4  ──► TX (SBUS)    │
│    GPIO 5  ──► RX           │
│                             │
│  USB-CDC ──► Serial Debug   │
│                             │
└─────────────────────────────┘
```

---

## Communication Protocols

### Serial Protocol (RC ↔ FC)

**Baud Rate**: 2 Mbps  
**Format**: 8N1

#### RC → FC Packet (0xAA 0x01)

```
Offset  Size  Description
------  ----  -----------
0       1     Sync: 0xAA
1       1     Type: 0x01
2       1     Channel count (N)
3       2N    Channel values (1000-2000, little-endian)
3+2N    2     CRC16
```

#### FC → RC Telemetry (0x55 0x02)

```
Offset  Size  Description
------  ----  -----------
0       1     Sync: 0x55
1       1     Type: 0x02
2       1     Flags (armed, failsafe, etc.)
3       6     Attitude (roll, pitch, yaw × int16)
9       4     Altitude (int32, cm)
13      6     Gyro rates (3 × int16)
19      6     GPS lat/lon (if available)
25      2     Battery voltage (mV)
27      2     CRC16
```

#### RC Status (0x66 0x03)

```
Offset  Size  Description
------  ----  -----------
0       1     Sync: 0x66
1       1     Type: 0x03
2       4     Uptime (ms)
6       2     Battery voltage (mV)
8       1     Battery percent
9       1     RSSI (dBm)
10      1     Signal quality (%)
11      2     Packet loss (×10 %)
13      4     Latency (μs)
17      2     Temperature (×10 °C)
19      4     Free memory
23      1     CPU load (%)
24      1     ESP-NOW channel
25      1     Connection state
26      1     Error flags
27      2     CRC16
```

### ESP-NOW Protocol

**Max Payload**: 250 bytes  
**Data Rate**: 2 Mbps

#### RC Data Packet (Transmitter → Receiver)

```cpp
struct ESPNOWPacket {
    uint8_t magic;           // 0xAA
    uint8_t type;             // 0x01
    uint32_t timestamp;        // μs since boot
    uint8_t channelCount;     // 4-16
    uint16_t channels[16];     // 1000-2000
    int8_t rssi;              // Signal strength
    uint8_t flags;            // Low battery, etc.
};
```

### SPI Protocol (FC ↔ Companion)

**Clock**: 125 MHz  
**Mode**: SPI_MODE0

```
┌──────────┬──────────┬──────────┬──────────┬──────────┐
│   CMD    │  LENGTH  │          │          │          │
│  1 byte  │  2 bytes │  DATA    │  DATA    │   CRC    │
│          │ (BE)      │  N bytes │  ...     │  1 byte  │
└──────────┴──────────┴──────────┴──────────┴──────────┘
```

**Commands**:
- 0x01: PING
- 0x02: RC Channels
- 0x03: Telemetry
- 0x04: Settings

---

## RC System

### Modes

#### RECEIVER Mode (on drone)
- Receives RC via ESP-NOW
- Outputs SBUS/CRSF/Serial to FC
- Sends telemetry back to transmitter
- Auto-scans for transmitter

#### TRANSMITTER Mode (handheld)
- Receives RC from PC via USB
- Forwards via ESP-NOW to receiver
- Receives telemetry from receiver
- Forwards telemetry to PC

#### BRIDGE Mode
- Direct passthrough PC ↔ FC
- No wireless link
- For testing/debugging

### Auto-Pairing

The RC system uses ESP-NOW broadcast for auto-pairing:

1. **Receiver starts**: Enters SEARCHING state, plays audio cue
2. **Transmitter starts**: Broadcasts RC data on random channel
3. **Receiver finds TX**: Pairs, plays FOUND cue
4. **Connection established**: Both enter CONNECTED state
5. **Link lost**: Receiver enters RECONNECTING, auto-retries

### Connection States

| State | LED | Audio | Description |
|-------|-----|-------|-------------|
| DISCONNECTED | Off | - | No peer |
| SEARCHING | Blue blink | Searching tone | Scanning for TX |
| PAIRING | Blue rapid | - | Establishing link |
| CONNECTED | Green solid | Connected tone | Link active |
| RECONNECTING | Orange blink | Lost tone | Retrying connection |

---

## Audio Cues

The flight controller uses motors as audio buzzers:

### Tone Reference

| Tone | Frequency | Duration | Motors | Description |
|------|-----------|----------|--------|-------------|
| ARMED_SUCCESS | 800→1200 Hz | 200ms | 1 | Armed confirmation |
| DISARMED | 1200→800 Hz | 200ms | 1 | Disarmed |
| FAILSAFE_ENTER | 1000/800 Hz | 400ms | 1 | Failsafe triggered |
| FAILSAFE_EXIT | 1000→1200 Hz | 200ms | 1 | Failsafe cleared |
| LOW_BATTERY | 800 Hz | 150ms | 1 | Battery < 25% |
| CRITICAL_BATTERY | 600 Hz | 200ms | 1 | Battery < 15% |
| RC_SEARCHING | 600/900 Hz | 600ms | 1 | Looking for RC |
| RC_FOUND | 1200→1800 Hz | 150ms | 1 | RC paired |
| RC_LOST | 1000→800→600 Hz | 300ms | 1 | RC link lost |
| RC_CONNECTED | 800→1000→1200 Hz | 200ms | 1 | RC connected |
| GPS_LOCK | 1000→1400→1800 Hz | 200ms | 1 | GPS locked |
| CALIBRATION_START | 1000 Hz | 400ms | 1 | Cal started |
| CALIBRATION_COMPLETE | 800→1000→1200 Hz | 200ms | 1 | Cal done |
| ERROR | 400→300 Hz | 600ms | 1 | Error |
| READY | 1200 Hz | 240ms | 1 | System ready |
| FIND_DRONE | 1800/2000 Hz | 4s | ALL | Locate drone |

### Using Audio

```cpp
// Play predefined tone
motorAudio.playTone(MotorAudio::Tone::ARMED_SUCCESS);

// Custom tone
motorAudio.beep(2000, 500, 0x0F);  // 2kHz for 500ms, motors 0-3

// Update in loop
motorAudio.update();
```

---

## Ground Control Station

### Features

- **Real-time PID Tuning**: Live graphs of P/I/D terms
- **Joystick Support**: Gamepad/game controller
- **Keyboard Control**: WASD + QE for pitch/roll
- **Mavlink Protocol**: Standard drone communication
- **Connection Modes**: USB Serial, UDP, Relay

### Running GCS

```bash
cd DroneGCS
uv sync
uv run python -m drone_gcs
```

### Keyboard Controls

| Key | Action |
|-----|--------|
| W/S | Pitch forward/back |
| A/D | Roll left/right |
| Q/E | Yaw left/right |
| Space | Throttle up |
| Ctrl | Throttle down |
| R | Arm |
| Esc | Disarm |

### Connection Modes

| Mode | Description |
|------|-------------|
| USB Serial | Direct USB connection to FC |
| UDP | Network connection (drone IP) |
| Relay | RC module bridges PC ↔ FC |

---

## Serial Commands

Connect to FC via USB serial at 115200 baud:

| Command | Description |
|---------|-------------|
| `a` | Arm motors |
| `d` | Disarm motors |
| `s` | Show status |
| `f` | Show failsafe state |
| `r` | Return to RC control |
| `p` | Play find-drone siren |
| `t` | Run PID tune |

### Status Output

```
Mode: 0
Companion: Disconnected
Armed: NO
Control: RC
Failsafe: 0
```

---

## Performance

### Loop Rates

| Loop | Target Rate | Max Budget |
|------|-------------|------------|
| Fast Loop | 400 Hz | <2.5 ms |
| Attitude | 200 Hz | <5 ms |
| Position | 50 Hz | <20 ms |
| RC Input | 100 Hz | - |
| Telemetry | 100 Hz | Variable |

### Memory Usage

| Component | RAM | Flash |
|-----------|-----|-------|
| Flight Controller | 42 KB / 264 KB | 4 KB / 2 MB |
| RC Receiver | 38 KB / 320 KB | 737 KB / 8 MB |
| RC Transmitter | 38 KB / 320 KB | 737 KB / 8 MB |

---

## Safety

### Warnings

⚠️ **Motor Safety**: Always disarm before connecting battery  
⚠️ **Propeller Safety**: Remove props during firmware testing  
⚠️ **Failsafe Testing**: Test in safe environment only  
⚠️ **RC Link**: Verify RC connection before arming  
⚠️ **Battery**: Never fly with damaged battery  

### Failsafe Triggers

- RC signal loss (>500ms)
- Stuck controls (>3s no movement)
- Companion timeout (>2s)
- Low battery (<15%)

### Emergency Procedures

1. **RC Lost**: Drone hovers at throttle, waits for reconnection
2. **Companion Lost**: Returns to RC control automatically
3. **Critical Battery**: Lands immediately
4. **IMU Failure**: Disarms, prevents arming

---

## Troubleshooting

### FC Not Responding
- Check USB connection
- Verify firmware flashed (UF2 copied)
- Press BOOTSEL and reset Pico

### RC Not Connecting
- Receiver needs 3-5 seconds to scan
- Move transmitter closer
- Check LED status on receiver

### Motors Not Spinning
- Ensure armed (aux channel high)
- Check PWM signals with oscilloscope
- Verify motor wiring

### Audio Not Working
- Connect buzzer mode enabled
- Check volume setting
- Test with `p` command

---

## License

MIT License

## Support

For issues, feature requests, or contributions, please open an issue on the project repository.
