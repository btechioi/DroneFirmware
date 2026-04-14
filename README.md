# DroneFlightController

<p align="center">
  <img src="https://img.shields.io/badge/Platform-Raspberry%20Pi%20Pico-blue" alt="Platform">
  <img src="https://img.shields.io/badge/MCU-ESP32--C3-orange" alt="MCU">
  <img src="https://img.shields.io/badge/Framework-Arduino-00979D?style=flat&logo=arduino" alt="Framework">
  <img src="https://img.shields.io/badge/Python-3.12+-blueviolet?style=flat&logo=python" alt="Python">
  <img src="https://img.shields.io/badge/License-MIT-green" alt="License">
</p>

A modular drone flight control system with **ESP-NOW RC**, **companion computer support**, and **Ground Control Station**.

## Features

- **Standalone FC** - Raspberry Pi Pico flies without companion
- **Wireless RC** - ESP-NOW auto-pairing, no manual config
- **Hot-Plug Sensors** - GPS, barometer, optical flow auto-detected
- **Audio Feedback** - Motor buzzer for failsafe cues
- **PID Auto-Tuner** - Multiple methods built-in
- **Ground Control** - PyQt6 app with joystick/keyboard

## Quick Start

```bash
# Build all firmware
./build.sh

# Flash Pico (copy UF2)
cp firmware/firmware.uf2 /media/$USER/RPI-RP2/

# Flash ESP32
esptool.py --chip esp32c3 --port /dev/ttyUSB0 write_flash 0x0 firmware/rc_receiver_esp32c3.bin
```

## Architecture

```
┌──────────────┐    ESP-NOW    ┌──────────────┐
│ TRANSMITTER  │◄────────────►│   RECEIVER   │
│  (PC/Hand)   │              │   (On Drone)  │
└──────┬───────┘              └───────┬──────┘
       │ USB                           │ Serial
       ▼                              ▼
┌──────────────┐              ┌──────────────────┐
│   DroneGCS   │              │ Flight Controller │
│   (PyQt6)    │              │  Raspberry Pico   │
└──────────────┘              └────────┬─────────┘
                                      │ SPI
                                      ▼
                             ┌──────────────────┐
                             │   Companion      │
                             │   Pi Zero 2W     │
                             └──────────────────┘
```

## Hardware

| Component | Model | Required |
|-----------|-------|----------|
| Flight Controller | Raspberry Pi Pico | Yes |
| IMU | MPU6050 | Yes |
| RC Receiver | ESP32-C3/S3 | Yes |
| RC Transmitter | ESP32-C3/S3 | Optional |
| GPS | u-blox NEO-M8N | No |
| Barometer | BMP280 | No |
| Companion | Pi Zero 2W | No |

## Firmware

```
firmware/
├── firmware.uf2                 # Pico FC
├── rc_receiver_esp32c3.bin     # ESP32-C3 receiver
├── rc_receiver_esp32s3.bin     # ESP32-S3 receiver
├── rc_transmitter_esp32c3.bin  # ESP32-C3 transmitter
└── rc_transmitter_esp32s3.bin  # ESP32-S3 transmitter
```

## RC Modes

| Mode | Description |
|------|-------------|
| **RECEIVER** | ESP-NOW → SBUS/Serial → FC |
| **TRANSMITTER** | PC USB → ESP-NOW → Receiver |
| **BRIDGE** | Direct PC ↔ FC passthrough |

## Audio Cues

| Tone | Trigger |
|------|---------|
| Ready | System startup |
| Armed/Disarmed | Motor state change |
| RC Lost/Found | Signal status |
| Find Drone | Loud siren (all motors) |

## Performance

| Loop | Rate | Budget |
|------|------|--------|
| Fast Loop | 400 Hz | <2.5ms |
| Attitude | 200 Hz | <5ms |
| Position | 50 Hz | <20ms |

## Safety

> ⚠️ **Remove props** before firmware testing  
> ⚠️ **Verify RC link** before arming  
> ⚠️ **Test failsafes** in safe environment

## License

MIT
