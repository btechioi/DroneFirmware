# DroneFlightController

<p align="center">
  <a href="#">
    <img src="https://img.shields.io/github/actions/workflow/status/btechioi/DroneFirmware/build.yml?style=flat&label=Build" alt="Build">
  </a>
  <a href="#">
    <img src="https://img.shields.io/github/languages/code-size/btechioi/DroneFirmware?style=flat" alt="Code Size">
  </a>
  <a href="#">
    <img src="https://img.shields.io/github/license/btechioi/DroneFirmware?style=flat" alt="License">
  </a>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/Platform-Raspberry%20Pi%20Pico-CC0000?style=for-the-badge&logo=raspberrypi" alt="Platform">
  <img src="https://img.shields.io/badge/MCU-ESP32--C3-E7352C?style=for-the-badge" alt="MCU">
  <img src="https://img.shields.io/badge/Framework-Arduino-00979D?style=for-the-badge&logo=arduino" alt="Framework">
</p>

---

## 🚁 Overview

```
     ╭──────────────────────────────────────╮
     │           DRONE SYSTEM               │
     │  ┌─────────────────────────────┐     │
     │  │  🚀 Flight Controller       │     │
     │  │  ┌─────────────────────┐    │     │
     │  │  │    ██  ██  ██  ██  │    │     │
     │  │  │    M0  M1  M2  M3  │    │     │
     │  │  └─────────────────────┘    │     │
     │  │         │                   │     │
     │  │    ══════════════           │     │
     │  │         │ IMU               │     │
     │  └─────────┼───────────────────┘     │
     │            │                          │
     │  ┌─────────┴───────────────────┐     │
     │  │  📡 RC System               │     │
     │  │  ESP-NOW ◄──────► Transmit  │     │
     │  └─────────────────────────────┘     │
     ╰──────────────────────────────────────╯
```

A modular drone flight control system with **ESP-NOW RC**, **companion computer support**, and **Ground Control Station**.

---

## ✨ Features

| Feature | Description |
|---------|-------------|
| 🔌 **Standalone FC** | Raspberry Pi Pico flies without companion |
| 📡 **Wireless RC** | ESP-NOW auto-pairing, no manual config |
| 🔌 **Hot-Plug** | GPS, barometer, optical flow auto-detected |
| 🔊 **Audio** | Motor buzzer for failsafe cues |
| 🎛️ **PID Tuner** | Multiple methods built-in |
| 🖥️ **GCS** | PyQt6 app with joystick/keyboard |

---

## 🚀 Quick Start

```bash
# Clone and build
git clone https://github.com/btechioi/DroneFirmware.git
cd DroneFirmware
./build.sh

# Flash Pico
cp firmware/firmware.uf2 /media/$USER/RPI-RP2/

# Flash ESP32
esptool.py --chip esp32c3 --port /dev/ttyUSB0 write_flash 0x0 firmware/rc_receiver_esp32c3.bin
```

---

## 🏗️ Architecture

```
                        ┌─────────────────┐
                        │   Ground Station │
                        │     DroneGCS     │
                        │  🎮 Joystick     │
                        └────────┬────────┘
                                 │ USB/UDP
                    ┌────────────┴────────────┐
                    │                         │
              ╔═════╧═════╗             ╔═════╧═════╗
              ║ TRANSMITTER║             ║ RECEIVER  ║
              ║  (Handheld)║◄──ESP-NOW──►║ (On Drone)║
              ╚═════╤═════╝             ╚═════╤═════╝
                    │                         │
                    │ USB                      │ Serial 2Mbps
                    ▼                         ▼
              ┌───────────┐            ╔═══════════════════╗
              │   PC/GCS  │            ║ Flight Controller  ║
              └───────────┘            ║  Raspberry Pico    ║
                                       ╠═══════════════════╣
                                       ║  IMU │ Motors    ║
                                       ║  GPS │ Baro     ║
                                       ╚══════╧═══════════╝
                                                 │
                                            SPI │ 125MHz
                                                 ▼
                                       ╔═══════════════════╗
                                       ║   Companion       ║
                                       ║   Pi Zero 2W      ║
                                       ║  Autopilot/Vision ║
                                       ╚═══════════════════╝
```

---

## 📦 Hardware

| Component | Model | Status |
|-----------|-------|--------|
| 🖥️ Flight Controller | Raspberry Pi Pico | ✅ Required |
| ⚙️ IMU | MPU6050 | ✅ Required |
| 📡 RC Receiver | ESP32-C3/S3 | ✅ Required |
| 🎮 RC Transmitter | ESP32-C3/S3 | ⚙️ Optional |
| 🛰️ GPS | u-blox NEO-M8N | ⭕ Optional |
| 🌡️ Barometer | BMP280 | ⭕ Optional |
| 🤖 Companion | Pi Zero 2W | ⭕ Optional |

---

## 📁 Firmware Output

```
firmware/
├── firmware.uf2                 ████████████  Pico FC
├── rc_receiver_esp32c3.bin      ██████████    ESP32-C3
├── rc_receiver_esp32s3.bin      ██████████    ESP32-S3
├── rc_transmitter_esp32c3.bin   ██████████    ESP32-C3 TX
└── rc_transmitter_esp32s3.bin  ██████████    ESP32-S3 TX
```

---

## 📻 RC Modes

| Mode | Flow | Use Case |
|------|------|----------|
| **RECEIVER** | ESP-NOW → SBUS/Serial → FC | Drone-side RC |
| **TRANSMITTER** | PC USB → ESP-NOW → Receiver | Handheld/PC |
| **BRIDGE** | PC ↔ FC passthrough | Direct control |

---

## 🔊 Audio Cues

```
┌─────────────────────────────────────────────────────┐
│  🎵 Audio Feedback System                           │
├─────────────────────────────────────────────────────┤
│                                                     │
│   🚀 Ready     ▸ ● ─ ●  System startup            │
│   ✅ Armed      ▸ ● ● ●  Motors armed               │
│   ⏹️ Disarmed   ▸ ● ● ─  Motors stopped             │
│   📡 RC Lost    ▸ ● ─ ● ─  Signal lost             │
│   📡 RC Found   ▸ ● ● ●    Signal restored          │
│   🔍 Searching  ▸ ● ─ ● ─  Looking for RC          │
│   📍 Find Drone ▸ ♪♪♪♪♪♪  LOUD SIREN (all motors) │
│                                                     │
└─────────────────────────────────────────────────────┘
```

---

## ⚡ Performance

| Loop | Rate | Budget |
|------|------|--------|
| 🏃 Fast Loop | **400 Hz** | <2.5ms |
| 🎯 Attitude | **200 Hz** | <5ms |
| 📍 Position | **50 Hz** | <20ms |

---

## ⚠️ Safety

> ```diff
> - WARNING: Remove props before firmware testing
> - WARNING: Verify RC link before arming  
> - WARNING: Test failsafes in safe environment
> ```

---

## 📚 Project Structure

```
DroneFirmware/
├── 📄 README.md              # This file
├── 🔧 build.sh               # Build script
├── ⚙️  platformio.ini         # FC config
├── 📁 firmware/               # Built binaries
├── 📁 src/                    # Flight controller
│   ├── main/
│   └── systems/
│       ├── control/
│       ├── comms/
│       └── sensors/
├── 📁 esp32-rc/              # RC firmware
│   └── src/main.cpp
├── 📁 companion/              # Pi Zero code
│   └── companion/
├── 📁 DroneGCS/              # Ground station
│   └── drone_gcs/
└── 📄 SPEC.md                # Protocol specs
```

---

## 📜 License

<div align="center">

MIT License

Made with ❤️ by [btechioi](https://github.com/btechioi)

</div>
