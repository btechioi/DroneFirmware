<p align="center">
  <img src="https://capsule-render.vercel.app/api?type=waving&color=0:00e1ff,100:0055ff&height=200&section=header&text=DroneFlightController&fontSize=50&fontAlignY=35&animation=fadeIn&fontColor=ffffff"/>
</p>

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

A drone flight controller that runs on Raspberry Pi Pico, with ESP-NOW RC, companion computer support, and a desktop ground station.

## Hardware

- **FC**: Raspberry Pi Pico + MPU6050 IMU
- **RC**: ESP32-C3 or ESP32-S3 (auto-pairing via ESP-NOW)
- **Optional**: GPS, barometer, optical flow
- **Companion**: Raspberry Pi Zero 2W

## Build

```bash
./build.sh
```

This creates `firmware/` with:
- `firmware.uf2` - Flash to Pico
- `rc_receiver_esp32c3.bin` - ESP32-C3 receiver
- `rc_receiver_esp32s3.bin` - ESP32-S3 receiver
- `rc_transmitter_esp32c3.bin` - ESP32-C3 transmitter
- `rc_transmitter_esp32s3.bin` - ESP32-S3 transmitter

## Flash

**Pico**: Copy `firmware.uf2` to the Pico USB drive (hold BOOTSEL to mount)

**ESP32**:
```bash
esptool.py --chip esp32c3 --port /dev/ttyUSB0 write_flash 0x0 firmware/rc_receiver_esp32c3.bin
```

## How It Works

```
                    ┌─────────────┐
                    │   GCS/PC    │
                    │  (PyQt6)    │
                    └──────┬──────┘
                           │ USB
                    ┌──────┴──────┐
                    │             │
              ┌─────┴─────┐ ┌─────┴─────┐
              │ Transmitter│ │  Receiver │
              │ (optional) │ │           │
              └─────┬─────┘ └─────┬─────┘
                    │ ESP-NOW    │
                    └─────┬───────┘
                          │ Serial
                    ┌─────┴─────┐
                    │   Pico FC   │
                    └─────┬─────┘
                          │ SPI
                    ┌─────┴─────┐
                    │ Pi Zero 2W │
                    │ (optional) │
                    └───────────┘
```

The Pico flies standalone. The ESP32 RC module receives control signals via ESP-NOW and sends them to the FC over serial. The Pi Zero can take over control via SPI.

## Features

| Feature | Notes |
|---------|-------|
| 400Hz control loop | Fast response |
| Auto-tuning PIDs | Relay, Ziegler-Nichols, etc |
| Hot-plug sensors | GPS/baro auto-detected |
| Motor audio cues | Beeps for status/failsafe |
| Triple-redundant RC | Direct, ESP-NOW, companion |

## RC System

The RC module runs in one of three modes:

| Mode | Description |
|------|-------------|
| RECEIVER | Picks up ESP-NOW, outputs SBUS/CRSF/Serial |
| TRANSMITTER | Takes input from PC USB, broadcasts ESP-NOW |
| BRIDGE | Passes PC ↔ FC directly |

Auto-pairs with any ESP-NOW peer on startup.

## Audio

The motors act as buzzers for audio feedback:

- Short beep on arm/disarm
- Rising tone when paired
- Descending tone when signal lost
- All-motors siren when you need to find the drone

## GCS

Python PyQt6 app for ground control:

```bash
cd DroneGCS && uv sync && uv run python -m drone_gcs
```

Features joystick support, keyboard fallback (WASD + QE), real-time PID graphs.

## Safety

- Remove props before testing firmware
- Verify RC link before arming
- Failsafe kicks in after 500ms without signal

## License

MIT
