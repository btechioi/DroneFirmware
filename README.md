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

A drone flight controller built on Raspberry Pi Pico that flies standalone. No companion computer needed.

## Hardware

| Part | Model | Notes |
|------|-------|-------|
| FC | Raspberry Pi Pico | 133MHz, 264KB RAM |
| IMU | MPU6050 | Required |
| RC | ESP32-C3 or ESP32-S3 | Auto-pairing |
| GPS | u-blox NEO-M8N | Optional |
| Barometer | BMP280 | Optional |
| Companion | Pi Zero 2W | Optional |

## Quick Start

```bash
git clone https://github.com/btechioi/DroneFirmware.git
cd DroneFirmware
./build.sh
```

## Flash

**Pico**: Hold BOOTSEL, plug in USB, copy `firmware.uf2` to the drive.

**ESP32**:
```bash
esptool.py --chip esp32c3 --port /dev/ttyUSB0 write_flash 0x0 firmware/rc_receiver_esp32c3.bin
```

## System Overview

```
┌──────────────────────────────────────────────────────────────┐
│                        YOUR DRONE                            │
│                                                              │
│    ┌─────────────┐      ┌─────────────────────────────┐      │
│    │   ESP32     │      │       Raspberry Pi Pico     │      │
│    │   RC RX     │──────│  FC + IMU + Motors          │      │
│    └──────┬──────┘      └─────────────────────────────┘      │
│           │                       │                          │
│      ESP-NOW              SPI (optional)                     │
│           │                       │                          │
└───────────┼───────────────────────┼──────────────────────────┘
            │                       │
    ┌───────┴───────┐       ┌──────┴──────┐
    │   TRANSMITTER │       │  Pi Zero 2W │
    │  (handheld/PC)│       │  (optional) │
    └───────────────┘       └─────────────┘
```

**Minimum setup**: Pico + MPU6050 + ESP32-C3 RC receiver

## LED Status

| LED | State | Meaning |
|-----|-------|---------|
| Green solid | Armed | Ready to fly |
| Green slow blink | Disarmed | Waiting |
| Blue double blink | Companion | SPI connected |
| Red fast blink | Failsafe | Error occurred |

## Audio Cues

| Sound | When |
|-------|------|
| 1 beep | System ready |
| 2 ascending beeps | Armed |
| 2 descending beeps | Disarmed |
| Descending tone | RC signal lost |
| Ascending tone | RC paired |
| All-motor siren | Find drone mode |

## RC Modes

The ESP32 RC module has three modes:

- **RECEIVER** (default): On the drone, receives ESP-NOW, outputs SBUS/CRSF/Serial to FC
- **TRANSMITTER**: Handheld unit, takes input from PC, broadcasts ESP-NOW
- **BRIDGE**: Direct PC ↔ FC passthrough without wireless

Auto-pairs with any ESP-NOW peer. No MAC addresses or channels to configure.

## Serial Commands

Connect to Pico via USB serial at 115200 baud:

| Key | Action |
|-----|--------|
| `a` | Arm |
| `d` | Disarm |
| `s` | Status |
| `f` | Failsafe state |
| `r` | Return to RC |
| `p` | Find drone siren |

## Failsafe

If RC signal is lost for 500ms, the drone:
1. Maintains altitude at throttle ~1200
2. Waits for RC reconnection
3. Returns to RC control automatically

## Firmware Files

```
firmware/
├── firmware.uf2                  Pico FC (copy to USB drive)
├── rc_receiver_esp32c3.bin     ESP32-C3 RC
├── rc_receiver_esp32s3.bin     ESP32-S3 RC
├── rc_transmitter_esp32c3.bin  ESP32-C3 TX
└── rc_transmitter_esp32s3.bin  ESP32-S3 TX
```

## Performance

| Loop | Rate |
|------|------|
| Motor control | 400 Hz |
| Attitude | 200 Hz |
| Position | 50 Hz |

## Safety

- Remove props before firmware updates
- Check RC link before arming
- Test failsafe in a safe area

## License

MIT
