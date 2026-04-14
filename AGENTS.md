# Agent Instructions for DroneFirmware

## Project Structure

```
DroneFirmware/
├── src/                    # Flight Controller (Pico)
│   ├── main/main.cpp
│   ├── systems/control/
│   │   ├── pid.h/cpp
│   │   ├── motor_audio.h/cpp
│   │   └── led_status.h
│   ├── systems/comms/
│   │   └── spi_control.h/cpp
│   └── systems/sensors/
├── esp32-rc/              # RC Module (ESP32-C3/S3)
│   └── src/main.cpp
├── companion/              # Pi Zero companion
│   └── companion/main.py
├── DroneGCS/              # Ground Control Station
│   └── drone_gcs/
├── build.sh               # Build all firmware
└── firmware/              # Built binaries
```

## Build Commands

```bash
# Build everything
./build.sh

# Build individual
pio run                           # Pico FC
pio run -d esp32-rc -e esp32c3-rc  # ESP32 RC
```

## Key Files

### Flight Controller
- `src/main/main.cpp` - Main loop, failsafes
- `config/pins.h` - Pin definitions
- `src/systems/control/motor_audio.h` - Audio cues
- `src/systems/control/led_status.h` - LED status

### ESP32 RC
- `esp32-rc/src/main.cpp` - RC modes, ESP-NOW

## LED Status

| State | LED Pattern |
|-------|-------------|
| Armed | Solid |
| Disarmed | Slow blink |
| Failsafe | Fast blink |
| Companion connected | Double blink |

## Audio Cues

| Tone | Trigger |
|------|---------|
| ARMED_SUCCESS | Motors armed |
| DISARMED | Motors disarmed |
| RC_LOST | Signal lost |
| RC_FOUND | Signal restored |
| FIND_DRONE | All-motor siren |

## Serial Commands (FC)

| Key | Action |
|-----|--------|
| a | Arm |
| d | Disarm |
| s | Status |
| f | Failsafe state |
| r | Return to RC |
| p | Find drone siren |

## RC Modes

| Mode | Description |
|------|-------------|
| RECEIVER | ESP-NOW → SBUS/Serial → FC |
| TRANSMITTER | PC USB → ESP-NOW |
| BRIDGE | PC ↔ FC passthrough |

## Performance

| Loop | Rate |
|------|------|
| Fast Loop | 400 Hz |
| Attitude | 200 Hz |
| RC Input | 50 Hz |

## Type Conventions

- `uint8_t` not `unsigned char`
- `uint16_t` not `short`
- `uint32_t` not `unsigned long`

## Include Order

```cpp
#include <Arduino.h>  // First
#include "flight_mode.h"
#include "../config/pins.h"
```

## Safety

⚠️ Remove props before testing  
⚠️ Verify RC before arming  
⚠️ Test failsafe in safe area
