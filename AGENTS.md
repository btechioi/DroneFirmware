# Agent Instructions for DroneFirmware

## Project Structure

```
DroneFirmware/
├── src/                         # Flight Controller (Pico)
│   ├── main/main.cpp           # Entry point, main loop
│   ├── flight_mode.h           # FlightMode, FailSafeState enums
│   ├── globals.h               # Global variables
│   ├── config/pins.h          # Pin definitions
│   ├── config/constants.h      # Loop rates, limits
│   └── systems/
│       ├── sensors/
│       │   └── flight_controller.h  # IMU, sensors
│       ├── control/
│       │   ├── pid.h/cpp       # PID controller
│       │   ├── motor_audio.h/cpp  # Motor buzzer tones
│       │   └── led_status.h   # LED patterns
│       └── comms/
│           └── spi_control.h/cpp  # SPI companion link
├── esp32-rc/                   # RC Module (ESP32-C3/S3)
│   ├── platformio.ini          # Build envs: esp32c3-rc, esp32c3-tx, etc
│   └── src/main.cpp            # All RC logic, ESP-NOW, SBUS
├── companion/                   # Pi Zero companion
│   └── companion/
│       ├── main.py             # Entry point
│       ├── comms/spi_link.py   # SPI communication
│       └── autopilot/           # Waypoints, position hold
├── DroneGCS/                    # Ground Control Station
│   └── drone_gcs/
│       ├── ui/main_window.py   # PyQt6 UI
│       └── protocol/mavlink_protocol.py
├── build.sh                     # Build script
└── firmware/                    # Built binaries (after ./build.sh)
```

## Build Commands

```bash
# Build everything (creates firmware/)
./build.sh

# Build individual
pio run                                    # Pico FC only
pio run -d esp32-rc -e esp32c3-rc       # ESP32-C3 RC
pio run -d esp32-rc -e esp32c3-tx       # ESP32-C3 TX
```

## Key Files for Modification

### Flight Controller (Pico)

| File | Purpose |
|------|---------|
| `src/main/main.cpp` | Main loop, RC input, failsafes, audio cues, LED |
| `src/systems/control/led_status.h` | LED patterns (OFF, SOLID, BLINK_SLOW, etc) |
| `src/systems/control/motor_audio.h` | Motor buzzer tones |
| `config/pins.h` | Pin definitions, add LED_STATUS here |

### ESP32 RC

| File | Purpose |
|------|---------|
| `esp32-rc/src/main.cpp` | RC logic, ESP-NOW, modes |
| `esp32-rc/platformio.ini` | Add new build environments |

## LED Status Reference

### FC (GPIO 25)
```cpp
ledStatus.setState(LEDStatus::SOLID);        // Armed
ledStatus.setState(LEDStatus::BLINK_SLOW);  // Disarmed
ledStatus.setState(LEDStatus::BLINK_FAST);   // Failsafe
ledStatus.setState(LEDStatus::DOUBLE_BLINK); // Companion connected
ledStatus.update();                          // Call in loop
```

### ESP32 RC
```cpp
statusLED.setState(LEDController::OFF);
statusLED.setState(LEDController::SOLID);
statusLED.setState(LEDController::BLINK_SLOW);    // 1s interval
statusLED.setState(LEDController::BLINK_FAST);    // 200ms interval
statusLED.setState(LEDController::DOUBLE_BLINK);   // Two quick blinks
statusLED.setState(LEDController::SEARCHING);      // Slow pulse
statusLED.setState(LEDController::CONNECTED);      // Long blink
statusLED.update();  // Call in loop
```

## Audio Cues Reference

```cpp
motorAudio.playTone(MotorAudio::Tone::READY);
motorAudio.playTone(MotorAudio::Tone::ARMED_SUCCESS);
motorAudio.playTone(MotorAudio::Tone::DISARMED);
motorAudio.playTone(MotorAudio::Tone::RC_LOST);
motorAudio.playTone(MotorAudio::Tone::RC_FOUND);
motorAudio.playTone(MotorAudio::Tone::FAILSAFE_ENTER);
motorAudio.playTone(MotorAudio::Tone::FAILSAFE_EXIT);
motorAudio.playTone(MotorAudio::Tone::LOW_BATTERY);
motorAudio.playTone(MotorAudio::Tone::FIND_DRONE);  // All motors
motorAudio.update();  // Call in loop
```

## Serial Commands (FC)

Connect to Pico USB serial at 115200 baud:

| Key | Action |
|-----|--------|
| `a` | Arm motors |
| `d` | Disarm motors |
| `s` | Show status |
| `f` | Show failsafe state |
| `r` | Return to RC control |
| `p` | Play find-drone siren |

## Type Conventions

- `uint8_t` not `unsigned char`
- `uint16_t` not `short`
- `uint32_t` not `unsigned long`
- `int16_t` not `int` for fixed-width

## Include Order

```cpp
#include <Arduino.h>  // First - for platform types
#include "flight_mode.h"
#include "../config/pins.h"
#include "systems/control/motor_audio.h"
```

## Adding New LED Patterns

### FC (src/systems/control/led_status.h)
Add to `State` enum:
```cpp
enum State {
    OFF = 0,
    SOLID,
    BLINK_SLOW,    // Add new states here
    BLINK_FAST,
    PULSE,
    DOUBLE_BLINK,
    // NEW_STATE
};
```

### ESP32 RC (esp32-rc/src/main.cpp)
Add to `LEDController::State` enum:
```cpp
class LEDController {
public:
  enum State {
    OFF,
    SOLID,
    BLINK_SLOW,
    BLINK_FAST,
    DOUBLE_BLINK,
    SEARCHING,
    CONNECTED,
    // NEW_STATE
  };
```

## Performance Targets

| Loop | Rate | Budget |
|------|------|--------|
| Fast Loop | 400 Hz | <2.5ms |
| RC Input | 50 Hz | <20ms |
| Telemetry | 100 Hz | Variable |

## Safety

⚠️ Remove props before testing firmware  
⚠️ Verify RC link before arming  
⚠️ Test failsafe in safe area  
⚠️ Never arm with props on during development

## Testing Checklist

When modifying firmware:
1. Build succeeds: `pio run`
2. ESP32 builds: `pio run -d esp32-rc`
3. No new compiler warnings
4. Memory usage acceptable (RAM < 50%, Flash < 50%)
