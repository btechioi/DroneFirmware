# Agent Instructions for DroneFirmware

## Project Structure

```
DroneFirmware/
├── src/                              # Flight Controller (Pico)
│   ├── main/
│   │   ├── main.cpp                  # Entry point, main loop
│   │   ├── flight_mode.h            # FlightMode, FailSafeState enums
│   │   └── globals.h                # Global variables
│   ├── config/
│   │   ├── pins.h                   # Pin definitions
│   │   └── constants.h              # Loop rates, limits
│   └── systems/
│       ├── sensors/
│       │   └── flight_controller.h  # IMU, sensors, baro
│       ├── control/
│       │   ├── pid.h/cpp            # PID controller
│       │   ├── motor_audio.h/cpp    # Motor buzzer tones
│       │   └── led_status.h         # LED patterns
│       └── comms/
│           ├── spi_control.h/cpp    # SPI companion link
│           └── comms_module.h       # RC communication
├── esp32-rc/                        # RC Module (ESP32-C3/S3)
│   ├── platformio.ini               # Build environments
│   └── src/
│       └── main.cpp                 # RC logic, ESP-NOW, SBUS, CRSF
├── companion/                        # Pi Zero companion
│   └── companion/
│       ├── main.py                  # Entry point
│       ├── comms/spi_link.py       # SPI communication
│       ├── autopilot/autopilot.py  # Waypoints, position hold
│       └── vision/optical_flow.py   # PMW3901 sensor
├── DroneGCS/                        # Ground Control Station
│   └── drone_gcs/
│       ├── ui/main_window.py        # PyQt6 UI
│       ├── protocol/mavlink_protocol.py
│       ├── connection/connection_manager.py
│       └── control/joystick.py
├── build.sh                          # Build script
├── firmware/                         # Built binaries
├── SPEC.md                           # Protocol specifications
├── README.md                         # User documentation
└── platformio.ini                   # Pico FC platform config
```

## Build Commands

```bash
# Build everything (creates firmware/)
./build.sh

# Build individual components
pio run                              # Pico FC only
pio run -d esp32-rc -e esp32c3-rc   # ESP32-C3 RC Receiver
pio run -d esp32-rc -e esp32c3-tx   # ESP32-C3 RC Transmitter
pio run -d esp32-rc -e esp32-s3-rc   # ESP32-S3 RC Receiver
pio run -d esp32-rc -e esp32-s3-tx   # ESP32-S3 RC Transmitter

# Upload via PlatformIO
pio run --target upload                              # Pico
pio run -d esp32-rc -e esp32c3-rc --target upload   # ESP32
```

## Key Files for Modification

### Flight Controller (Pico)

| File | Purpose | Key Functions |
|------|---------|----------------|
| `src/main/main.cpp` | Main loop, RC input, failsafes | `setup()`, `loop()`, `checkFailsafes()`, `enterFailsafe()` |
| `src/main/flight_mode.h` | Enums | `FlightMode`, `FailSafeState`, `ControlSource` |
| `src/config/pins.h` | Pin definitions | `MOTOR_PINS[]`, `RC_PINS[]`, `LED_STATUS` |
| `src/config/constants.h` | Loop rates | `FAST_LOOP_HZ`, `FAST_LOOP_US` |
| `src/systems/control/led_status.h` | LED patterns | `LEDStatus::setState()`, `LEDStatus::update()` |
| `src/systems/control/motor_audio.h` | Motor buzzer | `MotorAudio::playTone()`, `MotorAudio::update()` |
| `src/systems/sensors/flight_controller.h` | Sensors | `fc.updateSensors()`, `fc.getIMU()` |

### ESP32 RC

| File | Purpose | Key Classes |
|------|---------|-------------|
| `esp32-rc/src/main.cpp` | RC logic | `LEDController`, `SBUSTransmitter`, `CRSFTransmitter`, `SerialProtocol` |
| `esp32-rc/platformio.ini` | Build envs | `esp32c3-rc`, `esp32c3-tx`, `esp32-s3-rc`, `esp32-s3-tx` |

## Enums Reference

### FlightMode (src/main/flight_mode.h)
```cpp
enum class FlightMode {
    MANUAL = 0,
    ALTHOLD = 1,
    POSHOLD = 2,
    WAYPOINT = 3,
    RTL = 4,
    TAKEOFF = 5,
    LAND = 6
};
```

### FailSafeState (src/main/flight_mode.h)
```cpp
enum class FailSafeState {
    NONE = 0,
    SIGNAL_LOSS = 1,
    LOW_BATTERY = 2,
    CRITICAL_SENSOR = 3
};
```

### ControlSource (src/main/main.cpp)
```cpp
enum class ControlSource {
    RC_RECEIVER = 0,
    COMPANION = 1,
    FAILSAFE = 2
};
```

## LED Status Reference

### FC (GPIO 25) - led_status.h
```cpp
class LEDStatus {
public:
    enum State {
        OFF = 0,
        SOLID,
        BLINK_SLOW,      // 500ms interval
        BLINK_FAST,      // 150ms interval
        PULSE,
        DOUBLE_BLINK     // Two quick blinks
    };
    
    enum Color {
        NONE = 0, RED, GREEN, BLUE, YELLOW, CYAN, MAGENTA, WHITE
    };
    
    void begin();
    void setState(State state);
    void setColor(Color color);
    void update();
};
```

### Usage in main.cpp:
```cpp
#include "systems/control/led_status.h"
LEDStatus ledStatus;

// In setup():
ledStatus.begin();

// In loop():
ledStatus.update();
updateLEDStatus(); // Custom function that sets state based on system state

// LED States:
// - SOLID GREEN: Armed
// - BLINK_SLOW GREEN: Disarmed, healthy
// - DOUBLE_BLINK BLUE: Companion connected
// - BLINK_FAST RED: Failsafe active
// - SOLID RED: Critical sensor failure
```

### ESP32 RC - LEDController class (esp32-rc/src/main.cpp)
```cpp
class LEDController {
public:
    enum State {
        OFF,
        SOLID,
        BLINK_SLOW,      // 1s interval
        BLINK_FAST,       // 200ms interval
        DOUBLE_BLINK,
        SEARCHING,        // Slow pulse
        CONNECTED         // Long blink
    };
    
    LEDController(uint8_t p);
    void begin();
    void setState(State s);
    void update();
};

LEDController statusLED(LED_PIN);  // LED_PIN = 8 (C3) or 48 (S3)

// In setup():
statusLED.begin();
statusLED.setState(LEDController::SEARCHING);

// In loop():
statusLED.update();

// LED States:
// - OFF: No peer
// - SEARCHING: Slow pulse, looking for peer
// - BLINK_FAST: Pairing/connecting
// - DOUBLE_BLINK: RC connected to transmitter
// - SOLID: Fully connected (RC + FC telemetry)
```

## Audio Cues Reference

### MotorAudio Class (src/systems/control/motor_audio.h)
```cpp
class MotorAudio {
public:
    enum Tone {
        READY,
        ARMED_SUCCESS,
        DISARMED,
        FAILSAFE_ENTER,
        FAILSAFE_EXIT,
        LOW_BATTERY,
        CRITICAL_BATTERY,
        GPS_LOCK,
        RC_SEARCHING,
        RC_FOUND,
        RC_LOST,
        RC_CONNECTED,
        CALIBRATION_START,
        CALIBRATION_COMPLETE,
        ERROR,
        FIND_DRONE
    };
    
    void begin();
    void setMotorOutputFunc(void (*callback)(uint8_t motor, uint16_t pwm));
    void playTone(Tone tone);
    void beep(uint16_t frequency, uint16_t duration, uint8_t motorMask = 0x01);
    void update();
};
```

### Usage in main.cpp:
```cpp
#include "systems/control/motor_audio.h"
MotorAudio motorAudio;

// Motor callback for audio output
void audioMotorCallback(uint8_t motor, uint16_t pwm) {
    if (motor < 4) {
        uint16_t duty = map(pwm, 1000, 2000, 0, 65535);
        analogWrite(MOTOR_PINS[motor], duty >> 4);
    }
}

// In setup():
motorAudio.begin();
motorAudio.setMotorOutputFunc(audioMotorCallback);
motorAudio.playTone(MotorAudio::Tone::READY);

// In loop():
motorAudio.update();

// Play tones on events:
motorAudio.playTone(MotorAudio::Tone::ARMED_SUCCESS);    // Arm
motorAudio.playTone(MotorAudio::Tone::DISARMED);          // Disarm
motorAudio.playTone(MotorAudio::Tone::RC_LOST);          // RC lost
motorAudio.playTone(MotorAudio::Tone::RC_FOUND);         // RC found
motorAudio.playTone(MotorAudio::Tone::LOW_BATTERY);      // Low battery
motorAudio.playTone(MotorAudio::Tone::FAILSAFE_ENTER);   // Failsafe
motorAudio.playTone(MotorAudio::Tone::FAILSAFE_EXIT);     // Failsafe clear
motorAudio.playTone(MotorAudio::Tone::FIND_DRONE);       // All motors siren

// Custom beep:
motorAudio.beep(2000, 500, 0x0F);  // 2kHz for 500ms, motors 0-3
```

## Serial Commands (FC)

Connect to Pico USB serial at 115200 baud:

| Key | Action | Code Location |
|-----|--------|---------------|
| `a` | Arm motors | `if (cmd == 'a') { motorsArmed = true; }` |
| `d` | Disarm motors | `if (cmd == 'd') { motorsArmed = false; failsafeState = NONE; }` |
| `s` | Show status | Prints mode, companion, armed state |
| `f` | Show failsafe state | Prints failsafe and control source enums |
| `r` | Return to RC control | `returnToRC()` if not critical sensor failsafe |
| `p` | Play find-drone siren | `motorAudio.playTone(MotorAudio::Tone::FIND_DRONE)` |

## Failsafe Triggers

### RC Signal Loss (500ms)
```cpp
// In checkFailsafes():
if (now - lastRCInputTime > RC_SIGNAL_TIMEOUT_MS && !rcSignalLost) {
    rcSignalLost = true;
    if (motorsArmed) {
        enterFailsafe(FailSafeState::SIGNAL_LOSS);
    }
}
```

### Stuck Controls (3s no movement)
```cpp
// Threshold: 20 units change
// Resets throttle to 1200, angles to 0
```

### Companion Timeout (2s)
```cpp
// Returns to RC control if companion times out
if (activeControl == ControlSource::COMPANION && motorsArmed) {
    returnToRC();
}
```

### IMU Failure
```cpp
// In setup():
if (!fc.hasIMU()) {
    enterFailsafe(FailSafeState::CRITICAL_SENSOR);
}
// Cannot arm or return to RC
```

## Pin Definitions (config/pins.h)

```cpp
constexpr uint8_t MOTOR_PINS[] = {12, 13, 14, 15};  // PWM outputs

constexpr uint8_t RC_PINS[] = {16, 17, 18, 19, 20, 21, 22, 26};  // Direct RC input

constexpr uint8_t LED_STATUS = 25;  // On-board LED

constexpr uint8_t IMU_SDA = 4;      // I2C
constexpr uint8_t IMU_SCL = 5;      // I2C

constexpr uint8_t RADIO_SPI_CS = 9;    // SPI
constexpr uint8_t RADIO_SPI_MISO = 8;   // SPI
constexpr uint8_t RADIO_SPI_MOSI = 11;  // SPI
constexpr uint8_t RADIO_SPI_SCK = 10;    // SPI
constexpr uint32_t SPI_SPEED = 125000000;  // 125 MHz
```

## ESP32 RC Pin Definitions (esp32-rc/src/main.cpp)

```cpp
// ESP32-C3
#define LED_PIN 8
#define BUTTON_PIN 9
#define STICK_ROLL_PIN 0
#define STICK_PITCH_PIN 1
#define STICK_THROTTLE_PIN 2
#define STICK_YAW_PIN 3
#define SWITCH_A_PIN 4
#define SWITCH_B_PIN 5
#define SWITCH_C_PIN 6
#define SWITCH_D_PIN 7

// ESP32-S3
#define LED_PIN 48
#define BUTTON_PIN 0
#define STICK_ROLL_PIN 1
#define STICK_PITCH_PIN 2
#define STICK_THROTTLE_PIN 3
#define STICK_YAW_PIN 4
```

## RC Modes (ESP32)

| Mode | Define | Description |
|------|--------|-------------|
| RECEIVER | `#define MODE_RECEIVER` | On drone, ESP-NOW → SBUS/CRSF/Serial → FC |
| TRANSMITTER | `#define MODE_TRANSMITTER` | Handheld, PC USB → ESP-NOW |
| BRIDGE | `#define MODE_BRIDGE` | Direct PC ↔ FC passthrough |

## Protocol Packet Formats

### RC → FC Serial (2 Mbps)
```
RC Channels: 0xAA, 0x01, count, ch[0]..ch[n], crc16
FC Telemetry: 0x55, 0x02, seq, flags, control, roll, pitch, yaw, alt, gyro[3], bat
RC Status: 0x66, 0x03, 48 bytes of status
```

### ESP-NOW
```
RC Data: magic(0xAA), type(0x01), timestamp, channelCount, channels[16], rssi, flags
```

### SPI (FC ↔ Companion)
```
CMD | LEN_BE | DATA | CRC
```

## Global Variables (src/main/globals.h)

```cpp
extern volatile bool motorsArmed;
extern volatile FlightMode currentFlightMode;
extern volatile FailSafeState failsafeState;
extern volatile ControlSource activeControl;
extern volatile float targetRoll, targetPitch, targetYaw;
extern volatile float baseThrottle;
extern volatile uint16_t motorPWM[4];
extern volatile uint16_t rcRoll, rcPitch, rcThrottle, rcYaw;
extern volatile uint16_t rcAux1, rcAux2, rcAux3, rcAux4;
extern volatile bool companionConnected;
extern volatile bool companionRCActive;
extern uint32_t lastCompanionHeartbeat;
extern uint32_t lastRCInputTime;
```

## Performance Targets

| Loop | Rate | Period | Budget |
|------|------|--------|--------|
| Fast Loop | 400 Hz | 2.5 ms | <2.5 ms |
| RC Input | 500 Hz | 2 ms | <2 ms |
| Telemetry | 100 Hz | 10 ms | Variable |
| Status Print | 10 Hz | 100 ms | - |

## Type Conventions

- `uint8_t` not `unsigned char`
- `uint16_t` not `short`
- `uint32_t` not `unsigned long`
- `int16_t` not `int` for fixed-width integers

## Include Order

```cpp
#include <Arduino.h>  // First - for platform types
#include "flight_mode.h"
#include "../config/pins.h"
#include "systems/control/motor_audio.h"
#include "systems/control/led_status.h"
```

## Adding New LED Patterns

### FC (src/systems/control/led_status.h)
1. Add state to `State` enum
2. Add case in `updateLED()` switch statement

### ESP32 RC (esp32-rc/src/main.cpp)
1. Add state to `LEDController::State` enum
2. Add case in `LEDController::update()` switch statement

## Adding New Audio Tones

### FC (src/systems/control/motor_audio.h)
1. Add tone to `Tone` enum
2. Add case in `playTone()` switch statement
3. Define frequency and duration in `playTone()` body

## Testing Checklist

When modifying firmware:
1. Build succeeds: `pio run`
2. ESP32 builds: `pio run -d esp32-rc`
3. No new compiler warnings
4. Memory usage acceptable (RAM < 50%, Flash < 50%)
5. LED patterns update correctly
6. Audio cues trigger on correct events
7. Failsafe triggers work as expected

## Common Patterns

### Time-based execution
```cpp
static uint32_t lastTime = 0;
if (now - lastTime >= INTERVAL_MS * 1000) {
    lastTime = now;
    // Do work
}
```

### RC arm/disarm
```cpp
if (rcAux1 > 1500 && !motorsArmed) {
    motorsArmed = true;
    motorAudio.playTone(MotorAudio::Tone::ARMED_SUCCESS);
} else if (rcAux1 < 1200 && motorsArmed) {
    motorsArmed = false;
    motorAudio.playTone(MotorAudio::Tone::DISARMED);
}
```

### Failsafe entry
```cpp
void enterFailsafe(FailSafeState state) {
    if (failsafeState != FailSafeState::NONE) return;
    failsafeState = state;
    activeControl = ControlSource::FAILSAFE;
    motorAudio.playTone(MotorAudio::Tone::RC_LOST);
}
```

## Safety

⚠️ Remove props before testing firmware  
⚠️ Verify RC link before arming  
⚠️ Test failsafe in safe area  
⚠️ Never arm with props on during development  
⚠️ Always disarm before connecting battery
