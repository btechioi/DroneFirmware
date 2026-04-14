# Agent Instructions for DroneFirmware

## Project Overview

This is a modular drone flight controller for Raspberry Pi Pico (RP2040). The architecture is designed to be:

1. **Sensor-Agnostic** - Only IMU is required; all other sensors are optional
2. **Hot-Pluggable** - Sensors are auto-detected at startup
3. **Gracefully Degradable** - Can fly with failed motors or sensors
4. **Highly Modular** - Each subsystem is independent

## Key Principles

### 1. Include Order
Headers must include their dependencies. Always include:
```cpp
#include <Arduino.h>  // First for platform types
#include "flight_mode.h"
#include "config/pins.h"
#include "config/constants.h"
```

### 2. Type Definitions
Use C++ types, not C types:
- `uint8_t` instead of `unsigned char`
- `int16_t` instead of `short`
- `uint32_t` instead of `unsigned long`

### 3. Optional Features
Always check sensor availability:
```cpp
if (sensorHub->isSensorAvailable("Barometer")) {
    // Use barometer
}
if (sensorHub->canAltitudeHold()) {
    // Enable altitude hold mode
}
```

### 4. IMU as Foundation
The IMU is the only required sensor. Everything builds on it:
```
IMU → Attitude → Altitude Estimation → Position Estimation
```

### 5. Graceful Degradation
If a motor fails:
1. Detect failure
2. Switch to degraded mode (tricycle/bicopter)
3. Continue flight or auto-land

### 6. Serial Debug Output
Use consistent serial output format:
```cpp
Serial.println("[MODULE] Message");
Serial.print("[MODULE] Value: ");
Serial.println(value);
```

## Module Responsibilities

### Sensors (`systems/sensors/`)
- Auto-detect on initialization
- Provide raw data + health status
- Support calibration
- SensorBase class for all sensors

### Control (`systems/control/`)
- Nested PID loops
- Motor mixing
- Flight mode management
- Rate → Attitude → Position cascades

### Communications (`systems/comms/`)
- Radio protocols (LoRa, NRF24, SPI)
- Telemetry transmission
- Command reception
- Hot-swappable at runtime

### Safety (`systems/safety/`)
- Failsafe monitoring
- Motor failure detection
- Degraded flight modes
- Auto-landing

### Calibration (`systems/calibration/`)
- Static calibration (factory)
- Dynamic calibration (in-flight)
- Drift detection
- Auto-recalibration

## File Naming Conventions

| Type | Suffix | Example |
|------|---------|---------|
| Header | `.h` | `sensor_modules.h` |
| Implementation | `.cpp` | `sensor_modules.cpp` |
| Base Class | `_base.h` | `sensor_base.h` |
| Module | `_sensor.h` | `imu_sensor.h` |

## Class Hierarchy

```
SensorModule (base)
├── IMUSensorModule
├── BaroSensorModule
├── GPSSensorModule
├── MagSensorModule
├── OpticalFlowModule
└── ...

NestedPID
├── RatePID
├── AttitudePID
└── PositionPID

FailsafeSystem
├── FailsafeManager
├── MotorFailureDetector
├── DegradedFlightController
└── AutoLandController
```

## Adding a New Sensor

1. Create sensor header in `systems/sensors/`:
```cpp
// new_sensor.h
#pragma once
#include "sensor_base.h"

class NewSensorModule : public SensorModule {
public:
    NewSensorModule();
    const char* getName() const override { return "NewSensor"; }
    bool detect() override;
    bool init() override;
    void update() override;
    
    // Sensor-specific methods
    float getValue() const { return value_; }

private:
    float value_;
};
```

2. Create implementation:
```cpp
// new_sensor.cpp
#include "new_sensor.h"

NewSensorModule::NewSensorModule() {
    detected_ = false;
    initialized_ = false;
}

bool NewSensorModule::detect() {
    // I2C/SPI detection code
    detected_ = true;
    return detected_;
}

bool NewSensorModule::init() {
    if (!detected_) return false;
    initialized_ = true;
    return true;
}

void NewSensorModule::update() {
    if (!initialized_) return;
    // Read sensor data
    healthy_ = true;
}
```

3. Register in SensorHub:
```cpp
// sensor_modules.cpp - SensorHub::begin()
newSensor_ = new NewSensorModule();
registerSensor(newSensor_);
```

## Adding a New Flight Mode

1. Add mode to enum:
```cpp
// flight_mode.h
enum class ControlMode {
    MANUAL = 0,
    // ... existing modes
    NEW_MODE = 10
};
```

2. Implement in AttitudeController:
```cpp
// attitude_controller.cpp - AttitudeController::update()
case ControlMode::NEW_MODE: {
    // Mode implementation
    rollRateOut = cascade.rateRoll.compute(targetRollRate_, rollRate);
    // ...
    break;
}
```

## Adding a Communication Protocol

1. Create protocol class:
```cpp
// new_protocol.h/cpp
class NewProtocol {
public:
    void begin();
    void send(const void* data, size_t len);
    size_t receive(void* buffer, size_t maxLen);
};
```

2. Add to CommsHub for runtime switching.

## Testing Checklist

When modifying any module:

- [ ] Build succeeds (`pio run`)
- [ ] Sensor detection works
- [ ] Calibration completes
- [ ] Motor output correct
- [ ] Failsafe triggers work
- [ ] Memory usage acceptable
- [ ] CPU load acceptable

## Performance Targets

| Subsystem | Rate | Budget |
|----------|------|--------|
| Fast Loop | 400Hz | <2.5ms |
| Attitude | 200Hz | <5ms |
| Position | 50Hz | <20ms |
| Telemetry | 10-100Hz | Variable |
| Sensors | 50-400Hz | Variable |

## Common Patterns

### Checking Sensor Health
```cpp
if (sensorHub->isSensorHealthy("GPS")) {
    // Use GPS data
}
```

### Applying Calibration
```cpp
float calibrated = raw - calibration.offset;
calibrated *= calibration.scale;
```

### Motor Output
```cpp
int16_t pwm = constrain(motorOutput, 1000, 2000);
analogWrite(motorPin, map(pwm, 1000, 2000, 0, 4095));
```

### Time Management
```cpp
uint32_t now = millis();
if (now - lastUpdate >= interval) {
    lastUpdate = now;
    // Do work
}
```

## Debug Tips

1. **Sensor Debug**: Print raw vs calibrated values
2. **Motor Debug**: Check PWM with oscilloscope
3. **Communication Debug**: Verify packet structure
4. **Failsafe Debug**: Log trigger conditions

## Safety Warnings

⚠️ **Motor Safety**: Always disarm before connecting battery
⚠️ **Propeller Safety**: Remove props during testing
⚠️ **Calibration**: Ensure drone is level and still
⚠️ **Failsafe**: Test failsafe triggers in safe environment
⚠️ **Radio**: Verify control before arming

## Ground Control Station (DroneGCS)

Located in `DroneGCS/` directory. Python PyQt6 application for testing and flying the drone.

### Running GCS
```bash
cd DroneGCS
uv sync
uv run python -m drone_gcs
```

### Connection
- Auto-detects "DroneFlightController" USB devices
- Supports USB Serial, UDP, Relay modes
- Mavlink protocol for communication

### PID Tuning Protocol
Custom Mavlink messages (ID 200-202):
- 200: Tuning commands (start/stop/abort)
- 201: Tuning status
- 202: PID gains (single or full profile)

## PID Auto-Tuner

Located in `src/systems/control/pid_tuner.h/cpp`.

### Tuning Methods
- ZIEGLER_NICHOLS (0): Ultimate gain oscillation
- RELAY (1): Åström-Hägglund relay feedback
- STEP_RESPONSE (2): Cohen-Coon first-order
- FREQUENCY_SWEEP (3): Bode plot analysis

### Tuning Axes
- ROLL_RATE, PITCH_RATE, YAW_RATE
- ROLL_ATTITUDE, PITCH_ATTITUDE, YAW_ATTITUDE
- ALTITUDE, POSITION

### Usage via GCS
1. Select method and axis
2. Click "Start Tuning"
3. Wait for completion
4. Apply gains
5. Save to profile slot
