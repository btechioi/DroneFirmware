# Drone FC Test Suite

Software-in-the-loop testing for the drone flight controller firmware.

## Setup

```bash
cd tests
uv sync
```

## Run Tests

```bash
# Run all tests
uv run python run_tests.py

# Or with pytest
uv run pytest -v
```

## Test Structure

```
tests/
├── fc_simulator.py         # FC simulator module
├── test_pid.py              # PID controller tests
├── test_motor_mixer.py     # Motor mixing tests
├── test_failsafe.py        # Failsafe system tests
├── test_rc_protocol.py      # RC input tests
├── test_telemetry.py        # Telemetry protocol tests
├── test_integration.py      # Integration tests
├── run_tests.py             # Test runner
└── pyproject.toml          # Dependencies
```

## Test Categories

### Unit Tests

- **test_pid.py**: PID controller (P, I, D, limits)
- **test_motor_mixer.py**: Motor mixing calculations
- **test_failsafe.py**: Failsafe triggers and recovery
- **test_rc_protocol.py**: RC channel encoding/decoding
- **test_telemetry.py**: Telemetry packet format

### Integration Tests

- **test_integration.py**: Full FC simulation

## Simulator Module

`fc_simulator.py` provides:

```python
from fc_simulator import (
    FlightControllerSimulator,
    RCChannels,
    PID,
    MotorMixer,
    FailsafeManager
)

# Create simulator
fc = FlightControllerSimulator()

# Arm
fc.arm()

# Send RC input
rc = RCChannels(throttle=1200, roll=1500, pitch=1500, yaw=1500)
fc.receive_rc(rc, timestamp_ms=0)

# Run control loop
dt = 0.0025  # 2.5ms = 400Hz
for _ in range(400):
    fc.update(dt)

# Get motor outputs
motors = fc.get_motor_outputs()
```

## Test Coverage

| Component | Tests |
|-----------|-------|
| PID | Proportional, Integral, Derivative, Limits |
| Motor Mixer | Roll, Pitch, Yaw, Combined |
| Failsafe | RC loss, Stuck controls, Recovery |
| RC Protocol | Encoding, Decoding, Arm/Disarm |
| Telemetry | Packet format, Round-trip |
| Integration | Full flight scenarios |
