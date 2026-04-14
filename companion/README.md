# Drone Companion Computer

Position estimation & autopilot for Raspberry Pi Zero 2W.

## Flight Modes

| Mode | Description |
|------|-------------|
| 0 | STABILIZE - Self-leveling |
| 1 | ALTHOLD - Altitude hold |
| 2 | POSHOLD - Position hold |
| 3 | WAYPOINT - Waypoint navigation |
| 4 | RTL - Return to launch |
| 5 | TAKEOFF - Autonomous takeoff |
| 6 | LAND - Autonomous landing |

## Usage

```bash
# Altitude hold
python3 -m companion.main --mode 1

# Waypoint mission  
python3 -m companion.main --mode 3 --waypoints 0,0,2 5,0,2 5,5,2

# Position hold with optical flow
python3 -m companion.main --mode 2 --enable-optical-flow
```

## Data Flow

```
ESP32/RC  ──── Serial ────►  Pi Zero  ──── SPI ────►  Pico
Radio                                  │                │
(NRF24/LoRa)                           │                ▼
                                      │          IMU → PID → Motors
                                      │                ▲
                                      └──────── Telemetry
```
