<p align="center">
  <img src="https://capsule-render.vercel.app/api?type=waving&color=0:00ff88,100:0066ff&height=180&section=header&text=Companion%20Computer&fontSize=40&fontAlignY=35&animation=fadeIn&fontColor=ffffff"/>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/Pi%20Zero%202W-CC0000?style=for-the-badge&logo=raspberrypi" alt="Pi Zero">
  <img src="https://img.shields.io/badge/SPI-125MHz-00979D?style=for-the-badge" alt="SPI">
</p>

Optional companion computer that runs on Raspberry Pi Zero 2W. Handles autopilot, waypoints, and optical flow position estimation.

## Flight Modes

| Mode | Description |
|------|-------------|
| STABILIZE | Self-leveling |
| ALTHOLD | Maintains altitude |
| POSHOLD | Holds position |
| WAYPOINT | Flies mission |
| RTL | Returns to launch |
| TAKEOFF | Auto takeoff |
| LAND | Auto landing |

## Run

```bash
cd companion
uv sync
uv run python -m companion.main --mode 2
```

## Data Flow

```
ESP32/RC ──Serial──► Pi Zero ──SPI──► Pico FC
                                     │
                                     ▼
                            IMU → PID → Motors
```

The Pi Zero receives RC and telemetry, can override control, and sends position commands back to the FC via SPI.

## Options

| Flag | Description |
|------|-------------|
| `--mode N` | Flight mode (0-6) |
| `--waypoints lat,lng,alt...` | Mission waypoints |
| `--enable-optical-flow` | Use PMW3901 sensor |
| `--log FILE` | Log to file |
