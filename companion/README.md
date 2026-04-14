<p align="center">
  <img src="https://capsule-render.vercel.app/api?type=waving&color=0:00ff88,100:0066ff&height=180&section=header&text=Companion%20Computer&fontSize=40&fontAlignY=35&animation=fadeIn&fontColor=ffffff"/>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/Pi%20Zero%202W-CC0000?style=for-the-badge&logo=raspberrypi" alt="Pi Zero">
  <img src="https://img.shields.io/badge/SPI-125MHz-00979D?style=for-the-badge" alt="SPI">
</p>

Optional companion computer for Raspberry Pi Zero 2W. Handles autopilot and position estimation.

## Run

```bash
cd companion && uv sync
uv run python -m companion.main --mode 2
```

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

## Options

| Flag | Description |
|------|-------------|
| `--mode N` | Flight mode |
| `--waypoints lat,lng,alt...` | Mission waypoints |
| `--enable-optical-flow` | PMW3901 sensor |
| `--spi-bus 0` | SPI bus number |
| `--spi-device 0` | SPI device |

## LED

| State | Meaning |
|-------|---------|
| Solid | Armed |
| Brief blink | SPI connected |
| Off | Idle |
