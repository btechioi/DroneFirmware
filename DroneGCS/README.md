<p align="center">
  <img src="https://capsule-render.vercel.app/api?type=waving&color=0:9933ff,100:6600cc&height=180&section=header&text=DroneGCS&fontSize=50&fontAlignY=35&animation=fadeIn&fontColor=ffffff"/>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/PyQt6-41CD52?style=for-the-badge&logo=qt" alt="PyQt6">
  <img src="https://img.shields.io/badge/Mavlink-659AD2?style=for-the-badge" alt="Mavlink">
</p>

Desktop ground control station for flying and tuning the drone.

## Install

```bash
cd DroneGCS
uv sync
```

## Run

```bash
uv run python -m drone_gcs
```

## Controls

| Key | Action |
|-----|--------|
| WASD | Roll/Pitch |
| Q/E | Yaw |
| Space/Ctrl | Throttle |
| R | Arm |
| Esc | Disarm |

Gamepad supported if you prefer.

## Connect

| Mode | How |
|------|-----|
| USB | Direct cable to FC |
| UDP | Network to drone IP |
| Relay | Through RC module |

The app auto-detects "DroneFlightController" USB devices.

## PID Tuning

Built-in auto-tuner supports:
- Ziegler-Nichols
- Relay feedback (Åström-Hägglund)
- Step response
- Frequency sweep

Four profile slots to save different tuning sets.

## Tabs

- **Flight** - Basic arm/disarm, mode select
- **PID** - Auto-tune, manual adjustment
- **Motor** - Test individual motors
- **Sensors** - View IMU/baro data
- **Messages** - Mavlink traffic log
