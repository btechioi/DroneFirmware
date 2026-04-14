# DroneGCS - Ground Control Station

<p align="center">
  <img src="https://img.shields.io/badge/Framework-PyQt6-blue" alt="Framework">
  <img src="https://img.shields.io/badge/Protocol-Mavlink-green" alt="Protocol">
  <img src="https://img.shields.io/badge/Python-3.10+-blueviolet?style=flat&logo=python" alt="Python">
</p>

PyQt6-based ground control station with **Mavlink protocol**, **joystick support**, and **real-time PID tuning**.

## Features

- **Joystick/Keyboard** - Full manual control
- **Real-time Graphs** - PID tuning visualization
- **Auto-detect** - Finds drone USB devices
- **PID Auto-tune** - Multiple methods

## Installation

```bash
cd DroneGCS
uv sync
```

## Running

```bash
uv run python -m drone_gcs
```

## Controls

| Input | Action |
|-------|--------|
| W/S | Pitch forward/back |
| A/D | Roll left/right |
| Q/E | Yaw left/right |
| Space | Throttle up |
| Ctrl | Throttle down |
| R | Arm |
| Esc | Disarm |

## Connection Modes

| Mode | Description |
|------|-------------|
| USB Serial | Direct USB to FC |
| UDP | Network (port 14550) |
| Relay | Via RC module |

## PID Tuning Methods

1. **Ziegler-Nichols** - Ultimate gain
2. **Relay (Åström-Hägglund)** - Relay feedback
3. **Step Response** - First-order ID
4. **Frequency Sweep** - Bode plot

## Requirements

- Python 3.10+
- PyQt6
- pymavlink
- pyserial
- numpy
- pyqtgraph
