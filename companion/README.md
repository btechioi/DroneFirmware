# Drone Companion Computer

<p align="center">
  <img src="https://img.shields.io/badge/Platform-Pi%20Zero%202W-CC0000?style=for-the-badge&logo=raspberrypi" alt="Platform">
  <img src="https://img.shields.io/badge/Link-SPI%20125MHz-00979D?style=for-the-badge" alt="Link">
  <img src="https://img.shields.io/badge/Python-3.12+-blueviolet?style=for-the-badge&logo=python" alt="Python">
</p>

---

## 🎯 Features

| ✨ Feature | 📝 Description |
|------------|----------------|
| 🚀 **SPI Link** | 125 MHz to flight controller |
| 🧭 **Autopilot** | Position hold, waypoints, RTL |
| 👁️ **Vision** | Optical flow position estimation |
| 📡 **RC Relay** | Passes RC through to FC |

---

## ✈️ Flight Modes

```
┌─────────────────────────────────────────────────────┐
│              FLIGHT MODE SELECTOR                    │
├─────┬──────────────────┬─────────────────────────────┤
│  0  │  STABILIZE      │  🌀 Self-leveling          │
├─────┼──────────────────┼─────────────────────────────┤
│  1  │  ALTHOLD        │  📏 Altitude hold           │
├─────┼──────────────────┼─────────────────────────────┤
│  2  │  POSHOLD        │  📍 Position hold           │
├─────┼──────────────────┼─────────────────────────────┤
│  3  │  WAYPOINT       │  🗺️  Waypoint navigation     │
├─────┼──────────────────┼─────────────────────────────┤
│  4  │  RTL            │  🏠  Return to launch       │
├─────┼──────────────────┼─────────────────────────────┤
│  5  │  TAKEOFF        │  🚀  Autonomous takeoff      │
├─────┼──────────────────┼─────────────────────────────┤
│  6  │  LAND           │  🛬  Autonomous landing      │
└─────┴──────────────────┴─────────────────────────────┘
```

---

## 📡 Data Flow

```
┌─────────────────────────────────────────────────────────────────┐
│                        DATA FLOW DIAGRAM                        │
└─────────────────────────────────────────────────────────────────┘

   ESP32/RC ◄──────────── Serial ─────────────► Companion ◄──┐
   (Radio)                                            │          │
                                                     │ SPI      │
                                                     ▼          │
                                              ┌───────────┐    │
                                              │   Pico    │    │
                                              │    FC     │    │
                                              └─────┬─────┘    │
                                                    │           │
                         ┌──────────────────────────┼───────────┘
                         │                          │
                         ▼                          ▼
                  ┌──────────────┐           ┌──────────────┐
                  │  IMU → PID  │           │  Telemetry   │
                  │  → Motors   │           │  ← Feedback  │
                  └──────────────┘           └──────────────┘
```

---

## 🚀 Usage

```bash
# Setup
cd companion
uv sync

# Altitude hold mode
uv run python -m companion.main --mode 1

# Waypoint mission
uv run python -m companion.main --mode 3 \
    --waypoints 0,0,2 5,0,2 5,5,2

# Position hold with optical flow
uv run python -m companion.main --mode 2 \
    --enable-optical-flow
```

---

## 🔧 Configuration

### Waypoints Format

```
--waypoints lat,lng,alt [lat,lng,alt]...
```

### Options

| Option | Description | Default |
|--------|-------------|---------|
| `--mode` | Flight mode (0-6) | 0 |
| `--waypoints` | Mission waypoints | none |
| `--enable-optical-flow` | Enable PMW3901 | false |
| `--log` | Log file path | auto |

---

## 📦 Components

```
companion/
├── companion/
│   ├── __init__.py
│   ├── main.py              # Entry point
│   ├── comms/
│   │   └── spi_link.py      # SPI communication
│   ├── autopilot/
│   │   └── autopilot.py     # Flight control
│   └── vision/
│       └── optical_flow.py  # Position estimation
└── pyproject.toml
```

---

## ⚙️ Requirements

- Raspberry Pi Zero 2W
- SPI enabled
- Python 3.10+
- SPI connection to Pico

---

## 📜 License

MIT • Made by [btechioi](https://github.com/btechioi)
