# DroneGCS - Ground Control Station

<p align="center">
  <img src="https://img.shields.io/badge/Framework-PyQt6-41CD52?style=for-the-badge&logo=qt" alt="Framework">
  <img src="https://img.shields.io/badge/Protocol-Mavlink-659AD2?style=for-the-badge" alt="Protocol">
  <img src="https://img.shields.io/badge/Python-3.10+-blueviolet?style=for-the-badge&logo=python" alt="Python">
</p>

---

## 🎯 Features

| ✨ Feature | 📝 Description |
|------------|----------------|
| 🎮 **Joystick** | Gamepad/controller support |
| ⌨️ **Keyboard** | WASD + QE controls |
| 📊 **Real-time** | PID tuning graphs |
| 🔍 **Auto-detect** | Finds drone USB devices |
| 🎛️ **PID Tuner** | Multiple auto-tune methods |

---

## 🕹️ Controls

```
┌─────────────────────────────────────────────────────┐
│                 KEYBOARD CONTROLS                     │
├─────────────────────────────────────────────────────┤
│                                                      │
│              ┌─────────────────┐                     │
│              │   W             │  ← Pitch Forward   │
│              ├─────────────────┤                     │
│         ┌────┤   S             │                    │
│         │ A  ├─────────────────┤  D ──► Roll       │
│         │    │                 │                    │
│   Yaw ◄─┤    └─────────────────┘                   │
│         │                                              │
│         │        ┌─────────────────┐                 │
│         │        │   SPACE         │  ← Throttle Up  │
│         └───────┤   CTRL          │  ← Throttle Down│
│                 └─────────────────┘                 │
│                                                      │
├─────────────────────────────────────────────────────┤
│   R ──► ARM    │    ESC ──► DISARM    │    T ──► Tune │
└─────────────────────────────────────────────────────┘
```

### Joystick Mapping

| Axis/Button | Action |
|-------------|--------|
| Left Stick X | Roll |
| Left Stick Y | Pitch |
| Right Stick X | Yaw |
| Right Stick Y | Throttle |
| Button A | Arm |
| Button B | Disarm |
| Button X | Mode Cycle |

---

## 🔗 Connection Modes

```
┌─────────────────────────────────────────────────────┐
│              CONNECTION OPTIONS                     │
├─────────────────────────────────────────────────────┤
│                                                      │
│  ┌──────────────┐  ┌──────────────┐  ┌────────────┐ │
│  │ USB SERIAL   │  │    UDP       │  │   RELAY    │ │
│  │              │  │              │  │            │ │
│  │  /dev/tty   │  │  14550       │  │ Via RC     │ │
│  │  USB0       │  │  192.168.x   │  │ Module     │ │
│  │              │  │              │  │            │ │
│  └──────────────┘  └──────────────┘  └────────────┘ │
│                                                      │
└─────────────────────────────────────────────────────┘
```

| Mode | Port | Use Case |
|------|------|----------|
| **USB Serial** | /dev/ttyUSB0 | Direct cable |
| **UDP** | 14550 | Network |
| **Relay** | Via RC | Wireless |

---

## 🎛️ PID Tuning Methods

```
┌─────────────────────────────────────────────────────┐
│              AUTO-TUNE METHODS                       │
└─────────────────────────────────────────────────────┘

 ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐
 │   ZIEGLER   │  │   RELAY     │  │    STEP     │  │  FREQUENCY  │
 │  NICHOLS    │  │ (Åström)    │  │  RESPONSE   │  │   SWEEP     │
 │             │  │             │  │             │  │             │
 │  Ultimate   │  │  Relay      │  │  First-     │  │  Bode       │
 │   Gain      │  │ Feedback    │  │  Order      │  │  Plot       │
 │             │  │             │  │             │  │             │
 │  🌀🌀🌀      │  │  📈📉📈📉   │  │   ╱╲       │  │  ∿∿∿∿∿∿   │
 │  Oscillation│  │  On/Off     │  │  ╱──╲      │  │  Frequency  │
 └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘
```

---

## 🖥️ UI Tabs

```
┌─────────────────────────────────────────────────────┐
│  DroneGCS                                             │
├────────┬────────┬────────┬────────┬─────────────────┤
│ Flight │  PID   │ Motor  │Sensors │ Messages         │
│ Control│ Tuning │ Test   │        │                  │
├────────┴────────┴────────┴────────┴─────────────────┤
│                                                      │
│   ┌──────────────────┐  ┌──────────────────┐          │
│   │   ATTITUDE       │  │   TELEMETRY      │          │
│   │                  │  │                   │          │
│   │      ●          │  │  Alt: 10.5m      │          │
│   │    ●   ●        │  │  Bat: 11.8V      │          │
│   │      ●          │  │  RSSI: -45dB     │          │
│   │                  │  │                   │          │
│   │  R: 0.0°        │  │                   │          │
│   │  P: 0.0°        │  │                   │          │
│   │  Y: 0.0°        │  │                   │          │
│   └──────────────────┘  └──────────────────┘          │
│                                                      │
└─────────────────────────────────────────────────────┘
```

| Tab | Features |
|-----|----------|
| **Flight** | Mode, arm/disarm, throttle |
| **PID** | Auto-tune, profile slots |
| **Motor** | Individual motor test |
| **Sensors** | IMU, barometer values |
| **Messages** | Mavlink log |

---

## 🚀 Quick Start

```bash
# Install dependencies
cd DroneGCS
uv sync

# Run GCS
uv run python -m drone_gcs
```

---

## 📦 Requirements

```
┌─────────────────────────────────┐
│         DEPENDENCIES            │
├─────────────────────────────────┤
│  🐍 Python     3.10+          │
│  🖼️ PyQt6      UI Framework   │
│  📡 pymavlink  Mavlink proto   │
│  🔌 pyserial   Serial comms    │
│  📊 numpy      Data processing│
│  📈 pyqtgraph  Real-time plots │
└─────────────────────────────────┘
```

---

## 📜 License

MIT • Made by [btechioi](https://github.com/btechioi)
