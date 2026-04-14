# ESP32 RC Transmitter/Receiver

<p align="center">
  <a href="#">
    <img src="https://img.shields.io/badge/Chip-ESP32--C3/S3-E7352C?style=for-the-badge" alt="Chip">
  </a>
  <a href="#">
    <img src="https://img.shields.io/badge/Wireless-ESP--NOW-003366?style=for-the-badge" alt="Wireless">
  </a>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/Channels-16-FC4C02?style=flat" alt="16 Channels">
  <img src="https://img.shields.io/badge/Protocol-SBUS%20%7C%20CRSF-00979D?style=flat&logo=arduino" alt="Protocols">
  <img src="https://img.shields.io/badge/Audio-Cues-9933FF?style=flat" alt="Audio">
</p>

---

## 🎯 Features

| ✨ Feature | 📝 Description |
|------------|----------------|
| 🔗 **Auto-Pairing** | No manual configuration |
| 🔀 **Multi-Protocol** | SBUS, CRSF, Serial |
| 📊 **16 Channels** | Full RC channel support |
| 📡 **Telemetry** | Status to PC and FC |
| 🔊 **Audio Cues** | Connection feedback |
| ⚡ **Low Power** | Efficient ESP32-C3 |

---

## 🔄 Modes

### RECEIVER (Default) 🛸

```
    ┌─────────────────────────────────────────────┐
    │                   RECEIVER                   │
    │                                              │
    │   ◄─────────── ESP-NOW ────────────►        │
    │   │             │                           │
    │   │    ┌────────┴────────┐                  │
    │   │    │                 │                  │
    │   │    ▼                 ▼                  │
    │   │  [SBUS]    [CRSF]    [Serial]          │
    │   │    │         │          │               │
    │   │    └─────────┼──────────┘               │
    │   │              ▼                          │
    │   │    ═══════════════════════               │
    │   │         To Flight Controller             │
    └─────────────────────────────────────────────┘
```

### TRANSMITTER 📱

```
    ┌──────────────────────────────┐
    │        TRANSMITTER           │
    │                              │
    │   ════════════════          │
    │          │ From PC          │
    │          ▼                   │
    │   ┌──────────────┐           │
    │   │   Serial RX  │           │
    │   └──────┬───────┘           │
    │          │                    │
    │          ▼                    │
    │   ════════════════════════    │
    │       ESP-NOW Broadcast       │
    │   ════════════════════════    │
    └──────────────────────────────┘
              │
              │ ESP-NOW
              ▼
         [RECEIVER]
```

### BRIDGE 🌉

```
    PC ◄═══ USB ═══► [BRIDGE] ════ Serial ═══► FC
```

---

## 📌 Pinout

### ESP32-C3

```
    ┌─────────────────────────┐
    │        ESP32-C3         │
    │                         │
    │    [LED]   [BTN]       │
    │      ●       ○         │
    │                        │
    │   GPIO 8    GPIO 9     │
    │                        │
    │  ┌──┬──┬──┬──┬──┬──┐  │
    │  │  │  │  │  │  │  │  │
    │  │TX│RX│  │  │  │  │  │
    │  │ 4│ 5│  │  │  │  │  │
    │  └──┴──┴──┴──┴──┴──┘  │
    │                        │
    │  ┌──┬──┬──┬──┬──┬──┐  │
    │  │  │  │  │  │  │  │  │
    │  │A0│A1│A2│A3│S1│S2│  │
    │  │ 0│ 1│ 2│ 3│ 4│ 5│  │
    │  └──┴──┴──┴──┴──┴──┘  │
    │                        │
    └─────────────────────────┘
```

| Function | GPIO | Notes |
|----------|------|-------|
| 🔴 LED | 8 | Status LED |
| 🔘 Button | 9 | Boot button |
| 📤 UART TX | 4 | To FC |
| 📥 UART RX | 5 | From FC |
| 🎮 ADC 0-3 | 0-3 | Joysticks |
| 🔀 Switch A-D | 4-7 | Aux switches |

### ESP32-S3

| Function | GPIO | Notes |
|----------|------|-------|
| 🔴 LED | 48 | Status LED |
| 📤 UART TX | 17 | To FC |
| 📥 UART RX | 18 | From FC |
| 🎮 ADC 1-4 | 1-4 | Joysticks |

---

## 📡 Protocols

| Protocol | Speed | Format | Use |
|----------|-------|--------|-----|
| 🔢 **Serial** | 2 Mbps | Binary | High speed |
| 📻 **SBUS** | 100 Kbps | 8E2 | Futaba |
| 🚀 **CRSF** | 420 Kbps | ELRS | ExpressLRS |

---

## 🔵 Connection States

```
    ┌────────────────────────────────────────────┐
    │           CONNECTION FLOW                   │
    └────────────────────────────────────────────┘
    
    DISCONNECTED          💤
         │
         ▼
    ┌─────────┐    No peer    ┌───────────┐
    │SEARCHING│◄─────────────►│RECONNECTING│
    │ 🔵 ●    │               │  🟠 ●     │
    │         │   + peer      │            │
    └────┬────┘               └─────┬──────┘
         │                            │
         │    ESP-NOW linked          │
         ▼                            ▼
    ┌─────────┐               ┌───────────┐
    │ PAIRING │─────────────►│ CONNECTED │
    │ 🟢 ●●   │               │  🟢 ●●●  │
    └─────────┘               └───────────┘
```

| State | LED | Signal |
|-------|-----|--------|
| 💤 DISCONNECTED | ⬛ Off | No peer |
| 🔍 SEARCHING | 🔵 Slow blink | Scanning |
| 🔗 PAIRING | 🟢 Fast blink | Linking |
| ✅ CONNECTED | 🟢 Solid | Active |
| 🔄 RECONNECTING | 🟠 Slow blink | Retrying |

---

## 🔊 Audio Feedback

| Event | Sound | Pattern |
|-------|-------|---------|
| 🔍 Searching | beep-beep | ● ─ ● ─ |
| ✅ Connected | 3 beeps | ● ● ● |
| ❌ Lost | sad tone | ● ─ ● ─ ● |
| ✅ Reconnected | 2 beeps | ● ● |

---

## 🛠️ Building

```bash
# Build receiver
pio run -d esp32-rc -e esp32c3-rc

# Build transmitter
pio run -d esp32-rc -e esp32c3-tx

# Upload
pio run -d esp32-rc -e esp32c3-rc --target upload \
    --upload-port /dev/ttyUSB0
```

---

## 📊 Memory

```
    ┌─────────────────────────────────┐
    │         MEMORY USAGE            │
    ├─────────────────────────────────┤
    │                                 │
    │  RAM   ████████░░░░░░  11.6%  │
    │         ~38 KB / 320 KB         │
    │                                 │
    │  FLASH ██████████████░  21.9%  │
    │         ~730 KB / 8 MB          │
    │                                 │
    └─────────────────────────────────┘
```

---

## 📜 License

MIT • Made by [btechioi](https://github.com/btechioi)
