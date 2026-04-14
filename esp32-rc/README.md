# ESP32 RC Transmitter/Receiver

<p align="center">
  <img src="https://img.shields.io/badge/Chip-ESP32--C3/S3-orange" alt="Chip">
  <img src="https://img.shields.io/badge/Wireless-ESP--NOW-blue" alt="Wireless">
  <img src="https://img.shields.io/badge/Channels-16-red" alt="Channels">
  <img src="https://img.shields.io/badge/Protocol-SBUS--CRSF-green" alt="Protocol">
</p>

ESP32-C3/S3 based RC transmitter/receiver with **ESP-NOW wireless**, auto-pairing, and comprehensive telemetry.

## Features

- **Auto-Pairing** - No manual configuration required
- **Multi-Protocol** - SBUS, CRSF, Serial output
- **16 Channels** - Full RC channel support
- **Full Telemetry** - Status sent to PC and FC
- **Audio Cues** - Connection status feedback
- **Low Power** - Efficient ESP32-C3 design

## Modes

### RECEIVER (Default)
```
ESP-NOW ← [TRANSMITTER] ← [PC USB]
                ↓
    ┌───────────────────────────┐
    │        RECEIVER           │
    │                           │
    │  RC → SBUS/CRSF → FC     │
    │  Telemetry ↔ Serial      │
    │  Status → ESP-NOW → TX    │
    └───────────────────────────┘
```

### TRANSMITTER
```
[PC USB] → Serial → [TRANSMITTER] → ESP-NOW → [RECEIVER]
```

### BRIDGE
```
[PC USB] → Serial ↔ [BRIDGE] ↔ Serial → [FC]
```

## Pinout

### ESP32-C3

| Function | GPIO | Notes |
|----------|------|-------|
| LED | 8 | Status LED |
| UART1 TX | 4 | To FC |
| UART1 RX | 5 | From FC |
| ADC0-3 | 0-3 | Joysticks |
| Switch A-D | 4-7 | Aux switches |

### ESP32-S3

| Function | GPIO | Notes |
|----------|------|-------|
| LED | 48 | Status LED |
| UART1 TX | 17 | To FC |
| UART1 RX | 18 | From FC |
| ADC1-4 | 1-4 | Joysticks |

## Protocols

| Protocol | Speed | Format |
|----------|-------|--------|
| Serial | 2 Mbps | Binary |
| SBUS | 100 Kbps | 8E2 |
| CRSF | 420 Kbps | ELRS compatible |

## Connection States

| State | LED | Description |
|-------|-----|-------------|
| DISCONNECTED | Off | No peer |
| SEARCHING | Slow blink | Scanning for TX |
| PAIRING | Fast blink | Establishing link |
| CONNECTED | Solid | Link active |
| RECONNECTING | Slow blink | Retrying |

## Building

```bash
# Receiver
pio run -d esp32-rc -e esp32c3-rc

# Transmitter
pio run -d esp32-rc -e esp32c3-tx

# Upload
pio run -d esp32-rc -e esp32c3-rc --target upload --upload-port /dev/ttyUSB0
```

## Memory

- RAM: ~38 KB (11.6%)
- Flash: ~730 KB (21.9%)
