<p align="center">
  <img src="https://capsule-render.vercel.app/api?type=waving&color=0:ff6b35,100:ff0000&height=180&section=header&text=ESP32%20RC%20Module&fontSize=45&fontAlignY=35&animation=fadeIn&fontColor=ffffff"/>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/ESP32-C3/S3-CC0000?style=for-the-badge" alt="Chip">
  <img src="https://img.shields.io/badge/ESP--NOW-003366?style=for-the-badge" alt="Wireless">
</p>

Wireless RC module for drones. Handles the link between your transmitter and the flight controller.

## Modes

**RECEIVER** (default) - sits on the drone
- Listens for ESP-NOW transmissions
- Outputs SBUS, CRSF, or serial to FC
- Sends telemetry back to transmitter

**TRANSMITTER** - handheld controller
- Takes RC input from PC via USB
- Broadcasts over ESP-NOW

**BRIDGE** - direct passthrough
- Passes PC ↔ FC without wireless

## Wiring

### ESP32-C3

| GPIO | Function |
|------|----------|
| 4 | UART TX to FC |
| 5 | UART RX from FC |
| 8 | Status LED |
| 0-3 | ADC inputs |

### ESP32-S3

| GPIO | Function |
|------|----------|
| 17 | UART TX to FC |
| 18 | UART RX from FC |
| 48 | Status LED |

## Build

```bash
# Receiver
pio run -d esp32-rc -e esp32c3-rc

# Transmitter  
pio run -d esp32-rc -e esp32c3-tx
```

## Flash

```bash
esptool.py --chip esp32c3 --port /dev/ttyUSB0 write_flash 0x0 firmware/rc_receiver_esp32c3.bin
```

## Auto-Pairing

The module scans for any ESP-NOW broadcaster on startup. No MAC address or channel configuration needed. Once paired, it reconnects automatically if the link drops.

## Status LED

| State | LED |
|-------|-----|
| Off | No peer found |
| Slow blink | Searching |
| Fast blink | Pairing |
| Solid | Connected |

## Memory

- RAM: ~38 KB
- Flash: ~730 KB
