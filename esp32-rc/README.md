<p align="center">
  <img src="https://capsule-render.vercel.app/api?type=waving&color=0:ff6b35,100:ff0000&height=180&section=header&text=ESP32%20RC%20Module&fontSize=45&fontAlignY=35&animation=fadeIn&fontColor=ffffff"/>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/ESP32-C3/S3-CC0000?style=for-the-badge" alt="Chip">
  <img src="https://img.shields.io/badge/ESP--NOW-003366?style=for-the-badge" alt="Wireless">
</p>

Wireless RC module for drones. Handles the link between your transmitter and the flight controller.

## Build

```bash
pio run -d esp32-rc -e esp32c3-rc    # Receiver
pio run -d esp32-rc -e esp32c3-tx    # Transmitter
```

## Flash

```bash
esptool.py --chip esp32c3 --port /dev/ttyUSB0 write_flash 0x0 firmware/rc_receiver_esp32c3.bin
```

## Modes

| Mode | Description |
|------|-------------|
| RECEIVER | Picks up ESP-NOW, outputs SBUS/CRSF/Serial to FC |
| TRANSMITTER | Takes input from PC, broadcasts ESP-NOW |
| BRIDGE | Direct PC ↔ FC passthrough |

## LED Patterns

| State | LED |
|-------|-----|
| Slow blink | Searching for peer |
| Fast blink | Pairing |
| Double blink | RC connected |
| Solid | Fully connected |

## Pinout

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
| 17 | UART TX |
| 18 | UART RX |
| 48 | Status LED |

## Auto-Pairing

The module scans for any ESP-NOW broadcaster on startup. No MAC addresses or channels to configure.

## Memory

- RAM: ~38 KB
- Flash: ~730 KB
