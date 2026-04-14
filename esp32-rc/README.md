# ESP32-C3 RC Transmitter/Receiver

ESP32-C3 based RC transmitter/receiver module with ESP-NOW wireless, multiple protocol support, and comprehensive telemetry.

## Features

- **Auto-Pairing**: Automatically connects to any ESP-NOW transmitter
- **Multi-Protocol**: SBUS, CRSF, and Serial output
- **16 Channels**: Up to 16 RC channels supported
- **Full Telemetry**: Comprehensive status sent to PC and FC
- **Audio Cues**: Connection status and find-drone buzzer
- **Low Power**: Efficient ESP32-C3 design

## Modes

### RECEIVER Mode (Default)
Plugs into the drone, receives RC from transmitter, outputs to FC.

```
ESP-NOW ← [TRANSMITTER] ← [PC/USB]
                ↓
    ┌───────────────────────────────┐
    │         RECEIVER              │
    │                               │
    │  RC → SBUS → FC              │
    │  RC → CRSF → FC             │
    │  RC → Serial → FC            │
    │                               │
    │  Telemetry ← Serial ← FC    │
    │  Telemetry → ESP-NOW → TX    │
    │  Status → Serial → PC        │
    └───────────────────────────────┘
```

### TRANSMITTER Mode
Handheld controller, receives RC from PC, sends via ESP-NOW.

```
[PC/USB] → Serial → [TRANSMITTER] → ESP-NOW → [RECEIVER]
                     ↓
              Telemetry ← ESP-NOW
```

### BRIDGE Mode
Direct PC ↔ FC passthrough.

```
[PC/USB] → Serial ↔ [BRIDGE] ↔ Serial → [FC]
```

## Pinout

### ESP32-C3

| Function | GPIO | Notes |
|----------|------|-------|
| LED | 8 | Status LED |
| Button | 9 | Boot button |
| UART1 TX | 4 | To FC |
| UART1 RX | 5 | From FC |
| ADC0 | 0 | Joystick Y |
| ADC1 | 1 | Joystick X |
| ADC2 | 2 | Throttle |
| ADC3 | 3 | Rudder |
| Switch A | 4 | Mode switch |
| Switch B | 5 | Aux |
| Switch C | 6 | Aux |
| Switch D | 7 | Aux |

### ESP32-S3

| Function | GPIO | Notes |
|----------|------|-------|
| LED | 48 | Status LED |
| Button | 0 | Boot button |
| UART1 TX | 17 | To FC |
| UART1 RX | 18 | From FC |
| ADC1 | 1-4 | Joysticks/Switches |

## Protocols

### Serial Protocol (2 Mbps)

Binary only, no human-readable output.

| Packet | Sync | Type | Size |
|--------|------|------|------|
| RC Channels | 0xAA | 0x01 | 3+2N bytes |
| FC Telemetry | 0x55 | 0x02 | 80 bytes |
| RC Status | 0x66 | 0x03 | 48 bytes |

### SBUS Output (100Kbps, 8E2)

16 channels, inverted signal, standard SBUS format.

### CRSF Output (420Kbps)

16 channels, ELRS/CRSF protocol compatible.

## Connection States

| State | LED | Audio |
|-------|-----|-------|
| DISCONNECTED | Off | - |
| SEARCHING | Slow blink | beep-beep |
| PAIRING | Fast blink | - |
| CONNECTED | Solid | ✓✓✓ |
| RECONNECTING | Slow blink | 💔 sad |

## RC Status Structure (48 bytes)

```cpp
struct RCStatus {
    uint32_t uptime;           // Time since boot (ms)
    float batteryVoltage;       // Battery voltage (V)
    int8_t batteryPercent;     // Battery percentage
    int8_t txRssi;            // Signal strength (dBm)
    int8_t txSignalQuality;   // Signal quality (0-100%)
    uint16_t txPacketLoss;    // Packet loss (%)
    uint32_t txLatency;       // Latency (μs)
    float temperature;         // Module temp (°C)
    uint32_t freeMemory;       // Free RAM (bytes)
    uint8_t cpuLoad;          // CPU usage (%)
    uint8_t espnowChannel;    // WiFi channel
    uint8_t connectionState;   // 0=disconnected, 1=searching, 3=connected
    uint8_t errorFlags;       // Error flags
};
```

## Building

```bash
# ESP32-C3
pio run -e esp32c3-rc

# ESP32-S3
pio run -e esp32s3-rc

# Upload
pio run -e esp32c3-rc --target upload
```

## Configuration

### Mode Selection
Edit `src/main.cpp`:

```cpp
#define MODE_RECEIVER    // Default: Receiver on drone
// #define MODE_TRANSMITTER // Handheld controller
// #define MODE_BRIDGE      // PC ↔ FC passthrough
```

### Peer MAC Address
For fixed pairing, set peer MAC:

```cpp
uint8_t peerMAC[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
```

Default: Auto-scan (any ESP-NOW transmitter).

## Audio Cues

| Event | Sound | Description |
|-------|-------|-------------|
| Searching | 🔍 beep-beep | Looking for RC |
| Connected | ✓✓✓ | RC paired successfully |
| Lost | 💔 sad tone | RC link lost |
| Reconnected | ✓✓ | RC reconnected |

## Memory Usage

- RAM: ~38KB (11.6%)
- Flash: ~730KB (21.9%)
