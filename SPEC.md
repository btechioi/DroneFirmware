# DroneFlightController - Protocol Specification

## Overview

Binary protocols for the drone system:
1. **FC** - Raspberry Pi Pico
2. **RC** - ESP32-C3/S3 (ESP-NOW + Serial)
3. **Companion** - Pi Zero 2W (SPI)
4. **GCS** - PyQt6 desktop app (Mavlink)

## Connection Diagram

```
PC/GCS ──USB──► ESP32 TX ──ESP-NOW──► ESP32 RX ──Serial──► Pico FC
                                                    │
                                                    │ SPI
                                                    ▼
                                             Pi Zero 2W
```

## 1. RC → FC Serial (2 Mbps)

Binary protocol, 8N1.

### RC Channels Packet (0xAA 0x01)
```
Offset  Size  Description
------  ----  -----------
0       1     Sync: 0xAA
1       1     Type: 0x01
2       1     Channel count (N)
3       2N    Channel values (1000-2000, little-endian)
3+2N    2     CRC16
```

### FC Telemetry Packet (0x55 0x02)
```
Offset  Size  Description
------  ----  -----------
0       1     Sync: 0x55
1       1     Type: 0x02
2       1     Flags (armed, failsafe)
3       6     Attitude (roll, pitch, yaw × int16)
9       4     Altitude (int32, cm)
13      6     Gyro rates
19      2     Battery (mV)
```

### RC Status Packet (0x66 0x03)
```
Offset  Size  Description
------  ----  -----------
0       1     Sync: 0x66
1       1     Type: 0x03
2       4     Uptime (ms)
6       2     Battery voltage (mV)
8       1     Battery percent
9       1     RSSI (dBm)
10      1     Signal quality (%)
11      2     Packet loss (×10 %)
13      4     Latency (μs)
17      2     Temperature (×10 °C)
19      4     Free memory
23      1     CPU load (%)
24      1     ESP-NOW channel
25      1     Connection state
26      1     Error flags
```

## 2. FC ↔ Companion SPI (125 MHz)

```
┌────────┬──────┬────────┬────────┬────────┐
│  CMD   │ LEN  │  DATA  │  DATA  │  CRC   │
│  1B   │  2B  │ N bytes │  ...   │  1B   │
└────────┴──────┴────────┴────────┴────────┘
```

| Command | Description |
|---------|-------------|
| 0x01 | PING |
| 0x02 | RC Channels |
| 0x03 | Telemetry |
| 0x04 | Settings |

## 3. RC ↔ TX ESP-NOW

### RC Data Packet
```cpp
struct ESPNOWPacket {
    uint8_t magic;           // 0xAA
    uint8_t type;            // 0x01
    uint32_t timestamp;       // μs
    uint8_t channelCount;
    uint16_t channels[16];   // 1000-2000
    int8_t rssi;
    uint8_t flags;
};
```

## 4. Mavlink (GCS ↔ FC)

Standard messages + custom PID tuning (ID 200-204).

| ID | Name | Direction |
|----|------|-----------|
| 200 | PID_TUNE_CMD | GCS → FC |
| 201 | PID_TUNE_STATUS | FC → GCS |
| 202 | PID_TUNE_RESULT | FC → GCS |
| 203 | PID_GAINS | ↔ |

### PID Tuning Methods
| Value | Method |
|-------|--------|
| 0 | ZIEGLER_NICHOLS |
| 1 | RELAY |
| 2 | STEP_RESPONSE |
| 3 | FREQUENCY_SWEEP |

## 5. RC Channel Mapping

| Channel | Input | Range |
|---------|-------|-------|
| 1 | Roll | 1000-2000 |
| 2 | Pitch | 1000-2000 |
| 3 | Throttle | 1000-2000 |
| 4 | Yaw | 1000-2000 |
| 5 | Aux1 | 1000/2000 (Arm) |
| 6 | Aux2 | Mode |

## 6. Failsafe States

| State | Trigger | Action |
|-------|---------|--------|
| SIGNAL_LOSS | No RC 500ms | Hover at throttle |
| LOW_BATTERY | <20% | Land |
| CRITICAL_SENSOR | IMU fail | Disarm |

## 7. Data Types

| Type | Size |
|------|------|
| uint8_t | 1 byte |
| uint16_t | 2 bytes |
| uint32_t | 4 bytes |
| int16_t | 2 bytes |
| float | 4 bytes |
