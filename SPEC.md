# DroneFirmware Protocol Specification

## 1. RC → FC Serial Protocol

**Baud**: 2 Mbps, 8N1

### RC Channels Packet (0xAA 0x01)
```
Offset  Size  Description
------  ----  -----------
0       1     Sync: 0xAA
1       1     Type: 0x01
2       1     Channel count (N)
3       2N    Channel values (1000-2000, little-endian)
3+2N   2     CRC16
```

### FC Telemetry Packet (0x55 0x02)
```
Offset  Size  Description
------  ----  -----------
0       1     Sync: 0x55
1       1     Type: 0x02
2       1     Seq number
3       1     Flags (bit 0: armed, bit 1: companion)
4       1     Control source
5       6     Attitude (roll, pitch, yaw × int16, ÷100 for rad)
11      4     Altitude (int32, cm)
15      6     Gyro rates (×3 int16)
21      2     Battery voltage (mV)
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

## 2. ESP-NOW Protocol

### RC Data Packet (TX → RX)
```cpp
struct ESPNOWPacket {
    uint8_t magic;           // 0xAA
    uint8_t type;             // 0x01
    uint32_t timestamp;       // μs since boot
    uint8_t channelCount;    // 4-16
    uint16_t channels[16];    // 1000-2000
    int8_t rssi;
    uint8_t flags;
};
```

## 3. SPI Protocol (FC ↔ Companion)

**Speed**: 125 MHz

| Command | Description |
|---------|-------------|
| 0x01 | PING/PONG |
| 0x02 | RC Channels |
| 0x03 | Telemetry |
| 0x04 | Settings |

## 4. Mavlink (GCS ↔ FC)

### Custom Messages

| ID | Name | Direction | Payload |
|----|------|-----------|---------|
| 200 | PID_TUNE_CMD | GCS → FC | cmd, method, axis, amplitude |
| 201 | PID_TUNE_STATUS | FC → GCS | state, progress |
| 202 | PID_TUNE_RESULT | FC → GCS | kp, ki, kd |
| 203 | PID_GAINS | ↔ | axis, kp, ki, kd |

### PID Tuning Methods
| Value | Method |
|-------|--------|
| 0 | ZIEGLER_NICHOLS |
| 1 | RELAY |
| 2 | STEP_RESPONSE |
| 3 | FREQUENCY_SWEEP |

## 5. Enums

### FlightMode (src/main/flight_mode.h)
```cpp
enum class FlightMode {
    MANUAL = 0,
    ALTHOLD = 1,
    POSHOLD = 2,
    WAYPOINT = 3,
    RTL = 4,
    TAKEOFF = 5,
    LAND = 6
};
```

### FailSafeState
```cpp
enum class FailSafeState {
    NONE = 0,
    SIGNAL_LOSS = 1,
    LOW_BATTERY = 2,
    CRITICAL_SENSOR = 3
};
```

### ControlSource
```cpp
enum class ControlSource {
    RC_RECEIVER = 0,
    COMPANION = 1,
    FAILSAFE = 2
};
```

## 6. RC Channel Mapping

| Channel | Function | Range |
|---------|----------|-------|
| 0 | Roll | 1000-2000 |
| 1 | Pitch | 1000-2000 |
| 2 | Throttle | 1000-2000 |
| 3 | Yaw | 1000-2000 |
| 4 | Aux1 (Arm) | 1000/2000 |
| 5 | Aux2 (Mode) | 1000/2000 |

## 7. Failsafe Triggers

| Trigger | Timeout | Action |
|---------|---------|--------|
| RC Signal Loss | 500ms | Hover at throttle 1200 |
| Stuck Controls | 3s | Reset to neutral |
| Companion Timeout | 2s | Return to RC |
| IMU Failure | - | Disarm immediately |

## 8. Constants

```cpp
// Loop rates
FAST_LOOP_HZ = 400
FAST_LOOP_US = 2500

// RC
RC_SIGNAL_TIMEOUT_MS = 500
STUCK_CONTROL_TIMEOUT_MS = 3000

// Companion
COMPANION_HEARTBEAT_TIMEOUT_MS = 2000
PING_INTERVAL_MS = 500

// RC channels
STUCK_CONTROL_THRESHOLD = 20
```
