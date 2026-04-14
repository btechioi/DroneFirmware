# DroneFirmware Protocol Specification

Complete protocol documentation for:
1. RC → FC Serial Protocol
2. FC → RC Telemetry Protocol
3. RC Status Protocol
4. ESP-NOW Protocol
5. SPI Protocol (FC ↔ Companion)
6. Mavlink Protocol (GCS ↔ FC)

---

## 1. RC → FC Serial Protocol

**Baud Rate**: 2,000,000 (2 Mbps)  
**Format**: 8N1 (8 data bits, no parity, 1 stop bit)  
**Flow**: RC Module → Flight Controller

### RC Channels Packet (0xAA 0x01)

Transmits RC control channels from RC module to FC.

```
Offset  Size  Type      Description
------  ----  --------  -----------
0       1     uint8     Sync: 0xAA
1       1     uint8     Type: 0x01
2       1     uint8     Channel count (N, typically 8)
3       2N    uint16[]  Channel values (little-endian)
                       Each channel: 1000-2000 μs
3+2N    2     uint16    CRC16 (sum of bytes 0 to 3+2N-1)
```

**Example for 8 channels (19 bytes):**
```
Byte 0:  0xAA (sync)
Byte 1:  0x01 (type)
Byte 2:  0x08 (8 channels)
Byte 3:  0xDC, 0x05 (ch[0] = 1500)
Byte 5:  0xDC, 0x05 (ch[1] = 1500)
Byte 7:  0xE8, 0x03 (ch[2] = 1000, throttle)
Byte 9:  0xDC, 0x05 (ch[3] = 1500)
Byte 11: 0xDC, 0x05 (ch[4] = 1500)
Byte 13: 0xDC, 0x05 (ch[5] = 1500)
Byte 15: 0xDC, 0x05 (ch[6] = 1500)
Byte 17: 0xDC, 0x05 (ch[7] = 1500)
Byte 19: 0x00, 0x00 (CRC16)
```

### Channel Mapping

| Index | Function | Typical Value | Notes |
|-------|----------|---------------|-------|
| 0 | Roll | 1000-2000 | 1500 = center |
| 1 | Pitch | 1000-2000 | 1500 = center |
| 2 | Throttle | 1000-2000 | 1000 = min |
| 3 | Yaw | 1000-2000 | 1500 = center |
| 4 | Aux1 | 1000/2000 | Arm switch |
| 5 | Aux2 | 1000/2000 | Mode select |
| 6 | Aux3 | 1000/2000 | Reserved |
| 7 | Aux4 | 1000/2000 | Reserved |

---

## 2. FC → RC Telemetry Protocol

**Baud Rate**: 2,000,000 (2 Mbps)  
**Format**: 8N1  
**Flow**: Flight Controller → RC Module

### FC Telemetry Packet (0x55 0x02)

FC sends real-time flight data to RC module for forwarding.

```
Offset  Size  Type      Description
------  ----  --------  -----------
0       1     uint8     Sync: 0x55
1       1     uint8     Type: 0x02
2       1     uint8     Sequence number (increments)
3       1     uint8     Flags
                       Bit 0: armed (1) / disarmed (0)
                       Bit 1: companion connected (1)
                       Bit 4-7: failsafe state
4       1     uint8     Control source
                       0 = RC_RECEIVER
                       1 = COMPANION
                       2 = FAILSAFE
5       2     int16     Roll angle (×100 rad, divide by 100 for radians)
7       2     int16     Pitch angle (×100 rad)
9       2     int16     Yaw angle (×100 rad)
11      4     int32     Altitude (cm, from barometer)
15      2     int16     Gyro X rate (×100 rad/s)
17      2     int16     Gyro Y rate (×100 rad/s)
19      2     int16     Gyro Z rate (×100 rad/s)
21      2     uint16    Battery voltage (mV)
23      2     uint16    CRC16
```

---

## 3. RC Status Protocol

**Baud Rate**: 2,000,000 (2 Mbps)  
**Format**: 8N1  
**Flow**: RC Module → FC and PC

### RC Status Packet (0x66 0x03)

Comprehensive status from RC module.

```
Offset  Size  Type      Description
------  ----  --------  -----------
0       1     uint8     Sync: 0x66
1       1     uint8     Type: 0x03
2       4     uint32    Uptime since boot (ms)
6       2     uint16    Battery voltage (mV)
8       1     int8      Battery percent (0-100)
9       1     int8      RSSI (dBm, negative value)
10      1     uint8     Signal quality (0-100%)
11      2     uint16    Packet loss (×10, so 50 = 5%)
13      4     uint32    Link latency (μs)
17      2     int16     Temperature (×10 °C, so 235 = 23.5°C)
19      4     uint32    Free memory (bytes)
23      1     uint8     CPU load (0-100%)
24      1     uint8     ESP-NOW WiFi channel (1-13)
25      1     uint8     Connection state
                       0 = DISCONNECTED
                       1 = SEARCHING
                       2 = PAIRING
                       3 = CONNECTED
                       4 = RECONNECTING
26      1     uint8     Error flags
                       Bit 0: voltage low
                       Bit 1: temperature high
                       Bit 2: memory low
27      2     uint16    Reserved
29      2     uint16    CRC16
```

---

## 4. ESP-NOW Protocol

**Physical Layer**:
- Protocol: ESP-NOW (based on WiFi)
- Frequency: 2.4 GHz (WiFi channels 1-11)
- Max Range: ~100m line of sight
- Max Payload: 250 bytes
- Data Rate: 2 Mbps
- Encryption: None (default)

### RC Data Packet (Transmitter → Receiver)

Broadcast from RC transmitter to receiver.

```
Offset  Size  Type      Description
------  ----  --------  -----------
0       1     uint8     Magic: 0xAA
1       1     uint8     Type: 0x01
2       4     uint32    Timestamp (μs since boot)
6       1     uint8     Channel count (N, 4-16)
7       2N    uint16[]  Channel values (little-endian, 1000-2000)
7+2N    1     int8      RSSI (dBm)
8+2N    1     uint8     Flags
                       Bit 0: low battery
                       Bit 1: signal weak
```

### Telemetry Packet (Receiver → Transmitter)

Optional telemetry from FC forwarded via ESP-NOW.

```
Offset  Size  Type      Description
------  ----  --------  -----------
0       1     uint8     Magic: 0x55
1       1     uint8     Type: 0x02
2       4     uint32    Timestamp
6       1     uint8     Length of telemetry data
7       N     uint8[]   Telemetry data from FC
```

### Connection States

| State | Value | LED Pattern | Description |
|-------|-------|-------------|-------------|
| DISCONNECTED | 0 | Off | No peer found |
| SEARCHING | 1 | Slow pulse | Scanning for TX |
| PAIRING | 2 | Fast blink | Establishing link |
| CONNECTED | 3 | Solid | Link active |
| RECONNECTING | 4 | Slow blink | Lost link, retrying |

---

## 5. SPI Protocol (FC ↔ Companion)

**Speed**: 125 MHz (125,000,000 Hz)  
**Mode**: SPI Mode 0 (CPOL=0, CPHA=0)  
**Bit Order**: MSB First  
**Voltage**: 3.3V

### Frame Structure

```
┌────────┬──────┬────────┬────────┬────────┐
│  CMD   │ LEN  │  DATA  │  DATA  │  CRC   │
│  1B   │  2B  │ N bytes │  ...   │  1B   │
└────────┴──────┴────────┴────────┴────────┘
```

| Offset | Field | Type | Description |
|--------|-------|------|-------------|
| 0 | CMD | uint8 | Command type |
| 1 | LEN_L | uint8 | Length low byte |
| 2 | LEN_H | uint8 | Length high byte |
| 3 | DATA | uint8[] | Payload data |
| 3+N | CRC | uint8 | XOR of all bytes |

### Command Types

| CMD | Name | Direction | Description |
|-----|------|-----------|-------------|
| 0x01 | PING | ↔ | Heartbeat/keepalive |
| 0x02 | RC_CHANNELS | Comp→FC | RC control input |
| 0x03 | TELEMETRY | FC→Comp | Flight data |
| 0x04 | SETTINGS | ↔ | Configuration |
| 0x05 | RAW_SENSOR | FC→Comp | Raw IMU data |
| 0x10 | MOTOR_OUTPUT | FC→Comp | Motor PWM values |
| 0xFF | NOP | ↔ | No operation |

### PING (0x01)

```
Offset  Size  Type      Description
------  ----  --------  -----------
0       1     uint8     CMD: 0x01
1       2     uint16    Length: 0x0005
3       4     uint32    Timestamp (ms since boot)
7       1     uint8     Flags
                       Bit 0: armed
                       Bit 1: connected
8       1     uint8     CRC
```

### RC Channels (0x02)

```
Offset  Size  Type      Description
------  ----  --------  -----------
0       1     uint8     CMD: 0x02
1       2     uint16    Length: 0x000D
3       2     uint16    Roll (1000-2000)
5       2     uint16    Pitch
7       2     uint16    Throttle
9       2     uint16    Yaw
11      2     uint16    Aux1
13      2     uint16    Aux2
15      1     uint8     CRC
```

### Telemetry (0x03)

```
Offset  Size  Type      Description
------  ----  --------  -----------
0       1     uint8     CMD: 0x03
1       2     uint16    Length: 0x0019 (25)
3       4     uint32    Timestamp
7       1     uint8     Armed (0/1) + Mode (bits 4-7)
8       4     float     Roll angle (radians)
12      4     float     Pitch angle (radians)
16      4     float     Yaw angle (radians)
20      4     float     Altitude (meters)
24      4     float     Battery voltage (V)
28      1     uint8     CRC
```

### Heartbeat Timeout

- FC expects PING every 500ms
- Companion expects PONG within 100ms
- If no PING for 2s, companion assumes FC disconnected
- If no PONG for 2s, FC assumes companion disconnected

---

## 6. Mavlink Protocol (GCS ↔ FC)

Used for Ground Control Station communication.

### Standard Messages

| ID | Name | Direction | Rate | Description |
|----|------|-----------|------|-------------|
| 0 | HEARTBEAT | ↔ | 1 Hz | System heartbeat |
| 1 | SYS_STATUS | → | 1 Hz | System status |
| 30 | ATTITUDE | → | 100 Hz | Roll, pitch, yaw |
| 33 | GLOBAL_POSITION_INT | → | 10 Hz | GPS position |
| 35 | RC_CHANNELS_RAW | → | 50 Hz | RC channel values |
| 69 | RC_CHANNELS_OVERRIDE | ← | 50 Hz | Manual control |
| 74 | VFR_HUD | → | 10 Hz | Flight data display |

### Custom Messages (ID 200-204)

#### PID_TUNE_CMD (200) - GCS → FC

```
Field        Type    Offset  Description
-----------  ------  ------  -----------
command      uint8   0       0x01=Start, 0x02=Stop, 0x03=Abort
method       uint8   1       0-3 (see methods below)
axis         uint8   2       0-7 (see axes below)
amplitude    float   3       Excitation amplitude
```

**Tuning Methods:**

| Value | Method | Description |
|-------|--------|-------------|
| 0 | ZIEGLER_NICHOLS | Ultimate gain oscillation |
| 1 | RELAY | Åström-Hägglund relay feedback |
| 2 | STEP_RESPONSE | First-order step response |
| 3 | FREQUENCY_SWEEP | Bode plot analysis |

**Tuning Axes:**

| Value | Axis | PID |
|-------|------|-----|
| 0 | ROLL_RATE | Roll rate |
| 1 | PITCH_RATE | Pitch rate |
| 2 | YAW_RATE | Yaw rate |
| 3 | ROLL_ATTITUDE | Roll angle |
| 4 | PITCH_ATTITUDE | Pitch angle |
| 5 | YAW_ATTITUDE | Yaw angle |
| 6 | ALTITUDE | Altitude hold |
| 7 | POSITION | Position control |

#### PID_TUNE_STATUS (201) - FC → GCS

```
Field        Type    Offset  Description
-----------  ------  ------  -----------
state        uint8   0       0=IDLE, 1=WAITING, 2=COLLECTING, 3=ANALYZING, 4=APPLYING, 5=COMPLETE, 6=FAILED
method       uint8   1       Current method
axis         uint8   2       Current axis
progress     uint16  3       Progress (0-10000 = 0.00-100.00%)
timestamp    uint32  5       Tune start time (ms)
active       uint8   9       1=actively tuning
```

#### PID_TUNE_RESULT (202) - FC → GCS

```
Field          Type    Offset  Description
-------------  ------  ------  -----------
success        uint8   0       1=success, 0=failed
kp             float   1       Proportional gain
ki             float   5       Integral gain
kd             float   9       Derivative gain
overshoot      float   13      Overshoot percentage
settling_time  float   17      Settling time (s)
rise_time      float   21      Rise time (s)
tune_time      uint32  25      Total tuning time (ms)
```

#### PID_GAINS (203) - ↔

**Single Axis (13 bytes):**
```
Field  Type    Offset  Description
-----  ------  ------  -----------
axis   uint8   0       0-7
kp     float   1       Proportional
ki     float   5       Integral
kd     float   9       Derivative
```

**Full Profile (120 bytes):**
```
Offset  Field           Type    Description
------  -----           ------  -----------
0       roll_rate_kp    float
4       roll_rate_ki    float
8       roll_rate_kd    float
12      pitch_rate_kp   float
16      pitch_rate_ki   float
20      pitch_rate_kd   float
24      yaw_rate_kp     float
28      yaw_rate_ki     float
32      yaw_rate_kd     float
36      roll_att_kp     float
40      roll_att_ki     float
44      roll_att_kd     float
48      pitch_att_kp    float
52      pitch_att_ki    float
56      pitch_att_kd    float
60      yaw_att_kp      float
64      yaw_att_ki      float
68      yaw_att_kd      float
72      alt_kp          float
76      alt_ki          float
80      alt_kd          float
84      pos_kp          float
88      pos_ki          float
92      pos_kd          float
96      version         uint32
100     timestamp       uint32
104     reserved        16B
```

#### PID_PROFILE_CMD (204) - GCS → FC

```
Field   Type    Offset  Description
-----   ------  ------  -----------
cmd     uint8   0       0x10=Save, 0x11=Load, 0x12=Reset, 0x21=Request all
slot    uint8   1       Profile slot (0-3)
```

---

## 7. Failsafe Protocol

### Failsafe States

| State | Value | Trigger | Action |
|-------|-------|---------|--------|
| NONE | 0 | Normal operation | - |
| SIGNAL_LOSS | 1 | No RC for 500ms | Hover at throttle 1200 |
| LOW_BATTERY | 2 | Battery < 20% | Land |
| CRITICAL_SENSOR | 3 | IMU failure | Disarm |

### Failsafe Recovery

1. **RC Signal Lost**:
   - Wait 500ms for signal
   - If armed: enter SIGNAL_LOSS failsafe
   - Maintain hover at throttle 1200
   - Auto-recover when RC returns

2. **Battery Low**:
   - Warning at 25%
   - Critical at 15%
   - Auto-land sequence

3. **IMU Failure**:
   - Immediate disarm
   - Prevent re-arm
   - Solid red LED

---

## 8. Data Types

| Type | Size | Range | Description |
|------|------|-------|-------------|
| int8 | 1 | -128 to 127 | Signed byte |
| uint8 | 1 | 0 to 255 | Unsigned byte |
| int16 | 2 | -32768 to 32767 | Signed 16-bit |
| uint16 | 2 | 0 to 65535 | Unsigned 16-bit |
| int32 | 4 | ±2.1×10⁹ | Signed 32-bit |
| uint32 | 4 | 0 to 4.3×10⁹ | Unsigned 32-bit |
| float | 4 | IEEE 754 | Single precision |

### Endianness

- All multi-byte integers are **little-endian** unless specified
- RC channel values are in microseconds (1000-2000)
- Angles in radians
- Altitude in meters (or cm for serial)

---

## 9. Constants

### RC Values
```cpp
RC_MIN = 1000        // Minimum PWM (μs)
RC_MAX = 2000        // Maximum PWM (μs)
RC_CENTER = 1500     // Neutral PWM (μs)
RC_ARM_HIGH = 1500   // Aux > this to arm
RC_ARM_LOW = 1200    // Aux < this to disarm
```

### Timeouts
```cpp
RC_SIGNAL_TIMEOUT_MS = 500       // RC signal loss
STUCK_CONTROL_TIMEOUT_MS = 3000   // Stuck controls
STUCK_CONTROL_THRESHOLD = 20     // Movement threshold
COMPANION_HEARTBEAT_MS = 500     // PING interval
COMPANION_TIMEOUT_MS = 2000       // No PONG timeout
```

### Loop Rates
```cpp
FAST_LOOP_HZ = 400      // Main control loop
FAST_LOOP_US = 2500    // Loop period (μs)
RC_UPDATE_HZ = 50      // RC input rate
TELEMETRY_HZ = 100     // Telemetry rate
```

---

## 10. Error Handling

### Error Flags

| Flag | Bit | Description |
|------|-----|-------------|
| VOLTAGE_LOW | 0 | Battery voltage critical |
| TEMP_HIGH | 1 | Module temperature high |
| MEMORY_LOW | 2 | Free memory < 10KB |
| IMU_ERROR | 3 | IMU communication failed |
| RADIO_ERROR | 4 | Radio communication error |

### Recovery Procedures

1. **RC Loss**: Hover → wait 30s → land
2. **Battery Low**: Warning beep → auto-land
3. **IMU Error**: Immediate disarm → prevent arm
4. **Radio Error**: Try reconnection → failsafe

---

## 11. Revision History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2026-04-14 | Initial specification |
