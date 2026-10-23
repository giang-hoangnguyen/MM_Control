# TUE USB Binary Communication Protocol & Payload Specification

## 1. Overview

The **TUE USB Protocol** is a lightweight, reliable, high-performance binary request/response protocol designed for embedded motion-control systems. It communicates over USB CDC (Virtual COM Port) between a host system (Python client, ROS 2 node, GUI) and the STM32F407VET6 motion controller (`MM_Control`).

Key design highlights:
- **Packet Delimitation**: Explicit 2-byte preamble (`0xAA 0x55`) for synchronization and stream recovery.
- **Data Integrity**: CRC-16/CCITT-FALSE across sequence, command, status, length, and payload.
- **Request/Response Matching**: Monotonically incrementing 8-bit sequence number echoed in responses.
- **Defensive Parsing**: Byte-oriented state machine impervious to arbitrary USB packet fragmentation and concatenation.
- **Endianness**: Explicit Little-Endian serialization for all multi-byte fields (no compiler struct padding hazards).

---

## 2. Frame Structure

Every frame transmitted over USB CDC conforms to the following binary structure:

```
+--------+--------+--------+--------+--------+--------+--------+-----------------+--------+--------+
| SOF1   | SOF2   | SEQ    | CMD    | STATUS | LEN_L  | LEN_H  | PAYLOAD (0..N)  | CRC_L  | CRC_H  |
| 0xAA   | 0x55   | uint8  | uint8  | uint8  | uint8  | uint8  | N bytes         | uint8  | uint8  |
+--------+--------+--------+--------+--------+--------+--------+-----------------+--------+--------+
|<-- 2B Preamble->|<--------------------- 5B Header ------------>|<--- 0..256B --->|<-- 2B CRC-16 ->|
```

### 2.1 Header Field Definitions

| Offset | Field | Type | Description |
|:---|:---|:---|:---|
| 0 | `SOF1` | `uint8_t` | Start of Frame byte 1: Fixed magic constant `0xAA` |
| 1 | `SOF2` | `uint8_t` | Start of Frame byte 2: Fixed magic constant `0x55` |
| 2 | `SEQ` | `uint8_t` | Sequence counter (`0x00`–`0xFF`). Set by the host; echoed by MCU in response |
| 3 | `CMD` | `uint8_t` | Command identifier (`0x01`–`0xFF`) |
| 4 | `STATUS`| `uint8_t` | In Request: Fixed `0x00`. In Response: Execution status code (see §2.2) |
| 5 | `LEN_L` | `uint8_t` | Payload length LSB (`uint16_t` little-endian, `0` to `256`) |
| 6 | `LEN_H` | `uint8_t` | Payload length MSB (`uint16_t` little-endian) |
| 7 | `PAYLOAD`| `uint8_t[N]` | Command-specific payload (length = `LEN`) |
| 7+N | `CRC_L` | `uint8_t` | CRC-16 low byte (little-endian) |
| 8+N | `CRC_H` | `uint8_t` | CRC-16 high byte (little-endian) |

- **Minimum Frame Size**: 9 bytes (`LEN = 0`).
- **Maximum Frame Size**: 265 bytes (`LEN = 256`).
- **Maximum Payload Size**: 256 bytes (`TUE_USB_MAX_PAYLOAD`).

### 2.2 Status Codes

| Code | Symbol | Meaning |
|:---|:---|:---|
| `0x00` | `STATUS_OK` | Request processed successfully |
| `0x01` | `STATUS_ERR_UNKNOWN_CMD` | Command ID is not supported by firmware |
| `0x02` | `STATUS_ERR_INVALID_LEN` | Payload length does not match command expectation |
| `0x03` | `STATUS_ERR_INVALID_PARAM`| Parameter within payload is out of bounds or invalid |
| `0x04` | `STATUS_ERR_CRC` | Frame CRC check failed |
| `0x05` | `STATUS_ERR_BUSY` | MCU subsystem is busy (e.g. transfer ongoing) |
| `0x06` | `STATUS_ERR_EXEC` | Execution failure (hardware fault, timeout, etc.) |

### 2.3 CRC-16 Specification

The integrity of every frame is protected by **CRC-16/CCITT-FALSE**:
- **Polynomial**: `0x1021` ($x^{16} + x^{12} + x^5 + 1$)
- **Initial Value**: `0xFFFF`
- **Input Reflection**: False (MSB-first)
- **Output Reflection**: False (MSB-first)
- **XOR Out**: `0x0000`
- **Check Value**: For ASCII string `"123456789"` (`0x31`..`0x39`), CRC is `0x29B1`.

**CRC Calculation Range**:
The CRC is computed over all bytes following the preamble up to the end of the payload:
$$\text{CRC} = \text{crc16}(\text{SEQ}, \text{CMD}, \text{STATUS}, \text{LEN\_L}, \text{LEN\_H}, \text{PAYLOAD}[0 \dots N-1])$$

---

## 3. Command Groups & Payload Definitions

Commands are allocated into domain-specific functional groups:

| Group Range | Subsystem | Description |
|:---|:---|:---|
| `0x01` – `0x0F` | **System** | General board diagnostics, ping, info, reset, onboard LED |
| `0x10` – `0x1F` | **DC Motors** | 4-channel H-bridge PWM, direction, braking, fault monitoring |
| `0x20` – `0x2F` | **Stepper Motors** | 4-channel step/dir pulses, enable, velocity, positioning |
| `0x30` – `0x3F` | **Servo (STS3215)** | Half-duplex UART bus servo control & memory table access |
| `0x40` – `0x4F` | **Sensors** | IMU, optical/magnetic encoders, analog sensors, battery |
| `0x50` – `0x5F` | **Bus Passthrough**| CAN bus, I2C, SPI, UART raw transfers |
| `0x60` – `0x6F` | **Config & Modes** | Control modes, state machine transitions, flash memory |

---

### 3.1 System Commands (`0x01`–`0x0F`)

| ID | Name | Direction | Req Len | Resp Len | Status | Description |
|:---|:---|:---|:---|:---|:---|:---|
| `0x01` | `CMD_SYS_PING` | Host $\leftrightarrow$ MCU | 0..256 | 0..256 | **Implemented** | Echo test for latency and link health |
| `0x02` | `CMD_SYS_GET_INFO` | Host $\rightarrow$ MCU | 0 | 19 | **Implemented** | Reads board ID, firmware version, and 96-bit MCU UID |
| `0x03` | `CMD_SYS_RESET` | Host $\rightarrow$ MCU | 4 | 0 | Defined but not implemented | Software reboot via `NVIC_SystemReset` |
| `0x04` | `CMD_SYS_LED_CTRL` | Host $\leftrightarrow$ MCU | 2..4 | 2 | **Implemented** | Control or query state of onboard status LED |
| `0x05` | `CMD_SYS_GET_STATUS`| Host $\rightarrow$ MCU | 0 | 16 | Defined but not implemented | Query uptime, FreeRTOS free heap, CPU load |
| `0x06`–`0x0F` | -- | -- | -- | -- | **Reserved** | Reserved for future system extensions |

#### `0x01` `CMD_SYS_PING`
- **Request Payload**: Arbitrary byte payload of length $N$ ($0 \le N \le 256$).
- **Response Payload**: The identical $N$ bytes echoed back.

#### `0x02` `CMD_SYS_GET_INFO`
- **Request Payload**: None (`LEN = 0`).
- **Response Payload**: 19 bytes:
  | Offset | Field | Type | Size | Unit | Description |
  |:---|:---|:---|:---|:---|:---|
  | 0 | `board_id` | `char[4]` | 4 | ASCII | Board signature (`"MM01"`) |
  | 4 | `fw_major` | `uint8_t` | 1 | none | Firmware major version |
  | 5 | `fw_minor` | `uint8_t` | 1 | none | Firmware minor version |
  | 6 | `fw_patch` | `uint8_t` | 1 | none | Firmware patch version |
  | 7 | `mcu_uid` | `uint8_t[12]` | 12 | hex | STM32 96-bit Unique Device ID (`UID_BASE = 0x1FFF7A10`) |

#### `0x04` `CMD_SYS_LED_CTRL`
- **Hardware Connection**: `LED_MCU` is on **GPIO Port E, Pin 5 (`PE5`)**. Driving PE5 HIGH (`SET`) turns the LED ON; driving PE5 LOW (`RESET`) turns the LED OFF.
- **Request Payload**: 2 or 4 bytes:
  | Offset | Field | Type | Size | Unit / Values | Description |
  |:---|:---|:---|:---|:---|:---|
  | 0 | `led_index` | `uint8_t` | 1 | `0` | LED identifier (`0 = LED_MCU` onboard LED) |
  | 1 | `mode` | `uint8_t` | 1 | Enum | `0x00`: Turn OFF<br>`0x01`: Turn ON<br>`0x02`: Toggle<br>`0xFF`: Query state only |
  | 2 | `param` | `uint16_t` | 2 | ms (optional) | Reserved parameter (e.g. blink duration) |
- **Response Payload**: 2 bytes:
  | Offset | Field | Type | Size | Unit / Values | Description |
  |:---|:---|:---|:---|:---|:---|
  | 0 | `led_index` | `uint8_t` | 1 | `0` | LED identifier |
  | 1 | `current_state` | `uint8_t` | 1 | `0` or `1` | Current pin state (`0 = OFF`, `1 = ON`) |

---

### 3.2 DC Motor Commands (`0x10`–`0x1F`)

Hardware mapping: 4 DC motor channels driven by TIM3/TIM4 PWM with feedback fault inputs (`DC_F1`..`DC_F4`).

| ID | Name | Direction | Req Len | Resp Len | Status | Description |
|:---|:---|:---|:---|:---|:---|:---|
| `0x10` | `CMD_DC_SET_DUTY` | Host $\rightarrow$ MCU | 4 | 2 | **Implemented** | Set motor direction and PWM duty cycle |
| `0x11` | `CMD_DC_STOP` | Host $\rightarrow$ MCU | 2 | 2 | **Implemented** | Stop/brake specified DC motors |
| `0x12` | `CMD_DC_GET_STATUS` | Host $\rightarrow$ MCU | 1 | 8 | **Implemented** | Read fault status, active duty, current |
| `0x13` | `CMD_DC_SET_PID` | Host $\rightarrow$ MCU | 14 | 2 | **Implemented** | Configure velocity PID parameters |
| `0x14`–`0x1F` | -- | -- | -- | -- | **Reserved** | Reserved for DC motor extensions |

#### `0x10` `CMD_DC_SET_DUTY`
- **Request Payload**: 4 bytes:
  | Offset | Field | Type | Size | Unit / Range | Description |
  |:---|:---|:---|:---|:---|:---|
  | 0 | `motor_id` | `uint8_t` | 1 | `1`–`4` | Motor channel (1 = M1, ..., 4 = M4) |
  | 1 | `direction` | `uint8_t` | 1 | Enum | `0`: Coast / Float<br>`1`: Clockwise (CW)<br>`2`: Counter-clockwise (CCW)<br>`3`: Active Brake |
  | 2 | `duty` | `uint16_t` | 2 | `0`–`10000` | PWM duty cycle in units of 0.01% (0 = 0.0%, 10000 = 100.0%) |
- **Response Payload**: 2 bytes: `[motor_id, ack_code]`.

---

### 3.3 Stepper Motor Commands (`0x20`–`0x2F`)

Hardware mapping: 4 Stepper channels (`STEP_1`..`STEP_4`, `STEP_DIR1`..`STEP_DIR4`, common `STEP_ENABLE`).

| ID | Name | Direction | Req Len | Resp Len | Status | Description |
|:---|:---|:---|:---|:---|:---|:---|
| `0x20` | `CMD_STEP_ENABLE` | Host $\rightarrow$ MCU | 2 | 2 | **Implemented** | Enable or disable stepper drivers |
| `0x21` | `CMD_STEP_MOVE` | Host $\rightarrow$ MCU | 14 | 2 | **Implemented** | Command target position/steps with velocity profile |
| `0x22` | `CMD_STEP_STOP` | Host $\rightarrow$ MCU | 2 | 2 | **Implemented** | Immediately stop or decel stepper motion |
| `0x23` | `CMD_STEP_GET_POS` | Host $\rightarrow$ MCU | 1 | 9 | **Implemented** | Read current position and moving flag |
| `0x24` | `CMD_STEP_SET_HOMING`| Host $\rightarrow$ MCU | 2 | 2 | **Implemented** | Initiate sensor homing sequence |
| `0x25`–`0x2F` | -- | -- | -- | -- | **Reserved** | Reserved for stepper extensions |

#### `0x21` `CMD_STEP_MOVE`
- **Request Payload**: 14 bytes:
  | Offset | Field | Type | Size | Unit / Range | Description |
  |:---|:---|:---|:---|:---|:---|
  | 0 | `stepper_id` | `uint8_t` | 1 | `1`–`4` | Stepper channel |
  | 1 | `is_relative` | `uint8_t` | 1 | `0` or `1` | `0`: Absolute position, `1`: Relative steps |
  | 2 | `target_steps` | `int32_t` | 4 | microsteps | Target step count (signed) |
  | 6 | `speed_hz` | `uint32_t` | 4 | steps/sec | Maximum step pulse frequency |
  | 10 | `accel` | `uint32_t` | 4 | steps/sec² | Acceleration / deceleration rate |

---

### 3.4 STS3215 Servo Commands (`0x30`–`0x3F`)

Hardware mapping: Half-duplex serial servo bus on PA9 (`SERVO_TX`, USART1). Parameters map directly to `firmware/MEMORY_TABLE.md`.

| ID | Name | Direction | Req Len | Resp Len | Status | Description |
|:---|:---|:---|:---|:---|:---|:---|
| `0x30` | `CMD_SERVO_PING` | Host $\rightarrow$ MCU | 1 | 2 | **Implemented** | Ping servo by ID (`0`–`253`) |
| `0x31` | `CMD_SERVO_READ_REG` | Host $\rightarrow$ MCU | 3 | 3..N | **Implemented** | Read memory table register(s) |
| `0x32` | `CMD_SERVO_WRITE_REG`| Host $\rightarrow$ MCU | 3..N | 2 | **Implemented** | Write memory table register(s) |
| `0x33` | `CMD_SERVO_SET_POS` | Host $\rightarrow$ MCU | 7 | 2 | **Implemented** | Set target location (`0x2A`), speed (`0x2E`), accel (`0x29`) |
| `0x34` | `CMD_SERVO_GET_FEEDBACK`| Host $\rightarrow$ MCU | 1 | 13 | **Implemented** | Read position, speed, load, voltage, temp, status |
| `0x35` | `CMD_SERVO_TORQUE_ENABLE`| Host $\rightarrow$ MCU | 2 | 2 | **Implemented** | Toggle torque switch (`0x28`) |
| `0x36`–`0x3F` | -- | -- | -- | -- | **Reserved** | Reserved for servo extensions |

#### `0x34` `CMD_SERVO_GET_FEEDBACK`
- **Request Payload**: 1 byte (`[servo_id]`).
- **Response Payload**: 13 bytes:
  | Offset | Field | Memory Addr | Type | Size | Unit | Description |
  |:---|:---|:---|:---|:---|:---|:---|
  | 0 | `servo_id` | -- | `uint8_t` | 1 | ID | Servo identifier |
  | 1 | `position` | `0x38` | `int16_t` | 2 | steps | Current angular step position |
  | 3 | `speed` | `0x3A` | `int16_t` | 2 | steps/s | Current angular velocity |
  | 5 | `load` | `0x3C` | `int16_t` | 2 | 0.1% | Output load / drive duty |
  | 7 | `voltage` | `0x3E` | `uint8_t` | 1 | 0.1 V | Operating supply voltage |
  | 8 | `temperature` | `0x3F` | `uint8_t` | 1 | °C | Internal temperature |
  | 9 | `status` | `0x41` | `uint8_t` | 1 | bitmask | Servo error flags |
  | 10 | `current` | `0x45` | `int16_t` | 2 | 6.5 mA | Motor drive current |
  | 12 | `is_moving` | `0x42` | `uint8_t` | 1 | bool | Moving status |

---

### 3.5 Sensor Commands (`0x40`–`0x4F`)

Hardware mapping: SPI1 / SPI3 buses, I2C1, ADC channels.

| ID | Name | Direction | Req Len | Resp Len | Status | Description |
|:---|:---|:---|:---|:---|:---|:---|
| `0x40` | `CMD_SENSOR_READ_IMU` | Host $\rightarrow$ MCU | 0 | 14 | Defined but not implemented | 6-axis acceleration and angular rate |
| `0x41` | `CMD_SENSOR_READ_ENCODER`| Host $\rightarrow$ MCU | 1 | 9 | Defined but not implemented | Read optical/magnetic quadrature encoder count |
| `0x42` | `CMD_SENSOR_READ_ADC` | Host $\rightarrow$ MCU | 1 | 4 | Defined but not implemented | Read analog channel raw value & calibrated mV |
| `0x43` | `CMD_SENSOR_READ_GPIO` | Host $\rightarrow$ MCU | 0 | 4 | Defined but not implemented | Read status of digital endstops / inputs |
| `0x44`–`0x4F` | -- | -- | -- | -- | **Reserved** | Reserved for sensor extensions |

---

### 3.6 Communication Bus Passthrough (`0x50`–`0x5F`)

Hardware mapping: CAN1 (875 kbit/s), I2C1, SPI1 (CS1..CS3), SPI3 (CS1, CS2, CS1D4).

| ID | Name | Direction | Req Len | Resp Len | Status | Description |
|:---|:---|:---|:---|:---|:---|:---|
| `0x50` | `CMD_BUS_CAN_TX` | Host $\rightarrow$ MCU | 12..20 | 2 | Defined but not implemented | Transmit CAN 2.0B standard/extended frame |
| `0x51` | `CMD_BUS_CAN_RX` | Host $\rightarrow$ MCU | 0 | 12..20 | Defined but not implemented | Poll or pop received CAN message from queue |
| `0x52` | `CMD_BUS_I2C_TRANSFER` | Host $\rightarrow$ MCU | 4..N | 0..M | Defined but not implemented | I2C master write followed by optional read |
| `0x53` | `CMD_BUS_SPI_TRANSFER` | Host $\rightarrow$ MCU | 3..N | 0..N | Defined but not implemented | SPI full-duplex transfer with selected CS |
| `0x54`–`0x5F` | -- | -- | -- | -- | **Reserved** | Reserved for communication bus extensions |

---

### 3.7 Configuration & Control Modes (`0x60`–`0x6F`)

| ID | Name | Direction | Req Len | Resp Len | Status | Description |
|:---|:---|:---|:---|:---|:---|:---|
| `0x60` | `CMD_CFG_SET_MODE` | Host $\rightarrow$ MCU | 1 | 2 | Defined but not implemented | Set operational mode (Idle, Manual, Auto, E-Stop)|
| `0x61` | `CMD_CFG_GET_MODE` | Host $\rightarrow$ MCU | 0 | 4 | Defined but not implemented | Query current mode and active fault flags |
| `0x62` | `CMD_CFG_SAVE_FLASH` | Host $\rightarrow$ MCU | 4 | 2 | Defined but not implemented | Save current calibration/parameters to internal Flash|
| `0x63` | `CMD_CFG_LOAD_DEFAULT` | Host $\rightarrow$ MCU | 4 | 2 | Defined but not implemented | Restore factory configuration defaults |
| `0x64`–`0x6F` | -- | -- | -- | -- | **Reserved** | Reserved for configuration extensions |

---

## 4. Engineering Rules & Modularity

1. **Protocol Synchrony**: Whenever a command or payload changes, update this document, `firmware/Core/Inc/tue_usb.h`, and `firmware/python/tue_usb/defs.py` concurrently.
2. **Never Reuse IDs**: Command IDs must remain strictly unique and permanent. Deprecated commands are retired, never reassigned.
3. **Explicit Serialization**: Transmissions must serialize fields explicitly (little-endian bytes) rather than passing unpadded C `struct` representations.
4. **Non-blocking Callbacks**: USB CDC receive callbacks (`CDC_Receive_FS`) must never block or parse frames directly in ISR context; data is handed off to a FreeRTOS stream buffer or queue for worker task execution.
