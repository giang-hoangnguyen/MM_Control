# TUE USB Python Client Library

Reusable Python client library and tools for the **STM32F407VET6** motion-control board (`MM_Control`) over USB CDC.

## Features

- **Decoupled Architecture**: Framing and serialization (`tue_usb.protocol`) are independent of serial transport, allowing reuse in ROS 2 nodes, simulation harnesses, or UI dashboards.
- **Robust Framing**: Preamble sync markers (`0xAA 0x55`), CRC-16/CCITT-FALSE verification, sequence counter matching, and timeout handling.
- **Stream Resilient**: Reassembles fragmented USB packets and unpacks concatenated frames seamlessly.
- **Thread-safe**: Protected client interface with auto-incrementing sequence numbers.
- **Complete Test Suite**: Unit tests verifying CRC calculations, framing, and fragmentation recovery.

---

## Installation

### Prerequisites
- Python 3.8 or higher.
- `pip` package manager.

### Install Dependencies
From the `firmware/python/` directory:

```bash
pip install -r requirements.txt
```

Or install the package in editable development mode:

```bash
pip install -e .
```

---

## Quick Start: Python API

### 1. System & LED Diagnostics
```python
from tue_usb import TueClient, LedMode

with TueClient(port="COM3") as client:
    # 1. Ping the controller
    echo = client.ping(b"HELLO_STM32")
    print(f"Ping response: {echo}")

    # 2. Query board info and STM32 Unique ID
    info = client.get_info()
    print(f"Board: {info['board_id']}, FW: {info['version_str']}, UID: {info['mcu_uid']}")

    # 3. Control the onboard LED (PE5)
    client.set_led(led_index=0, state=True)   # Turn ON
    client.toggle_led(led_index=0)            # Toggle
```

### 2. DC Motor Control
```python
from tue_usb import TueClient, DcDirection, DcBrakeMode

with TueClient(port="COM3") as client:
    # Set Motor 1 to Clockwise at 60.00% duty cycle (6000 / 10000)
    client.dc_set_duty(motor_id=1, direction=DcDirection.CW, duty=6000)

    # Read Motor 1 status (direction, duty, fault, current)
    status = client.dc_get_status(motor_id=1)
    print(f"Motor 1: Dir={status['direction'].name}, Duty={status['duty']/100:.1f}%, Fault={status['fault']}")

    # Stop all DC motors with active dynamic braking
    client.dc_stop(motor_id=0, brake_mode=DcBrakeMode.ACTIVE)
```

### 3. Stepper Motor Control
```python
from tue_usb import TueClient

with TueClient(port="COM3") as client:
    # Enable all stepper drivers (common STEP_ENABLE on PC5)
    client.step_enable(stepper_id=0, enable=True)

    # Move Stepper 1 by 3200 microsteps at 2000 Hz, accel 1000 steps/s²
    client.step_move(stepper_id=1, target_steps=3200, speed_hz=2000, accel=1000, is_relative=True)

    # Query current position and moving flag
    pos = client.step_get_pos(stepper_id=1)
    print(f"Stepper 1: Current Pos={pos['current_steps']}, Moving={pos['is_moving']}")

    # Soft stop
    client.step_stop(stepper_id=1, hard_stop=False)
```

### 4. STS3215 Bus Servo Control (USART1 Half-Duplex DMA)
```python
from tue_usb import TueClient, Sts3215Reg

with TueClient(port="COM3") as client:
    # Ping servo ID 1
    sid, ack = client.servo_ping(servo_id=1)
    print(f"Servo {sid} ping: {'OK' if ack == 0 else 'FAIL'}")

    # Move servo 1 to 2048 (middle position) with speed limit 1500 steps/s
    client.servo_set_pos(servo_id=1, position=2048, speed=1500, accel=50)

    # Read complete telemetry feedback
    fb = client.servo_get_feedback(servo_id=1)
    print(f"Servo 1: Pos={fb['position']}, Spd={fb['speed']} steps/s, Load={fb['load_percent']}%, "
          f"Volt={fb['voltage_v']}V, Temp={fb['temperature_degc']}°C, Curr={fb['current_ma']}mA")

    # Read arbitrary register (e.g. Baud Rate at 0x06)
    _, _, baud_bytes = client.servo_read_reg(servo_id=1, reg_addr=Sts3215Reg.BAUDRATE, read_len=1)
    print(f"Baud register: 0x{baud_bytes.hex()}")

    # Toggle torque (1 = ON, 0 = OFF, 128 = calibrate position to 2048)
    client.servo_torque_enable(servo_id=1, enable=True)
```

---

## API Reference & CLI Options

| Domain | Client Method (`TueClient`) | Command ID | CLI Example (`motor_control.py`) |
|:---|:---|:---|:---|
| **System** | `ping(data)` | `0x01` (`CMD_SYS_PING`) | `python examples/motor_control.py --port COM3 --info` |
| **System** | `get_info()` | `0x02` (`CMD_SYS_GET_INFO`) | `python examples/motor_control.py --port COM3 --info` |
| **System** | `set_led(idx, state)` | `0x04` (`CMD_SYS_LED_CTRL`) | `python examples/led_control.py --port COM3 --on` |
| **DC Motor** | `dc_set_duty(id, dir, duty)` | `0x10` (`CMD_DC_SET_DUTY`) | `python examples/motor_control.py --dc 1 --dc-duty 5000 --dc-dir cw` |
| **DC Motor** | `dc_stop(id, brake_mode)` | `0x11` (`CMD_DC_STOP`) | `python examples/motor_control.py --dc-stop` |
| **DC Motor** | `dc_get_status(id)` | `0x12` (`CMD_DC_GET_STATUS`) | `python examples/motor_control.py --dc 1 --status` |
| **DC Motor** | `dc_set_pid(id, kp, ki, kd)` | `0x13` (`CMD_DC_SET_PID`) | *(Python API)* |
| **Stepper** | `step_enable(id, enable)` | `0x20` (`CMD_STEP_ENABLE`) | `python examples/motor_control.py --step-enable` |
| **Stepper** | `step_move(id, steps, speed, acc)` | `0x21` (`CMD_STEP_MOVE`) | `python examples/motor_control.py --step 1 --step-move 2000 --speed 1000` |
| **Stepper** | `step_stop(id, hard_stop)` | `0x22` (`CMD_STEP_STOP`) | `python examples/motor_control.py --step 1 --step-stop` |
| **Stepper** | `step_get_pos(id)` | `0x23` (`CMD_STEP_GET_POS`) | `python examples/motor_control.py --step 1 --step-pos` |
| **Stepper** | `step_set_homing(id, dir)` | `0x24` (`CMD_STEP_SET_HOMING`)| *(Python API)* |
| **Servo** | `servo_ping(id)` | `0x30` (`CMD_SERVO_PING`) | `python examples/motor_control.py --servo 1 --servo-ping` |
| **Servo** | `servo_read_reg(id, reg, len)` | `0x31` (`CMD_SERVO_READ_REG`) | *(Python API)* |
| **Servo** | `servo_write_reg(id, reg, data)` | `0x32` (`CMD_SERVO_WRITE_REG`)| *(Python API)* |
| **Servo** | `servo_set_pos(id, pos, spd, acc)` | `0x33` (`CMD_SERVO_SET_POS`) | `python examples/motor_control.py --servo 1 --servo-pos 2048` |
| **Servo** | `servo_get_feedback(id)` | `0x34` (`CMD_SERVO_GET_FEEDBACK`)| `python examples/motor_control.py --servo 1 --servo-fb` |
| **Servo** | `servo_torque_enable(id, enable)` | `0x35` (`CMD_SERVO_TORQUE_ENABLE`)| `python examples/motor_control.py --servo 1 --servo-torque 1` |
| **Interactive** | — | — | `python examples/motor_control.py --interactive` |

---

## Running Unit Tests

Run the test suite from `firmware/python/`:

```bash
python -m unittest discover -s tests -v
```

Or using `pytest`:

```bash
pytest tests -v
```

