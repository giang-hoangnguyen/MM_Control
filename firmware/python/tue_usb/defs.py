"""
TUE USB Binary Protocol Constants and Definitions.
"""

from enum import IntEnum

# Frame delimiter and sizing
SOF1: int = 0xAA
SOF2: int = 0x55
HEADER_SIZE: int = 7  # SOF1(1) + SOF2(1) + SEQ(1) + CMD(1) + STATUS(1) + LEN(2)
CRC_SIZE: int = 2     # CRC16(2)
MIN_FRAME_SIZE: int = HEADER_SIZE + CRC_SIZE  # 9 bytes
MAX_PAYLOAD_SIZE: int = 256
MAX_FRAME_SIZE: int = HEADER_SIZE + MAX_PAYLOAD_SIZE + CRC_SIZE  # 265 bytes


class StatusCode(IntEnum):
    """Protocol response status codes."""
    OK = 0x00
    ERR_UNKNOWN_CMD = 0x01
    ERR_INVALID_LEN = 0x02
    ERR_INVALID_PARAM = 0x03
    ERR_CRC = 0x04
    ERR_BUSY = 0x05
    ERR_EXEC = 0x06

    @classmethod
    def describe(cls, code: int) -> str:
        descriptions = {
            cls.OK: "Success / OK",
            cls.ERR_UNKNOWN_CMD: "Unknown or unimplemented Command ID",
            cls.ERR_INVALID_LEN: "Invalid payload length",
            cls.ERR_INVALID_PARAM: "Invalid parameter in payload",
            cls.ERR_CRC: "CRC-16 checksum failure",
            cls.ERR_BUSY: "Device or peripheral is busy",
            cls.ERR_EXEC: "Command execution error",
        }
        return descriptions.get(code, f"Unknown error code 0x{code:02X}")


class CommandId(IntEnum):
    """Protocol command identifiers."""
    # 0x01–0x0F: System Commands
    SYS_PING = 0x01
    SYS_GET_INFO = 0x02
    SYS_RESET = 0x03
    SYS_LED_CTRL = 0x04
    SYS_GET_STATUS = 0x05

    # 0x10–0x1F: DC Motor Commands
    DC_SET_DUTY = 0x10
    DC_STOP = 0x11
    DC_GET_STATUS = 0x12
    DC_SET_PID = 0x13

    # 0x20–0x2F: Stepper Motor Commands
    STEP_ENABLE = 0x20
    STEP_MOVE = 0x21
    STEP_STOP = 0x22
    STEP_GET_POS = 0x23
    STEP_SET_HOMING = 0x24

    # 0x30–0x3F: Servo Commands (STS3215)
    SERVO_PING = 0x30
    SERVO_READ_REG = 0x31
    SERVO_WRITE_REG = 0x32
    SERVO_SET_POS = 0x33
    SERVO_GET_FEEDBACK = 0x34
    SERVO_TORQUE_ENABLE = 0x35

    # 0x40–0x4F: Sensor Commands
    SENSOR_READ_IMU = 0x40
    SENSOR_READ_ENCODER = 0x41
    SENSOR_READ_ADC = 0x42
    SENSOR_READ_GPIO = 0x43

    # 0x50–0x5F: Bus Passthrough Commands
    BUS_CAN_TX = 0x50
    BUS_CAN_RX = 0x51
    BUS_I2C_TRANSFER = 0x52
    BUS_SPI_TRANSFER = 0x53

    # 0x60–0x6F: Configuration and Control Modes
    CFG_SET_MODE = 0x60
    CFG_GET_MODE = 0x61
    CFG_SAVE_FLASH = 0x62
    CFG_LOAD_DEFAULT = 0x63


class LedMode(IntEnum):
    """LED control operating modes for CMD_SYS_LED_CTRL."""
    OFF = 0x00
    ON = 0x01
    TOGGLE = 0x02
    QUERY = 0xFF


class DcDirection(IntEnum):
    """DC motor direction modes."""
    FLOAT = 0  # Coast / High-Z
    CW = 1     # Clockwise
    CCW = 2    # Counter-Clockwise
    BRAKE = 3  # Active Dynamic Brake


class DcBrakeMode(IntEnum):
    """DC motor stop brake modes."""
    COAST = 0   # Coast / Float
    ACTIVE = 1  # Active Dynamic Brake


class StepperMode(IntEnum):
    """Stepper motion mode."""
    ABSOLUTE = 0
    RELATIVE = 1


class StepperStopMode(IntEnum):
    """Stepper stop mode."""
    DECEL = 0       # Decelerate / soft stop
    IMMEDIATE = 1   # Emergency / hard stop


class Sts3215Reg(IntEnum):
    """STS3215 Memory Table register addresses."""
    # EEPROM (Read-only or Read/Write)
    FIRMWARE_MAJOR = 0x00
    FIRMWARE_MINOR = 0x01
    SERVO_MAJOR_VER = 0x03
    SERVO_VER = 0x04
    ID = 0x05
    BAUDRATE = 0x06
    RETURN_DELAY = 0x07
    RESPONSE_LEVEL = 0x08
    MIN_ANGLE_LIMIT = 0x09
    MAX_ANGLE_LIMIT = 0x0B
    MAX_TEMP_LIMIT = 0x0D
    MAX_VOLT_LIMIT = 0x0E
    MIN_VOLT_LIMIT = 0x0F
    MAX_TORQUE = 0x10
    PROTECTIVE_CURRENT = 0x1C
    OPERATING_MODE = 0x21

    # SRAM (Control & Feedback)
    TORQUE_SWITCH = 0x28
    ACCELERATION = 0x29
    GOAL_POSITION = 0x2A
    RUNNING_TIME = 0x2C
    RUNNING_SPEED = 0x2E
    TORQUE_LIMIT = 0x30
    LOCK_SYMBOL = 0x37
    CURRENT_POSITION = 0x38
    CURRENT_SPEED = 0x3A
    CURRENT_LOAD = 0x3C
    CURRENT_VOLTAGE = 0x3E
    CURRENT_TEMPERATURE = 0x3F
    ASYNC_WRITE_FLAG = 0x40
    STATUS = 0x41
    IS_MOVING = 0x42
    CURRENT_MA = 0x45
