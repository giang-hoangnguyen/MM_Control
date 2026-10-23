"""
TUE USB Protocol Framing, Serialization, and Streaming Parser.

This module is completely independent of serial communication and can be reused
by ROS 2 nodes, GUIs, or simulation harnesses.
"""

from dataclasses import dataclass
import struct
from typing import Any, Dict, List, Optional, Tuple, Union

from .crc import crc16_ccitt
from .defs import (
    CRC_SIZE,
    HEADER_SIZE,
    MAX_PAYLOAD_SIZE,
    MIN_FRAME_SIZE,
    SOF1,
    SOF2,
    DcBrakeMode,
    DcDirection,
    LedMode,
    StatusCode,
    StepperMode,
    StepperStopMode,
)
from .exceptions import TueCrcError, TueError


@dataclass
class TueFrame:
    """Represents a decoded TUE USB binary frame."""
    seq: int
    cmd: int
    status: int
    payload: bytes
    crc: int

    @property
    def is_ok(self) -> bool:
        """True if the frame status is StatusCode.OK."""
        return self.status == StatusCode.OK


def encode_frame(
    seq: int,
    cmd: int,
    status: int = StatusCode.OK,
    payload: bytes = b"",
) -> bytes:
    """Encode a protocol frame into binary wire format.

    Args:
        seq: Sequence number (0..255).
        cmd: Command ID (0..255).
        status: Status code (0 for requests, or status enum in responses).
        payload: Command payload bytes (max 256 bytes).

    Returns:
        Packed bytes including preamble, header, payload, and CRC-16.

    Raises:
        ValueError: If payload length exceeds MAX_PAYLOAD_SIZE.
    """
    payload_len = len(payload)
    if payload_len > MAX_PAYLOAD_SIZE:
        raise ValueError(
            f"Payload size {payload_len} exceeds maximum allowed {MAX_PAYLOAD_SIZE} bytes"
        )

    # Frame header: [SOF1, SOF2, SEQ, CMD, STATUS, LEN_L, LEN_H]
    header = struct.pack(
        "<BBBBBH",
        SOF1,
        SOF2,
        seq & 0xFF,
        cmd & 0xFF,
        status & 0xFF,
        payload_len & 0xFFFF,
    )

    # Calculate CRC-16 over SEQ through end of payload (everything except SOF1, SOF2)
    crc_data = header[2:] + payload
    crc = crc16_ccitt(crc_data)

    # Append little-endian CRC-16
    return header + payload + struct.pack("<H", crc)


def decode_frame(raw_bytes: bytes) -> TueFrame:
    """Decode a single complete frame from raw bytes.

    Args:
        raw_bytes: Buffer containing a complete frame.

    Returns:
        Decoded TueFrame object.

    Raises:
        TueError: If frame is truncated or SOF is invalid.
        TueCrcError: If CRC verification fails.
    """
    if len(raw_bytes) < MIN_FRAME_SIZE:
        raise TueError(f"Frame length {len(raw_bytes)} is shorter than minimum {MIN_FRAME_SIZE}")

    sof1, sof2, seq, cmd, status, payload_len = struct.unpack_from("<BBBBBH", raw_bytes, 0)
    if sof1 != SOF1 or sof2 != SOF2:
        raise TueError(f"Invalid frame start markers: 0x{sof1:02X} 0x{sof2:02X}")

    expected_total_len = HEADER_SIZE + payload_len + CRC_SIZE
    if len(raw_bytes) < expected_total_len:
        raise TueError(
            f"Truncated frame: expected {expected_total_len} bytes, got {len(raw_bytes)}"
        )

    payload = raw_bytes[HEADER_SIZE : HEADER_SIZE + payload_len]
    (received_crc,) = struct.unpack_from("<H", raw_bytes, HEADER_SIZE + payload_len)

    # Verify CRC
    crc_data = raw_bytes[2 : HEADER_SIZE + payload_len]
    computed_crc = crc16_ccitt(crc_data)
    if computed_crc != received_crc:
        raise TueCrcError(
            f"CRC mismatch: computed 0x{computed_crc:04X}, received 0x{received_crc:04X}"
        )

    return TueFrame(
        seq=seq,
        cmd=cmd,
        status=status,
        payload=payload,
        crc=received_crc,
    )


class TueFrameParser:
    """Streaming byte-oriented parser state machine.

    Correctly reassembles fragmented USB packets and unpacks concatenated frames.
    Recovers cleanly after noise or dropped bytes.
    """

    STATE_WAIT_SOF1 = 0
    STATE_WAIT_SOF2 = 1
    STATE_SEQ = 2
    STATE_CMD = 3
    STATE_STATUS = 4
    STATE_LEN_L = 5
    STATE_LEN_H = 6
    STATE_PAYLOAD = 7
    STATE_CRC_L = 8
    STATE_CRC_H = 9

    def __init__(self) -> None:
        self._state: int = self.STATE_WAIT_SOF1
        self._seq: int = 0
        self._cmd: int = 0
        self._status: int = 0
        self._len: int = 0
        self._payload: bytearray = bytearray()
        self._crc_l: int = 0
        self._crc: int = 0

    def reset(self) -> None:
        """Reset parser state machine to initial idle state."""
        self._state = self.STATE_WAIT_SOF1
        self._seq = 0
        self._cmd = 0
        self._status = 0
        self._len = 0
        self._payload = bytearray()
        self._crc_l = 0
        self._crc = 0

    def feed(self, data: bytes) -> List[TueFrame]:
        """Feed incoming byte chunks into the state machine.

        Args:
            data: Arbitrary chunk of incoming serial bytes.

        Returns:
            List of fully assembled and CRC-validated TueFrame instances.
        """
        frames: List[TueFrame] = []

        for b in data:
            if self._state == self.STATE_WAIT_SOF1:
                if b == SOF1:
                    self._state = self.STATE_WAIT_SOF2

            elif self._state == self.STATE_WAIT_SOF2:
                if b == SOF2:
                    self._state = self.STATE_SEQ
                elif b == SOF1:
                    # Consecutive 0xAA bytes
                    self._state = self.STATE_WAIT_SOF2
                else:
                    self._state = self.STATE_WAIT_SOF1

            elif self._state == self.STATE_SEQ:
                self._seq = b
                self._state = self.STATE_CMD

            elif self._state == self.STATE_CMD:
                self._cmd = b
                self._state = self.STATE_STATUS

            elif self._state == self.STATE_STATUS:
                self._status = b
                self._state = self.STATE_LEN_L

            elif self._state == self.STATE_LEN_L:
                self._len = b
                self._state = self.STATE_LEN_H

            elif self._state == self.STATE_LEN_H:
                self._len |= b << 8
                if self._len > MAX_PAYLOAD_SIZE:
                    # Invalid payload size, drop and resync
                    self._state = self.STATE_WAIT_SOF1
                elif self._len == 0:
                    self._payload = bytearray()
                    self._state = self.STATE_CRC_L
                else:
                    self._payload = bytearray()
                    self._state = self.STATE_PAYLOAD

            elif self._state == self.STATE_PAYLOAD:
                self._payload.append(b)
                if len(self._payload) >= self._len:
                    self._state = self.STATE_CRC_L

            elif self._state == self.STATE_CRC_L:
                self._crc_l = b
                self._state = self.STATE_CRC_H

            elif self._state == self.STATE_CRC_H:
                self._crc = self._crc_l | (b << 8)

                # Validate CRC
                header_data = struct.pack(
                    "<BBBH",
                    self._seq,
                    self._cmd,
                    self._status,
                    self._len,
                )
                computed_crc = crc16_ccitt(header_data + bytes(self._payload))

                if computed_crc == self._crc:
                    frames.append(
                        TueFrame(
                            seq=self._seq,
                            cmd=self._cmd,
                            status=self._status,
                            payload=bytes(self._payload),
                            crc=self._crc,
                        )
                    )

                self._state = self.STATE_WAIT_SOF1

            else:
                self._state = self.STATE_WAIT_SOF1

        return frames


# ==============================================================================
# Domain Payload Helpers
# ==============================================================================


def encode_led_cmd(
    led_index: int = 0,
    mode: Union[int, LedMode] = LedMode.TOGGLE,
    param: int = 0,
) -> bytes:
    """Encode payload for CMD_SYS_LED_CTRL (0x04).

    Args:
        led_index: Index of onboard LED (0 = LED_MCU).
        mode: LedMode enum (OFF, ON, TOGGLE, QUERY).
        param: Optional parameter (e.g. blink duration).

    Returns:
        Serialized payload bytes.
    """
    mode_val = int(mode) & 0xFF
    return struct.pack("<BBH", led_index & 0xFF, mode_val, param & 0xFFFF)


def decode_led_response(payload: bytes) -> Tuple[int, int]:
    """Decode response payload for CMD_SYS_LED_CTRL.

    Args:
        payload: Received response payload (2 bytes: led_index, current_state).

    Returns:
        Tuple of (led_index, current_state: 0=OFF, 1=ON).

    Raises:
        TueError: If payload length is less than 2 bytes.
    """
    if len(payload) < 2:
        raise TueError(f"LED response payload too short: expected >= 2 bytes, got {len(payload)}")
    led_index, current_state = struct.unpack_from("<BB", payload, 0)
    return led_index, current_state


def decode_info_response(payload: bytes) -> Dict[str, Any]:
    """Decode response payload for CMD_SYS_GET_INFO.

    Args:
        payload: Received response payload (19 bytes).

    Returns:
        Dictionary containing board_id, version, and mcu_uid.

    Raises:
        TueError: If payload length is less than 19 bytes.
    """
    if len(payload) < 19:
        raise TueError(f"Info response payload too short: expected 19 bytes, got {len(payload)}")

    board_id_bytes = payload[0:4]
    try:
        board_id = board_id_bytes.decode("ascii", errors="replace")
    except Exception:
        board_id = board_id_bytes.hex()

    fw_major, fw_minor, fw_patch = struct.unpack_from("<BBB", payload, 4)
    uid_bytes = payload[7:19]
    uid_hex = uid_bytes.hex().upper()

    return {
        "board_id": board_id,
        "fw_major": fw_major,
        "fw_minor": fw_minor,
        "fw_patch": fw_patch,
        "version_str": f"{fw_major}.{fw_minor}.{fw_patch}",
        "mcu_uid": uid_hex,
    }


# ==============================================================================
# DC Motor Payloads (0x10–0x13)
# ==============================================================================


def encode_dc_set_duty(
    motor_id: int,
    direction: Union[int, DcDirection],
    duty: int,
) -> bytes:
    """Encode payload for CMD_DC_SET_DUTY (0x10).

    Args:
        motor_id: Motor channel (1..4).
        direction: Direction mode (FLOAT=0, CW=1, CCW=2, BRAKE=3).
        duty: PWM duty cycle (0..10000 = 0.00%..100.00%).

    Returns:
        4-byte serialized request payload.
    """
    if not (1 <= motor_id <= 4):
        raise ValueError(f"Invalid motor_id {motor_id}: must be 1..4")
    if not (0 <= duty <= 10000):
        raise ValueError(f"Invalid duty {duty}: must be 0..10000 (0.01% units)")
    return struct.pack("<BBH", motor_id & 0xFF, int(direction) & 0xFF, duty & 0xFFFF)


def decode_dc_set_duty_response(payload: bytes) -> Tuple[int, int]:
    """Decode response for CMD_DC_SET_DUTY.

    Returns:
        Tuple of (motor_id, ack_code: 0=OK).
    """
    if len(payload) < 2:
        raise TueError(f"DC set duty response too short: expected 2 bytes, got {len(payload)}")
    return struct.unpack_from("<BB", payload, 0)


def encode_dc_stop(
    motor_id: int = 0,
    brake_mode: Union[int, DcBrakeMode] = DcBrakeMode.ACTIVE,
) -> bytes:
    """Encode payload for CMD_DC_STOP (0x11).

    Args:
        motor_id: Motor channel (0 = all motors, 1..4 = specific channel).
        brake_mode: Brake mode (COAST=0, ACTIVE=1).

    Returns:
        2-byte serialized request payload.
    """
    return struct.pack("<BB", motor_id & 0xFF, int(brake_mode) & 0xFF)


def decode_dc_stop_response(payload: bytes) -> Tuple[int, int]:
    """Decode response for CMD_DC_STOP.

    Returns:
        Tuple of (motor_id, ack_code: 0=OK).
    """
    if len(payload) < 2:
        raise TueError(f"DC stop response too short: expected 2 bytes, got {len(payload)}")
    return struct.unpack_from("<BB", payload, 0)


def encode_dc_get_status(motor_id: int) -> bytes:
    """Encode payload for CMD_DC_GET_STATUS (0x12).

    Args:
        motor_id: Motor channel (1..4).
    """
    if not (1 <= motor_id <= 4):
        raise ValueError(f"Invalid motor_id {motor_id}: must be 1..4")
    return struct.pack("<B", motor_id & 0xFF)


def decode_dc_get_status_response(payload: bytes) -> Dict[str, Any]:
    """Decode response for CMD_DC_GET_STATUS (8 bytes).

    Returns:
        Dictionary with motor_id, direction, duty, fault, current_ma.
    """
    if len(payload) < 8:
        raise TueError(f"DC status response too short: expected 8 bytes, got {len(payload)}")
    motor_id, direction, duty, fault, current_ma, _ = struct.unpack_from("<BBHBHB", payload, 0)
    return {
        "motor_id": motor_id,
        "direction": DcDirection(direction) if direction in DcDirection._value2member_map_ else direction,
        "duty": duty,
        "fault": bool(fault),
        "current_ma": current_ma,
    }


def encode_dc_set_pid(
    motor_id: int,
    kp: float,
    ki: float,
    kd: float,
) -> bytes:
    """Encode payload for CMD_DC_SET_PID (0x13).

    Args:
        motor_id: Motor channel (1..4).
        kp: Proportional gain (float32).
        ki: Integral gain (float32).
        kd: Derivative gain (float32).
    """
    if not (1 <= motor_id <= 4):
        raise ValueError(f"Invalid motor_id {motor_id}: must be 1..4")
    return struct.pack("<BfffB", motor_id & 0xFF, float(kp), float(ki), float(kd), 0)


def decode_dc_set_pid_response(payload: bytes) -> Tuple[int, int]:
    """Decode response for CMD_DC_SET_PID.

    Returns:
        Tuple of (motor_id, ack_code: 0=OK).
    """
    if len(payload) < 2:
        raise TueError(f"DC set PID response too short: expected 2 bytes, got {len(payload)}")
    return struct.unpack_from("<BB", payload, 0)


# ==============================================================================
# Stepper Motor Payloads (0x20–0x24)
# ==============================================================================


def encode_step_enable(stepper_id: int = 0, enable: bool = True) -> bytes:
    """Encode payload for CMD_STEP_ENABLE (0x20).

    Args:
        stepper_id: Stepper channel (0 = all, 1..4 = specific channel).
        enable: True to enable drivers, False to disable.
    """
    return struct.pack("<BB", stepper_id & 0xFF, 1 if enable else 0)


def decode_step_enable_response(payload: bytes) -> Tuple[int, int]:
    """Decode response for CMD_STEP_ENABLE.

    Returns:
        Tuple of (stepper_id, ack_code: 0=OK).
    """
    if len(payload) < 2:
        raise TueError(f"Stepper enable response too short: expected 2 bytes, got {len(payload)}")
    return struct.unpack_from("<BB", payload, 0)


def encode_step_move(
    stepper_id: int,
    target_steps: int,
    speed_hz: int = 1000,
    accel: int = 500,
    is_relative: bool = True,
) -> bytes:
    """Encode payload for CMD_STEP_MOVE (0x21).

    Args:
        stepper_id: Stepper channel (1..4).
        target_steps: Target microsteps (signed 32-bit).
        speed_hz: Step pulse rate in Hz (steps/s).
        accel: Acceleration rate in steps/s².
        is_relative: True for relative move, False for absolute move.
    """
    if not (1 <= stepper_id <= 4):
        raise ValueError(f"Invalid stepper_id {stepper_id}: must be 1..4")
    return struct.pack(
        "<BBiII",
        stepper_id & 0xFF,
        1 if is_relative else 0,
        int(target_steps),
        int(speed_hz),
        int(accel),
    )


def decode_step_move_response(payload: bytes) -> Tuple[int, int]:
    """Decode response for CMD_STEP_MOVE.

    Returns:
        Tuple of (stepper_id, ack_code: 0=OK).
    """
    if len(payload) < 2:
        raise TueError(f"Stepper move response too short: expected 2 bytes, got {len(payload)}")
    return struct.unpack_from("<BB", payload, 0)


def encode_step_stop(stepper_id: int = 0, hard_stop: bool = False) -> bytes:
    """Encode payload for CMD_STEP_STOP (0x22).

    Args:
        stepper_id: Stepper channel (0 = all, 1..4 = specific channel).
        hard_stop: False for deceleration stop, True for immediate emergency stop.
    """
    return struct.pack("<BB", stepper_id & 0xFF, 1 if hard_stop else 0)


def decode_step_stop_response(payload: bytes) -> Tuple[int, int]:
    """Decode response for CMD_STEP_STOP.

    Returns:
        Tuple of (stepper_id, ack_code: 0=OK).
    """
    if len(payload) < 2:
        raise TueError(f"Stepper stop response too short: expected 2 bytes, got {len(payload)}")
    return struct.unpack_from("<BB", payload, 0)


def encode_step_get_pos(stepper_id: int) -> bytes:
    """Encode payload for CMD_STEP_GET_POS (0x23).

    Args:
        stepper_id: Stepper channel (1..4).
    """
    if not (1 <= stepper_id <= 4):
        raise ValueError(f"Invalid stepper_id {stepper_id}: must be 1..4")
    return struct.pack("<B", stepper_id & 0xFF)


def decode_step_get_pos_response(payload: bytes) -> Dict[str, Any]:
    """Decode response for CMD_STEP_GET_POS (9 bytes).

    Returns:
        Dictionary with stepper_id, current_steps, is_moving, fault.
    """
    if len(payload) < 9:
        raise TueError(f"Stepper get pos response too short: expected 9 bytes, got {len(payload)}")
    stepper_id, current_steps, is_moving, fault, _ = struct.unpack_from("<BiBBH", payload, 0)
    return {
        "stepper_id": stepper_id,
        "current_steps": current_steps,
        "is_moving": bool(is_moving),
        "fault": fault,
    }


def encode_step_set_homing(stepper_id: int, direction: int = 0) -> bytes:
    """Encode payload for CMD_STEP_SET_HOMING (0x24).

    Args:
        stepper_id: Stepper channel (1..4).
        direction: Homing direction/mode flag.
    """
    if not (1 <= stepper_id <= 4):
        raise ValueError(f"Invalid stepper_id {stepper_id}: must be 1..4")
    return struct.pack("<BB", stepper_id & 0xFF, direction & 0xFF)


def decode_step_set_homing_response(payload: bytes) -> Tuple[int, int]:
    """Decode response for CMD_STEP_SET_HOMING.

    Returns:
        Tuple of (stepper_id, ack_code: 0=OK).
    """
    if len(payload) < 2:
        raise TueError(f"Stepper homing response too short: expected 2 bytes, got {len(payload)}")
    return struct.unpack_from("<BB", payload, 0)


# ==============================================================================
# STS3215 Bus Servo Payloads (0x30–0x35)
# ==============================================================================


def encode_servo_ping(servo_id: int) -> bytes:
    """Encode payload for CMD_SERVO_PING (0x30).

    Args:
        servo_id: Servo bus ID (0..253).
    """
    return struct.pack("<B", servo_id & 0xFF)


def decode_servo_ping_response(payload: bytes) -> Tuple[int, int]:
    """Decode response for CMD_SERVO_PING.

    Returns:
        Tuple of (servo_id, ack_code: 0=OK).
    """
    if len(payload) < 2:
        raise TueError(f"Servo ping response too short: expected 2 bytes, got {len(payload)}")
    return struct.unpack_from("<BB", payload, 0)


def encode_servo_read_reg(servo_id: int, reg_addr: int, read_len: int) -> bytes:
    """Encode payload for CMD_SERVO_READ_REG (0x31).

    Args:
        servo_id: Servo bus ID (0..253).
        reg_addr: Memory table starting register address (0..0x45).
        read_len: Number of bytes to read (1..64).
    """
    return struct.pack("<BBB", servo_id & 0xFF, reg_addr & 0xFF, read_len & 0xFF)


def decode_servo_read_reg_response(payload: bytes) -> Tuple[int, int, bytes]:
    """Decode response for CMD_SERVO_READ_REG.

    Returns:
        Tuple of (servo_id, reg_addr, data_bytes).
    """
    if len(payload) < 2:
        raise TueError(f"Servo read reg response too short: expected >= 2 bytes, got {len(payload)}")
    servo_id, reg_addr = struct.unpack_from("<BB", payload, 0)
    return servo_id, reg_addr, payload[2:]


def encode_servo_write_reg(servo_id: int, reg_addr: int, data: bytes) -> bytes:
    """Encode payload for CMD_SERVO_WRITE_REG (0x32).

    Args:
        servo_id: Servo bus ID (0..253).
        reg_addr: Memory table starting register address (0..0x45).
        data: Raw bytes to write into registers.
    """
    return struct.pack("<BB", servo_id & 0xFF, reg_addr & 0xFF) + data


def decode_servo_write_reg_response(payload: bytes) -> Tuple[int, int]:
    """Decode response for CMD_SERVO_WRITE_REG.

    Returns:
        Tuple of (servo_id, ack_code: 0=OK).
    """
    if len(payload) < 2:
        raise TueError(f"Servo write reg response too short: expected 2 bytes, got {len(payload)}")
    return struct.unpack_from("<BB", payload, 0)


def encode_servo_set_pos(
    servo_id: int,
    position: int,
    speed: int = 0,
    accel: int = 0,
) -> bytes:
    """Encode payload for CMD_SERVO_SET_POS (0x33).

    Args:
        servo_id: Servo bus ID (0..253).
        position: Goal position in steps (0..4095 in position mode, signed 16-bit).
        speed: Speed limit in steps/s (0 = maximum speed).
        accel: Acceleration profile (0..254).
    """
    return struct.pack("<BhhBB", servo_id & 0xFF, int(position), int(speed), int(accel) & 0xFF, 0)


def decode_servo_set_pos_response(payload: bytes) -> Tuple[int, int]:
    """Decode response for CMD_SERVO_SET_POS.

    Returns:
        Tuple of (servo_id, ack_code: 0=OK).
    """
    if len(payload) < 2:
        raise TueError(f"Servo set pos response too short: expected 2 bytes, got {len(payload)}")
    return struct.unpack_from("<BB", payload, 0)


def encode_servo_get_feedback(servo_id: int) -> bytes:
    """Encode payload for CMD_SERVO_GET_FEEDBACK (0x34).

    Args:
        servo_id: Servo bus ID (0..253).
    """
    return struct.pack("<B", servo_id & 0xFF)


def decode_servo_get_feedback_response(payload: bytes) -> Dict[str, Any]:
    """Decode response for CMD_SERVO_GET_FEEDBACK (13 bytes).

    Returns:
        Dictionary with telemetry: position, speed, load_percent, voltage_v,
        temperature_degc, status_mask, current_ma, is_moving.
    """
    if len(payload) < 13:
        raise TueError(f"Servo feedback response too short: expected 13 bytes, got {len(payload)}")

    servo_id, pos, speed, load, volt_raw, temp_degc, status_mask, curr_raw, moving = struct.unpack_from(
        "<BhhhBBBhB", payload, 0
    )

    return {
        "servo_id": servo_id,
        "position": pos,
        "speed": speed,
        "load_percent": round(load * 0.1, 1),
        "voltage_v": round(volt_raw * 0.1, 1),
        "temperature_degc": temp_degc,
        "status_mask": status_mask,
        "current_ma": round(curr_raw * 6.5, 1),
        "is_moving": bool(moving),
        "voltage_raw": volt_raw,
        "current_raw": curr_raw,
        "load_raw": load,
    }


def encode_servo_torque_enable(servo_id: int, enable: Union[bool, int] = True) -> bytes:
    """Encode payload for CMD_SERVO_TORQUE_ENABLE (0x35).

    Args:
        servo_id: Servo bus ID (0..253).
        enable: True/1 to enable torque, False/0 to disable torque,
                or 128 to calibrate middle position to 2048.
    """
    if isinstance(enable, bool):
        val = 1 if enable else 0
    else:
        val = int(enable) & 0xFF
    return struct.pack("<BB", servo_id & 0xFF, val)


def decode_servo_torque_enable_response(payload: bytes) -> Tuple[int, int]:
    """Decode response for CMD_SERVO_TORQUE_ENABLE.

    Returns:
        Tuple of (servo_id, ack_code: 0=OK).
    """
    if len(payload) < 2:
        raise TueError(f"Servo torque response too short: expected 2 bytes, got {len(payload)}")
    return struct.unpack_from("<BB", payload, 0)
