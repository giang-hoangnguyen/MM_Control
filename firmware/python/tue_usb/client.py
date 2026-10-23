"""
TUE USB Serial Client.

Provides a thread-safe, high-level client interface communicating over USB CDC using pyserial.
"""

import time
import threading
from typing import Any, Dict, List, Optional, Tuple, Union

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None  # type: ignore

from .defs import (
    CommandId,
    DcBrakeMode,
    DcDirection,
    LedMode,
    StatusCode,
    StepperMode,
    StepperStopMode,
    Sts3215Reg,
)
from .exceptions import (
    TueConnectionError,
    TueError,
    TueProtocolError,
    TueTimeoutError,
)
from .protocol import (
    TueFrame,
    TueFrameParser,
    decode_dc_get_status_response,
    decode_dc_set_duty_response,
    decode_dc_set_pid_response,
    decode_dc_stop_response,
    decode_info_response,
    decode_led_response,
    decode_servo_get_feedback_response,
    decode_servo_ping_response,
    decode_servo_read_reg_response,
    decode_servo_set_pos_response,
    decode_servo_torque_enable_response,
    decode_servo_write_reg_response,
    decode_step_enable_response,
    decode_step_get_pos_response,
    decode_step_move_response,
    decode_step_set_homing_response,
    decode_step_stop_response,
    encode_dc_get_status,
    encode_dc_set_duty,
    encode_dc_set_pid,
    encode_dc_stop,
    encode_frame,
    encode_led_cmd,
    encode_servo_get_feedback,
    encode_servo_ping,
    encode_servo_read_reg,
    encode_servo_set_pos,
    encode_servo_torque_enable,
    encode_servo_write_reg,
    encode_step_enable,
    encode_step_get_pos,
    encode_step_move,
    encode_step_set_homing,
    encode_step_stop,
)


class TueClient:
    """Thread-safe USB CDC communication client for TUE motion controllers."""

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        timeout: float = 1.0,
    ) -> None:
        """Initialize the TUE USB Client.

        Args:
            port: Serial device path (e.g. "COM3" on Windows, "/dev/ttyACM0" on Linux).
            baudrate: Baudrate (ignored by virtual USB CDC, but kept for pyserial compatibility).
            timeout: Default timeout in seconds for operations.
        """
        if serial is None:
            raise ImportError(
                "pyserial is required to use TueClient. Install with: pip install pyserial"
            )

        self._port_name: str = port
        self._baudrate: int = baudrate
        self._default_timeout: float = timeout
        self._ser: Optional[serial.Serial] = None
        self._seq: int = 0
        self._lock = threading.Lock()
        self._parser = TueFrameParser()

    @staticmethod
    def list_ports() -> List[str]:
        """List all available serial port device names."""
        if serial is None:
            return []
        ports = serial.tools.list_ports.comports()
        return [p.device for p in ports]

    def open(self) -> None:
        """Open the serial port connection."""
        if self._ser is not None and self._ser.is_open:
            return

        try:
            self._ser = serial.Serial(
                port=self._port_name,
                baudrate=self._baudrate,
                timeout=0.05,  # Non-blocking poll chunks
                write_timeout=self._default_timeout,
            )
            self._parser.reset()
            # Flush any stale data
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()
        except Exception as e:
            raise TueConnectionError(
                f"Failed to open serial port '{self._port_name}': {e}"
            ) from e

    def close(self) -> None:
        """Close the serial port connection."""
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    @property
    def is_open(self) -> bool:
        """Check if serial connection is open."""
        return self._ser is not None and self._ser.is_open

    def __enter__(self) -> "TueClient":
        self.open()
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        self.close()

    def _next_seq(self) -> int:
        seq = self._seq
        self._seq = (self._seq + 1) & 0xFF
        return seq

    def send_command(
        self,
        cmd: Union[int, CommandId],
        payload: bytes = b"",
        timeout: Optional[float] = None,
    ) -> TueFrame:
        """Send a request frame and synchronously await the corresponding response.

        Args:
            cmd: Command ID.
            payload: Payload bytes.
            timeout: Timeout in seconds (defaults to client timeout).

        Returns:
            The received and validated TueFrame response.

        Raises:
            TueTimeoutError: If response is not received within timeout.
            TueProtocolError: If MCU returns a non-OK status.
            TueConnectionError: If connection is broken or closed.
        """
        if not self.is_open:
            self.open()

        cmd_val = int(cmd) & 0xFF
        deadline = time.time() + (timeout if timeout is not None else self._default_timeout)

        with self._lock:
            assert self._ser is not None
            seq = self._next_seq()
            raw_frame = encode_frame(seq=seq, cmd=cmd_val, status=StatusCode.OK, payload=payload)

            # Flush input buffer before sending request
            self._ser.reset_input_buffer()
            self._parser.reset()

            try:
                self._ser.write(raw_frame)
                self._ser.flush()
            except Exception as e:
                raise TueConnectionError(f"Serial write error: {e}") from e

            # Read response until matching SEQ and CMD are found
            while time.time() < deadline:
                chunk = self._ser.read(self._ser.in_waiting or 1)
                if chunk:
                    frames = self._parser.feed(chunk)
                    for frame in frames:
                        if frame.seq == seq and frame.cmd == cmd_val:
                            if not frame.is_ok:
                                raise TueProtocolError(frame.status)
                            return frame
                else:
                    time.sleep(0.002)

            raise TueTimeoutError(
                f"Timed out waiting for response to CMD 0x{cmd_val:02X} (seq={seq})"
            )

    # ==========================================================================
    # High-Level Subsystem APIs
    # ==========================================================================

    def ping(self, data: bytes = b"TUE_PING") -> bytes:
        """Send a ping echo request.

        Args:
            data: Arbitrary bytes to echo.

        Returns:
            Echoed payload bytes from MCU.
        """
        resp = self.send_command(CommandId.SYS_PING, payload=data)
        return resp.payload

    def get_info(self) -> Dict[str, Any]:
        """Query board ID, firmware version, and STM32 Unique ID.

        Returns:
            Dictionary with board info fields.
        """
        resp = self.send_command(CommandId.SYS_GET_INFO)
        return decode_info_response(resp.payload)

    def set_led(self, led_index: int = 0, state: bool = True) -> bool:
        """Set onboard LED state.

        Args:
            led_index: Index of LED (0 = LED_MCU).
            state: True for ON, False for OFF.

        Returns:
            Current pin state (True = ON, False = OFF).
        """
        mode = LedMode.ON if state else LedMode.OFF
        payload = encode_led_cmd(led_index, mode)
        resp = self.send_command(CommandId.SYS_LED_CTRL, payload=payload)
        _, current_state = decode_led_response(resp.payload)
        return bool(current_state)

    def toggle_led(self, led_index: int = 0) -> bool:
        """Toggle onboard LED state.

        Args:
            led_index: Index of LED (0 = LED_MCU).

        Returns:
            Current pin state after toggle (True = ON, False = OFF).
        """
        payload = encode_led_cmd(led_index, LedMode.TOGGLE)
        resp = self.send_command(CommandId.SYS_LED_CTRL, payload=payload)
        _, current_state = decode_led_response(resp.payload)
        return bool(current_state)

    def get_led(self, led_index: int = 0) -> bool:
        """Query onboard LED state without altering it.

        Args:
            led_index: Index of LED (0 = LED_MCU).

        Returns:
            Current pin state (True = ON, False = OFF).
        """
        payload = encode_led_cmd(led_index, LedMode.QUERY)
        resp = self.send_command(CommandId.SYS_LED_CTRL, payload=payload)
        _, current_state = decode_led_response(resp.payload)
        return bool(current_state)

    # ==========================================================================
    # DC Motor APIs (0x10–0x13)
    # ==========================================================================

    def dc_set_duty(
        self,
        motor_id: int,
        direction: Union[int, DcDirection],
        duty: int,
        timeout: Optional[float] = None,
    ) -> Tuple[int, int]:
        """Set DC motor direction and PWM duty cycle.

        Args:
            motor_id: Motor channel (1..4).
            direction: Direction mode (FLOAT=0, CW=1, CCW=2, BRAKE=3).
            duty: PWM duty cycle (0..10000 = 0.00%..100.00%).
            timeout: Optional response timeout in seconds.

        Returns:
            Tuple of (motor_id, ack_code: 0=OK).
        """
        payload = encode_dc_set_duty(motor_id, direction, duty)
        resp = self.send_command(CommandId.DC_SET_DUTY, payload=payload, timeout=timeout)
        return decode_dc_set_duty_response(resp.payload)

    def dc_stop(
        self,
        motor_id: int = 0,
        brake_mode: Union[int, DcBrakeMode] = DcBrakeMode.ACTIVE,
        timeout: Optional[float] = None,
    ) -> Tuple[int, int]:
        """Stop or brake DC motor.

        Args:
            motor_id: Motor channel (0 = all motors, 1..4 = specific channel).
            brake_mode: Brake mode (COAST=0, ACTIVE=1).
            timeout: Optional response timeout in seconds.

        Returns:
            Tuple of (motor_id, ack_code: 0=OK).
        """
        payload = encode_dc_stop(motor_id, brake_mode)
        resp = self.send_command(CommandId.DC_STOP, payload=payload, timeout=timeout)
        return decode_dc_stop_response(resp.payload)

    def dc_get_status(
        self,
        motor_id: int,
        timeout: Optional[float] = None,
    ) -> Dict[str, Any]:
        """Get DC motor status.

        Args:
            motor_id: Motor channel (1..4).
            timeout: Optional response timeout in seconds.

        Returns:
            Dictionary with motor_id, direction, duty, fault, current_ma.
        """
        payload = encode_dc_get_status(motor_id)
        resp = self.send_command(CommandId.DC_GET_STATUS, payload=payload, timeout=timeout)
        return decode_dc_get_status_response(resp.payload)

    def dc_set_pid(
        self,
        motor_id: int,
        kp: float,
        ki: float,
        kd: float,
        timeout: Optional[float] = None,
    ) -> Tuple[int, int]:
        """Configure velocity PID parameters for a DC motor.

        Args:
            motor_id: Motor channel (1..4).
            kp: Proportional coefficient.
            ki: Integral coefficient.
            kd: Derivative coefficient.
            timeout: Optional response timeout in seconds.

        Returns:
            Tuple of (motor_id, ack_code: 0=OK).
        """
        payload = encode_dc_set_pid(motor_id, kp, ki, kd)
        resp = self.send_command(CommandId.DC_SET_PID, payload=payload, timeout=timeout)
        return decode_dc_set_pid_response(resp.payload)

    # ==========================================================================
    # Stepper Motor APIs (0x20–0x24)
    # ==========================================================================

    def step_enable(
        self,
        stepper_id: int = 0,
        enable: bool = True,
        timeout: Optional[float] = None,
    ) -> Tuple[int, int]:
        """Enable or disable stepper driver outputs.

        Args:
            stepper_id: Stepper channel (0 = all, 1..4 = specific channel).
            enable: True to enable drivers, False to disable.
            timeout: Optional response timeout in seconds.

        Returns:
            Tuple of (stepper_id, ack_code: 0=OK).
        """
        payload = encode_step_enable(stepper_id, enable)
        resp = self.send_command(CommandId.STEP_ENABLE, payload=payload, timeout=timeout)
        return decode_step_enable_response(resp.payload)

    def step_move(
        self,
        stepper_id: int,
        target_steps: int,
        speed_hz: int = 1000,
        accel: int = 500,
        is_relative: bool = True,
        timeout: Optional[float] = None,
    ) -> Tuple[int, int]:
        """Command stepper to move steps with target speed and acceleration profile.

        Args:
            stepper_id: Stepper channel (1..4).
            target_steps: Step distance or target coordinate (signed 32-bit).
            speed_hz: Maximum pulse frequency in steps/second.
            accel: Acceleration/deceleration rate in steps/second².
            is_relative: True for relative move, False for absolute position move.
            timeout: Optional response timeout in seconds.

        Returns:
            Tuple of (stepper_id, ack_code: 0=OK).
        """
        payload = encode_step_move(stepper_id, target_steps, speed_hz, accel, is_relative)
        resp = self.send_command(CommandId.STEP_MOVE, payload=payload, timeout=timeout)
        return decode_step_move_response(resp.payload)

    def step_stop(
        self,
        stepper_id: int = 0,
        hard_stop: bool = False,
        timeout: Optional[float] = None,
    ) -> Tuple[int, int]:
        """Stop stepper motion.

        Args:
            stepper_id: Stepper channel (0 = all, 1..4 = specific channel).
            hard_stop: False for deceleration stop, True for immediate emergency stop.
            timeout: Optional response timeout in seconds.

        Returns:
            Tuple of (stepper_id, ack_code: 0=OK).
        """
        payload = encode_step_stop(stepper_id, hard_stop)
        resp = self.send_command(CommandId.STEP_STOP, payload=payload, timeout=timeout)
        return decode_step_stop_response(resp.payload)

    def step_get_pos(
        self,
        stepper_id: int,
        timeout: Optional[float] = None,
    ) -> Dict[str, Any]:
        """Get stepper current position and motion status.

        Args:
            stepper_id: Stepper channel (1..4).
            timeout: Optional response timeout in seconds.

        Returns:
            Dictionary with stepper_id, current_steps, is_moving, fault.
        """
        payload = encode_step_get_pos(stepper_id)
        resp = self.send_command(CommandId.STEP_GET_POS, payload=payload, timeout=timeout)
        return decode_step_get_pos_response(resp.payload)

    def step_set_homing(
        self,
        stepper_id: int,
        direction: int = 0,
        timeout: Optional[float] = None,
    ) -> Tuple[int, int]:
        """Initiate homing sequence / reset position to zero.

        Args:
            stepper_id: Stepper channel (1..4).
            direction: Homing mode or direction flag.
            timeout: Optional response timeout in seconds.

        Returns:
            Tuple of (stepper_id, ack_code: 0=OK).
        """
        payload = encode_step_set_homing(stepper_id, direction)
        resp = self.send_command(CommandId.STEP_SET_HOMING, payload=payload, timeout=timeout)
        return decode_step_set_homing_response(resp.payload)

    # ==========================================================================
    # STS3215 Bus Servo APIs (0x30–0x35)
    # ==========================================================================

    def servo_ping(
        self,
        servo_id: int,
        timeout: Optional[float] = None,
    ) -> Tuple[int, int]:
        """Ping an STS3215 servo by bus ID.

        Args:
            servo_id: Servo bus ID (0..253).
            timeout: Optional response timeout in seconds.

        Returns:
            Tuple of (servo_id, ack_code: 0=OK).
        """
        payload = encode_servo_ping(servo_id)
        resp = self.send_command(CommandId.SERVO_PING, payload=payload, timeout=timeout)
        return decode_servo_ping_response(resp.payload)

    def servo_read_reg(
        self,
        servo_id: int,
        reg_addr: Union[int, Sts3215Reg],
        read_len: int,
        timeout: Optional[float] = None,
    ) -> Tuple[int, int, bytes]:
        """Read arbitrary registers from an STS3215 servo memory table.

        Args:
            servo_id: Servo bus ID (0..253).
            reg_addr: Memory table starting register address (0..0x45).
            read_len: Number of bytes to read (1..64).
            timeout: Optional response timeout in seconds.

        Returns:
            Tuple of (servo_id, reg_addr, data_bytes).
        """
        payload = encode_servo_read_reg(servo_id, int(reg_addr), read_len)
        resp = self.send_command(CommandId.SERVO_READ_REG, payload=payload, timeout=timeout)
        return decode_servo_read_reg_response(resp.payload)

    def servo_write_reg(
        self,
        servo_id: int,
        reg_addr: Union[int, Sts3215Reg],
        data: bytes,
        timeout: Optional[float] = None,
    ) -> Tuple[int, int]:
        """Write arbitrary registers to an STS3215 servo memory table.

        Args:
            servo_id: Servo bus ID (0..253).
            reg_addr: Memory table starting register address (0..0x45).
            data: Raw bytes to write into registers.
            timeout: Optional response timeout in seconds.

        Returns:
            Tuple of (servo_id, ack_code: 0=OK).
        """
        payload = encode_servo_write_reg(servo_id, int(reg_addr), data)
        resp = self.send_command(CommandId.SERVO_WRITE_REG, payload=payload, timeout=timeout)
        return decode_servo_write_reg_response(resp.payload)

    def servo_set_pos(
        self,
        servo_id: int,
        position: int,
        speed: int = 0,
        accel: int = 0,
        timeout: Optional[float] = None,
    ) -> Tuple[int, int]:
        """Command STS3215 servo to goal position with target speed and acceleration profile.

        Args:
            servo_id: Servo bus ID (0..253).
            position: Goal position in steps (0..4095 in position mode, signed 16-bit).
            speed: Speed limit in steps/s (0 = maximum speed).
            accel: Acceleration profile (0..254).
            timeout: Optional response timeout in seconds.

        Returns:
            Tuple of (servo_id, ack_code: 0=OK).
        """
        payload = encode_servo_set_pos(servo_id, position, speed, accel)
        resp = self.send_command(CommandId.SERVO_SET_POS, payload=payload, timeout=timeout)
        return decode_servo_set_pos_response(resp.payload)

    def servo_get_feedback(
        self,
        servo_id: int,
        timeout: Optional[float] = None,
    ) -> Dict[str, Any]:
        """Query STS3215 servo comprehensive status feedback telemetry.

        Args:
            servo_id: Servo bus ID (0..253).
            timeout: Optional response timeout in seconds.

        Returns:
            Dictionary with telemetry: position, speed, load_percent, voltage_v,
            temperature_degc, status_mask, current_ma, is_moving.
        """
        payload = encode_servo_get_feedback(servo_id)
        resp = self.send_command(CommandId.SERVO_GET_FEEDBACK, payload=payload, timeout=timeout)
        return decode_servo_get_feedback_response(resp.payload)

    def servo_torque_enable(
        self,
        servo_id: int,
        enable: Union[bool, int] = True,
        timeout: Optional[float] = None,
    ) -> Tuple[int, int]:
        """Enable or disable servo torque.

        Args:
            servo_id: Servo bus ID (0..253).
            enable: True to enable torque, False to disable torque,
                    or 128 to calibrate middle position to 2048.
            timeout: Optional response timeout in seconds.

        Returns:
            Tuple of (servo_id, ack_code: 0=OK).
        """
        payload = encode_servo_torque_enable(servo_id, enable)
        resp = self.send_command(CommandId.SERVO_TORQUE_ENABLE, payload=payload, timeout=timeout)
        return decode_servo_torque_enable_response(resp.payload)
