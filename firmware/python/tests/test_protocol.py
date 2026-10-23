import os
import sys
import unittest

# Ensure package is discoverable regardless of working directory
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from tue_usb.crc import crc16_ccitt, verify_crc16
from tue_usb.defs import (
    CRC_SIZE,
    HEADER_SIZE,
    MAX_PAYLOAD_SIZE,
    MIN_FRAME_SIZE,
    SOF1,
    SOF2,
    CommandId,
    DcBrakeMode,
    DcDirection,
    LedMode,
    StatusCode,
    StepperMode,
    StepperStopMode,
    Sts3215Reg,
)
from tue_usb.exceptions import TueCrcError, TueError
from tue_usb.protocol import (
    TueFrame,
    TueFrameParser,
    decode_dc_get_status_response,
    decode_dc_set_duty_response,
    decode_dc_set_pid_response,
    decode_dc_stop_response,
    decode_frame,
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


class TestCrc16(unittest.TestCase):
    """Test CRC-16/CCITT-FALSE implementation."""

    def test_standard_check_vector(self):
        """Standard check value for b'123456789' must equal 0x29B1."""
        test_data = b"123456789"
        crc = crc16_ccitt(test_data)
        self.assertEqual(crc, 0x29B1)
        self.assertTrue(verify_crc16(test_data, 0x29B1))

    def test_empty_buffer(self):
        """Empty buffer should return initial CRC value 0xFFFF."""
        self.assertEqual(crc16_ccitt(b""), 0xFFFF)

    def test_single_byte(self):
        """Single byte calculation check."""
        crc = crc16_ccitt(b"\x00")
        self.assertIsInstance(crc, int)
        self.assertTrue(0 <= crc <= 0xFFFF)

    def test_verify_failure(self):
        """verify_crc16 should return False for mismatched CRC."""
        self.assertFalse(verify_crc16(b"123456789", 0x1234))


class TestFrameEncodingDecoding(unittest.TestCase):
    """Test frame encoding and decoding."""

    def test_encode_empty_payload(self):
        """Encode frame with 0 payload bytes."""
        frame_bytes = encode_frame(seq=1, cmd=CommandId.SYS_PING, status=StatusCode.OK, payload=b"")
        self.assertEqual(len(frame_bytes), MIN_FRAME_SIZE)  # 9 bytes
        self.assertEqual(frame_bytes[0], SOF1)
        self.assertEqual(frame_bytes[1], SOF2)
        self.assertEqual(frame_bytes[2], 1)                   # SEQ
        self.assertEqual(frame_bytes[3], CommandId.SYS_PING) # CMD
        self.assertEqual(frame_bytes[4], StatusCode.OK)      # STATUS
        self.assertEqual(frame_bytes[5], 0)                   # LEN_L
        self.assertEqual(frame_bytes[6], 0)                   # LEN_H

    def test_encode_decode_roundtrip(self):
        """Encode and decode frame round-trip."""
        payload = b"Hello STM32F407!"
        encoded = encode_frame(
            seq=42,
            cmd=CommandId.SYS_PING,
            status=StatusCode.OK,
            payload=payload,
        )

        decoded = decode_frame(encoded)
        self.assertEqual(decoded.seq, 42)
        self.assertEqual(decoded.cmd, CommandId.SYS_PING)
        self.assertEqual(decoded.status, StatusCode.OK)
        self.assertEqual(decoded.payload, payload)
        self.assertTrue(decoded.is_ok)

    def test_decode_truncated_frame(self):
        """Decoding truncated bytes must raise TueError."""
        frame_bytes = encode_frame(seq=1, cmd=0x01, payload=b"TestData")
        with self.assertRaises(TueError):
            decode_frame(frame_bytes[:5])

    def test_decode_corrupted_crc(self):
        """Corrupted CRC byte must raise TueCrcError."""
        frame_bytes = bytearray(encode_frame(seq=1, cmd=0x01, payload=b"TestData"))
        frame_bytes[-1] ^= 0xFF  # Corrupt CRC MSB
        with self.assertRaises(TueCrcError):
            decode_frame(bytes(frame_bytes))

    def test_decode_bad_sof(self):
        """Corrupted SOF markers must raise TueError."""
        frame_bytes = bytearray(encode_frame(seq=1, cmd=0x01, payload=b"TestData"))
        frame_bytes[0] = 0x00
        with self.assertRaises(TueError):
            decode_frame(bytes(frame_bytes))

    def test_payload_exceeding_max_raises(self):
        """Payload exceeding 256 bytes must raise ValueError."""
        oversized = bytes(257)
        with self.assertRaises(ValueError):
            encode_frame(seq=1, cmd=0x01, payload=oversized)


class TestStreamingParser(unittest.TestCase):
    """Test TueFrameParser streaming fragmentation and concatenation handling."""

    def setUp(self):
        self.parser = TueFrameParser()

    def test_parse_single_complete_frame(self):
        """Feed a single complete frame in one chunk."""
        data = encode_frame(seq=10, cmd=CommandId.SYS_PING, payload=b"EchoData")
        frames = self.parser.feed(data)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].seq, 10)
        self.assertEqual(frames[0].payload, b"EchoData")

    def test_parse_fragmented_byte_by_byte(self):
        """Simulate worst-case USB fragmentation by feeding 1 byte at a time."""
        data = encode_frame(seq=55, cmd=CommandId.SYS_LED_CTRL, payload=b"\x00\x01\x00\x00")
        all_frames = []
        for byte_val in data:
            frames = self.parser.feed(bytes([byte_val]))
            all_frames.extend(frames)

        self.assertEqual(len(all_frames), 1)
        frame = all_frames[0]
        self.assertEqual(frame.seq, 55)
        self.assertEqual(frame.cmd, CommandId.SYS_LED_CTRL)
        self.assertEqual(frame.payload, b"\x00\x01\x00\x00")

    def test_parse_concatenated_frames(self):
        """Feed multiple frames in a single chunk."""
        frame1 = encode_frame(seq=1, cmd=0x01, payload=b"Frame1")
        frame2 = encode_frame(seq=2, cmd=0x02, payload=b"Frame2")
        frame3 = encode_frame(seq=3, cmd=0x04, payload=b"\x00\x02")

        combined = frame1 + frame2 + frame3
        frames = self.parser.feed(combined)

        self.assertEqual(len(frames), 3)
        self.assertEqual(frames[0].seq, 1)
        self.assertEqual(frames[0].payload, b"Frame1")
        self.assertEqual(frames[1].seq, 2)
        self.assertEqual(frames[1].payload, b"Frame2")
        self.assertEqual(frames[2].seq, 3)
        self.assertEqual(frames[2].payload, b"\x00\x02")

    def test_recovery_from_garbage_preamble(self):
        """Parser must discard noise bytes and synchronize onto valid frame."""
        garbage = b"\x00\xFF\xAA\x12\x34\xFE\xAA\xAA\xAA"
        valid = encode_frame(seq=99, cmd=CommandId.SYS_PING, payload=b"Recovered")

        frames = self.parser.feed(garbage + valid)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].seq, 99)
        self.assertEqual(frames[0].payload, b"Recovered")

    def test_crc_error_dropped(self):
        """Frame with corrupted CRC must be dropped by parser without crashing."""
        corrupted = bytearray(encode_frame(seq=7, cmd=0x01, payload=b"BadCRC"))
        corrupted[-1] ^= 0x55  # Damage CRC

        valid = encode_frame(seq=8, cmd=0x01, payload=b"GoodFrame")

        frames = self.parser.feed(bytes(corrupted) + valid)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].seq, 8)
        self.assertEqual(frames[0].payload, b"GoodFrame")


class TestCommandPayloadHelpers(unittest.TestCase):
    """Test LED and Info payload encoding/decoding."""

    def test_led_command_encoding(self):
        """Test encoding LED commands for various modes."""
        # OFF
        payload_off = encode_led_cmd(led_index=0, mode=LedMode.OFF)
        self.assertEqual(payload_off, b"\x00\x00\x00\x00")

        # ON
        payload_on = encode_led_cmd(led_index=0, mode=LedMode.ON)
        self.assertEqual(payload_on, b"\x00\x01\x00\x00")

        # TOGGLE
        payload_toggle = encode_led_cmd(led_index=0, mode=LedMode.TOGGLE)
        self.assertEqual(payload_toggle, b"\x00\x02\x00\x00")

        # QUERY
        payload_query = encode_led_cmd(led_index=0, mode=LedMode.QUERY)
        self.assertEqual(payload_query, b"\x00\xFF\x00\x00")

    def test_led_response_decoding(self):
        """Test decoding LED response payload."""
        led_idx, state = decode_led_response(b"\x00\x01")
        self.assertEqual(led_idx, 0)
        self.assertEqual(state, 1)

        led_idx, state = decode_led_response(b"\x00\x00")
        self.assertEqual(led_idx, 0)
        self.assertEqual(state, 0)

    def test_info_response_decoding(self):
        """Test decoding get_info response payload."""
        # 4 bytes "MM01", versions 1.0.0, 12 bytes UID
        dummy_uid = b"\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0A\x0B\x0C"
        payload = b"MM01\x01\x00\x00" + dummy_uid
        info = decode_info_response(payload)

        self.assertEqual(info["board_id"], "MM01")
        self.assertEqual(info["fw_major"], 1)
        self.assertEqual(info["fw_minor"], 0)
        self.assertEqual(info["fw_patch"], 0)
        self.assertEqual(info["version_str"], "1.0.0")
        self.assertEqual(info["mcu_uid"], "0102030405060708090A0B0C")


class TestDcMotorPayloads(unittest.TestCase):
    """Test DC motor payload encoding and decoding."""

    def test_dc_set_duty_encode_decode(self):
        payload = encode_dc_set_duty(motor_id=1, direction=DcDirection.CW, duty=7500)
        self.assertEqual(len(payload), 4)
        self.assertEqual(payload[0], 1)
        self.assertEqual(payload[1], DcDirection.CW)
        # 7500 = 0x1D4C -> 4C 1D
        self.assertEqual(payload[2], 0x4C)
        self.assertEqual(payload[3], 0x1D)

        # Response
        motor_id, ack = decode_dc_set_duty_response(b"\x01\x00")
        self.assertEqual(motor_id, 1)
        self.assertEqual(ack, 0)

    def test_dc_stop_encode_decode(self):
        payload = encode_dc_stop(motor_id=0, brake_mode=DcBrakeMode.ACTIVE)
        self.assertEqual(payload, b"\x00\x01")

        motor_id, ack = decode_dc_stop_response(b"\x00\x00")
        self.assertEqual(motor_id, 0)
        self.assertEqual(ack, 0)

    def test_dc_get_status_encode_decode(self):
        payload = encode_dc_get_status(motor_id=3)
        self.assertEqual(payload, b"\x03")

        # 8-byte status response: motor_id=3, dir=CCW(2), duty=5000(0x1388), fault=0, current=150mA, res=0
        raw_resp = b"\x03\x02\x88\x13\x00\x96\x00\x00"
        status = decode_dc_get_status_response(raw_resp)
        self.assertEqual(status["motor_id"], 3)
        self.assertEqual(status["direction"], DcDirection.CCW)
        self.assertEqual(status["duty"], 5000)
        self.assertFalse(status["fault"])
        self.assertEqual(status["current_ma"], 150)

    def test_dc_set_pid_encode_decode(self):
        payload = encode_dc_set_pid(motor_id=2, kp=1.5, ki=0.2, kd=0.05)
        self.assertEqual(len(payload), 14)
        self.assertEqual(payload[0], 2)

        motor_id, ack = decode_dc_set_pid_response(b"\x02\x00")
        self.assertEqual(motor_id, 2)
        self.assertEqual(ack, 0)

    def test_dc_invalid_params_raise(self):
        with self.assertRaises(ValueError):
            encode_dc_set_duty(motor_id=5, direction=DcDirection.CW, duty=1000)
        with self.assertRaises(ValueError):
            encode_dc_set_duty(motor_id=1, direction=DcDirection.CW, duty=10001)
        with self.assertRaises(ValueError):
            encode_dc_get_status(motor_id=0)


class TestStepperMotorPayloads(unittest.TestCase):
    """Test Stepper motor payload encoding and decoding."""

    def test_step_enable_encode_decode(self):
        payload = encode_step_enable(stepper_id=0, enable=True)
        self.assertEqual(payload, b"\x00\x01")

        payload_off = encode_step_enable(stepper_id=2, enable=False)
        self.assertEqual(payload_off, b"\x02\x00")

        stepper_id, ack = decode_step_enable_response(b"\x02\x00")
        self.assertEqual(stepper_id, 2)
        self.assertEqual(ack, 0)

    def test_step_move_encode_decode(self):
        payload = encode_step_move(
            stepper_id=1,
            target_steps=10000,
            speed_hz=2000,
            accel=1000,
            is_relative=True,
        )
        self.assertEqual(len(payload), 14)
        self.assertEqual(payload[0], 1)
        self.assertEqual(payload[1], 1)  # relative

        stepper_id, ack = decode_step_move_response(b"\x01\x00")
        self.assertEqual(stepper_id, 1)
        self.assertEqual(ack, 0)

    def test_step_stop_encode_decode(self):
        payload = encode_step_stop(stepper_id=0, hard_stop=True)
        self.assertEqual(payload, b"\x00\x01")

        stepper_id, ack = decode_step_stop_response(b"\x00\x00")
        self.assertEqual(stepper_id, 0)
        self.assertEqual(ack, 0)

    def test_step_get_pos_encode_decode(self):
        payload = encode_step_get_pos(stepper_id=2)
        self.assertEqual(payload, b"\x02")

        # 9 bytes: stepper_id=2, pos=5000 (0x00001388), is_moving=1, fault=0, reserved=0
        raw_resp = b"\x02\x88\x13\x00\x00\x01\x00\x00\x00"
        pos_info = decode_step_get_pos_response(raw_resp)
        self.assertEqual(pos_info["stepper_id"], 2)
        self.assertEqual(pos_info["current_steps"], 5000)
        self.assertTrue(pos_info["is_moving"])
        self.assertEqual(pos_info["fault"], 0)

    def test_step_set_homing_encode_decode(self):
        payload = encode_step_set_homing(stepper_id=1, direction=0)
        self.assertEqual(payload, b"\x01\x00")

        stepper_id, ack = decode_step_set_homing_response(b"\x01\x00")
        self.assertEqual(stepper_id, 1)
        self.assertEqual(ack, 0)

    def test_step_invalid_params_raise(self):
        with self.assertRaises(ValueError):
            encode_step_move(stepper_id=0, target_steps=100)
        with self.assertRaises(ValueError):
            encode_step_get_pos(stepper_id=5)


class TestServoPayloads(unittest.TestCase):
    """Test STS3215 Bus Servo payload encoding and decoding."""

    def test_servo_ping_encode_decode(self):
        payload = encode_servo_ping(servo_id=1)
        self.assertEqual(payload, b"\x01")

        servo_id, ack = decode_servo_ping_response(b"\x01\x00")
        self.assertEqual(servo_id, 1)
        self.assertEqual(ack, 0)

    def test_servo_read_reg_encode_decode(self):
        payload = encode_servo_read_reg(servo_id=1, reg_addr=Sts3215Reg.ID, read_len=1)
        self.assertEqual(payload, b"\x01\x05\x01")

        # Response: servo_id=1, reg=0x05, data=[0x01]
        servo_id, reg_addr, data = decode_servo_read_reg_response(b"\x01\x05\x01")
        self.assertEqual(servo_id, 1)
        self.assertEqual(reg_addr, 0x05)
        self.assertEqual(data, b"\x01")

    def test_servo_write_reg_encode_decode(self):
        payload = encode_servo_write_reg(servo_id=1, reg_addr=Sts3215Reg.TORQUE_SWITCH, data=b"\x01")
        self.assertEqual(payload, b"\x01\x28\x01")

        servo_id, ack = decode_servo_write_reg_response(b"\x01\x00")
        self.assertEqual(servo_id, 1)
        self.assertEqual(ack, 0)

    def test_servo_set_pos_encode_decode(self):
        payload = encode_servo_set_pos(servo_id=1, position=2048, speed=1000, accel=50)
        self.assertEqual(len(payload), 7)
        self.assertEqual(payload[0], 1)
        # 2048 = 0x0800 -> 00 08
        self.assertEqual(payload[1:3], b"\x00\x08")
        # 1000 = 0x03E8 -> E8 03
        self.assertEqual(payload[3:5], b"\xE8\x03")
        self.assertEqual(payload[5], 50)
        self.assertEqual(payload[6], 0)

        servo_id, ack = decode_servo_set_pos_response(b"\x01\x00")
        self.assertEqual(servo_id, 1)
        self.assertEqual(ack, 0)

    def test_servo_get_feedback_encode_decode(self):
        payload = encode_servo_get_feedback(servo_id=1)
        self.assertEqual(payload, b"\x01")

        # 13-byte response:
        # id=1, pos=2048, spd=0, load=100 (10.0%), volt=120 (12.0V), temp=35°C,
        # status=0, current=50 (325.0mA), moving=0
        raw_fb = (
            b"\x01"                  # servo_id = 1
            b"\x00\x08"              # pos = 2048
            b"\x00\x00"              # speed = 0
            b"\x64\x00"              # load = 100
            b"\x78"                  # volt = 120 (12.0V)
            b"\x23"                  # temp = 35°C
            b"\x00"                  # status = 0
            b"\x32\x00"              # current = 50
            b"\x00"                  # moving = 0
        )
        fb = decode_servo_get_feedback_response(raw_fb)
        self.assertEqual(fb["servo_id"], 1)
        self.assertEqual(fb["position"], 2048)
        self.assertEqual(fb["speed"], 0)
        self.assertEqual(fb["load_percent"], 10.0)
        self.assertEqual(fb["voltage_v"], 12.0)
        self.assertEqual(fb["temperature_degc"], 35)
        self.assertEqual(fb["status_mask"], 0)
        self.assertEqual(fb["current_ma"], 325.0)
        self.assertFalse(fb["is_moving"])

    def test_servo_torque_enable_encode_decode(self):
        payload_on = encode_servo_torque_enable(servo_id=1, enable=True)
        self.assertEqual(payload_on, b"\x01\x01")

        payload_calib = encode_servo_torque_enable(servo_id=1, enable=128)
        self.assertEqual(payload_calib, b"\x01\x80")

        servo_id, ack = decode_servo_torque_enable_response(b"\x01\x00")
        self.assertEqual(servo_id, 1)
        self.assertEqual(ack, 0)

    def test_servo_short_payload_raises(self):
        with self.assertRaises(TueError):
            decode_servo_get_feedback_response(b"\x01\x00\x00")


if __name__ == "__main__":
    unittest.main()
