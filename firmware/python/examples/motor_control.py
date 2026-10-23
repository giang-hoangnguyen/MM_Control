"""
TUE USB Motor and Servo Control Example Script.

Demonstrates controlling DC motors, Stepper motors, and STS3215 bus servos
over USB CDC using the tue_usb Python client library.

Examples:
    # Scan for available COM ports
    python motor_control.py --scan

    # Test DC motor 1 at 50% duty CW
    python motor_control.py --port COM3 --dc 1 --dc-duty 5000 --dc-dir cw

    # Read DC motor 1 status
    python motor_control.py --port COM3 --dc 1 --status

    # Stop all DC motors with active brake
    python motor_control.py --port COM3 --dc-stop

    # Enable all stepper drivers and move stepper 1 by 2000 steps
    python motor_control.py --port COM3 --step 1 --step-move 2000 --speed 1000

    # Read stepper 1 position
    python motor_control.py --port COM3 --step 1 --step-pos

    # Ping STS3215 servo 1
    python motor_control.py --port COM3 --servo 1 --servo-ping

    # Query STS3215 servo 1 telemetry feedback
    python motor_control.py --port COM3 --servo 1 --servo-fb

    # Move STS3215 servo 1 to position 2048 (middle)
    python motor_control.py --port COM3 --servo 1 --servo-pos 2048

    # Start interactive motor testing shell
    python motor_control.py --port COM3 --interactive
"""
#!/usr/bin/env python3
import argparse
import os
import sys
import time

# Ensure package import works when running directly from repo
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from tue_usb import (
    DcBrakeMode,
    DcDirection,
    Sts3215Reg,
    TueClient,
    TueError,
    TueProtocolError,
    TueTimeoutError,
)


def auto_detect_port() -> str:
    """Attempt to detect or suggest an available serial port."""
    ports = TueClient.list_ports()
    if not ports:
        print("[ERROR] No serial ports detected! Please connect the STM32 USB cable.")
        sys.exit(1)
    if len(ports) == 1:
        print(f"[INFO] Auto-selected single available port: {ports[0]}")
        return ports[0]
    print(f"[INFO] Multiple ports found: {', '.join(ports)}")
    print(f"[INFO] Defaulting to: {ports[1]} (use --port to specify)")
    return ports[1]


def run_interactive(client: TueClient) -> None:
    """Run an interactive console session for motor and servo testing."""
    print("\n=======================================================")
    print("      TUE USB Motion Controller Interactive Console     ")
    print("=======================================================")
    print("Commands:")
    print("  dc <id> <cw|ccw|float|brake> <duty 0..10000> - Set DC motor duty")
    print("  dc status <id>                                - Read DC motor status")
    print("  dc stop [id]                                  - Stop DC motor(s)")
    print("  step enable [0|1]                             - Enable/disable steppers")
    print("  step move <id> <steps> [speed_hz] [accel]     - Move stepper")
    print("  step pos <id>                                 - Query stepper position")
    print("  step stop [id]                                - Stop stepper(s)")
    print("  servo ping <id>                               - Ping STS3215 servo")
    print("  servo pos <id> <position 0..4095> [speed]     - Set servo goal position")
    print("  servo fb <id>                                 - Read servo telemetry")
    print("  servo torque <id> <0|1|128>                   - Toggle torque (128=calib)")
    print("  info                                          - Board ID & firmware")
    print("  exit                                          - Quit\n")

    while True:
        try:
            line = input("MOTION > ").strip()
        except (KeyboardInterrupt, EOFError):
            print("\nExiting.")
            break

        if not line:
            continue
        parts = line.split()
        cmd = parts[0].lower()

        if cmd in ("exit", "quit", "q"):
            break

        try:
            if cmd == "info":
                info = client.get_info()
                print(f"-> Board: {info['board_id']} | FW: v{info['version_str']} | UID: {info['mcu_uid']}")

            elif cmd == "dc":
                if len(parts) >= 4 and parts[1].isdigit():
                    m_id = int(parts[1])
                    dir_str = parts[2].lower()
                    duty = int(parts[3])
                    dirs = {"cw": DcDirection.CW, "ccw": DcDirection.CCW, "float": DcDirection.FLOAT, "brake": DcDirection.BRAKE}
                    if dir_str not in dirs:
                        print(f"Unknown direction '{dir_str}', use cw, ccw, float, or brake")
                        continue
                    m, ack = client.dc_set_duty(m_id, dirs[dir_str], duty)
                    print(f"-> DC Motor {m}: Duty set to {duty/100:.2f}% ({dir_str.upper()}) [ACK={ack}]")
                elif len(parts) >= 3 and parts[1].lower() == "status":
                    m_id = int(parts[2])
                    st = client.dc_get_status(m_id)
                    print(f"-> DC Motor {st['motor_id']}: Direction={st['direction'].name}, Duty={st['duty']/100:.2f}%, Fault={st['fault']}, Current={st['current_ma']} mA")
                elif len(parts) >= 2 and parts[1].lower() == "stop":
                    m_id = int(parts[2]) if len(parts) >= 3 else 0
                    m, ack = client.dc_stop(m_id, DcBrakeMode.ACTIVE)
                    print(f"-> DC Motor(s) {m} stopped [ACK={ack}]")
                else:
                    print("Usage: dc <id> <cw|ccw|float|brake> <duty> OR dc status <id> OR dc stop [id]")

            elif cmd == "step":
                sub = parts[1].lower() if len(parts) > 1 else ""
                if sub == "enable":
                    en = (int(parts[2]) != 0) if len(parts) > 2 else True
                    s, ack = client.step_enable(0, en)
                    print(f"-> Steppers {'ENABLED' if en else 'DISABLED'} [ACK={ack}]")
                elif sub == "move" and len(parts) >= 4:
                    s_id = int(parts[2])
                    steps = int(parts[3])
                    spd = int(parts[4]) if len(parts) > 4 else 1000
                    acc = int(parts[5]) if len(parts) > 5 else 500
                    s, ack = client.step_move(s_id, steps, speed_hz=spd, accel=acc, is_relative=True)
                    print(f"-> Stepper {s_id}: Moving {steps} steps at {spd} Hz (accel={acc}) [ACK={ack}]")
                elif sub == "pos" and len(parts) >= 3:
                    s_id = int(parts[2])
                    pos = client.step_get_pos(s_id)
                    print(f"-> Stepper {pos['stepper_id']}: Pos={pos['current_steps']} steps, Moving={pos['is_moving']}, Fault={pos['fault']}")
                elif sub == "stop":
                    s_id = int(parts[2]) if len(parts) > 2 else 0
                    s, ack = client.step_stop(s_id, hard_stop=False)
                    print(f"-> Stepper {s_id} stopped [ACK={ack}]")
                else:
                    print("Usage: step enable [0|1] OR step move <id> <steps> [speed] [accel] OR step pos <id> OR step stop [id]")

            elif cmd == "servo":
                sub = parts[1].lower() if len(parts) > 1 else ""
                if sub == "ping" and len(parts) >= 3:
                    sv_id = int(parts[2])
                    t0 = time.time()
                    sid, ack = client.servo_ping(sv_id)
                    dt = (time.time() - t0) * 1000.0
                    print(f"-> Servo {sid} Ping {'OK' if ack == 0 else 'FAIL'} ({dt:.1f} ms)")
                elif sub == "pos" and len(parts) >= 4:
                    sv_id = int(parts[2])
                    pos = int(parts[3])
                    spd = int(parts[4]) if len(parts) > 4 else 0
                    sid, ack = client.servo_set_pos(sv_id, pos, speed=spd)
                    print(f"-> Servo {sid}: Commanded Goal Pos={pos}, Speed={spd} [ACK={ack}]")
                elif sub == "fb" and len(parts) >= 3:
                    sv_id = int(parts[2])
                    fb = client.servo_get_feedback(sv_id)
                    print(f"-> Servo {fb['servo_id']} Telemetry:")
                    print(f"     Position:    {fb['position']} steps")
                    print(f"     Speed:       {fb['speed']} steps/s")
                    print(f"     Load:        {fb['load_percent']}%")
                    print(f"     Voltage:     {fb['voltage_v']} V")
                    print(f"     Temperature: {fb['temperature_degc']} °C")
                    print(f"     Current:     {fb['current_ma']} mA")
                    print(f"     Moving:      {fb['is_moving']}")
                elif sub == "torque" and len(parts) >= 4:
                    sv_id = int(parts[2])
                    val = int(parts[3])
                    sid, ack = client.servo_torque_enable(sv_id, enable=val)
                    print(f"-> Servo {sid}: Torque set to {val} [ACK={ack}]")
                else:
                    print("Usage: servo ping <id> OR servo pos <id> <pos> [speed] OR servo fb <id> OR servo torque <id> <val>")

            else:
                print(f"Unknown command: '{line}'")
        except TueError as e:
            print(f"[COMM ERROR] {e}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="TUE USB Motion Control CLI (DC, Stepper, STS3215 Servo)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--port", "-p", type=str, default=None, help="Serial port device")
    parser.add_argument("--scan", action="store_true", help="List available ports and exit")
    parser.add_argument("--interactive", "-i", action="store_true", help="Launch interactive control console")
    parser.add_argument("--info", action="store_true", help="Query board information")

    # DC Motor CLI options
    parser.add_argument("--dc", type=int, choices=[1, 2, 3, 4], help="Target DC motor channel (1..4)")
    parser.add_argument("--dc-duty", type=int, default=0, help="DC duty in 0.01%% units (0..10000)")
    parser.add_argument("--dc-dir", type=str, choices=["cw", "ccw", "float", "brake"], default="cw", help="DC direction")
    parser.add_argument("--dc-stop", action="store_true", help="Stop specified DC motor or all motors")
    parser.add_argument("--status", action="store_true", help="Query status of specified motor")

    # Stepper CLI options
    parser.add_argument("--step", type=int, choices=[1, 2, 3, 4], help="Target Stepper channel (1..4)")
    parser.add_argument("--step-enable", action="store_true", help="Enable stepper driver")
    parser.add_argument("--step-disable", action="store_true", help="Disable stepper driver")
    parser.add_argument("--step-move", type=int, help="Command target steps to move")
    parser.add_argument("--speed", type=int, default=1000, help="Stepper pulse speed in Hz (default: 1000)")
    parser.add_argument("--accel", type=int, default=500, help="Stepper acceleration (default: 500)")
    parser.add_argument("--step-pos", action="store_true", help="Query current stepper position")
    parser.add_argument("--step-stop", action="store_true", help="Stop stepper motion")

    # STS3215 Servo CLI options
    parser.add_argument("--servo", type=int, help="Target STS3215 Servo ID (0..253)")
    parser.add_argument("--servo-ping", action="store_true", help="Ping servo bus ID")
    parser.add_argument("--servo-pos", type=int, help="Command servo goal position (0..4095)")
    parser.add_argument("--servo-speed", type=int, default=0, help="Servo speed limit (0 = max)")
    parser.add_argument("--servo-fb", action="store_true", help="Read servo feedback telemetry")
    parser.add_argument("--servo-torque", type=int, choices=[0, 1, 128], help="Torque switch (0=OFF, 1=ON, 128=Calibrate)")

    args = parser.parse_args()

    if args.scan:
        ports = TueClient.list_ports()
        if ports:
            print(f"Available serial ports ({len(ports)}):")
            for p in ports:
                print(f"  - {p}")
        else:
            print("No serial ports found.")
        return

    port = args.port or auto_detect_port()

    print(f"Connecting to motion controller on {port}...")
    try:
        with TueClient(port=port) as client:
            print(f"Connected to {port} successfully.")

            if args.info:
                info = client.get_info()
                print("\n[Device Information]")
                print(f"  Board ID:         {info['board_id']}")
                print(f"  Firmware Version: {info['version_str']}")
                print(f"  STM32 96-bit UID: {info['mcu_uid']}")

            # DC Motor
            if args.dc is not None:
                if args.status:
                    st = client.dc_get_status(args.dc)
                    print(f"[DC M{args.dc}] Dir={st['direction'].name}, Duty={st['duty']/100:.2f}%, Fault={st['fault']}")
                elif args.dc_stop:
                    client.dc_stop(args.dc, DcBrakeMode.ACTIVE)
                    print(f"[DC M{args.dc}] Stopped.")
                elif args.dc_duty > 0 or args.dc_dir:
                    dirs = {"cw": DcDirection.CW, "ccw": DcDirection.CCW, "float": DcDirection.FLOAT, "brake": DcDirection.BRAKE}
                    client.dc_set_duty(args.dc, dirs[args.dc_dir], args.dc_duty)
                    print(f"[DC M{args.dc}] Set to {args.dc_duty/100:.2f}% {args.dc_dir.upper()}.")

            elif args.dc_stop:
                client.dc_stop(0, DcBrakeMode.ACTIVE)
                print("[DC] All motors stopped.")

            # Stepper
            if args.step_enable:
                client.step_enable(0, True)
                print("[STEP] Drivers enabled.")
            elif args.step_disable:
                client.step_enable(0, False)
                print("[STEP] Drivers disabled.")

            if args.step is not None:
                if args.step_pos:
                    pos = client.step_get_pos(args.step)
                    print(f"[STEP {args.step}] Pos={pos['current_steps']}, Moving={pos['is_moving']}")
                elif args.step_stop:
                    client.step_stop(args.step)
                    print(f"[STEP {args.step}] Stopped.")
                elif args.step_move is not None:
                    client.step_move(args.step, args.step_move, speed_hz=args.speed, accel=args.accel)
                    print(f"[STEP {args.step}] Moving {args.step_move} steps.")

            # Servo
            if args.servo is not None:
                if args.servo_ping:
                    sid, ack = client.servo_ping(args.servo)
                    print(f"[SERVO {args.servo}] Ping {'OK' if ack == 0 else 'FAIL'}")
                if args.servo_fb:
                    fb = client.servo_get_feedback(args.servo)
                    print(f"[SERVO {args.servo}] Pos={fb['position']}, Spd={fb['speed']}, Load={fb['load_percent']}%, Volt={fb['voltage_v']}V, Temp={fb['temperature_degc']}°C, Curr={fb['current_ma']}mA, Moving={fb['is_moving']}")
                if args.servo_pos is not None:
                    client.servo_set_pos(args.servo, args.servo_pos, speed=args.servo_speed)
                    print(f"[SERVO {args.servo}] Goal position set to {args.servo_pos}")
                if args.servo_torque is not None:
                    client.servo_torque_enable(args.servo, args.servo_torque)
                    print(f"[SERVO {args.servo}] Torque set to {args.servo_torque}")

            if args.interactive or (not any([args.info, args.dc, args.dc_stop, args.step_enable, args.step_disable, args.step, args.servo])):
                run_interactive(client)

    except TueTimeoutError as e:
        print(f"[TIMEOUT ERROR] {e}")
        sys.exit(2)
    except TueProtocolError as e:
        print(f"[PROTOCOL ERROR] {e}")
        sys.exit(3)
    except TueError as e:
        print(f"[COMMUNICATION ERROR] {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
