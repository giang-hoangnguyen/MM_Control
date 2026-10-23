"""
TUE USB Onboard LED Control Example Script.

Demonstrates controlling and querying the STM32F407 onboard status LED (LED_MCU, PE5)
over USB CDC using the tue_usb Python client library.

Examples:
    # Scan for available COM ports
    python led_control.py --scan

    # Turn onboard LED ON
    python led_control.py --port COM3 --on

    # Turn onboard LED OFF
    python led_control.py --port COM3 --off

    # Toggle onboard LED
    python led_control.py --port COM3 --toggle

    # Query current LED state
    python led_control.py --port COM3 --query

    # Blink LED 5 times (0.2s interval)
    python led_control.py --port COM3 --blink 5 --interval 0.2

    # Query board ID and firmware info
    python led_control.py --port COM3 --info

    # Interactive mode
    python led_control.py --port COM3 --interactive
"""
#!/usr/bin/env python3
import argparse
import os
import sys
import time

# Ensure package import works when running directly from repo
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from tue_usb import TueClient, TueError, TueTimeoutError, TueProtocolError


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
    print(f"[INFO] Defaulting to seccond port: {ports[1]} (use --port to specify)")
    return ports[1]


def run_interactive(client: TueClient) -> None:
    """Run an interactive console session to manipulate the LED."""
    print("\n--- Interactive Onboard LED Controller ---")
    print("Commands:")
    print("  1      - Turn LED ON")
    print("  0      - Turn LED OFF")
    print("  t      - Toggle LED")
    print("  q      - Query current state")
    print("  b <N>  - Blink N times (e.g. 'b 3')")
    print("  i      - Get device info")
    print("  p      - Ping MCU")
    print("  exit   - Quit\n")

    while True:
        try:
            cmd = input("LED > ").strip().lower()
        except (KeyboardInterrupt, EOFError):
            print("\nExiting.")
            break

        if not cmd:
            continue
        if cmd in ("exit", "quit", "x"):
            break

        try:
            if cmd == "1":
                state = client.set_led(led_index=0, state=True)
                print(f"-> LED is ON (state={state})")
            elif cmd == "0":
                state = client.set_led(led_index=0, state=False)
                print(f"-> LED is OFF (state={state})")
            elif cmd == "t":
                state = client.toggle_led(led_index=0)
                print(f"-> LED Toggled: now {'ON' if state else 'OFF'}")
            elif cmd == "q":
                state = client.get_led(led_index=0)
                print(f"-> Current LED State: {'ON' if state else 'OFF'}")
            elif cmd.startswith("b"):
                parts = cmd.split()
                count = int(parts[1]) if len(parts) > 1 else 3
                print(f"-> Blinking {count} times...")
                for i in range(count):
                    client.set_led(0, True)
                    time.sleep(0.2)
                    client.set_led(0, False)
                    time.sleep(0.2)
                print("-> Done.")
            elif cmd == "i":
                info = client.get_info()
                print(f"-> Board ID:    {info['board_id']}")
                print(f"-> Firmware:    v{info['version_str']}")
                print(f"-> MCU UID:     {info['mcu_uid']}")
            elif cmd == "p":
                t0 = time.time()
                resp = client.ping(b"PING_TEST")
                dt = (time.time() - t0) * 1000.0
                print(f"-> Ping OK: '{resp.decode('ascii', errors='replace')}' ({dt:.1f} ms)")
            else:
                print(f"Unknown command: '{cmd}'")
        except TueError as e:
            print(f"[ERROR] {e}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="TUE USB STM32F407 Onboard LED Controller",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--port",
        "-p",
        type=str,
        default=None,
        help="Serial port device (e.g. COM3 or /dev/ttyACM0)",
    )
    parser.add_argument(
        "--scan",
        action="store_true",
        help="Scan and print available serial ports then exit",
    )
    parser.add_argument(
        "--on",
        action="store_true",
        help="Turn onboard LED ON",
    )
    parser.add_argument(
        "--off",
        action="store_true",
        help="Turn onboard LED OFF",
    )
    parser.add_argument(
        "--toggle",
        action="store_true",
        help="Toggle onboard LED",
    )
    parser.add_argument(
        "--query",
        action="store_true",
        help="Query current LED state",
    )
    parser.add_argument(
        "--blink",
        type=int,
        metavar="COUNT",
        help="Blink LED specified number of times",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.25,
        help="Blink half-period interval in seconds (default: 0.25)",
    )
    parser.add_argument(
        "--ping",
        action="store_true",
        help="Ping MCU echo test",
    )
    parser.add_argument(
        "--info",
        action="store_true",
        help="Query board signature and MCU UID",
    )
    parser.add_argument(
        "--interactive",
        "-i",
        action="store_true",
        help="Start interactive console session",
    )

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

    print(f"Connecting to STM32 controller on {port}...")
    try:
        with TueClient(port=port) as client:
            print(f"Connected to {port} successfully.")

            if args.info:
                info = client.get_info()
                print("\n[Device Information]")
                print(f"  Board ID:         {info['board_id']}")
                print(f"  Firmware Version: {info['version_str']}")
                print(f"  STM32 96-bit UID: {info['mcu_uid']}")

            if args.ping:
                t0 = time.time()
                resp = client.ping(b"HELLO_STM32")
                dt = (time.time() - t0) * 1000.0
                print(f"[Ping] Echo payload: {resp.decode('ascii', errors='replace')} in {dt:.2f} ms")

            if args.on:
                state = client.set_led(led_index=0, state=True)
                print(f"[LED] State set to ON (readback={state})")

            elif args.off:
                state = client.set_led(led_index=0, state=False)
                print(f"[LED] State set to OFF (readback={state})")

            elif args.toggle:
                state = client.toggle_led(led_index=0)
                print(f"[LED] Toggled: now {'ON' if state else 'OFF'}")

            elif args.query:
                state = client.get_led(led_index=0)
                print(f"[LED] Current state: {'ON' if state else 'OFF'}")

            elif args.blink:
                count = args.blink
                interval = args.interval
                print(f"[LED] Blinking {count} times (interval: {interval}s)...")
                for i in range(count):
                    client.set_led(led_index=0, state=True)
                    time.sleep(interval)
                    client.set_led(led_index=0, state=False)
                    time.sleep(interval)
                print("[LED] Blinking sequence completed.")

            elif args.interactive or (not any([args.on, args.off, args.toggle, args.query, args.blink, args.ping, args.info])):
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
