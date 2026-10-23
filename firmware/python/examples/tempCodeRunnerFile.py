import argparse
import os
import sys
import time

# Import tue_usb từ thư mục cha của project
sys.path.insert(
    0,
    os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
)

from tue_usb import TueClient, TueError


def auto_detect_port():
    """Tự động tìm cổng serial của USB Motion Controller."""
    ports = TueClient.list_ports()

    if not ports:
        print("[ERROR] Không tìm thấy cổng serial!")
        print("Hãy kiểm tra kết nối USB với STM32.")
        sys.exit(1)

    if len(ports) == 1:
        print(f"[INFO] Sử dụng cổng: {ports[0]}")
        return ports[0]

    print("[INFO] Các cổng serial tìm thấy:")
    for i, port in enumerate(ports):
        print(f"  {i}: {port}")

    # Cho phép người dùng chọn cổng
    choice = input("Chọn số thứ tự cổng [0]: ").strip()
    index = int(choice) if choice else 0

    if index < 0 or index >= len(ports):
        print("[ERROR] Lựa chọn cổng không hợp lệ.")
        sys.exit(1)

    return ports[index]


def scan_servos(client, start_id=0, end_id=253, delay=0.01):
    """Quét ID servo STS3215 bằng lệnh ping."""
    found = []

    print(f"\nĐang scan servo ID {start_id} đến {end_id}...")
    print("-" * 45)

    start_time = time.time()

    for servo_id in range(start_id, end_id + 1):
        try:
            sid, ack = client.servo_ping(servo_id)

            if ack == 0:
                print(f"[FOUND] Servo phản hồi tại ID: {sid}")
                found.append(sid)

        except TueError:
            # Servo không phản hồi hoặc xảy ra lỗi giao tiếp
            pass

        if delay > 0:
            time.sleep(delay)

    elapsed = time.time() - start_time

    print("-" * 45)
    print(f"Hoàn tất scan trong {elapsed:.2f} giây.")

    if found:
        print(f"Tìm thấy {len(found)} servo:")
        print("Servo IDs:", ", ".join(map(str, found)))
    else:
        print("Không tìm thấy servo nào.")

    return found


def main():
    parser = argparse.ArgumentParser(
        description="Scan ID servo STS3215 qua TUE USB Controller"
    )

    parser.add_argument(
        "--port", "-p",
        type=str,
        default=None,
        help="Cổng serial, ví dụ COM3"
    )

    parser.add_argument(
        "--start",
        type=int,
        default=0,
        help="ID bắt đầu scan (mặc định: 0)"
    )

    parser.add_argument(
        "--end",
        type=int,
        default=253,
        help="ID kết thúc scan (mặc định: 253)"
    )

    parser.add_argument(
        "--delay",
        type=float,
        default=0.01,
        help="Thời gian nghỉ giữa hai lần ping, tính bằng giây"
    )

    args = parser.parse_args()

    if not (0 <= args.start <= args.end <= 253):
        parser.error("Yêu cầu: 0 <= start <= end <= 253")

    if args.delay < 0:
        parser.error("--delay phải lớn hơn hoặc bằng 0")

    port = args.port or auto_detect_port()

    print(f"[INFO] Kết nối Motion Controller qua {port}...")

    try:
        with TueClient(port=port) as client:
            print("[INFO] Kết nối thành công.")

            scan_servos(
                client,
                start_id=args.start,
                end_id=args.end,
                delay=args.delay
            )

    except TueError as e:
        print(f"[ERROR] Lỗi giao tiếp: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()