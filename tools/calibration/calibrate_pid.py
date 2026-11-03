
import argparse
import socket

ESP32_IP = "192.168.4.1"
PORT = 9004

MAX_KP = 10.0
MAX_KI = 10.0
MAX_KD = 10.0


def main():
    parser = argparse.ArgumentParser(
        description="Tune ball-and-plate PID gains over UDP."
    )
    parser.add_argument("--ip", default=ESP32_IP)
    args = parser.parse_args()

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.settimeout(2.0)

        print(f"Target: {args.ip}:{PORT}")
        print("Commands:")
        print("  get")
        print("  x <kp> <ki> <kd>")
        print("  y <kp> <ki> <kd>")
        print("  both <kp> <ki> <kd>")
        print("  q")
        print(f"Gain limits: Kp 0-{MAX_KP}, Ki 0-{MAX_KI}, Kd 0-{MAX_KD}")
        print("Example: x 2.0 0.0 0.0")

        while True:
            try:
                command = input("pid> ").strip()

                if command.lower() in {"q", "quit", "exit"}:
                    break

                parts = command.split()
                if not parts:
                    continue

                action = parts[0].lower()

                if action == "get":
                    if len(parts) != 1:
                        print("Usage: get")
                        continue

                    message = "GET"

                elif action in {"x", "y", "both"}:
                    if len(parts) != 4:
                        print(f"Usage: {action} <kp> <ki> <kd>")
                        continue

                    try:
                        kp, ki, kd = map(float, parts[1:])
                    except ValueError:
                        print("Gains must be numeric.")
                        continue

                    if not all(map(lambda value: value >= 0 and value < float("inf"),
                                   (kp, ki, kd))):
                        print("Gains must be finite and non-negative.")
                        continue

                    if not (kp <= MAX_KP and ki <= MAX_KI and kd <= MAX_KD):
                        print("Gain exceeds the configured limits.")
                        continue

                    message = f"SET {action.upper()} {kp} {ki} {kd}"

                else:
                    print("Unknown command. Use get, x, y, both, or q.")
                    continue

                try:
                    sock.sendto(message.encode(), (args.ip, PORT))
                    response, _ = sock.recvfrom(256)
                    print(response.decode(errors="replace").strip())

                except socket.timeout:
                    print("No response from ESP32-S3 (timeout).")

            except (EOFError, KeyboardInterrupt):
                print()
                break

    print("PID calibration ended.")


if __name__ == "__main__":
    main()
