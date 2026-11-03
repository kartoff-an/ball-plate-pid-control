import argparse
import socket

ESP32_IP = "192.168.4.1"
PORT = 9004

MIN_PULSE_US = 1000
MAX_PULSE_US = 3000


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default=ESP32_IP)
    args = parser.parse_args()

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        print(f"Connected logically to {args.ip}:{PORT}")
        print("Commands: 0 1500 | 1 1500 | q")
        print(f"Allowed pulse range: {MIN_PULSE_US}-{MAX_PULSE_US} us")

        while True:
            try:
                command = input("servo> ").strip()

                if command.lower() in {"q", "quit", "exit"}:
                    break

                parts = command.split()
                if len(parts) != 2:
                    print("Enter: <channel> <pulse_us>")
                    continue

                channel, pulse = map(int, parts)

                if channel not in (0, 1):
                    print("Channel must be 0 or 1.")
                    continue

                if not MIN_PULSE_US <= pulse <= MAX_PULSE_US:
                    print("Pulse is outside the permitted test range.")
                    continue

                message = f"SERVO {channel} {pulse}".encode()
                sock.sendto(message, (args.ip, PORT))
                print(f"Sent channel {channel}: {pulse} us")

            except (ValueError, EOFError):
                print("Invalid input.")
                break
            except KeyboardInterrupt:
                break


if __name__ == "__main__":
    main()