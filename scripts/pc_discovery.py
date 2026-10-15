import socket
import time

DISCOVERY_PORT = 9000
BROADCAST_IP = "192.168.4.255"
MESSAGE = b"HELO"

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

print(f"Broadcasting PC discovery to {BROADCAST_IP}:{DISCOVERY_PORT}")

try:
    while True:
        sock.sendto(MESSAGE, (BROADCAST_IP, DISCOVERY_PORT))
        time.sleep(1)
except KeyboardInterrupt:
    print("Discovery stopped.")
finally:
    sock.close()