"""Helper functions for network connections."""

import socket
import time


def is_port_open(host: str, port: int, timeout: float = 0.5) -> bool:
    """Check if a TCP port is open."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(timeout)
        try:
            sock.connect((host, port))
            return True
        except (ConnectionRefusedError, OSError):
            return False


def wait_for_port(port: int, timeout: float = 0.5):
    """Wait until a TCP port is open."""
    while not is_port_open("127.0.0.1", port):
        print(f"Waiting for port {port} to open...")
        time.sleep(timeout)
