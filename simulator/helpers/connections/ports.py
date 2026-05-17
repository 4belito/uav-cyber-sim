"""Helpers for plain TCP socket/port checks."""

import subprocess
import time


def is_port_listening(port: int) -> bool:
    """Return True if anything is in LISTEN state on the given TCP port.

    Uses `ss` rather than a socket connect so we never accidentally establish
    (and immediately drop) a real connection — ArduPilot SITL exits when its
    first client disconnects without exchanging data.
    """
    result = subprocess.run(
        ["ss", "-tlnH", f"sport = :{port}"],
        capture_output=True,
        text=True,
    )
    return bool(result.stdout.strip())


def wait_for_port(
    port: int,
    timeout: float = 0.5,
    startup_delay: float = 1.0,
    verbose: bool = False,
) -> None:
    """Wait until a TCP port is in LISTEN state, then pause for startup_delay.

    startup_delay gives the server (e.g. ArduPilot SITL) time to finish
    initialising after the socket is bound but before accepting MAVLink data.
    """
    while not is_port_listening(port):
        if verbose:
            print(f"Waiting for port {port} to open...")
        time.sleep(timeout)
    time.sleep(startup_delay)
