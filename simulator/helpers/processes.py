"""Utility for launching subprocesses, optionally in a visible terminal."""

import logging
import os
import platform
import signal
import time
from enum import StrEnum
from subprocess import DEVNULL, Popen


class SimProcess(StrEnum):
    """Enum for different types of simulation processes."""

    ARDUPILOT = "ardupilot"
    LOGIC = "logic"
    GCS = "gcs"
    ADSB_SOCAT = "adsb_socat"
    ADSB_INJECTOR = "adsb_injector"


def create_process(
    cmd: str,
    after: str = "exit",
    visible: bool = True,
    title: str = "Terminal",
    env_cmd: str | None = None,
    suppress_output: bool = False,
    terminal_geometry: str = "80x10",
    new_process_group: bool = False,
    cwd: str | None = None,
    env: dict[str, str] | None = None,  # 👈 ADD THIS
) -> Popen[bytes]:
    """Launch a subprocess, optionally in a visible terminal."""
    redirect = " > /dev/null 2>&1" if suppress_output else ""

    full_cmd = (
        (f"{env_cmd}; " if env_cmd else "")
        + f"{cmd}{redirect}"
        + (f"; {after}" if visible else "")
    )

    bash_cmd = ["bash", "-c", full_cmd]

    env = env if env is not None else os.environ.copy()

    # =========================
    # Visible terminal (Linux)
    # =========================
    if visible and platform.system() == "Linux":
        display_env = env.get("DISPLAY")
        if not display_env:
            raise RuntimeError("DISPLAY not set. X11 forwarding may not be active.")

        env["DISPLAY"] = display_env

        if "SSH_CONNECTION" in env or "REMOTE_CONTAINERS" in env:
            terminal_cmd = [
                "xterm",
                "-T",
                title,
                "-geometry",
                terminal_geometry,
                "-e",
            ] + bash_cmd
        else:
            terminal_cmd = [
                "gnome-terminal",
                "--title",
                title,
                f"--geometry={terminal_geometry}",
                "--",
            ] + bash_cmd

        return Popen(
            terminal_cmd,
            env=env,
            cwd=cwd,
            start_new_session=new_process_group,
        )

    # =========================
    # Headless execution
    # =========================
    if visible:
        raise OSError("Unsupported OS for visible terminal mode.")

    return Popen(
        bash_cmd,
        stdout=DEVNULL if suppress_output else None,
        stderr=DEVNULL if suppress_output else None,
        env=env,
        cwd=cwd,
        start_new_session=new_process_group,
    )


def terminate_process_group(
    proc: Popen[bytes],
    name: str,
    timeout: float = 1.0,
) -> None:
    """Terminate a process group gracefully, then forcefully if needed."""
    try:
        if proc.poll() is None:
            pgid = os.getpgid(proc.pid)
            os.killpg(pgid, signal.SIGTERM)
            logging.debug(f"process {name} terminated")
    except ProcessLookupError:
        return
    except Exception as e:
        logging.warning(f"Could not terminate process {name}: {e}")
        return

    time.sleep(timeout)

    try:
        if proc.poll() is None:
            pgid = os.getpgid(proc.pid)
            os.killpg(pgid, signal.SIGKILL)
            logging.debug(f"process {name} killed")
    except ProcessLookupError:
        return
    except Exception as e:
        logging.warning(f"Could not kill process {name}: {e}")
