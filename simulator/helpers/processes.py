"""Utility for launching subprocesses, optionally in a visible terminal."""

import logging
import os
import platform
import signal
import time
from subprocess import DEVNULL, Popen


def create_process(
    cmd: str,
    after: str = "exit",
    visible: bool = True,
    title: str = "Terminal",
    env_cmd: str | None = None,
    suppress_output: bool = False,
    terminal_geometry: str = "80x10",
    new_process_group: bool = False,
) -> Popen[bytes]:
    """Launch a subprocess, optionally in a visible terminal."""
    redirect = " > /dev/null 2>&1" if suppress_output else ""
    full_cmd = (
        (f"{env_cmd}; " if env_cmd else "")
        + f"{cmd}{redirect}"
        + (f"; {after}" if visible else "")
    )
    bash_cmd = ["bash", "-c", full_cmd]

    if visible and platform.system() == "Linux":
        display_env = os.environ.get("DISPLAY")
        if not display_env:
            raise RuntimeError("DISPLAY not set. X11 forwarding may not be active.")

        env = os.environ.copy()
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

        if new_process_group:
            return Popen(
                terminal_cmd,
                env=env,
                start_new_session=True,
            )

        return Popen(
            terminal_cmd,
            env=env,
        )

    if visible:
        raise OSError("Unsupported OS for visible terminal mode.")

    if new_process_group:
        return Popen(
            bash_cmd,
            stdout=DEVNULL if suppress_output else None,
            stderr=DEVNULL if suppress_output else None,
            env=os.environ.copy(),
            start_new_session=True,
        )

    return Popen(
        bash_cmd,
        stdout=DEVNULL if suppress_output else None,
        stderr=DEVNULL if suppress_output else None,
        env=os.environ.copy(),
    )


def terminate_process_group(
    proc: Popen[bytes],
    name: str,
    sysid: int,
    timeout: float = 1.0,
) -> None:
    """Terminate a process group gracefully, then forcefully if needed."""
    try:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGTERM)
            logging.info(f"process {name} for UAV {sysid} terminated")
    except ProcessLookupError:
        return
    except Exception as e:
        logging.warning(f"Could not terminate process {name} for UAV {sysid}: {e}")
        return

    time.sleep(timeout)

    try:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGKILL)
            logging.info(f"process {name} for UAV {sysid} killed")
    except ProcessLookupError:
        return
    except Exception as e:
        logging.warning(f"Could not kill process {name} for UAV {sysid}: {e}")
