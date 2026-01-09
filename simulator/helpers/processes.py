"""Utility for launching subprocesses, optionally in a visible terminal."""

import os
import platform
from subprocess import DEVNULL, Popen


def create_process(
    cmd: str,
    after: str = "exit",
    visible: bool = True,
    title: str = "Terminal",
    env_cmd: str | None = None,
    suppress_output: bool = False,
    terminal_geometry: str = "80x10",
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

        if "SSH_CONNECTION" in env or 'REMOTE_CONTAINERS' in env:
            return Popen(
                ["xterm", "-T", title, "-geometry", terminal_geometry, "-e"] + bash_cmd,
                env=env,
            )
        else:
            return Popen(
                [
                    "gnome-terminal",
                    "--title",
                    title,
                    f"--geometry={terminal_geometry}",
                    "--",
                ]
                + bash_cmd,
                env=env,
            )
    elif visible:
        raise OSError("Unsupported OS for visible terminal mode.")

    return Popen(
        bash_cmd,
        stdout=DEVNULL if suppress_output else None,
        stderr=DEVNULL if suppress_output else None,
        env=os.environ.copy(),
    )
