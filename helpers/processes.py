"""Utility for launching subprocesses, optionally in a visible terminal."""

import platform
from subprocess import DEVNULL, Popen

# def create_process(
#     cmd: str,
#     after: str = "exit",
#     visible: bool = True,
#     title: str = "Terminal",
#     env_cmd: str | None = None,
# ) -> Popen[bytes]:
#     """Launch a subprocess, optionally in a visible terminal."""
#     bash_cmd = [
#         "bash",
#         "-c",
#         (f"{env_cmd}; " if env_cmd else "") + f"{cmd}; {after}",
#     ]
#     if visible:
#         if platform.system() == "Linux":
#             return Popen(
#                 [
#                     "gnome-terminal",
#                     "--title",
#                     title,
#                     "--geometry=71x10",  # width=100 cols, height=30 rows
#                     "--",
#                 ]
#                 + bash_cmd
#             )
#         raise OSError("Unsupported OS for visible terminal mode.")
#     return Popen(bash_cmd)


def create_process(
    cmd: str,
    after: str = "exit",
    visible: bool = True,
    title: str = "Terminal",
    env_cmd: str | None = None,
    suppress_output: bool = False,
) -> Popen[bytes]:
    """Launch a subprocess, optionally in a visible terminal."""
    redirect = " > /dev/null 2>&1" if suppress_output else ""
    bash_cmd = [
        "bash",
        "-c",
        (f"{env_cmd}; " if env_cmd else "") + f"{cmd}{redirect}; {after}",
    ]
    if visible:
        if platform.system() == "Linux":
            return Popen(
                [
                    "gnome-terminal",
                    "--title",
                    title,
                    "--geometry=71x10",
                    "--",
                ]
                + bash_cmd
            )
        raise OSError("Unsupported OS for visible terminal mode.")
    return Popen(
        bash_cmd,
        stdout=DEVNULL if suppress_output else None,
        stderr=DEVNULL if suppress_output else None,
    )
