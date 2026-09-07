"""Tools to stop simulation processes and clean up log files."""

import contextlib
import glob
import os
import shutil
import subprocess
import time
from pathlib import Path

from simulator.config import (
    ARDU_LOGS_PATH,
    DATA_PATH,
    LOGS_PATH,
    RUNTIME_GAZEBO_MODELS,
    RUNTIME_GAZEBO_WORLDS,
)

# Patterns matched against the full command line (`pkill -f`), covering every
# process the simulator spawns. The spawn sites are `create_process` in
# `runtime/vehicle_launcher.py` (MITM, socat, ADS-B injector, logic, SITL),
# `sim.py` (GCS) and the visualizers (`gazebo/`, `QGroundControl/`).
#
# The `xterm`/`bash -c` wrappers used for visible terminals carry the inner
# command in their own command line, so matching the inner pattern kills them
# too — they need no entries of their own.
ALL_PROCESSES = [
    # Every Python child is launched as `python3 -m simulator.<module>`, so one
    # pattern covers logic, gcs, adsb_injector and mitm — and anything added
    # later, which is how `simulator.mitm` came to be missed before.
    "python3 -m simulator.",
    # ArduPilot SITL binaries; "arducopter" also matches "arducopter-heli".
    "arduplane",
    "arducopter",
    "ardurover",
    "ardusub",
    # Visualizers.
    "gazebo",
    "gzserver",
    "gzclient",
    "QGroundControl",
    # ADS-B virtual serial cable.
    "socat",
]

ALL_FOLDERS = [
    DATA_PATH,
    LOGS_PATH,
    ARDU_LOGS_PATH,
    RUNTIME_GAZEBO_MODELS,
    RUNTIME_GAZEBO_WORLDS,
]


def kill_processes(victims: list[str], wait_timeout: float = 2.0) -> None:
    """Kill all related processes and wait until they are gone."""
    for process in victims:
        subprocess.run(["pkill", "-9", "-f", process], check=False)

    deadline = time.monotonic() + wait_timeout
    while time.monotonic() < deadline:
        still_alive = [
            p for p in victims
            if subprocess.run(["pgrep", "-f", p], capture_output=True).returncode == 0
        ]
        if not still_alive:
            break
        time.sleep(0.05)


def clean_adsb_ptys() -> None:
    """Remove stale socat PTY symlinks from previous runs."""
    for path in glob.glob("/tmp/adsb_*"):
        with contextlib.suppress(OSError):
            os.unlink(path)


def del_folder(path: Path):
    """Ensure a clean folder by deleting and recreating it."""
    if path.exists():
        shutil.rmtree(path)


def _close_oracles() -> None:
    """Close all active Oracle ZMQ contexts in the current kernel."""
    from simulator.oracle import _active  # avoid circular import at module level
    for oracle in list(_active):
        oracle.close()


def _kill_stale_sim_sockets(
    base: int = 5760, span: int = 20, extra_ports: tuple[int, ...] = (11345,)
) -> None:
    """Kill any process holding a TCP port in the simulation range.

    SITL binds sequential ports starting at base (SERIAL0–SERIAL9). If a
    previous Oracle ZMQ socket in another kernel holds e.g. port base+5
    (RID_DOWN / SERIAL5), SITL will fail to bind it and exit.  `fuser -k`
    works without root on processes owned by the same user.

    ``extra_ports`` covers non-contiguous ports outside that range. The Gazebo
    master binds 11345; a stale ``gzserver`` left there makes the next Gazebo
    launch abort with "Address already in use", so it is freed here too.
    """
    ports = [f"{port}/tcp" for port in range(base, base + span)]
    ports += [f"{port}/tcp" for port in extra_ports]
    subprocess.run(["fuser", "-k", "-KILL", *ports], capture_output=True)
    time.sleep(0.5)


def clean(
    victim_processes: list[str] = ALL_PROCESSES,
    del_folders: list[Path] | None = None,
    reset_folders: list[Path] = ALL_FOLDERS,
) -> None:
    """End the simulation and free all ports it used."""
    if del_folders is None:
        del_folders = []
    _close_oracles()
    kill_processes(victim_processes)
    _kill_stale_sim_sockets()
    clean_adsb_ptys()
    for folder in reset_folders + del_folders:
        del_folder(folder)
    for folder in reset_folders:
        folder.mkdir(parents=True, exist_ok=True)
