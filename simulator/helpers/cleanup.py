"""Tools to stop simulation processes and clean up log files."""

import shutil
import subprocess
from pathlib import Path

from simulator.config import (
    ARDU_LOGS_PATH,
    DATA_PATH,
    LOGS_PATH,
    RUNTIME_GAZEBO_MODELS,
    RUNTIME_GAZEBO_WORLDS,
)

ALL_PROCESSES = [
    "QGroundControl",
    "arducopter",
    "gazebo",
    "gzserver",
    "gzclient",
    "mavproxy",
    "sim_vehicle.py",
    "socat",
    "adsb_injector.py",
    "logic.py",
    "gcs.py",
    "sim.py",
    "simulator.adsb_injector",
    "simulator.logic",
    "simulator.gcs",
    "exec bash",
]

ALL_FOLDERS = [
    DATA_PATH,
    LOGS_PATH,
    ARDU_LOGS_PATH,
    RUNTIME_GAZEBO_MODELS,
    RUNTIME_GAZEBO_WORLDS,
]


def kill_processes(victims: list[str]):
    """Kill all related processes or a given list of process names."""
    for process in victims:
        subprocess.run(["pkill", "-9", "-f", process], check=False)


def clean(
    victim_processes: list[str] = ALL_PROCESSES,
    del_folders: list[Path] = [],
    reset_folders: list[Path] = ALL_FOLDERS,
):
    """End the simulation."""
    kill_processes(victim_processes)
    for folder in reset_folders + del_folders:
        del_folder(folder)
    for folder in reset_folders:
        folder.mkdir(parents=True, exist_ok=True)


def del_folder(path: Path):
    """Ensure a clean folder by deleting and recreating it."""
    if path.exists():
        shutil.rmtree(path)
