"""Tools to stop simulation processes and clean up log files."""

import os
import shutil
from pathlib import Path
from typing import List

from config import DATA_PATH, LOGS_PATH

ALL_PROCESSES = [
    "QGroundControl",
    "arducopter",
    "gazebo",
    "mavproxy",
    "proxy.py",
    "run_many_uavs.py",
    "logic.py",
    "gcs.py",
    "run.py",
]


def kill_processes(victims: List[str]):
    """Kill all related processes or a given list of process names."""
    for process in victims:
        os.system(f"pkill -9 -f {process}")


def clean(
    victim_processes: List[str] = ALL_PROCESSES,
    del_folders: list[Path] = [],
    reset_folders: list[Path] = [DATA_PATH, LOGS_PATH],
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
