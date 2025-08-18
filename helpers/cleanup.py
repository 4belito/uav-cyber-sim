"""Tools to stop simulation processes and clean up log files."""

import os
import shutil
from pathlib import Path
from typing import List, Literal

from config import ARDU_LOGS_PATH, DATA_PATH, LOGS_PATH

ALL_PROCESSES = [
    "QGroundControl",
    "arducopter",
    "gazebo",
    "mavproxy",
    "proxy.py",
    "run_many_uavs.py",
    "logic.py",
    "gcs.py",
]

All = Literal["all"]


def kill_processes(victims: All | List[str] = "all"):
    """Kill all related processes or a given list of process names."""
    if victims == "all":
        victims = ALL_PROCESSES
    for process in victims:
        os.system(f"pkill -9 -f {process}")


def delete_logs():
    """Delete all proxy log files."""
    for file in os.listdir():
        if file.startswith("proxy_") and file.endswith(".log"):
            os.remove(file)


def clean(victims: All | List[str] = "all", sim_out: bool = True):
    """End the simulation."""
    kill_processes(victims)
    reset_folder(DATA_PATH)
    reset_folder(LOGS_PATH)
    if sim_out and ARDU_LOGS_PATH.exists():
        shutil.rmtree(ARDU_LOGS_PATH)


def reset_folder(path: str | Path):
    """Ensure a clean folder by deleting and recreating it."""
    path = Path(path)
    if path.exists():
        shutil.rmtree(path)
    path.mkdir()
