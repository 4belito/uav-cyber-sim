"""Tools to stop simulation processes and clean up log files."""

import os
import shutil
from pathlib import Path
from typing import List, Literal

from config import LOGS_PATH

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


def delete_missions():
    """Delete all mission files."""
    mission_path = os.path.join("plan", "missions")
    for file in os.listdir(mission_path):
        if file.startswith("mission_") and file.endswith(".waypoints"):
            os.remove(os.path.join(mission_path, file))


def clean(victims: All | List[str] = "all", sim_out: bool = True):
    """End the simulation."""
    kill_processes(victims)
    delete_logs()
    delete_missions()
    if sim_out and LOGS_PATH.exists():
        shutil.rmtree(LOGS_PATH)


def reset_folder(path: str | Path):
    """Ensure a clean folder by deleting and recreating it."""
    path = Path(path)
    if path.exists():
        shutil.rmtree(path)
    path.mkdir()
