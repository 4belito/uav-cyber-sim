"""Tools to stop simulation processes and clean up log files."""

import os
import shutil
from typing import List, Literal

from config import LOGS_PATH

ALL_PROCESSES = [
    "QGroundControl",
    "arducopter",
    "gazebo",
    "mavproxy",
    "proxy.py",
    "gcs.py",
]

All = Literal["all"]


def kill_processes(victims: All | List[str] = "all"):
    """Kill all related processes or a given list of process names."""
    if victims == "all":
        victims = ALL_PROCESSES
    for process in victims:
        os.system(f"pkill -9 -f {process}")


def clean(victims: All | List[str] = "all", sim_out: bool = True):
    """End the simulation."""
    kill_processes(victims)
    if sim_out and LOGS_PATH.exists():
        shutil.rmtree(LOGS_PATH)
