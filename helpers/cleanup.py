"""
Tools to stop simulation processes and clean up log files.
"""

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
]

All = Literal["all"]


def kill_processes(processes: All | List[str] = "all"):
    """Kill all related processes or a given list of process names."""
    if processes == "all":
        processes = ALL_PROCESSES
    for process in processes:
        os.system(f"pkill -9 -f {process}")


def clean(processes: All | List[str] = "all", sim_out: bool = True):
    """It ends the simulation"""
    kill_processes(processes)
    if sim_out and LOGS_PATH.exists():
        shutil.rmtree(LOGS_PATH)
