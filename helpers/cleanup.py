import os
from config import LOGS_PATH
import shutil

ALL_PROCESSES = [
    "QGroundControl",
    "arducopter",
    "gazebo",
    "mavproxy",
    "proxy.py",
]


def kill_processes(processes="all"):
    """Kill all the related processes"""
    if processes == "all":
        processes = ALL_PROCESSES
    for process in processes:
        os.system(f"pkill -9 -f {process}")


def clean(processes="all", sim_out=True):
    """It ends the simulation"""
    kill_processes(processes)
    if sim_out and LOGS_PATH.exists():
        shutil.rmtree(LOGS_PATH)
