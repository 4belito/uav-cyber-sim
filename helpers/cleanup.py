import os

ALL_PROCESSES = [
    "QGroundControl",
    "sim_vehicle.py",
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
