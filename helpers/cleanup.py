import os


def kill_processes(
    processes=["QGroundControl", "sim_vehicle.py", "arducopter", "gazebo", "mavproxy"]
):
    for process in processes:
        os.system(f"pkill -9 -f {process}")
