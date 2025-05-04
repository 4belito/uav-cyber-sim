"""Ground Control Station - Modular Version"""

import ast
import argparse
import time
import numpy as np
import matplotlib.pyplot as plt
from pymavlink import mavutil
import subprocess
import platform
from base_ports import GCS_BASE_PORT

SPC_DIM = 3
COLORS = ["green", "blue", "red"]


def square_path(origin, h, s):
    """create and square path from a home position an altitud for the flight and the side of the square"""
    wps = np.array(
        [[0, 0, 0], [0, 0, h], [0, s, h], [s, s, h], [s, 0, h], [0, 0, h], [0, 0, 0]],
        dtype=float,
    )
    return np.array(origin) + wps


def parse_arguments():
    """parse arguments"""
    parser = argparse.ArgumentParser(description="Multi-UAV waypoint sender")
    parser.add_argument(
        "--plans",
        type=str,
        required=True,
        help="List of square plans as a string, e.g., '[[(0,0,0),5,3],[(0,3,0),3,2]]'",
    )
    return ast.literal_eval(parser.parse_args().plans)


def compute_plot_bounds(pls):
    """compute plot bounds"""
    min_v = [float("inf")] * SPC_DIM
    max_v = [float("-inf")] * SPC_DIM
    for home, height, radius in pls:
        add = [radius, radius, height]
        for i in range(SPC_DIM):
            min_v[i] = min(min_v[i], home[i])
            max_v[i] = max(max_v[i], home[i] + add[i])
    return min_v, max_v


def setup_plot(min_v, max_v):
    """create the plot"""
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlim(min_v[0], max_v[0])
    ax.set_ylim(min_v[1], max_v[1])
    ax.set_zlim(min_v[2], max_v[2])
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("UAV Position")
    return fig, ax


def connect_uavs(n_uavs):
    """connect to uav"""
    conns = []
    for i in range(n_uavs):
        port = GCS_BASE_PORT + 10 * i
        conn = mavutil.mavlink_connection(f"udp:127.0.0.1:{port}", source_system=255)
        conn.wait_heartbeat()
        print(
            f"✅ GCS received System {conn.target_system} Heartbeat. Connected on port {port}."
        )
        conns.append(conn)
    return conns


def send_waypoints(pls, conns):
    """send waipoints to uavs"""
    for i, conn in enumerate(conns):
        home, height, radius = pls[i]
        waypoints = square_path(home, height, radius)
        for j, (x, y, z) in enumerate(waypoints):
            conn.mav.mission_item_send(
                1,
                0,
                j,
                mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0,
                1,
                0,
                0,
                0,
                0,
                x,
                y,
                z,
            )
            print(f"📤 UAV {conn.target_system} WP {j}: ({x:.1f},{y:.1f},{z:.1f})")
            time.sleep(0.5)


def receive_and_plot(conns, ax):
    """receive positions and plot"""
    while conns:
        for conn in conns.copy():
            i = conn.target_system
            msg = conn.recv_match(type="MISSION_ITEM", blocking=True, timeout=0.01)
            if msg:
                if msg.z == -1:
                    conns.remove(conn)
                else:
                    print(f"✅ Pos from uav {i}: ({msg.x:.2f},{msg.y:.2f},{msg.z:.2f})")
                    ax.scatter(msg.x, msg.y, msg.z, c=COLORS[i - 1], s=50, alpha=0.3)
                    plt.draw()
                    plt.pause(0.1)


def launch_proxies(n_uavs, visible=True):
    """
    Launch one MAVLink proxy process per UAV.

    Args:
        n_uavs (int): Number of UAVs
        visible (bool): If True, open each proxy in a new terminal window
    """
    procs = []
    for i in range(n_uavs):
        sysid = i + 1
        command = f"python3 proxy.py --sysid {sysid}"

        if visible:
            if platform.system() == "Darwin":  # macOS
                p = subprocess.Popen(
                    ["osascript", "-e", f'tell app "Terminal" to do script "{command}"']
                )
            elif platform.system() == "Linux":
                p = subprocess.Popen(
                    ["gnome-terminal", "--", "bash", "-c", f"{command}; exec bash"]
                )
            else:
                raise OSError("Unsupported OS for visible terminal mode.")
        else:
            # Background mode (silent, attachable)
            p = subprocess.Popen(command.split())

        print(f"🚀 Started proxy for UAV {sysid} (PID {p.pid})")
        procs.append(p)
    return procs


def terminate_proxies(procs):
    """Terminate all running proxy subprocesses"""
    for p in procs:
        p.terminate()
    print("🧹 All proxies terminated.")


if __name__ == "__main__":
    plans = parse_arguments()
    number_of_uavs = len(plans)
    min_values, max_vavlues = compute_plot_bounds(plans)
    _, axis = setup_plot(min_values, max_vavlues)
    proxy_procs = launch_proxies(number_of_uavs)
    connections = connect_uavs(number_of_uavs)
    send_waypoints(plans, connections)
    receive_and_plot(connections, axis)
    time.sleep(3)
    terminate_proxies(proxy_procs)
