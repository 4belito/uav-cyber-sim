"""Uav Logic(Ardupilot Substitute)"""

import time
import argparse
import numpy as np
import matplotlib.pyplot as plt
from pymavlink import mavutil

from base_ports import UAV_BASE_PORT


def parse_args():
    """parse"""
    parser = argparse.ArgumentParser(description="Vehicle Logic (PoC).")
    parser.add_argument("--sysid", type=int, help="System Index", default=1)
    return parser.parse_args()


def setup_plot():
    """stup plot"""
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("UAV Waypoints")
    return fig, ax


def setup_connection(sysid):
    """setup connection"""
    port = UAV_BASE_PORT + 10 * (sysid - 1)
    conn = mavutil.mavlink_connection(f"udpout:127.0.0.1:{port}", source_system=sysid)
    print("📤 UAV: Sending heartbeat to GCS...")
    return conn


def send_heartbeat(conn):
    """send hear beat"""
    conn.mav.heartbeat_send(
        type=mavutil.mavlink.MAV_TYPE_QUADROTOR,
        autopilot=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode=0,
        custom_mode=0,
        system_status=mavutil.mavlink.MAV_STATE_ACTIVE,
    )


def receive_waypoint(conn, wps, ax, color):
    """receive waypoints"""
    msg = conn.recv_match(type="MISSION_ITEM", blocking=False)
    if msg:
        wps.append((msg.x, msg.y, msg.z))
        print(
            f"✅ UAV {conn.target_system} Received WP: ({msg.x:.1f},{msg.y:.1f},{msg.z:.1f})"
        )
        ax.scatter(msg.x, msg.y, msg.z, c=color, s=50, alpha=0.3)
        plt.draw()
        plt.pause(0.1)


def send_position(conn, pos):
    """send position"""
    print(f"📤 UAV Sending position ({pos[0]:.1f},{pos[1]:.1f},{pos[2]:.1f}) to GCS.")
    conn.mav.mission_item_send(
        1,
        0,
        99,
        mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0,
        1,
        0,
        0,
        0,
        0,
        pos[0],
        pos[1],
        pos[2],
    )


def distance(x: tuple, y: tuple):
    """Compute Euclidean Distance between x and y tuples"""
    return np.linalg.norm(np.array(x) - np.array(y))


def update(pos: tuple, wp1: tuple, wp2: tuple, step: float = 0.1):
    """update position"""
    new_pos = np.array(pos) + step * (np.array(wp2) - np.array(wp1))
    return tuple(new_pos)


def run_uav_logic(conn, ax, sysid=1):
    """uav logic"""
    colors = ["green", "blue", "red"]
    wps = []
    pos = None
    wp_idx = 0

    while True:
        send_heartbeat(conn)
        wps_n = len(wps)
        if wps_n < 2 or wps[-1] != wps[0]:
            receive_waypoint(conn, wps, ax, colors[sysid])
        else:
            pos = pos or wps[0]
            send_position(conn, pos)
            if wp_idx < wps_n - 1:
                pos = update(pos, wp1=wps[wp_idx], wp2=wps[wp_idx + 1], step=0.1)
                dist = distance(pos, wps[wp_idx + 1])
                print(f"distance: {dist}")
                if dist < 0.01:
                    wp_idx += 1
            else:
                # This tells the GCS that the mission is over
                send_position(conn, (0, 0, -1))
                time.sleep(5)
                break

        time.sleep(0.1)


if __name__ == "__main__":
    args = parse_args()
    print(f"🚁 System {args.sysid}")

    _, axis = setup_plot()
    connection = setup_connection(args.sysid)
    run_uav_logic(connection, axis, sysid=args.sysid)
