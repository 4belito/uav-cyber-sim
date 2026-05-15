"""Multi-UAV MAVLink Proxy"""

import argparse
import time

from base_ports import GCS_BASE_PORT, UAV_BASE_PORT
from pymavlink import mavutil


def parse_arguments():
    """Parse a single system ID"""
    parser = argparse.ArgumentParser(description="Single UAV MAVLink Proxy")
    parser.add_argument(
        "--sysid",
        type=int,
        required=True,
        help="System ID of the UAV (e.g., 1)",
    )
    return parser.parse_args().sysid


def get_ports(idx):
    """Return GCS and UAV ports for a given sysid"""
    i = idx - 1
    uav_port = UAV_BASE_PORT + 10 * i
    gcs_port = GCS_BASE_PORT + 10 * i
    return gcs_port, uav_port


def start_proxy(idx):
    """Start bidirectional proxy for a given UAV sysid"""
    gcs_port, uav_port = get_ports(idx)

    print(f"\n🔁 Starting proxy for UAV {idx}")
    print(f"📡 GCS listens on port {gcs_port}")
    print(f"🛩️ Forwarding to UAV on port {uav_port}")

    gcs = mavutil.mavlink_connection(f"udpout:127.0.0.1:{gcs_port}", source_system=idx)
    uav = mavutil.mavlink_connection(f"udp:127.0.0.1:{uav_port}")

    try:
        while True:
            # GCS → UAV
            msg = gcs.recv_msg()
            if msg:
                print(f"➡️  UAV {idx} ← GCS: {msg.get_type()}")
                uav.mav.send(msg)

            # UAV → GCS
            msg = uav.recv_msg()
            if msg:
                print(f"⬅️  GCS ← UAV {idx}: {msg.get_type()}")
                gcs.mav.send(msg)

                if msg.get_type() == "MISSION_ITEM" and msg.z == -1:
                    print(f"🛑 UAV {idx} mission complete. Closing proxy.")
                    break
    finally:
        gcs.close()
        uav.close()
        print(f"✅ Proxy for UAV {idx} closed.")


if __name__ == "__main__":
    sysid = parse_arguments()
    start_proxy(sysid)
