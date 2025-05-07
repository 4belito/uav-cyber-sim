"""Multi-UAV MAVLink Proxy"""

import argparse

from config import VEH_BASE_PORT, GCS_BASE_PORT, ORC_BASE_PORT
from vehicle_logic import VehicleLogic
from plan.planner import State
import time

## this is temporal
# from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2


def send_done_until_ack(conn, idx, command_id=3000, max_attempts=1000):
    """
    Send 'DONE' via STATUSTEXT repeatedly until receiving a COMMAND_ACK.
    Assumes `conn` is a dedicated MAVLink connection for one UAV.
    """
    msg = mavlink2.MAVLink_statustext_message(severity=6, text=b"DONE")

    for attempt in range(max_attempts):
        print(f"📤 GCS ← UAV {idx}: Sending DONE (attempt {attempt + 1})")
        conn.mav.send(msg)

        start = time.time()
        while time.time() - start < 0.05:
            ack = conn.recv_match(type="COMMAND_ACK", blocking=False)
            if ack and ack.command == command_id:
                print("✅ ACK received. DONE message acknowledged.")
                return
            time.sleep(0.001)

    print("⚠️ No ACK received after max attempts.")


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
    uav_port = VEH_BASE_PORT + 10 * i
    gcs_port = GCS_BASE_PORT + 10 * i
    orc_port = ORC_BASE_PORT + 10 * i
    return gcs_port, uav_port, orc_port


def start_proxy(idx):
    """Start bidirectional proxy for a given UAV sysid"""
    # gcs_port, uav_port = get_ports(idx)

    print(f"\n🔁 Starting UAV {idx}")

    logic = VehicleLogic(sys_id=idx)

    try:
        while True:
            # UAV →
            msg = logic.ap_conn.recv_msg()
            if msg:
                # → GCS
                print(f"⬅️ GCS ← UAV {idx}: {msg.get_type()}")
                logic.cs_conn.mav.send(msg)
                # → ORC
                print(f"⬅️ ORC ← UAV {idx}: {msg.get_type()}")
                logic.oc_conn.mav.send(msg)

            # GCS →
            msg = logic.cs_conn.recv_msg()
            if msg:
                # → UAV
                print(f"➡️ UAV {idx} ← GCS: {msg.get_type()}")
                logic.ap_conn.mav.send(msg)

            # ORC →
            msg = logic.oc_conn.recv_msg()
            if msg:
                # → UAV
                print(f"➡️ UAV {idx} ← ORC: {msg.get_type()}")
                logic.ap_conn.mav.send(msg)

            if logic.plan.state == State.DONE:
                send_done_until_ack(logic.oc_conn, idx)
                break
            else:
                logic.act()
    finally:
        logic.cs_conn.close()
        logic.ap_conn.close()
        logic.oc_conn.close()
        print(f"✅ Process for uav {idx} logic closed.")


if __name__ == "__main__":
    sysid = parse_arguments()
    start_proxy(sysid)
