"""Multi-UAV MAVLink Proxy"""

# Third Party imports
import argparse
import time

from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2

# First Party imports
from config import BasePort
from plan.planner import State
from vehicle_logic import VehicleLogic


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


def create_connection(base_port: int, idx: int, is_in: bool = True):
    """Create and in or out connection and wait for geting the hearbeat in"""
    port = base_port + 10 * idx
    if is_in:
        conn = mavutil.mavlink_connection(f"udp:127.0.0.1:{port}")
        conn.wait_heartbeat()
    else:
        conn = mavutil.mavlink_connection(f"udpout:127.0.0.1:{port}")
    return conn


def start_proxy(system_id):
    """Start bidirectional proxy for a given UAV system_id"""
    i = system_id - 1
    ap_conn = create_connection(base_port=BasePort.ARP, idx=i, is_in=True)
    cs_conn = create_connection(base_port=BasePort.GCS, idx=i, is_in=False)
    oc_conn = create_connection(base_port=BasePort.ORC, idx=i, is_in=False)

    print(f"\n🔁 Starting Vehicle {system_id}")
    logic = VehicleLogic(ap_conn)

    try:
        while True:
            # ← VEH
            msg = ap_conn.recv_msg()
            if msg:
                msg_type = msg.get_type()
                # GCS ←
                print(f"⬅️ GCS ← VEH {system_id}: {msg_type}")
                cs_conn.mav.send(msg)
                # ORC ←
                print(f"⬅️ ORC ← VEH {system_id}: {msg_type}")
                oc_conn.mav.send(msg)

            # GCS →
            msg = cs_conn.recv_msg()
            if msg:
                # → VEH
                print(f"➡️ GCS → VEH {system_id}: {msg.get_type()}")
                ap_conn.mav.send(msg)

            # ORC →
            msg = oc_conn.recv_msg()
            if msg:
                # → UAV
                print(f"➡️ ORC → VEH {system_id}: {msg.get_type()}")
                ap_conn.mav.send(msg)

            if logic.plan.state == State.DONE:
                send_done_until_ack(oc_conn, system_id)
                break
            logic.act()
    finally:
        cs_conn.close()
        ap_conn.close()
        oc_conn.close()
        print(f"✅ Process for uav {system_id} logic closed.")


if __name__ == "__main__":
    sysid = parse_arguments()
    print(f"System id: {sysid}")
    start_proxy(system_id=sysid)
