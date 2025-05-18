"""Multi-UAV MAVLink Proxy"""

# Third Party imports
import argparse
import time
import socket
from select import select

from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2

# First Party imports
from config import BasePort
from plan.planner import State
from vehicle_logic import VehicleLogic

heartbeat_period = mavutil.periodic_event(1)


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


# taken from mavproxy
def send_heartbeat(master):
    if master.mavlink10():
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0
        )
    else:
        MAV_GROUND = 5
        MAV_AUTOPILOT_NONE = 4
        master.mav.heartbeat_send(MAV_GROUND, MAV_AUTOPILOT_NONE)


def create_connection_udp(base_port: int, idx: int, is_input: bool = False):
    """Create and in or out connection and wait for geting the hearbeat in"""
    port = base_port + 10 * idx
    if is_input:
        conn = mavutil.mavlink_connection(f"udp:127.0.0.1:{port}")
        conn.wait_heartbeat()
    else:
        conn = mavutil.mavlink_connection(f"udpout:127.0.0.1:{port}")
    return conn


def create_connection_tcp(base_port: int, idx: int, retries: int = 5):
    """Create and in or out connection and wait for geting the hearbeat in"""
    port = base_port + 10 * idx
    for attempt in range(retries):
        try:
            conn = mavutil.mavlink_connection(f"tcp:127.0.0.1:{port}")
            send_heartbeat(conn)
            conn.wait_heartbeat(timeout=0.1)
            print("✅ Heartbeat received")
            return conn
        except Exception as e:
            print(f"Retry {attempt+1}/{retries} failed: {e}")
            time.sleep(0.1)
    raise RuntimeError("Failed to connect to ArduPilot via TCP")


def start_proxy(system_id: int):
    """Start bidirectional proxy for a given UAV system_id"""
    i = system_id - 1
    ap_conn = create_connection_tcp(base_port=BasePort.ARP, idx=i)
    cs_conn = create_connection_udp(base_port=BasePort.GCS, idx=i)
    oc_conn = create_connection_udp(base_port=BasePort.ORC, idx=i)
    print(f"\n🔁 Starting Vehicle {system_id}")
    logic = VehicleLogic(ap_conn)
    print("ok-VehicleLogic")

    try:
        while True:
            if heartbeat_period.trigger():
                send_heartbeat(ap_conn)

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


if __name__ == "__main__":
    sysid = parse_arguments()
    print(f"System id: {sysid}")
    start_proxy(system_id=sysid)
