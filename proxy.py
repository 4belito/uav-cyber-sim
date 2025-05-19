"""Multi-UAV MAVLink Proxy"""

# Third Party imports
import argparse
import time
from typing import List

from pymavlink import mavutil
from pymavlink.mavutil import mavlink_connection as connect
from pymavlink.dialects.v20 import common as mavlink2

# First Party imports
from config import BasePort
from helpers.mavlink import MAVConnection, MAVCommand, MAVLinkMessage
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
def send_heartbeat(conn: MAVConnection):
    """Send a GCS heartbeat message to the UAV."""
    conn.mav.heartbeat_send(MAVCommand.TYPE_GCS, MAVCommand.AUTOPILOT_INVALID, 0, 0, 0)


def create_connection_udp(base_port: int, idx: int, is_input: bool = False):
    """Create and in or out connection and wait for geting the hearbeat in"""
    port = base_port + 10 * idx
    if is_input:
        conn: MAVConnection = connect(f"udp:127.0.0.1:{port}")  # type: ignore
        conn.wait_heartbeat()
    else:
        conn: MAVConnection = connect(f"udpout:127.0.0.1:{port}")  # type: ignore
    return conn


def create_connection_tcp(base_port: int, idx: int, retries: int = 5):
    """Create and in or out connection and wait for geting the hearbeat in"""
    port = base_port + 10 * idx
    for attempt in range(retries):
        try:
            conn: MAVConnection = connect(f"tcp:127.0.0.1:{port}")  # type: ignore
            send_heartbeat(conn)
            conn.wait_heartbeat()
            print("✅ Heartbeat received")
            return conn
        except (ConnectionError, TimeoutError) as e:
            print(f"Retry {attempt+1}/{retries} failed: {e}")
            time.sleep(0.1)
    raise RuntimeError("Failed to connect to ArduPilot via TCP")


def route_message(
    source: MAVConnection,
    targets: List[MAVConnection],
    labels: List[str],
    sysid: int = 1,
) -> None:
    """
    Receives a MAVLink message from the source and forwards it to the target connections.
    """
    msg = source.recv_msg()
    if msg:
        dispatch_message(msg, targets, labels, sysid)


def dispatch_message(
    msg: MAVLinkMessage,
    targets: List[MAVConnection],
    labels: List[str],
    sysid: int = 1,
) -> None:
    """
    Forwards a given MAVLink message to the target connections with logging.
    """
    msg_type = msg.get_type()
    msg_buff = msg.get_msgbuf()
    for conn, label in zip(targets, labels):
        print(f"{label} {sysid}: {msg_type}")
        conn.write(msg_buff)


def start_proxy(sysid: int, verbose: int = 1):
    """Start bidirectional proxy for a given UAV system_id"""
    i = sysid - 1
    ap_conn = create_connection_tcp(base_port=BasePort.ARP, idx=i)
    cs_conn = create_connection_udp(base_port=BasePort.GCS, idx=i)
    oc_conn = create_connection_udp(base_port=BasePort.ORC, idx=i)
    print(f"\n🚀 Starting Vehicle {sysid}")
    logic = VehicleLogic(ap_conn, verbose=verbose)
    print("ok-VehicleLogic")

    try:
        while True:
            if heartbeat_period.trigger():
                send_heartbeat(ap_conn)

            # VEH → GCS + ORC
            route_message(
                source=ap_conn,
                targets=[cs_conn, oc_conn],
                labels=["⬅️ GCS ← VEH", "⬅️ ORC ← VEH"],
                sysid=sysid,
            )

            # GCS → VEH
            route_message(
                source=cs_conn,
                targets=[ap_conn],
                labels=["➡️ GCS → VEH"],
                sysid=sysid,
            )

            # ORC → VEH
            route_message(
                source=oc_conn,
                targets=[ap_conn],
                labels=["➡️ ORC → VEH"],
                sysid=sysid,
            )

            if logic.plan.state == State.DONE:
                send_done_until_ack(oc_conn, system_id)
                break
            logic.act()
    finally:
        cs_conn.close()
        ap_conn.close()
        oc_conn.close()
        print(f"❎ Vehicle {system_id} logic stopped.")


def send_done_until_ack(
    conn: MAVConnection, idx: int, command_id: int = 3000, max_attempts: int = 100
):
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
    system_id = parse_arguments()
    print(f"System id: {system_id}")
    start_proxy(system_id, verbose=2)
