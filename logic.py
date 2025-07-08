"""Multi-UAV MAVLink Proxy."""

# Third Party imports
import argparse
import time

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink

# First Party imports
from config import BasePort
from mavlink.customtypes.connection import MAVConnection
from mavlink.enums import Autopilot, Type
from mavlink.util import CustomCmd, connect
from params.simulation import HEARTBEAT_PERIOD
from plan import Plan, State
from proxy import create_connection_udp
from vehicle_logic import VehicleLogic

######################################################################
# NOTE: The plans have to only be hardcoded when using the GUIDED mode.
mission = True
if not mission:
    plans = [
        Plan.auto(name="square_auto", mission_name=f"square_{i + 1}") for i in range(5)
    ]
########################################
# TODO: Refactor this module

heartbeat_period = mavutil.periodic_event(HEARTBEAT_PERIOD)


def main():
    """Entry point for the Multi-UAV MAVLink Proxy."""
    system_id, port_offset, verbose = parse_arguments()
    print(f"System id: {system_id}")
    start_proxy(system_id, port_offset, verbose=verbose or 1)


def parse_arguments() -> tuple[int, int, int | None]:
    """Parse a single system ID."""
    parser = argparse.ArgumentParser(description="Single UAV MAVLink Proxy")
    parser.add_argument(
        "--sysid",
        type=int,
        required=True,
        help="System ID of the UAV (e.g., 1)",
    )
    parser.add_argument(
        "--verbose",
        type=int,
        required=False,
        help="verbose value (0,1,2)",
    )
    parser.add_argument(
        "--port-offset", type=int, required=True, help="Port offset to use (e.g. 10)"
    )
    args = parser.parse_args()
    return (args.sysid, args.port_offset, args.verbose)


# taken from mavproxy
def send_heartbeat(conn: MAVConnection) -> None:
    """Send a GCS heartbeat message to the UAV."""
    conn.mav.heartbeat_send(Type.GCS, Autopilot.INVALID, 0, 0, 0)


def create_connection_tcp(base_port: int, offset: int) -> MAVConnection:
    """Create and in or out connection and wait for geting the hearbeat in."""
    port = base_port + offset
    conn = connect(f"tcpin:127.0.0.1:{port}")
    conn.wait_heartbeat()
    send_heartbeat(conn)
    print("✅ Heartbeat received")
    return conn


def start_proxy(sysid: int, port_offset: int, verbose: int = 1):
    """Start bidirectional proxy for a given UAV system_id."""
    i = sysid - 1
    ap_conn = create_connection_tcp(base_port=BasePort.VEH, offset=port_offset)
    cs_conn = create_connection_udp(base_port=BasePort.GCS, offset=port_offset)
    oc_conn = create_connection_udp(base_port=BasePort.ORC, offset=port_offset)

    print(f"\n🚀 Starting Vehicle {sysid} logic")
    logic = VehicleLogic(
        ap_conn,
        plan=(
            Plan.auto(name="auto", mission_name=f"mission_{sysid}")
            if mission
            else plans[i]
        ),
        verbose=verbose,
    )

    try:
        while True:
            if heartbeat_period.trigger():
                send_heartbeat(ap_conn)

            if logic.plan.state == State.DONE:
                send_done_until_ack(oc_conn, sysid)
                send_done_until_ack(cs_conn, sysid)
                break
            logic.act()
            time.sleep(0.01)
    finally:
        cs_conn.close()
        ap_conn.close()
        oc_conn.close()
        print(f"❎ Vehicle {sysid} logic stopped.")


def send_done_until_ack(conn: MAVConnection, idx: int, max_tries: float = float("inf")):
    """
    Send 'DONE' via STATUSTEXT repeatedly until receiving a COMMAND_ACK.
    Assumes `conn` is a dedicated MAVLink connection for one UAV.
    """
    msg = mavlink.MAVLink_statustext_message(severity=6, text=b"DONE")
    i = 0
    while i < max_tries:
        print(f"📤 GCS ← UAV {idx}: Sending DONE (attempt {i + 1})")
        conn.mav.send(msg)

        start = time.time()
        while time.time() - start < 0.05:
            ack = conn.recv_match(type="COMMAND_ACK", blocking=False)
            if ack and ack.command == CustomCmd.PLAN_DONE:
                print("✅ ACK received. DONE message acknowledged.")
                return
            time.sleep(0.001)
        i += 1

    print("⚠️ No ACK received after max attempts.")


if __name__ == "__main__":
    main()
