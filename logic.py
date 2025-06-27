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

### Hardcoded for now as part of a step-by-step development process
########## 5 UAVs ####################
# offsets = [  # east, north, up, heading
#     (0.0, 0.0, 0.0, 0.0),
#     (10.0, 0.0, 0.0, 45.0),
#     (-5.0, -10.0, 0.0, 225.0),
#     (-15.0, 0.0, 0.0, 0.0),
#     (0.0, -20.0, 0.0, 0.0),
# ]
# n_vehicles = len(offsets)
# local_paths = [Plan.create_square_path(side_len=5, alt=5) for _ in range(n_vehicles)]
# plans = [Plan.basic(wps=path, wp_margin=0.5) for path in local_paths]


# gcses = [
#     ("blue 🟦", Color.BLUE),
#     ("green 🟩", Color.GREEN),
#     ("yellow 🟨", Color.YELLOW),
#     ("orange 🟧", Color.ORANGE),
#     ("red 🟥", Color.RED),
# ]
# n_uavs_per_gcs = 12
# side_len = 5
# altitude = 5

# n_gcs = len(gcses)
# n_vehicles = n_gcs * n_uavs_per_gcs
# offsets = [
#     (i * 10 * side_len, j * 3 * side_len, 0, 0)
#     for i in range(n_gcs)
#     for j in range(n_uavs_per_gcs)
# ]

# local_paths = [
#     Plan.create_square_path(side_len=side_len, alt=altitude)
#       for _ in range(n_vehicles)
# ]
# plans = [Plan.basic(wps=path, wp_margin=0.5) for path in local_paths]


# homes = [offset[:3] for offset in offsets]


# offset = ENUPose(0, 0, 0, 0)  # east, north, up, heading
# # rel_path = Plan.create_square_path(side_len=5, alt=5)
# # plans = [Plan.basic(wps=rel_path, wp_margin=0.5)]
# homes = [ENU(*offset[:3])]  # we dont need this


######################################################################
plans = [
    Plan.auto(name="square_auto", mission_name=f"square_{i + 1}") for i in range(5)
]
# plans = [Plan.auto(name="square_auto", mission_name="square")]
########################################
# TODO: Refactor this module

heartbeat_period = mavutil.periodic_event(HEARTBEAT_PERIOD)


def main():
    """Entry point for the Multi-UAV MAVLink Proxy."""
    system_id, port_offset, verbose = parse_arguments()
    print(f"System id: {system_id}")
    start_proxy(system_id, port_offset, verbose=verbose or 1)


def parse_arguments() -> tuple[int, int, int]:
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
    logic = VehicleLogic(ap_conn, plan=plans[i], verbose=verbose)

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


def send_done_until_ack(conn: MAVConnection, idx: int, max_attempts: int = 100):
    """
    Send 'DONE' via STATUSTEXT repeatedly until receiving a COMMAND_ACK.
    Assumes `conn` is a dedicated MAVLink connection for one UAV.
    """
    msg = mavlink.MAVLink_statustext_message(severity=6, text=b"DONE")

    for attempt in range(max_attempts):
        print(f"📤 GCS ← UAV {idx}: Sending DONE (attempt {attempt + 1})")
        conn.mav.send(msg)

        start = time.time()
        while time.time() - start < 0.05:
            ack = conn.recv_match(type="COMMAND_ACK", blocking=False)
            if ack and ack.command == CustomCmd.PLAN_DONE:
                print("✅ ACK received. DONE message acknowledged.")
                return
            time.sleep(0.001)

    print("⚠️ No ACK received after max attempts.")


if __name__ == "__main__":
    main()
