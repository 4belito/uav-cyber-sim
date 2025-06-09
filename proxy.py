"""Multi-UAV MAVLink Proxy."""

import argparse
from queue import Queue
import subprocess
import time
from typing import List
import threading

from pymavlink import mavutil  # type: ignore
from pymavlink.dialects.v20 import common as mavlink2  # type: ignore
from pymavlink.mavutil import mavlink_connection as connect  # type: ignore

# First Party imports
from config import BasePort, Color
from helpers.mavlink import CustomCmd, MavCmd, MAVConnection, MAVLinkMessage
from params.simulation import HEARTBEAT_PERIOD
from plan import Plan

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


gcses = [
    ("blue 🟦", Color.BLUE),
    ("green 🟩", Color.GREEN),
    ("yellow 🟨", Color.YELLOW),
    # ("orange 🟧", Color.ORANGE),
    # ("red 🟥", Color.RED),
]
n_uavs_per_gcs = 5
side_len = 5
altitude = 5

n_gcs = len(gcses)
n_vehicles = n_gcs * n_uavs_per_gcs
offsets = [
    (i * 3 * side_len, j * 3 * side_len, 0, 0)
    for i in range(n_gcs)
    for j in range(n_uavs_per_gcs)
]

local_paths = [
    Plan.create_square_path(side_len=side_len, alt=altitude) for _ in range(n_vehicles)
]
plans = [Plan.basic(wps=path, wp_margin=0.5) for path in local_paths]


homes = [offset[:3] for offset in offsets]


# offset = (0, 0, 0, 0)  # east, north, up, heading
# local_path = Plan.create_square_path(side_len=5, alt=5)
# plans = [Plan.basic(wps=local_path, wp_margin=0.5)]
# homes = [offset[:3]]


########################################


heartbeat_period = mavutil.periodic_event(HEARTBEAT_PERIOD)


def main() -> None:
    """Parse arguments and launch the MAVLink proxy."""
    system_id = parse_arguments()
    print(f"System id: {system_id}")
    start_proxy(system_id, verbose=2)


def parse_arguments() -> int:
    """Parse a single system ID."""
    parser = argparse.ArgumentParser(description="Single UAV MAVLink Proxy")
    parser.add_argument(
        "--sysid",
        type=int,
        required=True,
        help="System ID of the UAV (e.g., 1)",
    )
    return parser.parse_args().sysid


# taken from mavproxy
def send_heartbeat(conn: MAVConnection) -> None:
    """Send a GCS heartbeat message to the UAV."""
    conn.mav.heartbeat_send(MavCmd.TYPE_GCS, MavCmd.AUTOPILOT_INVALID, 0, 0, 0)


def create_connection_udp(
    base_port: int, idx: int, is_input: bool = False
) -> MAVConnection:
    """Create and in or out connection and wait for geting the hearbeat in."""
    port = base_port + 10 * idx
    if is_input:
        conn: MAVConnection = connect(f"udp:127.0.0.1:{port}")  # type: ignore
        conn.wait_heartbeat()
    else:
        conn: MAVConnection = connect(f"udpout:127.0.0.1:{port}")  # type: ignore
        # send_heartbeat(conn)
    return conn


def create_connection_tcp(base_port: int, idx: int, retries: int = 5) -> MAVConnection:
    """Create and in or out connection and wait for geting the hearbeat in."""
    port = base_port + 10 * idx
    for attempt in range(retries):
        try:
            conn: MAVConnection = connect(f"tcp:127.0.0.1:{port}")  # type: ignore
            # send_heartbeat(conn)
            # conn.wait_heartbeat()
            print("✅ Heartbeat received")
            return conn
        except (ConnectionError, TimeoutError) as e:
            print(f"Retry {attempt + 1}/{retries} failed: {e}")
            time.sleep(0.1)
    raise RuntimeError("Failed to connect to ArduPilot via TCP")


class MessageRouter(threading.Thread):
    def __init__(
        self,
        source: MAVConnection,
        targets: list[Queue[bytes]],
        labels: list[str],
        sysid: int = 1,
    ):
        super().__init__()
        self.source = source
        self.targets = targets
        self.labels = labels
        self.sysid = sysid
        self.stop_event = threading.Event()

    def run(self):
        while not self.stop_event.is_set():
            try:
                msg: MAVLinkMessage | None = self.source.recv_match(
                    blocking=True, timeout=0.1
                )  # type: ignore
                if msg:
                    self.dispatch_message(msg)
            except:
                pass

    def stop(self):
        self.stop_event.set()

    def dispatch_message(self, msg: MAVLinkMessage):
        msg_type = msg.get_type()
        msg_buff = msg.get_msgbuf()
        for q, label in zip(self.targets, self.labels):
            print(f"{label} {self.sysid}: {msg_type}")
            q.put(msg_buff)
            # conn.write(msg_buff)


def route_message(
    source: MAVConnection,
    targets: List[MAVConnection],
    labels: List[str],
    sysid: int = 1,
) -> None:
    """
    Receive a MAVLink message from the source and forwards it to the target
    connections.
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
    """Forward a given MAVLink message to the target connections with logging."""
    msg_type = msg.get_type()
    msg_buff = msg.get_msgbuf()
    for conn, label in zip(targets, labels):
        print(f"{label} {sysid}: {msg_type}")
        conn.write(msg_buff)


def start_proxy(sysid: int, verbose: int = 1) -> None:
    """Start bidirectional proxy for a given UAV system_id."""
    i = sysid - 1

    # TODO: probably should start the logic from sim.py
    # this would also remove the need to hard-code the plan in logic.py
    subprocess.Popen(
        [
            # "gnome-terminal",
            # "--",
            "bash",
            "-c",
            f"source ~/.bashrc; conda activate uav-cyber-sim11; python3 logic.py --sysid {sysid}; exec bash",
        ]
    )
    ap_conn = create_connection_tcp(base_port=BasePort.ARP, idx=i)
    cs_conn = create_connection_udp(base_port=BasePort.GCS, idx=i)
    oc_conn = create_connection_udp(base_port=BasePort.ORC, idx=i)
    vh_conn = create_connection_tcp(base_port=5000, idx=i)
    ap_queue = Queue[bytes]()
    cs_queue = Queue[bytes]()
    oc_queue = Queue[bytes]()
    vh_queue = Queue[bytes]()
    print(f"\n🚀 Starting Vehicle {sysid}")

    # ARP → GCS + ORC + VEH
    router1 = MessageRouter(
        source=ap_conn,
        targets=[cs_queue, oc_queue, vh_queue],
        labels=["⬅️ GCS ← ARP", "⬅️ ORC ← ARP", "⬅️ VEH ← ARP"],
        sysid=sysid,
    )

    # GCS → ARP
    router2 = MessageRouter(
        source=cs_conn,
        targets=[ap_queue],
        labels=["➡️ GCS → ARP"],
        sysid=sysid,
    )

    # ORC → ARP
    router3 = MessageRouter(
        source=oc_conn,
        targets=[ap_queue],
        labels=["➡️ ORC → ARP"],
        sysid=sysid,
    )

    # VEH → ARP
    router4 = MessageRouter(
        source=vh_conn, targets=[ap_queue], labels=["⬅️ ARP ← VEH"], sysid=sysid
    )

    try:
        router1.start()
        router2.start()
        router3.start()
        router4.start()

        while True:
            while not oc_queue.empty():
                oc_conn.write(oc_queue.get())

            while not cs_queue.empty():
                cs_conn.write(cs_queue.get())

            while not ap_queue.empty():
                ap_conn.write(ap_queue.get())

            while not vh_queue.empty():
                vh_conn.write(vh_queue.get())

            time.sleep(0.01)
    finally:
        router1.stop()
        router2.stop()
        router3.stop()
        router4.stop()

        router1.join()
        router2.join()
        router3.join()
        router4.join()

        cs_conn.close()
        ap_conn.close()
        oc_conn.close()
        vh_conn.close()
        print(f"❎ Vehicle {sysid} logic stopped.")


def send_done_until_ack(conn: MAVConnection, idx: int, max_attempts: int = 100) -> None:
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
            if ack and ack.command == CustomCmd.PLAN_DONE:
                print("✅ ACK received. DONE message acknowledged.")
                return
            time.sleep(0.001)

    print("⚠️ No ACK received after max attempts.")


if __name__ == "__main__":
    main()
