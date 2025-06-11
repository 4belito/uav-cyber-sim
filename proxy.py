"""Multi-UAV MAVLink Proxy."""

import argparse
import threading
import time
from queue import Queue

from pymavlink import mavutil  # type: ignore
from pymavlink.mavutil import mavlink_connection as connect  # type: ignore

# First Party imports
from config import BasePort
from helpers.mavlink import MavCmd, MAVConnection, MAVLinkMessage
from params.simulation import HEARTBEAT_PERIOD

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
            # print("✅ Heartbeat received")
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
        sysid: int,
        stop_event: threading.Event,
    ):
        super().__init__()
        self.source = source
        self.targets = targets
        self.labels = labels
        self.sysid = sysid
        self.stop_event = stop_event

    def run(self):
        while not self.stop_event.is_set():
            try:
                msg = self.source.recv_match(blocking=True, timeout=0.1)
                if msg:
                    self.dispatch_message(msg)
            except:
                self.stop_event.set()

    def dispatch_message(self, msg: MAVLinkMessage):
        msg_type = msg.get_type()
        msg_buff = msg.get_msgbuf()
        for q, label in zip(self.targets, self.labels):
            print(f"{label} {self.sysid}: {msg_type}")
            q.put(msg_buff)


def start_proxy(sysid: int, verbose: int = 1) -> None:
    """Start bidirectional proxy for a given UAV system_id."""
    i = sysid - 1

    ap_conn = create_connection_tcp(base_port=BasePort.ARP, idx=i)
    cs_conn = create_connection_udp(base_port=BasePort.GCS, idx=i)
    oc_conn = create_connection_udp(base_port=BasePort.ORC, idx=i)
    vh_conn = create_connection_tcp(base_port=BasePort.VEH, idx=i)

    ap_queue = Queue[bytes]()
    cs_queue = Queue[bytes]()
    oc_queue = Queue[bytes]()
    vh_queue = Queue[bytes]()
    print(f"\n🚀 Starting Proxy {sysid}")

    stop_event = threading.Event()

    # ARP → GCS + ORC + VEH
    router1 = MessageRouter(
        source=ap_conn,
        targets=[cs_queue, oc_queue, vh_queue],
        labels=["⬅️ GCS ← ARP", "⬅️ ORC ← ARP", "⬅️ VEH ← ARP"],
        sysid=sysid,
        stop_event=stop_event,
    )

    # GCS → ARP
    router2 = MessageRouter(
        source=cs_conn,
        targets=[ap_queue],
        labels=["➡️ GCS → ARP"],
        sysid=sysid,
        stop_event=stop_event,
    )

    # ORC → ARP
    router3 = MessageRouter(
        source=oc_conn,
        targets=[ap_queue],
        labels=["➡️ ORC → ARP"],
        sysid=sysid,
        stop_event=stop_event,
    )

    # VEH → ARP
    router4 = MessageRouter(
        source=vh_conn,
        targets=[ap_queue],
        labels=["➡️ VEH → ARP"],
        sysid=sysid,
        stop_event=stop_event,
    )

    try:
        router1.start()
        router2.start()
        router3.start()
        router4.start()

        while not stop_event.is_set():
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
        router1.join()
        router2.join()
        router3.join()
        router4.join()

        cs_conn.close()
        ap_conn.close()
        oc_conn.close()
        vh_conn.close()
        print(f"❎ Proxy {sysid} stopped.")


if __name__ == "__main__":
    main()
