"""Multi-UAV MAVLink Proxy."""

import _csv
import argparse
import csv
import json
import os
import threading
import time
from queue import Queue
from typing import TextIO

import pymavlink.dialects.v20.ardupilotmega as mavlink
from pymavlink import mavutil

# First Party imports
from config import DATA_PATH, BasePort
from mavlink.customtypes.connection import MAVConnection
from mavlink.enums import Autopilot, DataStream, Type
from mavlink.util import connect, request_sensor_streams
from params.simulation import HEARTBEAT_PERIOD

heartbeat_period = mavutil.periodic_event(HEARTBEAT_PERIOD)


DATA_STREAM_IDS = [
    DataStream.RAW_SENSORS,
    DataStream.EXTENDED_STATUS,
    DataStream.POSITION,
    DataStream.EXTRA1,
    DataStream.EXTRA2,
]


def main() -> None:
    """Parse arguments and launch the MAVLink proxy."""
    system_id, port_offset, verbose = parse_arguments()
    start_proxy(system_id, port_offset, verbose)


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


def create_connection_udp(
    base_port: int, offset: int, is_input: bool = False
) -> MAVConnection:
    """Create and in or out connection and wait for geting the hearbeat in."""
    port = base_port + offset
    if is_input:
        conn: MAVConnection = connect(f"udp:127.0.0.1:{port}")  # type: ignore
        conn.wait_heartbeat()
    else:
        conn: MAVConnection = connect(f"udpout:127.0.0.1:{port}")  # type: ignore
        # send_heartbeat(conn)
    return conn


def create_connection_tcp(
    base_port: int, offset: int, retries: int = 20
) -> MAVConnection:
    """Create and in or out connection and wait for geting the hearbeat in."""
    port = base_port + offset
    for attempt in range(retries):
        try:
            conn: MAVConnection = connect(f"tcp:127.0.0.1:{port}")  # type: ignore
            return conn
        except (ConnectionError, TimeoutError) as e:
            print(f"Retry {attempt + 1}/{retries} failed: {e}")
            time.sleep(0.1)
    raise RuntimeError("Failed to connect to ArduPilot via TCP")


class MessageRouter(threading.Thread):
    def __init__(
        self,
        source: MAVConnection,
        targets: list[Queue[tuple[str, float, mavlink.MAVLink_message]]],
        labels: list[str],
        sender: str,
        sysid: int,
        stop_event: threading.Event,
        verbose: int = 1,
    ):
        super().__init__()
        self.source = source
        self.targets = targets
        self.labels = labels
        self.sender = sender
        self.sysid = sysid
        self.stop_event = stop_event
        self.verbose = verbose

    def run(self):
        while not self.stop_event.is_set():
            try:
                msg = self.source.recv_match(blocking=True, timeout=0.1)
                if msg:
                    self.dispatch_message(msg)
            except:
                self.stop_event.set()

    def dispatch_message(self, msg: mavlink.MAVLink_message):
        time_received = time.time()
        for q, label in zip(self.targets, self.labels):
            if self.verbose == 3:
                print(f"{label} {self.sysid}: {msg.get_type()}")
            q.put((self.sender, time_received, msg))


def write_and_log_message(
    q: Queue[tuple[str, float, mavlink.MAVLink_message]],
    conn: MAVConnection,
    log_writer: _csv.Writer,
    recipient: str,
):
    """Write the next message from a queue to the connection and log it."""
    sender, time_received, msg = q.get()
    conn.write(bytes(msg.get_msgbuf()))
    if msg.get_type() != "BAD_DATA":
        log_writer.writerow(
            [
                sender,
                recipient,
                time_received,
                time.time(),
                msg.to_json(),
            ]
        )


def write_and_log_with_sensors(
    q: Queue[tuple[str, float, mavlink.MAVLink_message]],
    conn: MAVConnection,
    log_writer: _csv.Writer,
    recipient: str,
    sensor_logs: dict[str, TextIO],
    sysid: int,
):
    """Write and log proxy and sensor messages in one step."""
    sender, time_received, msg = q.get()
    conn.write(bytes(msg.get_msgbuf()))
    if msg.get_type() != "BAD_DATA":
        # Log general proxy traffic
        log_writer.writerow(
            [
                sender,
                recipient,
                time_received,
                time.time(),
                msg.to_json(),
            ]
        )
        # Also log per-sensor messages
        msg_type = msg.get_type()
        if msg_type in {"RAW_IMU", "SCALED_PRESSURE", "GPS_RAW_INT"}:
            log_line = json.dumps(
                {
                    "sysid": sysid,
                    "sender": sender,
                    "time_received": time_received,
                    "time_logged": time.time(),
                    "msg": msg.to_dict(),
                }
            )
            if msg_type not in sensor_logs:
                path = DATA_PATH / "sensor_logs" / f"sensor_{sysid}_{msg_type}.log"
                os.makedirs(path.parent, exist_ok=True)
                sensor_logs[msg_type] = open(path, "a")
            sensor_logs[msg_type].write(log_line + "\n")
            sensor_logs[msg_type].flush()


def start_proxy(sysid: int, port_offset: int, verbose: int = 1) -> None:
    """Start bidirectional proxy for a given UAV system_id."""
    ap_conn = create_connection_tcp(base_port=BasePort.ARP, offset=port_offset)
    cs_conn = create_connection_udp(base_port=BasePort.GCS, offset=port_offset)
    oc_conn = create_connection_udp(base_port=BasePort.ORC, offset=port_offset)
    vh_conn = create_connection_tcp(base_port=BasePort.VEH, offset=port_offset)
    request_sensor_streams(vh_conn, stream_ids=DATA_STREAM_IDS, rate_hz=5)
    ap_queue = Queue[tuple[str, float, mavlink.MAVLink_message]]()
    cs_queue = Queue[tuple[str, float, mavlink.MAVLink_message]]()
    oc_queue = Queue[tuple[str, float, mavlink.MAVLink_message]]()
    vh_queue = Queue[tuple[str, float, mavlink.MAVLink_message]]()
    print(f"🚀 Starting Proxy {sysid}")

    stop_event = threading.Event()

    # ARP → ORC + VEH  X(+GCS)
    router1 = MessageRouter(
        source=ap_conn,
        targets=[cs_queue, oc_queue, vh_queue],  #
        labels=["⬅️ GCS ← ARP", "⬅️ ORC ← ARP", "⬅️ VEH ← ARP"],
        sysid=sysid,
        sender="ARP",
        stop_event=stop_event,
        verbose=verbose,
    )

    # GCS → ARP
    router2 = MessageRouter(
        source=cs_conn,
        targets=[ap_queue],
        labels=["➡️ GCS → ARP"],
        sysid=sysid,
        sender="GCS",
        stop_event=stop_event,
        verbose=verbose,
    )

    # ORC → ARP
    router3 = MessageRouter(
        source=oc_conn,
        targets=[ap_queue],
        labels=["➡️ ORC → ARP"],
        sysid=sysid,
        sender="ORC",
        stop_event=stop_event,
        verbose=verbose,
    )

    # VEH → ARP
    router4 = MessageRouter(
        source=vh_conn,
        targets=[ap_queue],
        labels=["➡️ VEH → ARP"],
        sysid=sysid,
        sender="VEH",
        stop_event=stop_event,
        verbose=verbose,
    )

    log_file = open(DATA_PATH / f"proxy_{sysid}.log", "w")
    log_writer = csv.writer(log_file)
    log_writer.writerow(
        ["sender", "recipient", "time_received", "time_sent", "message"]
    )
    sensor_log_files: dict[str, TextIO] = {}
    try:
        router1.start()
        router2.start()
        router3.start()
        router4.start()

        while not stop_event.is_set():
            while not oc_queue.empty():
                write_and_log_message(oc_queue, oc_conn, log_writer, "ORC")

            while not cs_queue.empty():
                write_and_log_message(cs_queue, cs_conn, log_writer, "GCS")

            while not ap_queue.empty():
                write_and_log_message(ap_queue, ap_conn, log_writer, "ARP")

            while not vh_queue.empty():
                # write_and_log_message(vh_queue, vh_conn, log_writer, "VEH")
                write_and_log_with_sensors(
                    vh_queue, vh_conn, log_writer, "VEH", sensor_log_files, sysid
                )

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

        log_file.close()

        print(f"❎ Proxy {sysid} stopped.")


if __name__ == "__main__":
    main()
