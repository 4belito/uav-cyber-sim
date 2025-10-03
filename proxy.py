"""Multi-UAV MAVLink Proxy."""

import _csv
import argparse
import csv
import json
import logging
import os
import threading
import time
from queue import Queue
from typing import TextIO

import pymavlink.dialects.v20.ardupilotmega as mavlink
import zmq
from pymavlink import mavutil

# First Party imports
from config import DATA_PATH, BasePort
from helpers.connections.mavlink.conn import create_tcp_conn, create_udp_conn
from helpers.connections.mavlink.customtypes.mavconn import MAVConnection
from helpers.connections.mavlink.enums import DataStream
from helpers.connections.mavlink.streams import request_sensor_streams
from helpers.connections.zeromq import create_zmq_socket
from helpers.setup_log import setup_logging
from params.simulation import HEARTBEAT_PERIOD

heartbeat_period = mavutil.periodic_event(HEARTBEAT_PERIOD)


DATA_STREAM_IDS = [
    DataStream.RAW_SENSORS,
    DataStream.EXTENDED_STATUS,
    DataStream.POSITION,
    DataStream.EXTRA1,
    DataStream.EXTRA2,
]
DATA_STREAM_RATE = 5


def main() -> None:
    """Parse arguments and launch the MAVLink proxy."""
    system_id, port_offset, verbose = parse_arguments()
    setup_logging(f"proxy_{system_id}", verbose=verbose, console_output=True)
    start_proxy(system_id, port_offset)


def start_proxy(sysid: int, port_offset: int) -> None:
    """Start bidirectional proxy for a given UAV system_id."""
    ap_conn = create_tcp_conn(
        base_port=BasePort.ARP, offset=port_offset, role="client", sysid=sysid
    )
    cs_conn = create_udp_conn(base_port=BasePort.GCS, offset=port_offset, mode="sender")
    vh_conn = create_tcp_conn(
        base_port=BasePort.VEH, offset=port_offset, role="client", sysid=sysid
    )
    request_sensor_streams(
        ap_conn, stream_ids=DATA_STREAM_IDS, rate_hz=DATA_STREAM_RATE
    )

    ap_queue = Queue[tuple[str, float, mavlink.MAVLink_message]]()
    cs_queue = Queue[tuple[str, float, mavlink.MAVLink_message]]()
    vh_queue = Queue[tuple[str, float, mavlink.MAVLink_message]]()
    zmq_ctx = zmq.Context()
    rid_sock = create_zmq_socket(zmq_ctx, zmq.PUB, BasePort.RID_DATA, port_offset)

    stop_event = threading.Event()

    # ARP → VEH + GCS
    router1 = MessageRouter(
        source=ap_conn,
        targets=[cs_queue, vh_queue],  # oc_queue,
        labels=["⬅️ GCS ← ARP", "⬅️ VEH ← ARP"],  # "⬅️ ORC ← ARP"
        sysid=sysid,
        sender="ARP",
        stop_event=stop_event,
    )

    # GCS → ARP
    router2 = MessageRouter(
        source=cs_conn,
        targets=[ap_queue],
        labels=["➡️ GCS → ARP"],
        sysid=sysid,
        sender="GCS",
        stop_event=stop_event,
    )

    # VEH → ARP
    router3 = MessageRouter(
        source=vh_conn,
        targets=[ap_queue],
        labels=["➡️ VEH → ARP"],
        sysid=sysid,
        sender="VEH",
        stop_event=stop_event,
    )
    logging.debug(f"Proxy {sysid}: MessageRouter threads created")

    log_file = open(DATA_PATH / f"proxy_{sysid}.log", "w")
    log_writer = csv.writer(log_file)
    log_writer.writerow(
        ["sender", "recipient", "time_received", "time_sent", "message"]
    )
    sensor_log_files: dict[str, TextIO] = {}
    logging.debug(f"Proxy {sysid}: CSV log file created")
    try:
        router1.start()
        router2.start()
        router3.start()

        while not stop_event.is_set():
            try:
                if stop_event.is_set():
                    break

                while not cs_queue.empty() and not stop_event.is_set():
                    write_and_log_message(cs_queue, cs_conn, log_writer, "GCS")

                while not ap_queue.empty() and not stop_event.is_set():
                    write_and_log_message(ap_queue, ap_conn, log_writer, "ARP")

                while not vh_queue.empty() and not stop_event.is_set():
                    record = write_and_log_message(vh_queue, vh_conn, log_writer, "VEH")
                    write_and_resend_sensor_readings(
                        record,
                        sensor_log_files,
                        sysid,
                        rid_sock,
                    )

                time.sleep(0.01)
            except EOFError as e:
                logging.error(f"EOF error (sysid {sysid}): {e}")
            except ConnectionResetError as e:
                logging.error(f"Connection reset (sysid {sysid}): {e}")
            except OSError as e:
                if e.errno == 5:  # Input/output error
                    logging.error(f"I/O error (sysid {sysid}): {e}")
                else:
                    logging.error(f"OS error in proxy main loop (sysid {sysid}): {e}")
                break
            except Exception as e:
                logging.error(f"Unexpected error (sysid {sysid}): {e}")
    finally:
        router1.join()
        router2.join()
        router3.join()
        try:
            cs_conn.close()
        except Exception as e:
            logging.error(f"Proxy {sysid}: Error closing GCS connection: {e}")
        try:
            ap_conn.close()
        except Exception as e:
            logging.error(f"Proxy {sysid}: Error closing ArduPilot connection: {e}")
        try:
            vh_conn.close()
        except Exception as e:
            logging.error(f"Proxy {sysid}: Error closing Vehicle connection: {e}")
        try:
            log_file.close()
        except Exception as e:
            logging.error(f"Proxy {sysid}: Error closing log file: {e}")
        rid_sock.close(linger=0)
        zmq_ctx.term()
        logging.info(f"Proxy {sysid} stopped.")


class MessageRouter(threading.Thread):
    """Threaded message router between MAVLink connections."""

    def __init__(
        self,
        source: MAVConnection,
        targets: list[Queue[tuple[str, float, mavlink.MAVLink_message]]],
        labels: list[str],
        sender: str,
        sysid: int,
        stop_event: threading.Event,
    ):
        super().__init__()
        self.source = source
        self.targets = targets
        self.labels = labels
        self.sender = sender
        self.sysid = sysid
        self.stop_event = stop_event

    def run(self):
        """Continuously receive messages and dispatch them until stopped."""
        while not self.stop_event.is_set():
            msg = self.source.recv_match(blocking=True, timeout=0.1)

            if msg and not self.stop_event.is_set():
                # Check for logic completion signal
                if (
                    msg.get_type() == "STATUSTEXT"
                    and hasattr(msg, "text")
                    and getattr(msg, "text", None) == "LOGIC_DONE"
                ):
                    logging.debug(
                        f"MessageRouter ({self.sender}): "
                        f"Received LOGIC_DONE, terminating proxy"
                    )
                    self.stop_event.set()
                    break

                self.dispatch_message(msg)

    def dispatch_message(self, msg: mavlink.MAVLink_message):
        """Send a message to all targets with timestamp and sender."""
        time_received = time.time()
        for q, label in zip(self.targets, self.labels):
            logging.debug(f"{label} {self.sysid}: {msg.get_type()}")
            q.put((self.sender, time_received, msg))


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


def write_and_log_message(
    q: Queue[tuple[str, float, mavlink.MAVLink_message]],
    conn: MAVConnection,
    log_writer: _csv.Writer,
    recipient: str,
):
    """Write the next message from a queue to the connection and log it."""
    sender, time_received, msg = q.get()

    msg_type = msg.get_type()
    conn.write(bytes(msg.get_msgbuf()))
    if msg_type != "BAD_DATA":
        log_writer.writerow(
            [
                sender,
                recipient,
                time_received,
                time.time(),
                msg.to_json(),
            ]
        )
    return sender, time_received, msg


def write_and_resend_sensor_readings(
    msg_record: tuple[str, float, mavlink.MAVLink_message],
    sensor_logs: dict[str, TextIO],
    sysid: int,
    rid_sock: zmq.Socket[bytes],
):
    """Log sensor readings to separate files and resend Remote ID data via ZMQ."""
    sender, time_received, msg = msg_record
    msg_type = msg.get_type()
    # Check if it's a sensor message to log separately
    if msg_type in {"GPS_RAW_INT", "RAW_IMU", "SCALED_PRESSURE"}:  # ,
        data = msg.to_dict()
        try:
            rid_sock.send_json(data, flags=zmq.NOBLOCK)  # type: ignore
        except Exception as e:
            logging.warning(
                f"Error sending Remote ID data for {msg_type} from {sysid}: {e}"
            )
        log_line = json.dumps(
            {
                "sysid": sysid,
                "sender": sender,
                "time_received": time_received,
                "time_logged": time.time(),
                "msg": data,
            }
        )

        file = sensor_logs.get(msg_type)
        if file is None:
            path = DATA_PATH / "sensor_logs" / f"sensor_{sysid}_{msg_type}.log"
            os.makedirs(path.parent, exist_ok=True)
            file = open(path, "a")
            sensor_logs[msg_type] = file

        file.write(log_line + "\n")
        file.flush()


if __name__ == "__main__":
    main()
