"""Multi-UAV MAVLink Proxy."""

import _csv
import argparse
import csv
import json
import logging
import os
import threading
import time
import traceback
from queue import Queue
from typing import Literal, TextIO

import pymavlink.dialects.v20.ardupilotmega as mavlink
import zmq
from pymavlink import mavutil

# First Party imports
from config import DATA_PATH, BasePort
from helpers.setup_log import setup_logging
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
DATA_STREAM_RATE = 5


def main() -> None:
    """Parse arguments and launch the MAVLink proxy."""
    system_id, port_offset, verbose = parse_arguments()
    # Set up logging for standalone proxy
    setup_logging(f"proxy_{system_id}", verbose=verbose, console_output=True)
    start_proxy(system_id, port_offset)


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
def send_heartbeat(conn: MAVConnection, sysid: int = 255) -> None:
    """Send a GCS heartbeat message to the UAV."""
    # Set the source system ID for this connection
    conn.mav.srcSystem = sysid
    conn.mav.heartbeat_send(Type.GCS, Autopilot.INVALID, 0, 0, 0)


def create_udp_conn(
    base_port: int,
    offset: int,
    mode: Literal["receiver", "sender"],
) -> MAVConnection:
    """Create a MAVLink-over-UDP connection."""
    port = base_port + offset
    if mode == "receiver":
        conn = connect(f"udp:127.0.0.1:{port}")  # listen for incoming
        conn.wait_heartbeat()
    else:  # mode == "sender"
        conn = connect(f"udpout:127.0.0.1:{port}")  # send-only
    return conn


def create_tcp_conn(
    base_port: int,
    offset: int,
    role: Literal["client", "server"] = "client",
    sysid: int = 255,
) -> MAVConnection:
    """Create and in or out connection and wait for geting the hearbeat in."""
    port = base_port + offset
    connection_string = f"tcp{'in' if role == 'server' else ''}:127.0.0.1:{port}"

    try:
        conn = connect(connection_string)
        # Set the source system ID for this connection
        conn.mav.srcSystem = sysid
        send_heartbeat(conn, sysid)
        conn.wait_heartbeat()
        # After receiving heartbeat, the target_system should be set correctly
        # But if it's still 0, we can force it to the expected sysid
        if hasattr(conn, "target_system") and conn.target_system == 0:
            conn.target_system = sysid
        return conn
    except Exception as e:
        logging.error(f"Failed to create TCP connection on port {port}: {e}")
        logging.error(f"TCP connection error traceback:\n{traceback.format_exc()}")
        raise


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
        logging.debug(f"MessageRouter ({self.sender}): Thread starting")

        while not self.stop_event.is_set():
            try:
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

            except EOFError as e:
                logging.error(
                    f"EOF error in MessageRouter ({self.sender}): Connection closed"
                )
                logging.debug(f"EOF details: {e}")
                self.stop_event.set()
                break
            except ConnectionResetError as e:
                logging.error(
                    f"Connection reset in MessageRouter ({self.sender}): "
                    f"Connection closed by peer"
                )
                logging.debug(f"ConnectionResetError details: {e}")
                self.stop_event.set()
                break
            except OSError as e:
                if e.errno == 5:  # Input/output error
                    logging.error(
                        f"I/O error in MessageRouter ({self.sender}): "
                        f"Connection terminated"
                    )
                    logging.debug(f"OSError details: {e}")
                else:
                    logging.error(f"OS error in MessageRouter ({self.sender}): {e}")
                    logging.error(f"Error type: {type(e).__name__}")
                    logging.error(f"Exception traceback:\n{traceback.format_exc()}")
                self.stop_event.set()
                break
            except Exception as e:
                logging.error(f"Unexpected error in MessageRouter ({self.sender}): {e}")
                logging.error(f"Error type: {type(e).__name__}")
                logging.error(f"Exception traceback:\n{traceback.format_exc()}")
                self.stop_event.set()
                break

        logging.debug(f"MessageRouter ({self.sender}): Thread exiting")

    def dispatch_message(self, msg: mavlink.MAVLink_message):
        """Send a message to all targets with timestamp and sender."""
        time_received = time.time()
        for q, label in zip(self.targets, self.labels):
            logging.debug(f"{label} {self.sysid}: {msg.get_type()}")
            q.put((self.sender, time_received, msg))


def write_and_log_message(
    q: Queue[tuple[str, float, mavlink.MAVLink_message]],
    conn: MAVConnection,
    log_writer: _csv.Writer,
    recipient: str,
):
    """Write the next message from a queue to the connection and log it."""
    sender, time_received, msg = q.get()

    try:
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
    except (ConnectionResetError, OSError, EOFError) as e:
        logging.debug(f"Connection closed while writing to {recipient}: {e}")
        # Don't re-raise, just skip this message
    except Exception as e:
        logging.error(f"Unexpected error writing to {recipient}: {e}")
        logging.error(f"Exception traceback:\n{traceback.format_exc()}")
        # Don't re-raise, just skip this message


def write_and_log_with_sensors(
    q: Queue[tuple[str, float, mavlink.MAVLink_message]],
    conn: MAVConnection,
    log_writer: _csv.Writer,
    recipient: str,
    sensor_logs: dict[str, TextIO],
    sysid: int,
    rid_sock: zmq.Socket[bytes],
):
    """Write and log proxy and sensor messages in one step."""
    sender, time_received, msg = q.get()
    msg_type = msg.get_type()

    try:
        conn.write(bytes(msg.get_msgbuf()))
    except (ConnectionResetError, OSError, EOFError) as e:
        logging.debug(f"Connection closed while writing to {recipient}: {e}")
        return  # Skip processing if connection is dead
    except Exception as e:
        logging.error(f"Unexpected error writing to {recipient}: {e}")
        return  # Skip processing if write failed

    if msg_type == "BAD_DATA":
        return

    now = time.time()

    # Log general proxy traffic
    log_writer.writerow(
        [
            sender,
            recipient,
            time_received,
            now,
            msg.to_json(),
        ]
    )

    # Check if it's a sensor message to log separately
    if msg_type in {"RAW_IMU", "SCALED_PRESSURE", "GPS_RAW_INT"}:
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
                "time_logged": now,
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


def start_proxy(sysid: int, port_offset: int) -> None:
    """Start bidirectional proxy for a given UAV system_id."""
    logging.debug(f"Proxy {sysid}: Creating connections...")

    logging.debug(
        f"Proxy {sysid}: Creating ArduPilot TCP connection on port "
        f"{BasePort.ARP + port_offset}..."
    )
    ap_conn = create_tcp_conn(
        base_port=BasePort.ARP, offset=port_offset, role="client", sysid=sysid
    )
    logging.debug(f"Proxy {sysid}: ArduPilot connection created")

    logging.debug(f"Proxy {sysid}: Creating GCS UDP connection...")
    cs_conn = create_udp_conn(base_port=BasePort.GCS, offset=port_offset, mode="sender")
    logging.debug(f"Proxy {sysid}: GCS connection created")

    oc_conn = create_udp_conn(base_port=BasePort.ORC, offset=port_offset, mode="sender")

    logging.debug(f"Proxy {sysid}: Creating Vehicle TCP connection...")
    vh_conn = create_tcp_conn(
        base_port=BasePort.VEH, offset=port_offset, role="client", sysid=sysid
    )
    logging.debug(f"Proxy {sysid}: Vehicle connection created")

    logging.debug(f"Proxy {sysid}: Requesting sensor streams...")
    request_sensor_streams(
        ap_conn, stream_ids=DATA_STREAM_IDS, rate_hz=DATA_STREAM_RATE
    )
    logging.debug(f"Proxy {sysid}: Sensor streams requested")

    logging.debug(f"Proxy {sysid}: Creating message queues...")
    ap_queue = Queue[tuple[str, float, mavlink.MAVLink_message]]()
    cs_queue = Queue[tuple[str, float, mavlink.MAVLink_message]]()
    oc_queue = Queue[tuple[str, float, mavlink.MAVLink_message]]()
    vh_queue = Queue[tuple[str, float, mavlink.MAVLink_message]]()
    logging.debug(f"Proxy {sysid}: Message queues created")

    logging.debug(f"Proxy {sysid}: Setting up ZMQ...")
    zmq_ctx = zmq.Context()
    rid_sock = zmq_ctx.socket(zmq.PUB)
    rid_sock.bind(f"tcp://127.0.0.1:{BasePort.RID_DATA + port_offset}")
    logging.debug(f"Proxy {sysid}: ZMQ setup complete")

    logging.debug(f"Starting Proxy {sysid}")

    stop_event = threading.Event()

    logging.debug(f"Proxy {sysid}: Creating MessageRouter threads...")
    # ARP → ORC + VEH  X(+GCS)
    router1 = MessageRouter(
        source=ap_conn,
        targets=[cs_queue, oc_queue, vh_queue],
        labels=["⬅️ GCS ← ARP", "⬅️ ORC ← ARP", "⬅️ VEH ← ARP"],
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

    # ORC → ARP
    router3 = MessageRouter(
        source=oc_conn,
        targets=[ap_queue],
        labels=["➡️ ORC → ARP"],
        sysid=sysid,
        sender="ORC",
        stop_event=stop_event,
    )

    # VEH → ARP
    router4 = MessageRouter(
        source=vh_conn,
        targets=[ap_queue],
        labels=["➡️ VEH → ARP"],
        sysid=sysid,
        sender="VEH",
        stop_event=stop_event,
    )
    logging.debug(f"Proxy {sysid}: MessageRouter threads created")

    logging.debug(f"Proxy {sysid}: Creating CSV log file...")
    log_file = open(DATA_PATH / f"proxy_{sysid}.log", "w")
    log_writer = csv.writer(log_file)
    log_writer.writerow(
        ["sender", "recipient", "time_received", "time_sent", "message"]
    )
    sensor_log_files: dict[str, TextIO] = {}
    logging.debug(f"Proxy {sysid}: CSV log file created")
    try:
        logging.debug(f"Proxy {sysid}: Starting router threads...")
        router1.start()
        logging.debug(f"Proxy {sysid}: Router1 (ARP) started")
        router2.start()
        logging.debug(f"Proxy {sysid}: Router2 (GCS) started")
        router3.start()
        router4.start()
        logging.debug(f"Proxy {sysid}: Router4 (VEH) started")

        logging.debug(f"Proxy {sysid}: Entering main message processing loop")
        while not stop_event.is_set():
            try:
                # Check if we should continue processing queues
                if stop_event.is_set():
                    logging.debug(
                        f"Proxy {sysid}: Stop event set, breaking from main loop"
                    )
                    break

                while not oc_queue.empty():
                    write_and_log_message(oc_queue, oc_conn, log_writer, "ORC")

                while not cs_queue.empty() and not stop_event.is_set():
                    write_and_log_message(cs_queue, cs_conn, log_writer, "GCS")

                while not ap_queue.empty() and not stop_event.is_set():
                    write_and_log_message(ap_queue, ap_conn, log_writer, "ARP")

                while not vh_queue.empty() and not stop_event.is_set():
                    # write_and_log_message(vh_queue, vh_conn, log_writer, "VEH")
                    write_and_log_with_sensors(
                        vh_queue,
                        vh_conn,
                        log_writer,
                        "VEH",
                        sensor_log_files,
                        sysid,
                        rid_sock,
                    )

                time.sleep(0.01)
            except EOFError as e:
                logging.error(
                    f"EOF error in proxy main loop (sysid {sysid}): Connection closed"
                )
                logging.debug(f"EOF details: {e}")
                break
            except ConnectionResetError as e:
                logging.error(
                    f"Connection reset in proxy main loop (sysid {sysid}): "
                    f"Connection closed by peer"
                )
                logging.debug(f"ConnectionResetError details: {e}")
                break
            except OSError as e:
                if e.errno == 5:  # Input/output error
                    logging.error(
                        f"I/O error in proxy main loop (sysid {sysid}): "
                        f"Connection terminated"
                    )
                    logging.debug(f"OSError details: {e}")
                else:
                    logging.error(f"OS error in proxy main loop (sysid {sysid}): {e}")
                    logging.error(f"Error type: {type(e).__name__}")
                    logging.error(f"Exception traceback:\n{traceback.format_exc()}")
                break
            except Exception as e:
                logging.error(
                    f"Unexpected error in proxy main loop (sysid {sysid}): {e}"
                )
                logging.error(f"Error type: {type(e).__name__}")
                logging.error(f"Exception traceback:\n{traceback.format_exc()}")
                break
    finally:
        logging.info(f"Proxy {sysid}: Starting cleanup...")

        logging.debug(f"Proxy {sysid}: Waiting for router threads to stop...")
        router1.join()
        router2.join()
        router3.join()
        router4.join()
        logging.debug(f"Proxy {sysid}: All router threads stopped")

        logging.debug(f"Proxy {sysid}: Closing connections...")
        try:
            cs_conn.close()
            logging.debug(f"Proxy {sysid}: GCS connection closed")
        except Exception as e:
            logging.error(f"Proxy {sysid}: Error closing GCS connection: {e}")

        try:
            oc_conn.close()
            logging.debug(f"Proxy {sysid}: Oracle connection closed")
        except Exception as e:
            logging.error(f"Proxy {sysid}: Error closing Oracle connection: {e}")

        try:
            ap_conn.close()
            logging.debug(f"Proxy {sysid}: ArduPilot connection closed")
        except Exception as e:
            logging.error(f"Proxy {sysid}: Error closing ArduPilot connection: {e}")

        try:
            vh_conn.close()
            logging.debug(f"Proxy {sysid}: Vehicle connection closed")
        except Exception as e:
            logging.error(f"Proxy {sysid}: Error closing Vehicle connection: {e}")

        # rid_sock.close()
        # zmq_ctx.term()

        try:
            log_file.close()
            logging.debug(f"Proxy {sysid}: Log file closed")
        except Exception as e:
            logging.error(f"Proxy {sysid}: Error closing log file: {e}")

        logging.info(f"Proxy {sysid} stopped.")


if __name__ == "__main__":
    main()
