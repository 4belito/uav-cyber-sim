"""Multi-UAV MAVLink Logic."""

from __future__ import annotations

import argparse
import json
import logging
import threading
import time
from typing import Any, TypedDict

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from simulator.config import BasePort
from simulator.helpers.connections import (
    MAVConnection,
    create_tcp_conn,
    create_udp_conn,
    send_heartbeat,
)
from simulator.helpers.connections.mavlink.customenums.customcmd import CustomCmd
from simulator.helpers.connections.mavlink.enums import DataStream, MsgID
from simulator.helpers.connections.mavlink.streams import (
    ask_msg,
    request_sensor_streams,
)
from simulator.helpers.coordinates import ENU, GRA
from simulator.helpers.rid import RIDData, RIDManager
from simulator.helpers.setup_log import setup_logging
from simulator.params.simulation import (
    DATA_STREAM_FREQUENCY,
    HEARTBEAT_FREQUENCY,
    REMOTE_ID_FREQUENCY,
)
from simulator.planner import Action, Plan, PlanSpec, State, Step
from simulator.vehicle.router import MAVLinkRouter
from simulator.vehicle.state import VehicleState

DATA_STREAM_IDS = [
    DataStream.RAW_SENSORS,
    DataStream.EXTENDED_STATUS,
    DataStream.POSITION,
    DataStream.EXTRA1,  # Required to receive some ArduPilot custom telemetry
    # (e.g. ESC_TELEMETRY_*).If disabled, these messages are not streamed and the
    # UNKNOWN message decoding logic is never exercised.
    DataStream.EXTRA2,
]
RID_INTERVAL = int(1_000_000 / REMOTE_ID_FREQUENCY)

# TODO: Refactor this module
heartbeat_event = mavutil.periodic_event(HEARTBEAT_FREQUENCY)
rid_event = mavutil.periodic_event(REMOTE_ID_FREQUENCY)


def wait_for_vehicle_link(
    vehicle_state: VehicleState,
    timeout: float = 10.0,
) -> mavlink.MAVLink_heartbeat_message:
    """Wait until the first HEARTBEAT is received from the vehicle."""
    hb = vehicle_state.wait_for("HEARTBEAT", timeout=timeout)
    if hb is None:
        raise TimeoutError("Timed out waiting for vehicle HEARTBEAT")
    return hb


def main():
    """Entry point for the Multi-UAV MAVLink Logic."""
    config_path, verbose = parse_arguments()
    config = VehicleLogic.load_config(config_path)
    setup_logging(f"logic_{config['sysid']}", verbose=verbose or 1, console_output=True)
    start_logic(config)


# TODO: Remove monitored items from config
def start_logic(config: LogicConfig):
    """Start bidirectional proxy for a given UAV system_id."""
    sysid = config["sysid"]
    port_offset = config["port_offset"]
    gra_orign = GRA(**config["gra_origin_dict"])
    plan_spec = PlanSpec(**config["plan_spec"])

    # lg_conn = create_tcp_conn(
    #     base_port=BasePort.LOG,
    #     offset=port_offset,
    #     role="server",
    #     src_sysid=sysid,
    #     src_compid=140,  # free for custom modules, companion computers, routing modules
    # )
    ap_conn = create_tcp_conn(
        base_port=BasePort.ARP,
        offset=port_offset,
        role="client",
        src_sysid=sysid,
        src_compid=140,
    )
    logging.info(f"Vehicle {sysid}: Logic connection established")
    cs_conn = create_udp_conn(
        base_port=BasePort.GCS,
        offset=port_offset,
        mode="sender",
        src_sysid=sysid,
        src_compid=140,
    )
    logging.info(f"Vehicle {sysid}: GCS connection established")

    # Shared telemetry state
    vehicle_state = VehicleState()

    # Router stop signal
    router_stop = threading.Event()
    router = MAVLinkRouter(
        conn=ap_conn,
        state=vehicle_state,
        stop_event=router_stop,
    )
    logging.info("Waiting for MAVLink client connection...")
    ap_conn.wait_heartbeat()
    logging.info("MAVLink connection established")

    ask_msg(ap_conn, MsgID.GLOBAL_POSITION_INT, interval=RID_INTERVAL)

    request_sensor_streams(
        ap_conn,
        stream_ids=DATA_STREAM_IDS,
        rate_hz=DATA_STREAM_FREQUENCY,
    )

    router.start()
    logging.info(f"Vehicle {sysid}: MAVLink router started")
    hb = wait_for_vehicle_link(vehicle_state, timeout=10.0)
    logging.info(
        "Vehicle %s: first heartbeat received from system=%s component=%s",
        sysid,
        hb.get_srcSystem(),
        hb.get_srcComponent(),
    )
    rid_mnng = RIDManager(sysid, port_offset, gra_orign)
    rid_mnng.start()

    plan = Plan.build(plan_spec)
    logic = VehicleLogic(
        connection=ap_conn,
        plan=plan,
        gra_origin=gra_orign,
        vehicle_state=vehicle_state,
    )

    try:
        while True:
            if heartbeat_event.trigger():
                send_heartbeat(ap_conn)
                send_heartbeat(cs_conn)

            if rid_event.trigger() and rid_mnng.pending:
                try:
                    logic.rid = rid_mnng.data
                    rid_mnng.publish()
                except Exception as e:
                    logging.error(f"Error sending RID data: {e}")
                    pass
            if logic.plan.state == State.DONE:
                logic.send_done_msgs(cs_conn)
                break

            logic.act()
            time.sleep(0.01)
    finally:
        router_stop.set()
        router.join(timeout=1)

        cs_conn.close()
        ap_conn.close()
        rid_mnng.stop()
        logging.info(f"Vehicle {sysid} logic stopped")


class LogicConfig(TypedDict):
    """UAV logic configuration."""

    sysid: int
    gra_origin_dict: dict[str, float]
    port_offset: int
    monitored_items: list[int]
    plan_spec: dict[str, Any]


class VehicleLogic:
    """Handles the logic for executing a UAV's mission plan."""

    def __init__(
        self,
        connection: MAVConnection,
        plan: Plan,
        gra_origin: GRA,
        vehicle_state: VehicleState,
    ):
        # Vehicle Creation
        self.conn = connection
        self.sysid = connection.target_system
        self.name = f"Logic 🧠 {self.sysid}"
        self.gra_origin = gra_origin
        self.vehicle_state = vehicle_state

        # Plan
        self.plan = plan
        self.plan.bind(self.conn, self.gra_origin, self.vehicle_state)

        # Communication properties (positions are local)
        self.rid: RIDData | None = None

        logging.info(f"{self.name}: launching")

    def act(self):
        """Perform the next step in the mission plan."""
        self.plan.act()

    def send_done_msgs(self, cs_conn: MAVConnection) -> None:
        """Notify the GCS that the mission is done."""
        done_msg = mavlink.MAVLink_statustext_message(severity=6, text=b"LOGIC_DONE")
        logging.info(f"Proxy ← Logic {self.sysid}: Sending LOGIC_DONE")
        self.conn.mav.send(done_msg)  # This is tcp connection, no ack need it.
        self.send_msg_until_ack(cs_conn, done_msg, CustomCmd.LOGIC_DONE)

    def send_msg_until_ack(
        self,
        conn: MAVConnection,
        msg: mavlink.MAVLink_statustext_message,
        ack_cmd: CustomCmd,
        max_tries: float = float("inf"),
    ):
        """
        Send 'DONE' via STATUSTEXT repeatedly until receiving a COMMAND_ACK.
        Assumes `conn` is a dedicated MAVLink connection for one UAV.
        """
        i = 0
        while i < max_tries:
            logging.debug(f"GCS ← UAV {self.sysid}: Sending DONE (attempt {i + 1})")
            conn.mav.send(msg)
            start = time.time()
            while time.time() - start < 0.05:
                ack = conn.recv_match(type="COMMAND_ACK", blocking=False)
                if ack and ack.command == ack_cmd:
                    logging.info("ACK received. DONE message acknowledged")
                    return
                time.sleep(0.001)
            i += 1

        logging.warning("No ACK received after max attempts")

    @property
    def current_action(self) -> Action[Step] | None:
        """Return the current action being executed."""
        return self.plan.current

    @property
    def current_step(self) -> Step | None:
        """Return the current step within the current action."""
        if self.current_action is not None:
            return self.current_action.current
        else:
            return None

    @property
    def pos(self) -> ENU | None:
        """Return the current estimated position of the UAV."""
        return self.plan.curr_pos

    def is_onair(self) -> bool | None:
        """Return whether the UAV is currently airborne."""
        return self.plan.onair

    @property
    def target_pos(self) -> ENU | None:
        """Return the current step's target position, if any."""
        if self.current_step:
            return self.current_step.target_pos
        else:
            return None

    @staticmethod
    def load_config(config_path: str) -> LogicConfig:
        """Load logic configuration from a JSON file."""
        with open(config_path) as f:
            logic_config: LogicConfig = json.load(f)
        return logic_config


def parse_arguments() -> tuple[str, int | None]:
    """Parse a single system ID."""
    parser = argparse.ArgumentParser(description="Single UAV MAVLink Logic")
    parser.add_argument(
        "--config-path",
        type=str,
        required=True,
        help="Path to the logic configuration file (e.g. logic_config_1.json)",
    )
    parser.add_argument(
        "--verbose",
        type=int,
        required=False,
        help="verbosity level (e.g. 0,1,2,3)",
    )
    args = parser.parse_args()
    return (args.config_path, args.verbose)


if __name__ == "__main__":
    main()
