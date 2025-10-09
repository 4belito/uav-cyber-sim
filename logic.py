"""Multi-UAV MAVLink Proxy."""

from __future__ import annotations

import argparse
import json
import logging
import time
from enum import StrEnum
from typing import TypedDict

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from config import DATA_PATH, BasePort
from helpers.connections.mavlink.conn import (
    create_tcp_conn,
    create_udp_conn,
    send_heartbeat,
)
from helpers.connections.mavlink.customtypes.mavconn import MAVConnection
from helpers.connections.mavlink.enums import CmdCustom
from helpers.coordinates import ENU
from helpers.rid import RIDManager
from helpers.setup_log import setup_logging
from params.simulation import HEARTBEAT_FREQUENCY, REMOTE_ID_FREQUENCY
from plan import Action, Plan, State, Step

######################################################################
# NOTE: The plans have to only be hardcoded when using the GUIDED mode.
mission = True
if not mission:
    plans = [
        Plan.auto(name="square_auto", mission_path=f"plan/missions/square_{i + 1}")
        for i in range(5)
    ]
########################################
# TODO: Refactor this module
heartbeat_event = mavutil.periodic_event(HEARTBEAT_FREQUENCY)
rid_event = mavutil.periodic_event(REMOTE_ID_FREQUENCY)


def main():
    """Entry point for the Multi-UAV MAVLink Proxy."""
    config_path, verbose = parse_arguments()
    config = VehicleLogic.load_config(config_path)
    setup_logging(f"logic_{config['sysid']}", verbose=verbose or 1, console_output=True)
    start_logic(config)


def start_logic(config: LogicConfig):
    """Start bidirectional proxy for a given UAV system_id."""
    sysid = config["sysid"]
    port_offset = config["port_offset"]
    monitored_items = config["monitored_items"]

    i = sysid - 1
    lg_conn = create_tcp_conn(
        base_port=BasePort.LOG, offset=port_offset, role="server", sysid=sysid
    )

    cs_conn = create_udp_conn(base_port=BasePort.GCS, offset=port_offset, mode="sender")

    rid_mnng = RIDManager(sysid, port_offset)
    rid_mnng.start()

    plan = Plan.auto(
        name="auto",
        mission_path=str(DATA_PATH / f"mission_{sysid}.waypoints"),
        monitored_items=monitored_items,
    )
    logic = VehicleLogic(lg_conn, plan=plan if mission else plans[i])

    try:
        while True:
            if heartbeat_event.trigger():
                send_heartbeat(lg_conn)
                send_heartbeat(cs_conn)

            if rid_event.trigger():
                try:
                    rid_mnng.publish()
                except Exception as e:
                    logging.error(f"Error sending RID data: {e}")
                    pass

            if logic.plan.state == State.DONE:
                msg_proxy = mavlink.MAVLink_statustext_message(
                    severity=6, text=b"LOGIC_DONE"
                )
                logging.info(f"Proxy ← Logic {sysid}: Sending LOGIC_DONE")
                lg_conn.mav.send(msg_proxy)
                send_done_until_ack(cs_conn, sysid)
                break

            logic.act()
            time.sleep(0.01)
    finally:
        cs_conn.close()
        lg_conn.close()
        rid_mnng.stop()
        logging.info(f"Vehicle {sysid} logic stopped")


class LogicConfig(TypedDict):
    """UAV logic configuration."""

    sysid: int
    port_offset: int
    monitored_items: list[int]


def send_done_until_ack(conn: MAVConnection, idx: int, max_tries: float = float("inf")):
    """
    Send 'DONE' via STATUSTEXT repeatedly until receiving a COMMAND_ACK.
    Assumes `conn` is a dedicated MAVLink connection for one UAV.
    """
    msg = mavlink.MAVLink_statustext_message(severity=6, text=b"DONE")
    i = 0
    while i < max_tries:
        logging.debug(f"GCS ← UAV {idx}: Sending DONE (attempt {i + 1})")
        conn.mav.send(msg)

        start = time.time()
        while time.time() - start < 0.05:
            ack = conn.recv_match(type="COMMAND_ACK", blocking=False)
            if ack and ack.command == CmdCustom.PLAN_DONE:
                logging.info("ACK received. DONE message acknowledged")
                return
            time.sleep(0.001)
        i += 1

    logging.warning("No ACK received after max attempts")


class VehicleMode(StrEnum):
    """Defines operational modes for the UAV."""

    MISSION = "MISSION"  # or "MISSION", "WAYPOINT_NAV"
    AVOIDANCE = "AVOIDANCE"  # or "COLLISION_AVOIDANCE"


class VehicleLogic:
    """Handles the logic for executing a UAV's mission plan."""

    def __init__(
        self,
        connection: MAVConnection,
        plan: Plan,
        safety_radius: float = 5,
        radar_radius: float = 10,
    ):
        # Vehicle Creation
        self.conn = connection
        self.sysid = connection.target_system
        self.name = f"Logic 🧠 {self.sysid}"

        # Mode Properties
        self.mode = VehicleMode.MISSION
        self.plan = plan
        self.plan.bind(self.conn)
        self.back_mode = VehicleMode.MISSION

        # Communication properties (positions are local)
        self.safety_radius: float = safety_radius
        self.radar_radius: float = radar_radius

        logging.info(f"{self.name}: launching")

    def act(self):
        """Perform the next step in the mission plan."""
        self.plan.act()

    def set_mode(self, new_mode: VehicleMode) -> None:
        """Switch the vehicle to a new operational mode."""
        if new_mode != self.mode:
            logging.info(
                f"{self.name}: Vehicle {self.sysid} switched to mode: {new_mode}"
            )
            self.mode = new_mode

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
    parser = argparse.ArgumentParser(description="Single UAV MAVLink Proxy")
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
