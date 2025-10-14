"""Multi-UAV MAVLink Proxy."""

from __future__ import annotations

import argparse
import json
import logging
import time
from queue import Empty
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
from helpers.connections.mavlink.enums import CmdCustom, CopterMode
from helpers.coordinates import ENU, GRA, XY
from helpers.rid import RIDData, RIDManager
from helpers.setup_log import setup_logging
from params.simulation import HEARTBEAT_FREQUENCY, REMOTE_ID_FREQUENCY
from plan import Action, ActionNames, Plan, State, Step
from plan.actions import make_set_mode
from plan.actions.navegation import GoToGlobal

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
    gra_orign = GRA(**config["gra_origin_dict"])
    navegation_speed = config["navegation_speed"]

    lg_conn = create_tcp_conn(
        base_port=BasePort.LOG, offset=port_offset, role="server", sysid=sysid
    )

    cs_conn = create_udp_conn(base_port=BasePort.GCS, offset=port_offset, mode="sender")

    rid_mnng = RIDManager(sysid, port_offset, gra_orign)
    rid_mnng.start()

    plan = Plan.auto(
        name="auto",
        gra_origin=gra_orign,
        mission_path=str(DATA_PATH / f"mission_{sysid}.waypoints"),
        monitored_items=monitored_items,
        navegation_speed=navegation_speed,
    )
    logic = VehicleLogic(lg_conn, plan=plan, gra_origin=gra_orign)

    try:
        while True:
            if heartbeat_event.trigger():
                send_heartbeat(lg_conn)
                send_heartbeat(cs_conn)

            if rid_event.trigger() and rid_mnng.pending:
                try:
                    logic.rid = rid_mnng.data
                    rid_mnng.publish()
                except Exception as e:
                    logging.error(f"Error sending RID data: {e}")
                    pass
            # logging.info(f"Vehicle {sysid} plan_state: {logic.plan.state}")
            if logic.plan.state == State.DONE:
                logic.send_done_msgs(cs_conn)
                break
            try:
                o_rid = rid_mnng.received_rid.get_nowait()
                # TODO: handle multiple obstacles
                logging.debug(
                    f"Get RID:{o_rid and o_rid.enu_pos} from the received_queeue"
                )
                logic.check_avoidance(o_rid)
            except Empty:
                pass
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
    gra_origin_dict: dict[str, float]
    port_offset: int
    monitored_items: list[int]
    navegation_speed: float


class VehicleLogic:
    """Handles the logic for executing a UAV's mission plan."""

    def __init__(
        self,
        connection: MAVConnection,
        plan: Plan,
        gra_origin: GRA,
        safety_radius: float = 5,
    ):
        # Vehicle Creation
        self.conn = connection
        self.sysid = connection.target_system
        self.name = f"Logic 🧠 {self.sysid}"

        # Plan Properties
        self.plan = plan
        self.plan.bind(self.conn)

        # Avoidance Actions
        self.set_guided = make_set_mode(CopterMode.GUIDED)
        self.set_guided.bind(self.conn)
        self.set_auto = make_set_mode(CopterMode.AUTO)
        self.set_auto.bind(self.conn)
        self.mode = CopterMode.AUTO
        self.avoidance_action = Action[Step](name=ActionNames.AVOIDANCE, emoji="🚧")
        self.avoidance_action.bind(self.conn)
        self.gra_origin = gra_origin

        # Communication properties (positions are local)
        self.safety_radius: float = safety_radius
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
        self.send_msg_until_ack(cs_conn, done_msg, CmdCustom.LOGIC_DONE)

    def send_msg_until_ack(
        self,
        conn: MAVConnection,
        msg: mavlink.MAVLink_statustext_message,
        ack_cmd: CmdCustom,
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

    ## Passive avoidance
    def check_avoidance(self, o_rid: RIDData):
        """Check for nearby obstacles and perform avoidance maneuvers if necessary."""
        target_pos = self.plan.target_pos
        logging.debug(f"POS: {self.rid and self.rid.enu_pos} TARGET_POS: {target_pos}")
        if self.rid is None or target_pos is None:
            return
        pos = self.rid.enu_pos
        o_pos = o_rid.enu_pos
        obst_dist = ENU.distance(pos, o_pos)
        logging.debug(
            (
                f"Vehicle {self.sysid} obstacle {o_rid.sysid} at distance "
                f"{obst_dist:.2f} m compared to safety radius "
                f"{self.safety_radius} m"
            )
        )
        if obst_dist < self.safety_radius:
            logging.info(
                (
                    f"Vehicle {self.sysid} obstacle {o_rid.sysid} too close "
                    f"({obst_dist:.2f} m < {self.safety_radius} m), performing avoidance"
                )
            )
            avoid_pos = self.get_avoidance_pos(o_pos, target_pos, obst_dist)
            if avoid_pos is not None:
                # TODO: Refactor this logic
                logging.info(
                    f"Vehicle {self.sysid} avoiding to {avoid_pos}: "
                    f"mode: {self.mode} action_state: {self.avoidance_action.state}"
                )
                if (
                    self.mode == CopterMode.GUIDED
                    and self.avoidance_action.state == State.DONE  # check this later
                ):
                    gra_avoid_pos = self.gra_origin.to_abs(avoid_pos)
                    go_to_step = GoToGlobal(
                        wp=gra_avoid_pos, cause_text="avoidance", stop_asking=False
                    )
                    go_to_step.bind(self.conn)
                    self.avoidance_action.add(go_to_step)
                    logging.info(
                        f"Vehicle {self.sysid} continuing avoidance to {avoid_pos}"
                    )
                elif self.mode == CopterMode.AUTO:
                    gra_avoid_pos = self.gra_origin.to_abs(avoid_pos)
                    go_to_step = GoToGlobal(
                        wp=gra_avoid_pos, cause_text="avoidance", stop_asking=False
                    )
                    go_to_step.bind(self.conn)
                    self.avoidance_action.add(go_to_step)
                    while self.set_guided.state != State.DONE:
                        self.set_guided.act()
                    self.set_guided.reset()
                    self.mode = CopterMode.GUIDED
                    logging.info(f"Vehicle {self.sysid} switched to GUIDED mode")
        logging.debug(f"Vehicle {self.sysid} no avoidance needed")
        logging.debug(
            f"mode {self.mode.name}, plan_state {self.plan.current and self.plan.current.state}"
        )
        if self.mode == CopterMode.GUIDED:
            if self.avoidance_action.state == State.DONE:
                while self.set_auto.state != State.DONE:
                    self.set_auto.act()
                self.set_auto.reset()
                self.mode = CopterMode.AUTO
                logging.info(f"Vehicle {self.sysid} switched to AUTO mode")
            else:
                self.avoidance_action.act()

    def get_avoidance_pos(
        self,
        obst_pos: ENU,
        target_pos: ENU,
        obst_dist: float,
        direction: str = "left",
        safety_coef: float = 1 / 3,  # magic number to increase the distance
    ) -> ENU | None:
        """
        Send a velocity command in body frame, orthogonal to the direction of wp.
        `direction` can be 'left' or 'right' (relative to wp direction).
        """
        # Normalize wp direction (ignore Z)
        if self.rid is None:
            raise ValueError("Current position is None")
        curr_pos = self.rid.enu_pos

        obj_dir = XY(*ENU.sub(obst_pos, curr_pos)[:2])
        target_dir = XY(*ENU.sub(target_pos, curr_pos)[:2])
        if XY.dot(obj_dir, target_dir) < 0:
            return None
        # distance = math.sqrt(self.safety_radius**2 - obst_dist**2) * safety_coef
        # distance = self.safety_radius * safety_coef
        # obj_dir = obj_dir.scale(distance / obj_dir.norm())
        # Get orthogonal direction
        if direction == "left":
            ortho = ENU(x=-obj_dir.y, y=obj_dir.x, z=0)
        elif direction == "right":
            ortho = ENU(x=obj_dir.y, y=-obj_dir.x, z=0)
        else:
            raise ValueError("Direction must be 'left' or 'right'")

        # Scale to desired speed
        return ENU.add(
            curr_pos, ortho.scale(self.safety_radius * safety_coef / ortho.norm())
        )


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
