"""Multi-UAV MAVLink Proxy."""

# Third Party imports
import argparse
import json
import time
from enum import StrEnum
from typing import TypedDict

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink

# First Party imports
from config import DATA_PATH, BasePort
from mavlink.customtypes.connection import MAVConnection
from mavlink.customtypes.location import ENU
from mavlink.enums import Autopilot, Type
from mavlink.util import CustomCmd, connect
from params.simulation import HEARTBEAT_PERIOD
from plan import Action, Plan, State, Step
from proxy import create_connection_udp

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

heartbeat_period = mavutil.periodic_event(HEARTBEAT_PERIOD)


def main():
    """Entry point for the Multi-UAV MAVLink Proxy."""
    system_id, verbose = parse_arguments()
    port_offset = VehicleLogic.load_config(system_id)["port_offset"]
    start_proxy(system_id, port_offset, verbose=verbose or 1)


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
    vh_conn = create_connection_tcp(base_port=BasePort.VEH, offset=port_offset)
    cs_conn = create_connection_udp(base_port=BasePort.GCS, offset=port_offset)
    oc_conn = create_connection_udp(base_port=BasePort.ORC, offset=port_offset)

    logic = VehicleLogic(
        vh_conn,
        plan=(
            Plan.auto(
                name="auto", mission_path=str(DATA_PATH / f"mission_{sysid}.waypoints")
            )
            if mission
            else plans[i]
        ),
        verbose=verbose,
    )

    try:
        while True:
            if heartbeat_period.trigger():
                send_heartbeat(vh_conn)

            if logic.plan.state == State.DONE:
                send_done_until_ack(oc_conn, sysid)
                send_done_until_ack(cs_conn, sysid)
                break
            logic.act()
            time.sleep(0.01)
    finally:
        cs_conn.close()
        vh_conn.close()
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


class LogicConfig(TypedDict):
    """UAV logic configuration."""

    sysid: int
    port_offset: int


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
        verbose: int = 1,
    ):
        # Vehicle Creation
        self.conn = connection
        self.sysid = connection.target_system
        self.name = f"Logic 🧠 {self.sysid}"
        self.verbose = verbose

        # Mode Properties
        self.mode = VehicleMode.MISSION
        self.plan = plan
        self.plan.bind(self.conn, verbose)
        self.back_mode = VehicleMode.MISSION

        # Communication properties (positions are local)
        self.safety_radius: float = safety_radius
        self.radar_radius: float = radar_radius

        if verbose:
            print(f"{self.name}: 🚀 lauching")

    def act(self):
        """Perform the next step in the mission plan."""
        self.plan.act()

    def set_mode(self, new_mode: VehicleMode) -> None:
        """Switch the vehicle to a new operational mode."""
        if new_mode != self.mode:
            print(f"{self.name}: Vehicle {self.sysid} switched to mode: 🔁 {new_mode}")
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
    def load_config(sysid: int) -> LogicConfig:
        """Load GCS configuration from a JSON file via command line argument."""
        config_path = DATA_PATH / f"logic_config_{sysid}.json"
        with config_path.open() as f:
            logic_config: LogicConfig = json.load(f)
        return logic_config


def parse_arguments() -> tuple[int, int | None]:
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
        help="verbosity level (e.g. 0,1,2,3)",
    )
    args = parser.parse_args()
    return (args.sysid, args.verbose)


if __name__ == "__main__":
    main()
