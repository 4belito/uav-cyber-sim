"""Multi-Vehicle MAVLink Logic."""

from __future__ import annotations

import argparse
import json
import logging
import time

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from simulator.config import DATA_PATH, LOGS_PATH, BasePort
from simulator.configs import LogicConfig
from simulator.entities.riddata import RIDData
from simulator.helpers.connections import (
    MAVConnection,
    create_tcp_conn,
    create_udp_conn,
    send_heartbeat,
    # wait_for_port,
)
from simulator.helpers.connections.mavlink.customenums.customcmd import CustomCmd
from simulator.helpers.connections.mavlink.enums import DataStream, MsgID
from simulator.helpers.connections.mavlink.streams import (
    ask_msg,
    request_sensor_streams,
)
from simulator.helpers.coordinates import ENU, GRA
from simulator.helpers.logging.data_logger import DataLogger
from simulator.helpers.logging.setup_log import setup_logging
from simulator.helpers.math import connection_id
from simulator.params.simulation import (
    DATA_STREAM_FREQUENCY,
    HEARTBEAT_FREQUENCY,
    REMOTE_ID_FREQUENCY,
)
from simulator.planner import Action, Plan, PlanSpec, State, Step
from simulator.runtime.vehicle.mav_manager import MAVLinkManager
from simulator.runtime.vehicle.rid_manager import RIDManager
from simulator.runtime.vehicle.state import VehicleStateP

DATA_STREAM_IDS = [
    DataStream.RAW_SENSORS,
    DataStream.EXTENDED_STATUS,
    DataStream.POSITION,
    DataStream.EXTRA1,  # Required to receive some ArduPilot custom telemetry
    DataStream.EXTRA2,
]
RID_INTERVAL = int(1_000_000 / REMOTE_ID_FREQUENCY)

# TODO: Refactor this module
heartbeat_event = mavutil.periodic_event(HEARTBEAT_FREQUENCY)
rid_event = mavutil.periodic_event(REMOTE_ID_FREQUENCY)


def wait_for_vehicle_link(
    vehicle_state: VehicleStateP,
    timeout: float = 10.0,
) -> mavlink.MAVLink_heartbeat_message:
    """Wait until the first HEARTBEAT is received from the vehicle."""
    hb = vehicle_state.wait_for("HEARTBEAT", timeout=timeout)
    if hb is None:
        raise TimeoutError("Timed out waiting for vehicle HEARTBEAT")
    return hb


def main():
    """Entry point for the Multi-Vehicle MAVLink Logic."""
    config_path, verbose = parse_arguments()
    config = VehicleLogic.load_config(config_path)
    setup_logging(
        LOGS_PATH / "logics" / f"logic_{config['sysid']}.log",
        verbose=verbose or 1,
        console_output=True,
    )
    start_logic(config)


# TODO: Remove monitored items from config
def start_logic(config: LogicConfig):
    """Start bidirectional proxy for a given Vehicle system_id."""
    sysid = config["sysid"]
    port_offset = config["port_offset"]
    gra_orign = GRA(**config["gra_origin_dict"])
    plan_spec = PlanSpec(**config["plan_spec"])

    # wait_for_port(BasePort.ARP + port_offset, verbose=True)
    ap_conn = create_tcp_conn(
        base_port=BasePort.ARP,
        offset=port_offset,
        role="client",
        src_sysid=connection_id(sysid),
        src_compid=140,
    )
    logging.debug(f"Vehicle {sysid}: Logic connection established")
    cs_conn = create_udp_conn(
        base_port=BasePort.GCS,
        offset=port_offset,
        mode="sender",
        src_sysid=1,
        src_compid=140,
    )
    logging.debug(f"Vehicle {sysid}: GCS connection established")

    # Shared telemetry state
    data_logger = DataLogger(path=DATA_PATH / "msgs", sysid=sysid)
    rid_mng = RIDManager(sysid, port_offset, gra_orign, data_logger=data_logger)
    # Router stop signal
    mav_mng = MAVLinkManager(
        conn=ap_conn,
        data_logger=data_logger,
    )

    ap_conn.wait_heartbeat()
    logging.debug("MAVLink connection established")

    msg = ask_msg(ap_conn, MsgID.GLOBAL_POSITION_INT, interval=RID_INTERVAL)
    mav_mng.send(msg)

    stream_msgs = request_sensor_streams(
        ap_conn,
        stream_ids=DATA_STREAM_IDS,
        rate_hz=DATA_STREAM_FREQUENCY,
    )
    for msg in stream_msgs.values():
        mav_mng.send(msg)

    mav_mng.start()
    logging.debug(f"Vehicle {sysid}: MAVLink router started")
    hb = wait_for_vehicle_link(mav_mng.state, timeout=10.0)
    logging.debug(
        "Vehicle %s: first heartbeat received from system=%s component=%s",
        sysid,
        hb.get_srcSystem(),
        hb.get_srcComponent(),
    )

    rid_mng.start()

    plan = Plan.build(plan_spec)
    logic = VehicleLogic(
        plan=plan,
        gra_origin=gra_orign,
        mav_manager=mav_mng,
    )

    try:
        while True:
            if heartbeat_event.trigger():
                send_heartbeat(ap_conn)
                send_heartbeat(cs_conn)
            if rid_event.trigger():
                pos = mav_mng.state.get("GLOBAL_POSITION_INT")
                if pos:
                    rid_mng.update(pos.to_dict())
                if rid_mng.pending:
                    try:
                        logic.rid = rid_mng.data
                        rid_mng.publish()
                    except Exception as e:
                        logging.error(f"Error sending RID data: {e}")
                        pass
            if logic.plan.state == State.DONE:
                logic.send_done_msgs(cs_conn)
                break

            logic.act()
            time.sleep(0.01)
    finally:
        # 1. stop producers
        mav_mng.stop()
        rid_mng.stop()

        # 2. close connections
        cs_conn.close()

        # 3. close logger
        data_logger.close()

        logging.info(f"Vehicle {sysid} logic stopped")


class VehicleLogic:
    """Handles the logic for executing a Vehicle's mission plan."""

    def __init__(
        self,
        plan: Plan,
        gra_origin: GRA,
        mav_manager: MAVLinkManager,
    ):
        # Vehicle Creation
        self.conn = mav_manager.conn
        self.sysid = mav_manager.data_logger.sysid
        self.name = f"Logic 🧠 {self.sysid}"
        self.gra_origin = gra_origin
        self.vehicle_state = mav_manager.state

        # Plan
        self.plan = plan
        # TODO: Pass the plan alread binded
        self.plan.bind(self.gra_origin, mav_manager)

        # Communication properties (positions are local)
        self.rid: RIDData | None = None

        logging.info(f"{self.name}: launching")

    def act(self):
        """Perform the next step in the mission plan."""
        self.plan.act()
        time.sleep(0.01)  # Avoid busy loop if plan.act() returns immediately

    def send_done_msgs(self, cs_conn: MAVConnection) -> None:
        """Notify the GCS that the mission is done."""
        done_msg = mavlink.MAVLink_statustext_message(severity=6, text=b"LOGIC_DONE")
        logging.info(f"GCS ← Logic {self.sysid}: Sending LOGIC_DONE")
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
        Assumes `conn` is a dedicated MAVLink connection for one Vehicle.
        """
        i = 0
        while i < max_tries:
            logging.debug(f"GCS ← Vehicle {self.sysid}: Sending DONE (attempt {i + 1})")
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
        """Return the current estimated position of the Vehicle."""
        return self.plan.curr_pos

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
    parser = argparse.ArgumentParser(description="Single Vehicle MAVLink Logic")
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
