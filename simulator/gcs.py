"""
Define the GCS class to monitor UAVs through MAVLink messages and run GCS
instances.
"""

import argparse
import json
import logging
import os
import pickle
import time
from concurrent import futures
from typing import TypedDict

import zmq
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from simulator.config import DATA_PATH, ENV_CMD_ARP, ENV_CMD_PYT, BasePort
from simulator.helpers.connections import (
    MAVConnection,
    create_udp_conn,
    create_zmq_socket,
)
from simulator.helpers.connections.mavlink.customenums.customcmd import CustomCmd
from simulator.helpers.coordinates import GRAs
from simulator.helpers.processes import create_process
from simulator.helpers.setup_log import setup_logging
from simulator.monitor import UAVMonitor
from simulator.params.simulation import HEARTBEAT_FREQUENCY

heartbeat_event = mavutil.periodic_event(HEARTBEAT_FREQUENCY)


def main():
    """Run a GCS instance to monitor UAVs."""
    config_path, verbose = parse_arguments()
    with open(config_path) as f:
        config = json.load(f)
    setup_logging(f"GCS_{config['name']}", verbose=verbose, console_output=True)
    gcs = GCS(**config)
    gcs.run()


class UAVGCSConfig(TypedDict):
    """TypedDict for UAV configuration in the GCS."""

    sysid: int
    port_offset: int
    ardupilot_cmd: str
    logic_cmd: str
    proxy_cmd: str


class GCSConfig(TypedDict):
    """Ground Control Station (GCS) Configuration."""

    name: str
    port_offset: int
    uavs: list[UAVGCSConfig]
    terminals: list[str]
    suppress: list[str]


class GCS(UAVMonitor):
    """Ground Control Station class extending Oracle with trajectory logging."""

    def __init__(
        self,
        uavs: list[UAVGCSConfig],
        name: str,
        port_offset: int,
        terminals: list[str],
        suppress: list[str],
    ) -> None:
        # Configure logging for this GCS process
        self.name = name
        self.uavs = uavs
        self.sysids = [uavconfig["sysid"] for uavconfig in uavs]
        self.n_uavs = len(self.sysids)
        self.terminals = set(terminals)
        self.suppress = set(suppress)
        self.conns = self._launch_vehicles()  # NOTE: add self.conn =

        self.zmq_ctx = zmq.Context()
        self.orc_sock = create_zmq_socket(
            self.zmq_ctx, zmq.PUB, BasePort.GCS_ZMQ, port_offset
        )

        super().__init__(dict(zip(self.sysids, self.conns)))
        self.paths: dict[int, GRAs] = {sysid: [] for sysid in self.sysids}

        logging.info(f" GCS {self.name} started with {self.n_uavs} UAVs")

    ###
    def run(self):
        """Run the GCS monitoring loop until all UAVs complete their missions."""
        with futures.ThreadPoolExecutor() as executor:
            executor.map(self._monitor_uav, self.sysids)

        logging.info("All UAVs assigned have completed their missions")
        self.orc_sock.send_string("DONE")  # type: ignore
        logging.info("DONE message sent to Oracle")
        self.orc_sock.close(linger=0)
        self.zmq_ctx.term()

        trajectory_file = DATA_PATH / f"trajectories_{self.name}.pkl"
        with open(trajectory_file, "wb") as file:
            pickle.dump(self.paths, file)
        logging.info(f"Trajectories saved to '{trajectory_file}'")

    def save_pos(self):
        """Save the current global position of each UAV to their trajectory path."""
        for sysid, pos in self.pos.items():
            self.paths[sysid].append(pos)

    def _monitor_uav(self, sysid: int):
        logging.info(f"Monitoring UAV {sysid}")
        while not self.is_plan_done(sysid):
            self.get_global_pos(sysid)
            self.save_pos()

    def _handle_message(self, sysid: int, msg: mavlink.MAVLink_message) -> bool:
        """Process MAVLink messages from a UAV."""
        if msg.get_type() == "STATUSTEXT":
            text = getattr(msg, "text", b"")
            if isinstance(text, bytes):
                text = text.decode(errors="ignore")

            if text == "LOGIC_DONE":
                logging.info(f"GCS: received LOGIC_DONE from UAV {sysid}")

                # Send COMMAND_ACK back
                ack = mavutil.mavlink.MAVLink_command_ack_message(
                    command=CustomCmd.LOGIC_DONE,
                    result=0,
                )
                self.conns[sysid].mav.send(ack)
                logging.info(f"GCS: sent LOGIC_DONE ACK to UAV {sysid}")

                # Mark UAV as done
                self.remove_uav(sysid)
                return True

        return False

    def _launch_vehicles(self) -> list[MAVConnection]:
        """Launch ArduPilot and logic processes for each UAV."""
        with futures.ThreadPoolExecutor() as executor:
            conns = list(executor.map(self._launch_vehicle, range(self.n_uavs)))
        return conns

    def _launch_vehicle(self, i: int) -> MAVConnection:
        uav_config = self.uavs[i]
        sysid = uav_config["sysid"]

        # -----------------------
        # 1. ADS-B virtual cable
        # -----------------------
        socat_cmd = (
            f"socat -d -d "
            f"pty,raw,echo=0,link=/tmp/adsb_{sysid}_ardupilot "
            f"pty,raw,echo=0,link=/tmp/adsb_{sysid}_injector"
        )

        p = create_process(
            socat_cmd,
            after="exec bash",
            visible="adsb_socat" in self.terminals,
            suppress_output="adsb_socat" in self.suppress,
            title=f"ADSB socat: Vehicle {sysid}",
        )
        logging.debug(f"ADSB socat for vehicle {sysid} launched (PID {p.pid})")
        self._wait_for_pty(f"/tmp/adsb_{sysid}_injector")

        # -----------------------
        # 2. ADS-B injector
        # -----------------------
        adsb_cmd = (
            f"python3 -m simulator.adsb_injector "
            f"--uart /tmp/adsb_{sysid}_injector "
            f"--port-offset {uav_config['port_offset']}"
        )

        p = create_process(
            adsb_cmd,
            after="exec bash",
            visible="adsb_injector" in self.terminals,
            suppress_output="adsb_injector" in self.suppress,
            title=f"ADSB injector: Vehicle {sysid}",
            env_cmd=ENV_CMD_PYT,
        )
        logging.debug(f"ADSB injector for vehicle {sysid} launched (PID {p.pid})")

        # -----------------------
        # 3. ArduPilot + Proxy + Logic
        # -----------------------
        p = create_process(
            uav_config["logic_cmd"],
            after="exec bash",
            visible="logic" in self.terminals,
            suppress_output="logic" in self.suppress,
            title=f"UAV logic: Vehicle {sysid}",
            env_cmd=ENV_CMD_PYT,
        )  # "exit"
        logging.debug(f"UAV logic for vehicle {sysid} launched (PID {p.pid})")

        p = create_process(
            uav_config["ardupilot_cmd"],
            after="exec bash",
            visible="launcher" in self.terminals,
            suppress_output="launcher" in self.suppress,
            title=f"ArduPilot SITL Launcher: Vehicle {sysid}",
            env_cmd=ENV_CMD_ARP,
        )  # "exit"
        logging.debug(f"ArduPilot SITL vehicle {sysid} launched (PID {p.pid})")

        conn = create_udp_conn(
            base_port=BasePort.GCS,
            offset=uav_config["port_offset"],
            mode="receiver",
            src_sysid=255,  # estándar GCS sysid
            src_compid=190,  # estándar GCS commponent ID
        )
        logging.info(f"UAV {sysid} connected")
        return conn

    @staticmethod
    def load_config(config_path: str) -> GCSConfig:
        """Load GCS configuration from a JSON file via command line argument."""
        with open(config_path) as f:
            gcs_config: GCSConfig = json.load(f)
        return gcs_config

    def _wait_for_pty(self, path: str, timeout: float = 3.0):
        t0 = time.time()
        while not os.path.exists(path):
            if time.time() - t0 > timeout:
                raise RuntimeError(f"PTY not created: {path}")
            time.sleep(0.05)


def parse_arguments() -> tuple[str, int]:
    """Parse List of GCS system IDs and GCS name."""
    parser = argparse.ArgumentParser(description="Single GCS")
    parser.add_argument(
        "--config-path",
        type=str,
        required=True,
        help="Path to the GCS configuration file (e.g. gcs_config_1.json)",
    )
    parser.add_argument(
        "--verbose",
        type=int,
        required=False,
        default=1,
        help="Verbosity level (0=silent, 1=normal, 2=debug)",
    )
    args = parser.parse_args()
    return args.config_path, args.verbose


if __name__ == "__main__":
    main()
