"""
Launch the OS processes that make up one simulated vehicle.

Exactly one component owns a vehicle's processes: the first GCS assigned to it,
or the Simulator itself when the vehicle is monitored by no GCS at all. Both
call in here so the launch order (MITM -> socat -> ADS-B -> logic -> SITL) is
defined in a single place.
"""

from __future__ import annotations

import logging
import os
import time
from typing import TYPE_CHECKING

from simulator.config import ARDU_LOGS_PATH, ENV_CMD_ARP, ENV_CMD_PYT
from simulator.helpers.processes import SimProcess, create_process

if TYPE_CHECKING:
    from collections.abc import Container
    from subprocess import Popen

    from simulator.configs import GCSVehicleConfig

VehicleProcesses = dict[SimProcess, "Popen[bytes]"]


def wait_for_pty(path: str, timeout: float = 3.0) -> None:
    """Block until socat has created the PTY symlink at `path`."""
    t0 = time.time()
    while not os.path.exists(path):
        if time.time() - t0 > timeout:
            raise RuntimeError(f"PTY not created: {path}")
        time.sleep(0.05)


def launch_vehicle(
    veh_config: GCSVehicleConfig,
    terminals: Container[SimProcess],
    suppress: Container[SimProcess],
) -> VehicleProcesses:
    """
    Spawn every process of one vehicle and return them keyed by role.

    Launch order is MITM -> socat -> ADS-B -> logic -> SITL. The MITM goes first
    so its UDP listeners are bound before Logic and the GCS retarget their
    senders at it.
    """
    sysid = veh_config["sysid"]
    procs: VehicleProcesses = {}
    mitm_enabled = veh_config.get("mitm", False)

    # MITM proxy (interposed on GCS<->Logic links)
    if mitm_enabled and veh_config["mitm_cmd"]:
        p_mitm = create_process(
            veh_config["mitm_cmd"],
            after="exec bash",
            visible=SimProcess.MITM in terminals,
            suppress_output=SimProcess.MITM in suppress,
            title=f"MITM: Vehicle {sysid}",
            env_cmd=ENV_CMD_PYT,
            new_process_group=True,
        )
        logging.debug(f"MITM proxy for vehicle {sysid} launched (PID {p_mitm.pid})")
        procs[SimProcess.MITM] = p_mitm

    # ADS-B virtual cable
    p_socat = create_process(
        veh_config["socat_cmd"],
        after="exec bash",
        visible=SimProcess.ADSB_SOCAT in terminals,
        suppress_output=SimProcess.ADSB_SOCAT in suppress,
        title=f"ADSB socat: Vehicle {sysid}",
        new_process_group=True,
    )
    logging.debug(f"ADSB socat for vehicle {sysid} launched (PID {p_socat.pid})")
    procs[SimProcess.ADSB_SOCAT] = p_socat
    wait_for_pty(f"/tmp/adsb_{sysid}_injector")

    # ADS-B injector
    p_adsb = create_process(
        veh_config["adsb_cmd"],
        after="exec bash",
        visible=SimProcess.ADSB_INJECTOR in terminals,
        suppress_output=SimProcess.ADSB_INJECTOR in suppress,
        title=f"ADSB injector: Vehicle {sysid}",
        env_cmd=ENV_CMD_PYT,
        new_process_group=True,
    )
    logging.debug(f"ADSB injector for vehicle {sysid} launched (PID {p_adsb.pid})")
    procs[SimProcess.ADSB_INJECTOR] = p_adsb

    # Logic
    p_logic = create_process(
        veh_config["logic_cmd"],
        after="exec bash",
        visible=SimProcess.LOGIC in terminals,
        suppress_output=SimProcess.LOGIC in suppress,
        title=f"Vehicle logic: Vehicle {sysid}",
        env_cmd=ENV_CMD_PYT,
        new_process_group=True,
    )
    logging.debug(f"Vehicle logic for vehicle {sysid} launched (PID {p_logic.pid})")
    procs[SimProcess.LOGIC] = p_logic

    # ArduPilot SITL
    ardu_log_folder = ARDU_LOGS_PATH / f"veh_{sysid}"
    ardu_log_folder.mkdir(parents=True, exist_ok=True)
    p_ard = create_process(
        veh_config["ardupilot_cmd"],
        after="exec bash",
        visible=SimProcess.ARDUPILOT in terminals,
        suppress_output=SimProcess.ARDUPILOT in suppress,
        title=f"ArduPilot SITL Launcher: Vehicle {sysid}",
        env_cmd=ENV_CMD_ARP,
        new_process_group=True,
        cwd=str(ardu_log_folder),
    )
    logging.debug(f"ArduPilot SITL vehicle {sysid} launched (PID {p_ard.pid})")
    procs[SimProcess.ARDUPILOT] = p_ard

    return procs
