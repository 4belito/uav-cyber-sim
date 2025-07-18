"""
Multi-UAV Mission Launcher.

This script launches a multi-UAV simulation using predefined mission plans
and visualizers. It assigns UAVs to Ground Control Stations (GCS) by color,
creates square paths with random delays, and monitors mission completion.

Run from command line for faster output than notebooks:

    python multi_uav_launcher.py
"""

import random
import signal
from collections import defaultdict

from config import Color
from helpers import clean

# from helpers import clean  # , local2global
from helpers.cleanup import ALL_PROCESSES
from mavlink.customtypes.location import ENUPose, GRAPose
from plan import Plan
from simulator import (
    QGC,
    ConfigGazebo,
    ConfigNovis,
    ConfigQGC,
    Gazebo,
    NoVisualizer,
    Simulator,
)

signal.signal(signal.SIGTTIN, signal.SIG_IGN)
ALL_PROCESSES.remove("run_many_uavs.py")


def main():
    """Launch a multi-UAV simulation and monitors mission completion."""
    clean(ALL_PROCESSES)

    # Create Plans
    gra_origin = GRAPose(-35.3633280, 149.1652241, 0, 90)  # east, north, up, heading
    enu_origin = ENUPose(0, 0, gra_origin.alt, gra_origin.heading)

    gcs = [Color.RED, Color.ORANGE, Color.GREEN, Color.BLUE]
    n_uavs_per_gcs = 10
    side_len = 10
    altitude = 5
    max_delay = 3

    base_homes = ENUPose.list(
        [
            (i * 50, j * 3 * side_len, 0, 0)
            for i in range(len(gcs))
            for j in range(n_uavs_per_gcs)
        ]
    )
    base_paths = [
        Plan.create_square_path(side_len=side_len, alt=altitude, heading=0)
        for _ in base_homes
    ]

    colors = [color for color in gcs for _ in range(n_uavs_per_gcs)]

    msn_delays = [random.randint(0, max_delay) for _ in base_homes]

    # Assign vehicles to GCS (by color)
    gcs_sysids: dict[str, list[int]] = defaultdict(list)
    for i, color in enumerate(colors, start=1):
        gcs_sysids[f"{color.name} {color.emoji}"].append(i)

    # Gazebo Configuration
    gaz_config = ConfigGazebo(
        origin=enu_origin, world_path="simulator/gazebo/worlds/runway3.world"
    )
    for path, home, c in zip(base_paths, base_homes, colors):
        gaz_config.add(base_path=path, base_home=home, color=c)
    # gaz_config.show()

    # QGroundControl Configuration
    qgc_config = ConfigQGC(origin=gra_origin)
    for path, home, color, delay in zip(base_paths, base_homes, colors, msn_delays):
        qgc_config.add(base_path=path, base_home=home, color=color, mission_delay=delay)
    # qgc_config.show()

    # No Visualizer
    novis_config = ConfigNovis(origin=gra_origin)
    for home in base_homes:
        novis_config.add(base_home=home)

    # Visualization Parameters
    novis = NoVisualizer(novis_config)  # type: ignore  # noqa: F841
    gaz = Gazebo(gaz_config, gra_origin)  # type: ignore  # noqa: F841
    qgc = QGC(qgc_config)  # type: ignore  # noqa: F841

    # Launch Simulator
    simulator = Simulator(
        visualizers=[novis],
        gcs_sysids=gcs_sysids,
        missions=[veh.mission for veh in qgc_config.vehicles],
        terminals=["gcs"],
        verbose=1,
    )
    orac = simulator.launch()

    # Main loop: check all UAVs for completion
    orac.run()
    print("🎉 All UAVs have completed their missions!")


if __name__ == "__main__":
    main()
