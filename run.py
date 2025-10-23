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

from config import Color
from helpers import clean
from helpers.cleanup import ALL_PROCESSES
from helpers.coordinates import ENUPose, GRAPose
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
    gra_origin = GRAPose(lat=-35.3633280, lon=149.1652241, alt=0, heading=90)
    enu_origin = ENUPose(x=0, y=0, z=gra_origin.alt, heading=gra_origin.heading)

    gcs_colors = [Color.RED, Color.ORANGE, Color.GREEN, Color.BLUE]  #
    n_uavs_per_gcs = 60
    side_len = 10
    altitude = 5
    max_delay = 3  # sec

    # novis (i * 50 * side_len, j * 50 * side_len, 0, 0)  4x60 100
    # qgc (i * side_len / 10, j * side_len / 3, 0, 0)     4x30 100
    # gaz (i * 50, j * 3 * side_len, 0, 0)                 3x3 10
    base_homes = ENUPose.list(
        [
            (i * 50 * side_len, j * 3 * side_len, 0, 0)
            for i in range(len(gcs_colors))
            for j in range(n_uavs_per_gcs)
        ]
    )
    base_paths = [
        Plan.create_square_path(side_len=side_len, alt=altitude, heading=0)
        for _ in base_homes
    ]

    colors = [color for color in gcs_colors for _ in range(n_uavs_per_gcs)]

    msn_delays = [random.randint(0, max_delay) for _ in base_homes]

    ## Assign vehicles to GCS (by color)
    gcs_sysids = {
        f"{color.name}_{color.emoji}": list(
            range(i * n_uavs_per_gcs + 1, (i + 1) * n_uavs_per_gcs + 1)
        )
        for i, color in enumerate(gcs_colors)
    }

    # Gazebo Configuration
    gaz_config = ConfigGazebo(
        origin=enu_origin, world_path="simulator/gazebo/worlds/runway3.world"
    )

    for path, home, c in zip(base_paths, base_homes, colors):
        gaz_config.add(base_path=path, base_home=home, color=c)

    # QGroundControl Configuration
    qgc_config = ConfigQGC(origin=gra_origin)

    for path, home, color, delay in zip(base_paths, base_homes, colors, msn_delays):
        qgc_config.add(base_path=path, base_home=home, color=color, mission_delay=delay)

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
        gra_origin=gra_origin,
        visualizer=novis,
        gcs_system_ids=gcs_sysids,
        missions=[veh.mission for veh in qgc_config.vehicles],
        terminals=["gcs"],
        verbose=1,
    )

    orac = simulator.launch()

    orac.run()
    orac.wait_for_trajectory_files()
    orac.plot_trajectories(gra_origin)


if __name__ == "__main__":
    main()
