"""
Multi-UAV Mission Launcher.

This script mirrors the `5-many_uavs` notebook but exposes CLI switches to pick
between the lightweight run (default), Gazebo, or QGroundControl visualizers.

Usage:
    python run.py --visualizer novis|gazebo|qgc
"""

import argparse
from typing import Callable, Tuple, cast

from config import DATA_PATH, Color
from helpers import ALL_PROCESSES, clean
from helpers.coordinates import ENUPose, GRAPose
from planner import Plan
from planner.plans.auto import AutoPlan
from simulator import QGC, Gazebo, NoVisualizer, Simulator
from simulator.gazebo.preview import GazMarker
from simulator.QGroundControl.qgc import QGCMarker
from simulator.vehicle import SimVehicle, Vehicle
from simulator.visualizer import Visualizer

VISUALIZER_CHOICES = ("novis", "gazebo", "QGroundControl")
GAZEBO_WORLD = "simulator/gazebo/worlds/runway.world"

vis_trajs: dict[str, Callable[[int, int, float], Tuple[float, float, float, float]]] = {
    "novis": lambda i, j, side_len: (i * 50 * side_len, j * 50 * side_len, 0.0, 0.0),
    "QGroundControl": lambda i, j, side_len: (
        (i - 4) * 1.5 * side_len,
        j * side_len,
        0.0,
        0.0,
    ),
    "gazebo": lambda i, j, side_len: (
        ((i - 1) - 0.25) * 1.75 * side_len,
        j * 3 * side_len,
        0.0,
        0.0,
    ),
}

vis_gcs_colors: dict[str, list[Color]] = {
    "novis": [Color.RED, Color.ORANGE, Color.GREEN, Color.BLUE],
    "QGroundControl": [Color.RED, Color.ORANGE, Color.GREEN, Color.BLUE],
    "gazebo": [Color.RED, Color.GREEN, Color.BLUE],
}
vis_uavs_per_gcs: dict[str, int] = {
    "novis": 60,
    "QGroundControl": 25,
    "gazebo": 3,
}


def parse_args():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description="Launch many-UAV mission runs")
    parser.add_argument(
        "--visualizer",
        choices=VISUALIZER_CHOICES,
        default="novis",
        help="Visualizer to use (default: novis/headless)",
    )
    return parser.parse_args()


def make_visualizer(
    choice: str, gra_origin: GRAPose, enu_origin: ENUPose
) -> Visualizer[Vehicle]:
    """Create the requested visualizer."""
    match choice:
        case "gazebo":
            gaz = Gazebo(gra_origin, world_path=GAZEBO_WORLD)
            origin_gaz = GazMarker(
                name="origin",
                group="origin",
                pos=enu_origin.unpose(),
                color=Color.WHITE,
            )
            gaz.markers.append(origin_gaz)
            vis = gaz
        case "QGroundControl":
            qgc = QGC(gra_origin)
            origin_qgc = QGCMarker(
                name="origin",
                pos=gra_origin.unpose(),
                color=Color.WHITE,
            )
            qgc.markers.append(origin_qgc)
            vis = qgc
        case "novis":
            vis = NoVisualizer(gra_origin)
        case _:
            raise ValueError(f"Unknown visualizer choice: {choice}")
    return cast(Visualizer[Vehicle], vis)


def main():
    """Launch a multi-UAV simulation and monitors mission completion."""
    args = parse_args()
    clean(
        victim_processes=[
            proc for proc in ALL_PROCESSES if proc not in {args.visualizer, "run.py"}
        ]
    )

    ## Simulation Positions and Paths
    gra_origin = GRAPose(lat=-35.3633280, lon=149.1652241, alt=0, heading=0)
    enu_origin = ENUPose(x=0, y=0, z=gra_origin.alt, heading=gra_origin.heading)
    visualizer = make_visualizer(args.visualizer, gra_origin, enu_origin)

    gcs_colors = vis_gcs_colors[args.visualizer]
    n_uavs_per_gcs = vis_uavs_per_gcs[args.visualizer]
    side_len = 8
    altitude = 5

    base_homes = ENUPose.list(
        [
            vis_trajs[args.visualizer](i, j, side_len)
            for i in range(len(gcs_colors))
            for j in range(n_uavs_per_gcs)
        ]
    )

    base_paths = [
        Plan.create_square_path(side_len=side_len, alt=altitude, heading=0)
        for _ in base_homes
    ]

    enu_homes = enu_origin.to_abs_all(base_homes)
    gra_homes = gra_origin.to_abs_all(base_homes)
    enu_wptrajs = [
        enu_home.to_abs_all(base_path)
        for enu_home, base_path in zip(enu_homes, base_paths)
    ]
    gra_wptrajs = [
        gra_home.to_abs_all(base_path)
        for gra_home, base_path in zip(gra_homes, base_paths)
    ]

    ## Create Vehicles
    sysids = range(1, len(base_homes) + 1)
    colors: list[Color] = []
    for color in gcs_colors:
        colors.extend([color] * n_uavs_per_gcs)

    vehs: list[SimVehicle] = []
    for sysid, enu_home, gra_wptraj, enu_wptraj, color in zip(
        sysids, enu_homes, gra_wptrajs, enu_wptrajs, colors
    ):
        mission_path = DATA_PATH / f"mission_{sysid}.waypoints"
        plan = AutoPlan(
            name="simple_auto_plan",
            mission_path=str(mission_path),
        )
        plan.save_basic_mission(
            sysid=sysid,
            gra_wps=GRAPose.unpose_all(gra_wptraj),
        )

        veh = SimVehicle(
            sysid=sysid,
            gcs_name=f"{color.name}_{color.emoji}",
            plan=plan,
            color=color,
            home=enu_home,
            waypoints=ENUPose.unpose_all(enu_wptraj),
        )
        vehs.append(veh)

    simulator = Simulator(visualizer=visualizer, terminals=["gcs"], verbose=1)
    for veh in vehs:
        simulator.add_vehicle(veh)

    orac = simulator.launch()
    orac.run()


if __name__ == "__main__":
    main()
