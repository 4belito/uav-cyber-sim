import random
import signal
import time
from collections import defaultdict

from config import Color

# from helpers import clean  # , local2global
from mavlink.customtypes.location import ENUPose, GRAPose
from plan import Plan
from simulator import (
    ConfigGazebo,
    ConfigQGC,
    ConfigVis,
    NoneVisualizer,
    Simulator,
)

signal.signal(signal.SIGTTIN, signal.SIG_IGN)


def main():
    # Clean environment
    # clean()

    # Create Plans
    gra_origin = GRAPose(-35.3633280, 149.1652241, 0, 90)  # east, north, up, heading
    enu_origin = ENUPose(0, 0, gra_origin.alt, gra_origin.heading)

    gcs = [Color.GREEN, Color.ORANGE, Color.RED, Color.BLUE]
    n_uavs_per_gcs = 30
    side_len = 10
    altitude = 5
    max_delay = 10

    base_homes = ENUPose.list(
        [
            (i * 200, j * 200, 0, 0)
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
    msn_delays = [1 for _ in base_homes]

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
    # Optionally: gaz_config.show()

    # QGroundControl Configuration
    qgc_config = ConfigQGC(origin=gra_origin)
    for path, home, color, delay in zip(base_paths, base_homes, colors, msn_delays):
        qgc_config.add(base_path=path, base_home=home, color=color, mission_delay=delay)
    # Optionally: qgc_config.show()

    # No Simulator
    novis_config = ConfigVis[int]()
    for i, _ in enumerate(base_homes):
        novis_config.add_vehicle(i)

    # Visualization Parameters
    novis = NoneVisualizer(novis_config)
    # gaz = Gazebo(gaz_config, gra_origin)
    # qgc = QGC(qgc_config)

    # Launch Simulator
    simulator = Simulator(
        visualizers=[novis],
        gcs_sysids=gcs_sysids,
        missions=[veh.mission for veh in qgc_config.vehicles],
        terminals=["gcs"],
        verbose=1,
    )
    orac = simulator.launch()

    # Main loop: check all UAVs for completion, non-blocking
    while len(orac.conns):
        for sysid in list(orac.conns.keys()):
            if orac.is_plan_done(sysid):  # Make sure this is non-blocking!
                orac.remove(sysid)
        time.sleep(0.1)  # Prevent busy-waiting
    time.sleep(5)
    print("🎉 All UAVs have completed their missions!")


if __name__ == "__main__":
    main()
