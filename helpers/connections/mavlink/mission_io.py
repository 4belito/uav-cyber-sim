"""Helpers for saving/loading MAVLink missions (.waypoints files)."""

from pathlib import Path

from ...coordinates import GRAs
from .enums import CmdNav, Frame


def save_mission(path: Path, poses: GRAs, delay: int = 0) -> None:
    """
    Save a .waypoints file from a sequence of GRAPose ppositions.
    The file will include:
    - Home (index 0, altitude = 0).
    - Takeoff (index 1, to the first pose with alt).
    - Mission waypoints (indices 2 to N).
    - Return to launch (last index, with alt = 0).
    """
    WP = CmdNav.WAYPOINT.value
    TAKEOFF = CmdNav.TAKEOFF.value
    LAND = CmdNav.LAND.value
    REL_ALT = Frame.GLOBAL_RELATIVE_ALT.value
    DELAY = CmdNav.DELAY.value
    takeoff_idx = 1
    with path.open("w") as f:
        f.write("QGC WPL 110\n")

        # Home location
        home = poses[0]
        f.write(
            f"0\t0\t{REL_ALT}\t{WP}\t0\t0\t0\t0\t{home.lat:.7f}\t{home.lon:.7f}\t0.0\t1\n"
        )
        # Dalay mission (mission item with delay=0 does not work)
        if delay:
            f.write(
                f"1\t0\t{REL_ALT}\t{DELAY}\t{delay}\t0\t0\t0\t{home.lat:.7f}\t{home.lon:.7f}\t0.0\t1\n"
            )
            takeoff_idx += 1

        f.write(
            f"{takeoff_idx}\t0\t{REL_ALT}\t{TAKEOFF}\t0\t0\t0\t0\t{home.lat:.7f}\t{home.lon:.7f}\t{home.alt:.1f}\t1\n"
        )

        # Mission waypoints
        for i, pose in enumerate(poses[1:], start=takeoff_idx + 1):
            f.write(
                f"{i}\t0\t{REL_ALT}\t{WP}\t0\t0\t0\t0\t{pose.lat:.7f}\t{pose.lon:.7f}\t{pose.alt:.1f}\t1\n"
            )

        # Return to Launch (RTL)
        last = poses[-1]
        rtl_index = len(poses) + takeoff_idx
        f.write(
            f"{rtl_index}\t0\t{REL_ALT}\t{LAND}\t0\t0\t0\t0\t{last.lat:.7f}\t{last.lon:.7f}\t0.0\t1\n"
        )
