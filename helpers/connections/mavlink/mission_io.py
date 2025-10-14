"""Helpers for saving/loading MAVLink missions (.waypoints files)."""

from pathlib import Path

from ...coordinates import GRAs
from .enums import Cmd, CmdNav, Frame


def save_mission(
    path: Path, poses: GRAs, delay: int = 0, speed: float = 5.0, land: bool = True
) -> list[int]:
    """
    Write a mission to `path` using the QGroundControl `.waypoints` format.

    Returns the indices of the generated mission items corresponding to the
    user-provided poses. These indices are meant for downstream monitoring.
    """
    WP = CmdNav.WAYPOINT.value
    TAKEOFF = CmdNav.TAKEOFF.value
    LAND = CmdNav.LAND.value
    REL_ALT = Frame.GLOBAL_RELATIVE_ALT.value
    DELAY = CmdNav.DELAY.value
    SET_SPEED = Cmd.DO_CHANGE_SPEED.value
    idx = 0
    # first and last items(takeoff and land) are included for ask only not to reach
    msn_items: list[int] = []
    with path.open("w") as f:
        f.write("QGC WPL 110\n")

        # Home location
        home = poses[0]
        f.write(
            f"{idx}\t0\t{REL_ALT}\t{WP}\t0\t0\t0\t0\t{home.lat:.7f}\t{home.lon:.7f}\t0.0\t1\n"
        )
        idx += 1
        # Dalay mission (mission item with delay=0 does not work)
        if delay:
            f.write(
                f"{idx}\t0\t{REL_ALT}\t{DELAY}\t{delay}\t0\t0\t0\t{home.lat:.7f}\t{home.lon:.7f}\t0.0\t1\n"
            )
            idx += 1
        # Takeoff
        f.write(
            f"{idx}\t0\t{REL_ALT}\t{TAKEOFF}\t0\t0\t0\t0\t{home.lat:.7f}\t{home.lon:.7f}\t{home.alt:.1f}\t1\n"
        )
        msn_items.append(idx)
        idx += 1
        if speed != 5.0:
            # param1 = 0 → airspeed, 1 → ground speed, 2 → climb rate
            # param2 = target speed (in m/s)
            # param3 = throttle (usually -1 = unchanged)
            speed_type = 1
            throttle = -1
            f.write(
                f"{idx}\t0\t{REL_ALT}\t{SET_SPEED}\t{speed_type}\t{speed}\t{throttle}\t0\t0\t0\t0.0\t1\n"
            )
            msn_items.append(idx)
            idx += 1

        # Mission waypoints
        for i, pose in enumerate(poses[1:], start=idx):
            f.write(
                f"{i}\t0\t{REL_ALT}\t{WP}\t0\t0\t0\t0\t{pose.lat:.7f}\t{pose.lon:.7f}\t{pose.alt:.1f}\t1\n"
            )
            msn_items.append(i)
        idx += len(poses) - 1

        # land at the last waypoint
        if land:
            last = poses[-1]
            f.write(
                f"{idx}\t0\t{REL_ALT}\t{LAND}\t0\t0\t0\t0\t{last.lat:.7f}\t{last.lon:.7f}\t0.0\t1\n"
            )
            msn_items.append(idx)

        return msn_items
