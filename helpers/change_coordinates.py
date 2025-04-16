import math
from pymavlink import mavextra
import numpy as np


def heading_to_yaw(heading_deg):
    return -math.radians(heading_deg)


def GLOBAL_switch_LOCAL_NED(x, y, z):
    # https://mavlink.io/en/messages/common.html#MAV_FRAME_BODY_FRD
    return (x, -y, -z)


def global2local(
    positions: np.ndarray, homes: np.ndarray, pairwise=False
) -> np.ndarray:
    if pairwise:
        assert (
            positions.shape == homes.shape or homes.ndim == 1
        ), "number of positionsa and offsers must agree"
        new_pos = positions - homes
    else:
        new_pos = positions[None, :, :] - homes[:, None, :]
    return new_pos


def local2global(
    positions: np.ndarray, homes: np.ndarray, pairwise=False
) -> np.ndarray:
    """Computes UAV positions using NumPy broadcasting."""
    if pairwise:
        assert (
            positions.shape == homes.shape or homes.ndim == 1
        ), f"number of positions and homes must agree position shape{positions.shape} != {homes.shape}"
        uav_wps = positions + homes
    else:
        uav_wps = positions[None, :, :] + homes[:, None, :]
    return uav_wps


## Taken from sim_vehicle.py and modified to agree with gazeboo
def find_spawns(loc, homes):
    lat, lon, alt, heading = loc
    spawns = []
    for x_north, y_east, z, head in homes:
        if head is None:
            head = heading
        # Swap north and east for gps_offset, which expects (east, north)
        g = mavextra.gps_offset(lat, lon, y_east, x_north)
        spawns.append((g[0], g[1], alt + z, head))
    return spawns
