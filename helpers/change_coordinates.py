"""Utility functions for coordinate transformations."""

import math
from typing import List, Tuple

import numpy as np
from numpy.typing import NDArray
from pymavlink import mavextra  # type: ignore

Offset = Tuple[float, float, float, float]
Position = Tuple[float, float, float]


def heading_to_yaw(heading_deg: float) -> float:
    """Convert compass heading (deg) to yaw (rad)."""
    return -math.radians(heading_deg)


def global_switch_local_ned(pos: Position) -> Position:
    """Flip Y and Z to switch between global and local NED."""
    x, y, z = pos
    return (x, -y, -z)


def local2global_pos(pos: Position, home: Position) -> Position:
    """Convert local NED positions to global frame postions."""
    return (pos[0] + home[0], pos[1] + home[1], pos[2] + home[2])


def local2global(positions: NDArray[np.float64], home: Position) -> NDArray[np.float64]:
    """Convert local NED positions to global frame."""
    return positions + np.asarray(home)


def local2global_batch(
    positions: NDArray[np.float64], homes: NDArray[np.float64]
) -> NDArray[np.float64]:
    """Convert batches of local NED positions to global frame."""
    return positions[None, :, :] + homes[:, None, :]


def global2local(positions: NDArray[np.float64], home: Position) -> NDArray[np.float64]:
    """Convert global positions to local NED frame."""
    return positions - np.asarray(home)


def global2local_batch(
    positions: NDArray[np.float64], homes: NDArray[np.float64]
) -> NDArray[np.float64]:
    """Convert batches of global positions to local NED frame."""
    return positions[None, :, :] - homes[:, None, :]


## Taken from sim_vehicle.py
def find_spawns(loc: Offset, homes: List[Offset]) -> List[Offset]:
    """Compute GPS spawn locations from local NED offsets."""
    lat, lon, alt, _ = loc
    spawns: List[Offset] = []
    for x, y, z, head in homes:
        g = mavextra.gps_offset(lat, lon, x, y)  # type: ignore[reportUnknownMemberType]
        offset = (g[0], g[1], alt + z, head)
        spawns.append(offset)
    return spawns


def localned_to_global_relative_int(
    pos_local: Position, home_gps: Tuple[float, float, float]
) -> Tuple[int, int, float]:
    """
    Convert local NED (meters) to global relative coordinates in
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT frame.
    Returns (lat_int, lon_int, alt_m).
    """
    x, y, z = pos_local
    home_lat, home_lon, _ = home_gps

    # Convert x/y offset to GPS lat/lon
    lat, lon = mavextra.gps_offset(home_lat, home_lon, x, y)  # type: ignore

    # z is down in NED, so negate it to get up (positive altitude)
    alt = -z

    # Convert lat/lon to integer * 1e7 format
    lat_int = int(lat * 1e7)
    lon_int = int(lon * 1e7)

    return lat_int, lon_int, alt
