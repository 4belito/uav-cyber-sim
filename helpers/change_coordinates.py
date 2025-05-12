"""
Utility functions for coordinate transformations between local NED and global GPS
frames.
"""

import math
from typing import List, Tuple

import numpy as np
from numpy.typing import NDArray
from pymavlink import mavextra

Offset = Tuple[float, float, float, float]
Position = Tuple[float, float, float]


def heading_to_yaw(heading_deg: float) -> float:
    return -math.radians(heading_deg)


def global_switch_local_ned(pos: Position) -> Position:
    x, y, z = pos
    return (x, -y, -z)


def local2global(positions: NDArray[np.float64], home: Position) -> NDArray[np.float64]:
    return positions + np.asarray(home)


def local2global_batch(
    positions: NDArray[np.float64], homes: NDArray[np.float64]
) -> NDArray[np.float64]:
    return positions[None, :, :] + homes[:, None, :]


def global2local(positions: NDArray[np.float64], home: Position) -> NDArray[np.float64]:
    return positions - np.asarray(home)


def global2local_batch(
    positions: NDArray[np.float64], homes: NDArray[np.float64]
) -> NDArray[np.float64]:
    return positions[None, :, :] - homes[:, None, :]


## Taken from sim_vehicle.py
def find_spawns(loc: Offset, homes: List[Offset]) -> List[Offset]:
    lat, lon, alt, _ = loc
    spawns: List[Offset] = []  # Explicitly type the spawns list
    for x, y, z, head in homes:
        g = mavextra.gps_offset(lat, lon, x, y)  # Explicitly type g
        offset: Offset = (g[0], g[1], alt + z, head)  # Explicitly type offset
        spawns.append(offset)  # Append the offset to spawns
    return spawns
