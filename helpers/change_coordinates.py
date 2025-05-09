import math
from typing import List, Tuple

import numpy as np
from pymavlink import mavextra

Offset = Tuple[float, float, float, float]
Position = Tuple[float, float, float]


def heading_to_yaw(heading_deg: float) -> float:
    return -math.radians(heading_deg)


def GLOBAL_switch_LOCAL_NED(x: float, y: float, z: float) -> tuple:
    return (x, -y, -z)


def local2global(positions: Position, homes: Position):
    return np.asarray(positions) + np.asarray(homes)


def local2global_broadcast(positions: Position, homes: Position):
    return np.asarray(positions)[None, :, :] + np.asarray(homes)[:, None, :]


def global2local(positions: Position, homes: Position):
    return np.asarray(positions) - np.asarray(homes)


def global2local_broadcast(positions: Position, homes: Position):
    return np.asarray(positions)[None, :, :] - np.asarray(homes)[:, None, :]


## Taken from sim_vehicle.py
def find_spawns(loc: Offset, homes: List[Offset]) -> List[Offset]:
    lat, lon, alt, _ = loc
    spawns: List[Offset] = []  # Explicitly type the spawns list
    for x, y, z, head in homes:
        g = mavextra.gps_offset(lat, lon, x, y)  # Explicitly type g
        offset: Offset = (g[0], g[1], alt + z, head)  # Explicitly type offset
        spawns.append(offset)  # Append the offset to spawns
    return spawns
