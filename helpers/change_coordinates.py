import math

import numpy as np
from pymavlink import mavextra


def heading_to_yaw(heading_deg):
    return -math.radians(heading_deg)


def GLOBAL_switch_LOCAL_NED(x: float, y: float, z: float) -> tuple:
    return (x, -y, -z)


def local2global(positions, homes):
    return np.asarray(positions) + np.asarray(homes)


def local2global_broadcast(positions, homes):
    return np.asarray(positions)[None, :, :] + np.asarray(homes)[:, None, :]


def global2local(positions, homes):
    return np.asarray(positions) - np.asarray(homes)


def global2local_broadcast(positions, homes):
    return np.asarray(positions)[None, :, :] - np.asarray(homes)[:, None, :]


## Taken from sim_vehicle.py
def find_spawns(loc, homes):
    lat, lon, alt, heading = loc
    spawns = []
    for x, y, z, head in homes:
        if head is None:
            head = heading
        g = mavextra.gps_offset(lat, lon, x, y)
        spawns.append((g[0], g[1], alt + z, head))
    return spawns
