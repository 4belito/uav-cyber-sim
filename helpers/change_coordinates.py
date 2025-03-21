import math
from pymavlink import mavextra
import numpy as np
def heading_to_yaw(heading_deg):
    return -math.radians(heading_deg)

def GLOBAL_switch_LOCAL_NED(x,y,z):
    #https://mavlink.io/en/messages/common.html#MAV_FRAME_BODY_FRD
    return (x,-y,-z) 

def global2local(positions: np.ndarray, homes: np.ndarray,pairwise=False) -> np.ndarray:
    if pairwise:
        assert positions.shape == homes.shape or homes.ndim==1, 'number of positions and offsers must agree'
        new_pos = positions - homes
    else:
        new_pos =   positions[None, :, :] -homes[:, None, :]
    return new_pos

def local2global(positions: np.ndarray, homes: np.ndarray,pairwise=False) -> np.ndarray:
    """Computes UAV positions using NumPy broadcasting."""
    if pairwise:
        assert positions.shape == homes.shape or homes.ndim==1, 'number of positions and homes must agree'
        uav_wps = positions + homes
    else:
        uav_wps =   positions[None, :, :] + homes[:, None, :]
    return uav_wps



## Taken from sim_vehicle.py
def find_spawns(loc, homes):
    lat, lon, alt, heading = loc
    spawns = []
    for (x, y, z, head) in homes:
        if head is None:
            head = heading
        g = mavextra.gps_offset(lat, lon, x, y)
        spawns.append((g[0],g[1],alt+z,head))
    return spawns