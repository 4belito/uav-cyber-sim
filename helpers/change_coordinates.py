import math
from pymavlink import mavextra
import numpy as np
def heading_to_yaw(heading_deg):
    return -math.radians(heading_deg)

def GLOBAL_switch_LOCAL_NED(x,y,z):
    #https://mavlink.io/en/messages/common.html#MAV_FRAME_BODY_FRD
    return (x,-y,-z) 

def global2local(waypoints: np.ndarray, offsets: np.ndarray,pairwise=False) -> np.ndarray:
    """Computes UAV waypoints using NumPy broadcasting."""
    if pairwise:
        assert waypoints.shape == offsets.shape or offsets.ndim==1, 'number of waypoints and offsers must agree'
        uav_wps = waypoints - offsets
    else:
        uav_wps =   waypoints[None, :, :] -offsets[:, None, :]
    return uav_wps

## Taken from sim_vehicle.py
def find_spawns(loc, offsets):
    lat, lon, alt, heading = loc
    spawns = []
    for (x, y, z, head) in offsets:
        if head is None:
            head = heading
        g = mavextra.gps_offset(lat, lon, x, y)
        spawns.append((g[0],g[1],alt+z,head))
    return spawns