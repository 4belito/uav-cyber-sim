import math
from pymavlink import mavextra

def heading_to_yaw(heading_deg):
    return -math.radians(heading_deg)

def GLOBAL_switch_LOCAL_NED(x,y,z):
    #https://mavlink.io/en/messages/common.html#MAV_FRAME_BODY_FRD
    return (x,-y,-z) 


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