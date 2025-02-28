import math
from pymavlink import mavextra

def heading_to_yaw(heading_deg):
    return -math.radians(heading_deg)

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