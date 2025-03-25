from pymavlink import mavutil
import math
import numpy as np
from typing import Callable
from functools import partial
from typing import Optional

from plan.planner import Plan
from helpers.change_coordinates import GLOBAL_switch_LOCAL_NED


class VehicleLogic:
    def __init__(self,
                sys_id:int,
                home: np.ndarray,
                plan: Optional[Plan] = None, 
                verbose=1):
        self.sys_id = sys_id
        self.conn=mavutil.mavlink_connection(f'udp:127.0.0.1:{14551+10*(sys_id-1)}')
        self.conn.wait_heartbeat()
        self.home = home
        self.verbose = verbose
        self.plan= plan if plan is not None else Plan.basic()

        # Plan
        self.act_plan = partial(self.plan.act, connection=self.conn)

        print(f'vehicle {self.sys_id} created')

       
    def reset_plan(self):
        pass

    def current_action(self):
        return self.plan.current
    
    def current_step(self):
        return self.current_action().current

    def current_wp(self):
        return self.current_action()=='fly' and self.wps[self.wp_i]




    def get_global_position(self):
        msg = self.conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            # Convert from scaled integers (E7 format) to decimal degrees
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.alt / 1000  # Convert mm to meters
            return lat, lon, alt
        return None  # Return None if no data received


    def move_left(self, speed=0.5):
        """
        Moves the UAV left by overriding position commands with a velocity command.
        
        - speed: Speed in m/s (default 0.5 m/s)
        """
        print(f"Moving left at {speed} m/s")
        # Step 1: Cancel Position Control (ignore position commands)
        type_mask = int(0b010111000111)  # Only use velocity, ignore position & acceleration
        coordinate_frame = mavutil.mavlink.MAV_FRAME_BODY_NED  # Relative to drone

        # Step 2: Send Velocity Command
        vy = -speed  # Negative vy moves left

        msg = mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            10, self.sys, self.comp, coordinate_frame, type_mask,
            0, 0, 0,  # Position ignored
            0, vy, 0,  # Move left (velocity command)
            0, 0, 0,  # Acceleration ignored
            0, 0  # Ignore yaw
        )
        print('turn left msg will be sent')
        self.conn.mav.send(msg)
        print('turn left msgsent')
        # Wait for acknowledgment before continuing
        while not self.is_acknowledged():
            print(f"Waiting moving left acknowledge")
            pass  # Keep checking for an acknowledgment

            

    