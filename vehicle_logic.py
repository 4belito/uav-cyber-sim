import time
import json
from pymavlink import mavutil
import math
with open('mode_codes.json', 'r') as file:
    mode_code = json.load(file)

arm_code = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
takeoff_code = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
land_code = mavutil.mavlink.MAV_CMD_NAV_LAND
ext_state_code = mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE
req_msg_code=mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE
on_ground_code=mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
coordinate_frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED
#change_speed_code=mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED
lotier_code=mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM
type_mask = int(0b110111111000)

class VehicleLogic:
    def __init__(self,connection,
                takeoff_pos = None,
                plan=['check_prearm','check_pos_est','mode_stabilize','mode_guided','arm','takeoff','fly','land'], 
                waypoints=[(0,0,5)],
                wp_margin=0.1):
        self.conn=connection
        self.sys = connection.target_system
        self.comp = connection.target_component
        self.takeoff_pos = takeoff_pos
        # Plan
        self.plan = plan
        self.n_actions = len(plan)
    
        # waipoints
        self.wps=waypoints
        self.n_wps= len(self.wps)
        self.wp_margin=wp_margin
        self.reset_plan()
        print(f'vehicle {self.sys} created')


    def reset_plan(self):
        self.action_i = -1
        self.wp_i = 0
        self.is_done = True 
        self.is_reached = True

    def act(self,action):
        if action == 'check_pos_est':
            self.is_done = self.check_for_position_estimate()
        if action == 'check_prearm':
            self.is_done = self.check_for_prearm()
        if action == 'mode_stabilize':
            self.send_mode_stabilize()
            self.is_done = self.check_ack()
        if action == 'mode_guided':
            self.send_mode_guided()
            self.is_done = self.check_ack()
        if action == 'arm':
            self.send_arm()
            self.is_done = self.check_ack()
        if action == 'takeoff':
            wp=self.wps[0]
            self.send_takeoff(altitude=wp[-1])
            self.is_done = self.check_reached(point=wp)
            if self.is_done: self.wp_i+=1
        if action == 'fly':
            if self.wp_i < self.n_wps:
                wp=self.wps[self.wp_i]
                self.send_go_to(point=wp)
                if self.check_reached(point=wp):
                    self.wp_i+=1
            else:
                self.is_done = True
        if action == 'land':
            self.send_land()
            self.is_done=self.check_landed()
        if action == 'hover':
            self.hover(self)

    def act_plan(self):
        if self.is_done: 
            print(f'vehicle {self.sys}: action {self.plan[self.action_i]} is done')
            self.action_i+=1
            self.is_done = False
        if self.action_i < self.n_actions:
            self.act(self.plan[self.action_i])
            return True
        else:
            return False

    def check_for_position_estimate(self):
        """Wait until the UAV has a valid position estimate (EKF is ready)"""
        msg = self.conn.recv_match(type='EKF_STATUS_REPORT', blocking=True)
        if msg:
            return msg.flags & mavutil.mavlink.EKF_POS_HORIZ_ABS
        else:
            return False

    def check_for_prearm(self):
        """ Wait until all pre-arm checks pass """
        msg = self.conn.recv_match(type='SYS_STATUS', blocking=True)
        if msg:
            return msg.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK
        else: 
            return False

    def check_ack(self):
        msg = self.conn.recv_match(type='COMMAND_ACK', blocking=True)
        return msg and (not msg.result)

    def send_mode_stabilize(self):
        self.conn.set_mode(mode_code["STABILIZE"])

    def send_mode_guided(self):
        self.conn.set_mode(mode_code["GUIDED"])

    def send_arm(self):
        self.conn.mav.command_long_send(self.sys, self.comp, arm_code, 0, 1, 0, 0, 0, 0, 0, 0)
    
    def get_position(self):
        msg = self.conn.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        if msg:
            return (msg.x, msg.y, msg.z)

    def get_global_position(self):
        msg = self.conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            # Convert from scaled integers (E7 format) to decimal degrees
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.alt / 1000  # Convert mm to meters
            return lat, lon, alt
        return None  # Return None if no data received



    def check_reached(self,point):
        pos = self.get_position()
        if pos:
            wd_dist = math.dist(pos, point) 
            return wd_dist < self.wp_margin
        else:
            return False
        
    def check_landed(self):
        self.conn.mav.command_long_send(self.sys, self.comp,req_msg_code,0,ext_state_code,0, 0, 0, 0, 0, 0  )
        msg = self.conn.recv_match(type='EXTENDED_SYS_STATE', blocking=True)
        if msg:
            return msg.landed_state == on_ground_code
        else:
            return False
        
    def send_takeoff(self,altitude):
        self.conn.mav.command_long_send(self.sys, self.comp, takeoff_code, 0, 0, 0, 0, 0, 0,0,-altitude)
    
    def send_go_to(self,point):
        go_msg=mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                    10, self.sys, self.comp, coordinate_frame, type_mask, *point, 0, 0, 0, 0, 0, 0, 0, 0)
        self.conn.mav.send(go_msg)

    def send_land(self):
        self.conn.mav.command_long_send(self.sys, self.comp, land_code, 0, 0, 0, 0, 0, 0, 0, 0)

    def send_lotier(self):
        self.conn.mav.command_long_send(self.sys, self.comp, lotier_code, 0, 0, 0, 0, 0, 0, 0, 0)

    def collision_hover(self,r):
        hover_msg=mavutil.mavlink.MAVLink_collision_message(
                    0, # Collision data source
                    1, # Unique identifier, domain based on src field
                    6, # Action that is being taken to avoid this collision
                    2, # How concerned the aircraft is about this collision
                    1, # Estimated time until collision occurs
                    r, # Closest vertical distance between vehicle and object
                    r  # Closest horizontal distance between vehicle and object
                    )
        self.conn.mav.send(hover_msg)
