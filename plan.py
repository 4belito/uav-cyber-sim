import time
import json
from pymavlink import mavutil
import math
from helpers.change_coordinates import GLOBAL_switch_LOCAL_NED
with open('mode_codes.json', 'r') as file:
    mode_code = json.load(file)


from typing import Callable

#Mavlink shorts

# Commands
ARM_CODE = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
TAKEOFF_CODE = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
LAND_CODE  = mavutil.mavlink.MAV_CMD_NAV_LAND
REQ_MSG_CODE = mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE
LOITER_CODE = mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM
# CHANGE_SPEED_CODE = mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED

# Messages
EXT_STATE_CODE = mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE
ON_GROUND_CODE = mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND


# Combine required sensors using MAVLink constants
REQUIRED_SENSORS = (
    mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO |
    mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
    mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG |
    mavutil.mavlink.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
    mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS
)

EKF_REQUIRED_FLAGS = (
    mavutil.mavlink.EKF_ATTITUDE |
    mavutil.mavlink.EKF_VELOCITY_HORIZ |
    mavutil.mavlink.EKF_POS_HORIZ_ABS |
    mavutil.mavlink.EKF_POS_VERT_ABS
)






class VehicleLogic:
    def __init__(self,sys_id,
                home = None,
                plan=['check_prearm','check_pos_est','mode_stabilize','mode_guided','arm','takeoff','fly','land'], 
                waypoints=[(0,0,5)],
                wp_margin=0.1,
                verbose=1,
                wpnv_speed=None):
        self.conn=mavutil.mavlink_connection(f'udp:127.0.0.1:{14551+10*(sys_id-1)}')
        self.conn.wait_heartbeat()
        self.sys = self.conn.target_system
        self.comp = self.conn.target_component
        self.home = home
        # Plan
        self.plan = plan
        self.n_actions = len(plan)
        self.plan.append('off')
        if wpnv_speed: self.set_wp_nav_speed(wpnv_speed)

        # waipoints
        self.wps=waypoints
        self.n_wps= len(self.wps)
        self.wp_margin=wp_margin
        self.verbose = verbose
        self.reset_plan()
        print(f'vehicle {self.sys} created')

    def update_waypoints(self,new_waypoints):
        self.wps=new_waypoints
        self.n_wps= len(self.wps)
        self.wp_i = 0
        self.wp_reached=False
        
    def reset_plan(self):
        self.action_i = 0
        self.wp_i = 0
        self.wp_reached = False

    def current_action(self):
        return self.plan[self.action_i]

    def current_wp(self):
        return self.current_action()=='fly' and self.wps[self.wp_i]

    def act(self,action):
        if action == 'check_pos_est':
            self.check(self.is_position_estimated)
        if action == 'check_prearm':
            self.check(self.is_prearmed) 
        if action == 'mode_stabilize':
            self.send_mode_stabilize()
            self.check(self.is_acknowledged) 
        if action == 'mode_guided':
            self.send_mode_guided()
            self.check(self.is_acknowledged) 
        if action == 'arm':
            self.send_arm()
            self.check(self.is_acknowledged) 
        if action == 'takeoff':
            wp=self.wps[0]
            self.send_takeoff(altitude=wp[-1])
            self.wp_reached = self.is_reached(point=wp)
            if self.wp_reached:
                self.wp_i+=1
                self.action_i+=1
        if action == 'fly':
            wp=self.current_wp()
            self.send_local_position(point=wp)
            self.wp_reached = self.is_reached(point=wp)
            if self.wp_reached:
                self.wp_i+=1
                if self.wp_i == self.n_wps:
                    self.action_i+=1
        if action == 'land':
            self.send_land()
            self.check(self.is_landed) 
            
        if action == 'lotier':
            self.send_lotier()
            if self.verbose==1: print(f'vehicle {self.sys}: action lotier sent')


    def check(self,check:Callable[[], None]):
        if check():
            if self.verbose==1:self.inoform()
            self.action_i+=1

    def inoform(self):
        print(f'vehicle {self.sys}: action {self.current_action()} is done')
    
    def act_plan(self):
        if self.action_i < self.n_actions:
            self.act(self.current_action())
            return True
        else:
            return False

    # Check prearm
    def check_heartbeat(self):
        """Check if UAV is already armed"""
        msg = self.conn.recv_match(type="HEARTBEAT", blocking=True)
        if msg:
            if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                print("❌ UAV is already armed!")
                return False
            print("✅ UAV is disarmed and ready to arm")
            return True
        return False
    
    def check_ekf_status(self):
            """Check EKF status for safe arming"""
            msg = self.conn.recv_match(type="EKF_STATUS_REPORT", blocking=True)
            if msg:
                ekf_flags = msg.flags
                # Required EKF flags
                if (ekf_flags & EKF_REQUIRED_FLAGS) == EKF_REQUIRED_FLAGS:
                    print("✅ EKF is stable and ready for arming")
                    return True
                else:
                    print("❌ EKF not ready!")
            return False

    def check_gps_status(self):
        """Check GPS fix type"""
        msg = self.conn.recv_match(type="GPS_RAW_INT", blocking=True)
        if msg:
            gps_fix = msg.fix_type
            if gps_fix >= 3:  # 3D Fix or better
                print(f"✅ GPS Fix OK (Type {gps_fix})")
                return True
            else:
                print(f"❌ GPS Fix is weak (Type {gps_fix})")
        return False

    def check_sys_status(self):
        """Check system status (failsafe, battery, sensors)"""
        msg = self.conn.recv_match(type="SYS_STATUS", blocking=True)
        if msg:
            # Battery check
            if msg.battery_remaining > 20:  # More than 20% battery
                print(f"✅ Battery OK ({msg.battery_remaining}% remaining)")
                return True
            else:
                print(f"❌ Battery too low ({msg.battery_remaining}% remaining)")
            
            # Sensor check
            if msg and (msg.onboard_control_sensors_health & REQUIRED_SENSORS) == REQUIRED_SENSORS:
                print("✅ All required sensors are healthy! Ready for GPS-based autonomous flight")
                return True
            else:
                print("❌ Missing or unhealthy sensors! Not safe for autonomous flight")
        return False

    # Checking messages
    def is_position_estimated(self):
        """Wait until the UAV has a valid position estimate (EKF is ready)"""
        msg = self.conn.recv_match(type='EKF_STATUS_REPORT', blocking=True)
        return msg.flags & mavutil.mavlink.EKF_POS_HORIZ_ABS

    def is_prearmed(self):
        """ Wait until all pre-arm checks pass """
        msg = self.conn.recv_match(type='SYS_STATUS', blocking=True)
        if msg:
            return msg.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK
        else: 
            return False

    def is_acknowledged(self):
        print('ack?')
        msg = self.conn.recv_match(type='COMMAND_ACK', blocking=True)
        print('ack end')
        return msg and (not msg.result)
       
    def is_landed(self):
        self.conn.mav.command_long_send(self.sys, self.comp,REQ_MSG_CODE,0,EXT_STATE_CODE,0, 0, 0, 0, 0, 0  ) #parameter 4 is confirmation(it may be increased)
        msg = self.conn.recv_match(type='EXTENDED_SYS_STATE', blocking=True)
        if msg:
            return msg.landed_state == ON_GROUND_CODE
        else:
            return False

    def is_reached(self,point):
        pos = self.get_local_position()
        if pos:
            wd_dist = math.dist(pos, point) 
            return wd_dist < self.wp_margin
        else:
            return False

    # Sen Messages
    def send_mode_stabilize(self):
        self.conn.set_mode(mode_code["STABILIZE"])

    def send_mode_guided(self):
        self.conn.set_mode(mode_code["GUIDED"])

    def send_arm(self):
        self.conn.mav.command_long_send(self.sys, self.comp, ARM_CODE, 0, 1, 0, 0, 0, 0, 0, 0)
    
    def send_takeoff(self,altitude):
        self.conn.mav.command_long_send(self.sys, self.comp, TAKEOFF_CODE, 0, 0, 0, 0, 0, 0,0,altitude)
    





    # Get Messages
    def get_local_position(self):
        msg = self.conn.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        if msg:
            return GLOBAL_switch_LOCAL_NED(msg.x, msg.y, msg.z)

    def get_global_position(self):
        msg = self.conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            # Convert from scaled integers (E7 format) to decimal degrees
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.alt / 1000  # Convert mm to meters
            return lat, lon, alt
        return None  # Return None if no data received


      
    # Set messages
    def set_uav_velocity(self, vx=0, vy=0.0, vz=0.0, yaw_rate=0.0):
        """
        Sets the UAV's velocity without a duration, allowing other commands to use this speed.
        
        - vx: Forward speed (m/s)
        - vy: Sideways speed (m/s) (positive = right, negative = left)
        - vz: Vertical speed (m/s) (positive = down, negative = up)
        - yaw_rate: Rotation speed (rad/s)
        """

        type_mask = int(0b010111000111)  # Ignore position, use velocity only
        coordinate_frame = mavutil.mavlink.MAV_FRAME_BODY_NED  # Relative to drone

        # Create MAVLink message to set velocity
        msg = mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            10, self.sys, self.comp, coordinate_frame, type_mask,
            0, 0, 0,  # Position (ignored)
            vx, vy, vz,  # Velocity
            0, 0, 0,  # Acceleration (ignored)
            yaw_rate, 0  # Yaw & yaw_rate
        )
    
        # Send the velocity command
        self.conn.mav.send(msg)

        print(f"UAV {self.sys} velocity set: vx={vx}, vy={vy}, vz={vz}, yaw_rate={yaw_rate}")


    def send_local_position(self,point):
        type_mask = int(0b110111111000)
        point=GLOBAL_switch_LOCAL_NED(*point)
        coordinate_frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED
        go_msg=mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                    10, self.sys, self.comp, coordinate_frame, type_mask, *point, 0, 0, 0, 0, 0, 0, 0, 0)
        self.conn.mav.send(go_msg)
    
    def stop_position_control(self):
        """
        Stops the UAV from following previous position commands.
        """
        type_mask = int(0b111111111000)  # Ignore all movement except yaw
        coordinate_frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED

        msg = mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            10, self.sys, self.comp, coordinate_frame, type_mask,
            0, 0, 0,  # No movement
            0, 0, 0,  # No velocity
            0, 0, 0,  # No acceleration
            0, 0  # No yaw
        )

        self.conn.mav.send(msg)
        print("Position control stopped. UAV can now move freely.")

    def send_turn(self,direction):
    # Turn right (yaw 90 degrees to the right at 30 deg/sec)
        self.conn.mav.command_long_send(
            self.sys, self.comp,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,  # Confirmation
            90,  # Yaw angle (90 degrees to the right)
            50,  # Yaw rate (30 degrees/sec)
            direction,   # Direction (1 = right, -1 = left)
            1,   # Relative (0 = absolute, 1 = relative to current heading)
            0, 0, 0  # Unused parameters
        )
        print('turned')



    def send_global_position(self,point):
        coordinate_frame = mavutil.mavlink.MAVLink_set_position_target_global_int_message
        type_mask = int(0b110111111000)
        go_msg=mavutil.mavlink.MAVLink_set_position_target_global_int_message(
                    10, self.sys, self.comp, coordinate_frame, type_mask, *point, 0, 0, 0, 0, 0, 0, 0, 0)
        self.conn.mav.send(go_msg)


    def send_land(self):
        self.conn.mav.command_long_send(self.sys, self.comp, LAND_CODE , 0, 0, 0, 0, 0, 0, 0, 0)

    # def send_lotier(self):
    #     self.conn.mav.command_long_send(self.sys, self.comp, LOTIER_CODE, 0, 0, 0, 0, 0, 0, 0, 0)



    # def collision_hover(self,r):
    #     hover_msg=mavutil.mavlink.MAVLink_collision_message(
    #                 0, # Collision data source
    #                 1, # Unique identifier, domain based on src field
    #                 6, # Action that is being taken to avoid this collision
    #                 2, # How concerned the aircraft is about this collision
    #                 1, # Estimated time until collision occurs
    #                 r, # Closest vertical distance between vehicle and object
    #                 r  # Closest horizontal distance between vehicle and object
    #                 )
    #     self.conn.mav.send(hover_msg)

    
    def set_wp_nav_speed(self, speed_mps):
        """
        Sets the waypoint navigation speed (WPNAV_SPEED) in ArduPilot.
        
        - speed_mps: Speed in meters per second (e.g., 2.0 for 2 m/s)
        """
        param_name = b'WPNAV_SPEED'  # Name of the parameter
        speed_cmps = speed_mps * 100  # Convert m/s to cm/s (ArduPilot uses cm/s)

        # Send MAVLink command to set WPNAV_SPEED
        self.conn.mav.param_set_send(self.sys, self.comp, param_name, speed_cmps, mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )

        print(f"WPNAV_SPEED set to {speed_mps} m/s ({speed_cmps} cm/s)")

        # Wait for the parameter to be updated
        self.conn.recv_match(type='PARAM_VALUE', blocking=True)




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

            

    