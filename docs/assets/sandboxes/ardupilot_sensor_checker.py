"""
Simple script to check for active sensors on an ArduPilot-based vehicle.
It connects to the vehicle via MAVLink, listens for sensor-related messages,
and checks for relevant parameters indicating sensor presence and configuration.
"""

from pymavlink import mavutil


def check_sensor_messages(master):
    print("🔍 Checking for active sensor MAVLink messages...")

    # Define MAVLink message types associated with known sensors
    sensor_msgs = {
        "RAW_IMU": "IMU (accelerometer + gyro)",
        "HIGHRES_IMU": "High-resolution IMU",
        "SCALED_PRESSURE": "Barometer",
        "GPS_RAW_INT": "GPS",
        "GPS2_RAW": "Secondary GPS",
        "RAW_MAG": "Compass (magnetometer)",
        "OPTICAL_FLOW_RAD": "Optical Flow",
        "DISTANCE_SENSOR": "Proximity / RangeFinder",
        "OBSTACLE_DISTANCE": "Obstacle Avoidance",
        "ADSB_VEHICLE": "ADSB Receiver",
        "VISION_POSITION_ESTIMATE": "Vision/VIO",
        "IRLOCK_REPORT": "IRLock Landing Beacon",
        "VICON_POSITION_ESTIMATE": "Motion Capture",
        "UWB_DISTANCE": "UWB Beacon",
    }

    found = set()

    # Listen for messages for a few seconds
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        4,
        1,
    )

    print("Waiting for sensor messages (5 seconds)...")
    import time

    start = time.time()
    while time.time() - start < 5:
        msg = master.recv_match(blocking=False)
        if msg:
            msg_type = msg.get_type()
            if msg_type in sensor_msgs and msg_type not in found:
                print(f"✅ {sensor_msgs[msg_type]}: {msg_type}")
                found.add(msg_type)

    if not found:
        print(
            "⚠️  No sensor messages received in 5 seconds. Is the drone armed and sensors active?"
        )


def check_sensor_params(master):
    print("\n🔍 Checking for sensor-related parameters...")

    sensor_params = {
        "INS_ENABLE": "IMU enabled",
        "COMPASS_USE": "Compass enabled",
        "GPS_TYPE": "GPS module type",
        "GPS_TYPE2": "Second GPS module type",
        "RNGFND1_TYPE": "RangeFinder 1 type",
        "FLOW_TYPE": "Optical flow type",
        "BARO_ENABLE": "Barometer enabled",
        "ADSB_ENABLE": "ADSB receiver enabled",
        "EK3_SRC1_POSXY": "EKF Position XY source",
        "EK3_SRC1_YAW": "EKF Yaw source",
    }

    for param, description in sensor_params.items():
        try:
            value = master.param_fetch_one(param)
            print(f"✅ {description}: {value}")
        except:
            print(f"⚠️  {description} not found")


if __name__ == "__main__":
    print("🔌 Connecting to vehicle...")
    master = mavutil.mavlink_connection("udp:127.0.0.1:14550")  # Change if needed
    master.wait_heartbeat()
    print(
        "✅ Heartbeat received from system %u component %u"
        % (master.target_system, master.target_component)
    )

    check_sensor_messages(master)
    check_sensor_params(master)
