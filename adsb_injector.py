"""
ADS-B Injector for ArduPilot SITL.

This script demonstrates how to inject ADS-B telemetry data into an ArduPilot
sim_vehicle.py process via a virtual serial device on Linux.

It converts Remote ID beacon data to MAVLink ADSB_VEHICLE messages and sends
them to the SITL, allowing you to test ArduPilot's built-in collision avoidance.

SETUP:
------
1. Create a virtual serial port pair using socat:

   socat -d -d pty,raw,echo=0,link=/tmp/adsb_master pty,raw,echo=0,link=/tmp/adsb_slave &

   This creates two linked pseudo-terminals:
   - /tmp/adsb_master - Your Python script writes to this
   - /tmp/adsb_slave  - sim_vehicle.py reads from this

2. Start sim_vehicle.py with the virtual serial port:

   sim_vehicle.py -v ArduCopter -A "--serial3=uart:/tmp/adsb_slave:57600"

3. Configure ArduPilot parameters (in MAVProxy or your GCS):

   param set SERIAL3_PROTOCOL 2    # MAVLink2
   param set SERIAL3_BAUD 57       # 57600 baud
   param set ADSB_TYPE 1           # MAVLink
   param set AVD_ENABLE 1          # Enable avoidance
   param set AVD_F_DIST_XY 200     # Horizontal near-miss distance (m)
   param set AVD_F_DIST_Z 100      # Vertical near-miss distance (m)
   param set AVD_F_TIME 30         # Seconds ahead to predict collision
   param set AVD_F_ACTION 2        # Action: 2=Climb/Descend, 3=Move Horizontally

4. Run this script to inject ADS-B traffic

USAGE:
------
    python3 adsb_injector.py

    # Or with custom serial port:
    python3 adsb_injector.py --uart /tmp/adsb_master --baud 57600

DEPENDENCIES:
-------------
    pip install pymavlink pyserial
"""

#!/usr/bin/env python3

import argparse
import math
import time
from dataclasses import dataclass
from typing import Optional

try:
    from pymavlink import mavutil
    from pymavlink.dialects.v20 import common as mavlink2
except ImportError:
    print("ERROR: pymavlink not installed. Run: pip install pymavlink")
    exit(1)


# =============================================================================
# ADSB_VEHICLE Message Constants (from MAVLink common.xml)
# =============================================================================

# ADSB_ALTITUDE_TYPE
ADSB_ALTITUDE_TYPE_PRESSURE_QNH = 0  # Altitude from barometer using QNH
ADSB_ALTITUDE_TYPE_GEOMETRIC = 1  # Altitude from GNSS

# ADSB_EMITTER_TYPE
ADSB_EMITTER_TYPE_NO_INFO = 0
ADSB_EMITTER_TYPE_LIGHT = 1
ADSB_EMITTER_TYPE_SMALL = 2
ADSB_EMITTER_TYPE_LARGE = 3
ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE = 4
ADSB_EMITTER_TYPE_HEAVY = 5
ADSB_EMITTER_TYPE_HIGHLY_MANUV = 6
ADSB_EMITTER_TYPE_ROTORCRAFT = 7
ADSB_EMITTER_TYPE_UNASSIGNED = 8
ADSB_EMITTER_TYPE_GLIDER = 9
ADSB_EMITTER_TYPE_LIGHTER_AIR = 10
ADSB_EMITTER_TYPE_PARACHUTE = 11
ADSB_EMITTER_TYPE_ULTRA_LIGHT = 12
ADSB_EMITTER_TYPE_UNASSIGNED2 = 13
ADSB_EMITTER_TYPE_UAV = 14
ADSB_EMITTER_TYPE_SPACE = 15
ADSB_EMITTER_TYPE_UNASSIGNED3 = 16
ADSB_EMITTER_TYPE_EMERGENCY_SURFACE = 17
ADSB_EMITTER_TYPE_SERVICE_SURFACE = 18
ADSB_EMITTER_TYPE_POINT_OBSTACLE = 19

# ADSB_FLAGS (bitmask)
ADSB_FLAGS_VALID_COORDS = 1
ADSB_FLAGS_VALID_ALTITUDE = 2
ADSB_FLAGS_VALID_HEADING = 4
ADSB_FLAGS_VALID_VELOCITY = 8
ADSB_FLAGS_VALID_CALLSIGN = 16
ADSB_FLAGS_VALID_SQUAWK = 32
ADSB_FLAGS_SIMULATED = 64
ADSB_FLAGS_VERTICAL_VELOCITY_VALID = 128
ADSB_FLAGS_BARO_VALID = 256
ADSB_FLAGS_SOURCE_UAT = 32768


@dataclass
class RemoteIDBeacon:
    """
    Represents a Remote ID beacon (your existing data source).
    Adapt this class to match your actual Remote ID data structure.
    """

    # Unique identifier (can be derived from serial number, etc.)
    uas_id: str

    # Position (WGS84)
    latitude: float  # degrees
    longitude: float  # degrees
    altitude_msl: float  # meters above mean sea level

    # Velocity
    speed_horizontal: float  # m/s ground speed
    speed_vertical: float  # m/s (positive = climbing)
    heading: float  # degrees (0-360, 0=North)

    # Optional metadata
    callsign: Optional[str] = None
    operator_id: Optional[str] = None
    timestamp: Optional[float] = None


def remote_id_to_icao(uas_id: str) -> int:
    """
    Convert a Remote ID UAS identifier to a pseudo-ICAO address.

    Real ICAO addresses are 24-bit unique identifiers assigned by aviation
    authorities. For simulation purposes, we generate a deterministic
    24-bit hash from the UAS ID string.

    Args:
        uas_id: The Remote ID unique identifier string

    Returns:
        A 24-bit integer to use as ICAO address

    """
    # Use a simple hash, masked to 24 bits
    # In production, you might want a proper mapping scheme
    hash_val = hash(uas_id) & 0xFFFFFF

    # Avoid reserved ICAO ranges (0x000000 and some others)
    if hash_val < 0x000100:
        hash_val += 0x000100

    return hash_val


def remote_id_to_adsb_vehicle(
    beacon: RemoteIDBeacon, system_id: int = 1, component_id: int = 1
):
    """
    Convert a Remote ID beacon to a MAVLink ADSB_VEHICLE message.

    ADSB_VEHICLE message fields (from MAVLink common.xml):
    - ICAO_address: uint32_t - ICAO address
    - lat: int32_t - Latitude (degE7)
    - lon: int32_t - Longitude (degE7)
    - altitude_type: uint8_t - ADSB_ALTITUDE_TYPE enum
    - altitude: int32_t - Altitude in millimeters
    - heading: uint16_t - Course over ground (cdeg, 0-35999)
    - hor_velocity: uint16_t - Horizontal velocity (cm/s)
    - ver_velocity: int16_t - Vertical velocity (cm/s, positive=up)
    - callsign: char[9] - 8 char + null terminator
    - emitter_type: uint8_t - ADSB_EMITTER_TYPE enum
    - tslc: uint8_t - Time since last communication (seconds)
    - flags: uint16_t - ADSB_FLAGS bitmask
    - squawk: uint16_t - Squawk code (mode 3/A)

    Args:
        beacon: RemoteIDBeacon with position/velocity data
        system_id: MAVLink system ID to use
        component_id: MAVLink component ID to use

    Returns:
        Packed MAVLink message bytes

    """
    # Create a MAVLink connection for message encoding
    # We use a dummy file-like object since we just need encoding
    mav = mavlink2.MAVLink(None)
    mav.srcSystem = system_id
    mav.srcComponent = component_id

    # Convert Remote ID to ICAO address
    icao_address = remote_id_to_icao(beacon.uas_id)

    # Convert coordinates to degE7 (degrees * 1e7)
    lat = int(beacon.latitude * 1e7)
    lon = int(beacon.longitude * 1e7)

    # Altitude in millimeters
    altitude = int(beacon.altitude_msl * 1000)

    # Heading in centidegrees (0-35999)
    heading = int(beacon.heading * 100) % 36000

    # Velocities in cm/s
    hor_velocity = int(beacon.speed_horizontal * 100)
    ver_velocity = int(beacon.speed_vertical * 100)

    # Callsign (8 chars max, padded with spaces)
    if beacon.callsign:
        callsign = beacon.callsign[:8].ljust(8)
    else:
        # Generate from UAS ID if no callsign
        callsign = beacon.uas_id[:8].ljust(8)

    # Build flags bitmask
    flags = (
        ADSB_FLAGS_VALID_COORDS
        | ADSB_FLAGS_VALID_ALTITUDE
        | ADSB_FLAGS_VALID_HEADING
        | ADSB_FLAGS_VALID_VELOCITY
        | ADSB_FLAGS_VALID_CALLSIGN
        | ADSB_FLAGS_SIMULATED  # Mark as simulated traffic
    )

    if beacon.speed_vertical != 0:
        flags |= ADSB_FLAGS_VERTICAL_VELOCITY_VALID

    # Time since last communication (0 = just received)
    tslc = 0

    # Squawk code - use 1200 for VFR (standard in US)
    squawk = 1200

    # Create the ADSB_VEHICLE message
    msg = mav.adsb_vehicle_encode(
        ICAO_address=icao_address,
        lat=lat,
        lon=lon,
        altitude_type=ADSB_ALTITUDE_TYPE_GEOMETRIC,
        altitude=altitude,
        heading=heading,
        hor_velocity=hor_velocity,
        ver_velocity=ver_velocity,
        callsign=callsign.encode("ascii"),
        emitter_type=ADSB_EMITTER_TYPE_UAV,  # Remote ID = UAV
        tslc=tslc,
        flags=flags,
        squawk=squawk,
    )

    return msg.pack(mav)


class ADSBInjector:
    """Injects ADS-B messages into ArduPilot SITL via serial connection."""

    def __init__(self, uart: str, baudrate: int = 57600):
        """
        Initialize the ADS-B injector.

        Args:
            uart: Serial port path (e.g., /tmp/adsb_master or /dev/ttyUSB0)
            baudrate: Serial baud rate (default 57600 for ADS-B)

        """
        self.uart = uart
        self.baudrate = baudrate
        self.connection = None
        self.mav = None

    def connect(self):
        """Open the serial connection."""
        print(f"Connecting to {self.uart} at {self.baudrate} baud...")

        # Use mavutil for proper MAVLink connection handling
        # Format: device path directly, with baud parameter
        self.connection = mavutil.mavlink_connection(
            self.uart,
            baud=self.baudrate,
            source_system=1,
            source_component=156,  # MAV_COMP_ID_ADSB (156)
        )

        self.mav = self.connection.mav
        print("Connected!")

    def disconnect(self):
        """Close the serial connection."""
        if self.connection:
            self.connection.close()
            print("Disconnected.")

    def send_heartbeat(self):
        """
        Send a heartbeat message to identify as an ADS-B peripheral.
        ArduPilot expects periodic heartbeats from connected devices.
        """
        self.mav.heartbeat_send(
            type=mavlink2.MAV_TYPE_ADSB,
            autopilot=mavlink2.MAV_AUTOPILOT_INVALID,
            base_mode=0,
            custom_mode=0,
            system_status=mavlink2.MAV_STATE_ACTIVE,
        )

    def send_adsb_vehicle(self, beacon: RemoteIDBeacon):
        """
        Send an ADSB_VEHICLE message for a Remote ID beacon.

        Args:
            beacon: RemoteIDBeacon with position/velocity data

        """
        icao_address = remote_id_to_icao(beacon.uas_id)
        lat = int(beacon.latitude * 1e7)
        lon = int(beacon.longitude * 1e7)
        altitude = int(beacon.altitude_msl * 1000)
        heading = int(beacon.heading * 100) % 36000
        hor_velocity = int(beacon.speed_horizontal * 100)
        ver_velocity = int(beacon.speed_vertical * 100)

        callsign = (beacon.callsign or beacon.uas_id)[:8].ljust(8)

        flags = (
            ADSB_FLAGS_VALID_COORDS
            | ADSB_FLAGS_VALID_ALTITUDE
            | ADSB_FLAGS_VALID_HEADING
            | ADSB_FLAGS_VALID_VELOCITY
            | ADSB_FLAGS_VALID_CALLSIGN
            | ADSB_FLAGS_SIMULATED
            | ADSB_FLAGS_VERTICAL_VELOCITY_VALID
        )

        self.mav.adsb_vehicle_send(
            ICAO_address=icao_address,
            lat=lat,
            lon=lon,
            altitude_type=ADSB_ALTITUDE_TYPE_GEOMETRIC,
            altitude=altitude,
            heading=heading,
            hor_velocity=hor_velocity,
            ver_velocity=ver_velocity,
            callsign=callsign.encode("ascii"),
            emitter_type=ADSB_EMITTER_TYPE_UAV,
            tslc=0,
            flags=flags,
            squawk=1200,
        )


def generate_sample_traffic(
    center_lat: float, center_lon: float, center_alt: float
) -> RemoteIDBeacon:
    """
    Generate sample Remote ID beacon flying in circle around a point.

    Args:
        center_lat: Center latitude in degrees
        center_lon: Center longitude in degrees
        center_alt: Center altitude in meters

    Returns:
        RemoteIDBeacon object

    """
    t = time.time()

    radius_m = 7  # 5
    period_s = 60  # 10

    # Calculate position on circle
    angle = 2 * math.pi * t / period_s

    # Convert radius to degrees (approximate)
    radius_deg = radius_m / 111000  # ~111km per degree

    lat = center_lat + radius_deg * math.cos(angle)
    lon = center_lon + radius_deg * math.sin(angle) / math.cos(math.radians(center_lat))

    # Heading tangent to circle
    heading = math.degrees(angle + math.pi / 2) % 360

    # Speed = circumference / period
    speed = (2 * math.pi * radius_m) / period_s

    beacon = RemoteIDBeacon(
        uas_id="RIDTEST",
        latitude=lat,
        longitude=lon,
        altitude_msl=center_alt,
        speed_horizontal=speed,
        speed_vertical=0.0,
        heading=heading,
        callsign="TEST",
        timestamp=t,
    )

    return beacon


def main():
    parser = argparse.ArgumentParser(
        description="Inject ADS-B traffic into ArduPilot SITL from Remote ID beacons",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example setup:
  # Terminal 1: Create virtual serial port
  socat -d -d pty,raw,echo=0,link=/tmp/adsb_master pty,raw,echo=0,link=/tmp/adsb_slave &

  # Terminal 2: Start SITL
  sim_vehicle.py -v ArduCopter -A "--serial3=uart:/tmp/adsb_slave:57600"

  # Terminal 3: Run this script
  python3 adsb_injector.py --uart /tmp/adsb_master

  # In MAVProxy (Terminal 2), configure ADSB:
  param set SERIAL3_PROTOCOL 2
  param set SERIAL3_BAUD 57
  param set ADSB_TYPE 1
  param set AVD_ENABLE 1
        """,
    )

    parser.add_argument(
        "--uart",
        "-u",
        default="/tmp/adsb_1_injector",
        help="UART device (default: /tmp/adsb_1_injector)",
    )
    parser.add_argument(
        "--baud", "-b", type=int, default=57600, help="Baud rate (default: 57600)"
    )
    parser.add_argument(
        "--lat",
        type=float,
        default=-35.363262,  # SITL default location
        help="Center latitude for simulated traffic (default: SITL home)",
    )
    parser.add_argument(
        "--lon",
        type=float,
        default=149.165237,  # SITL default location
        help="Center longitude for simulated traffic (default: SITL home)",
    )
    parser.add_argument(
        "--alt",
        type=float,
        default=100.0,
        help="Center altitude for simulated traffic in meters (default: 100)",
    )
    parser.add_argument(
        "--rate", type=float, default=5.0, help="Update rate in Hz (default: 5)"
    )

    args = parser.parse_args()

    # Create injector
    injector = ADSBInjector(args.uart, args.baud)

    try:
        injector.connect()

        print(f"  Lat: {args.lat}")
        print(f"  Lon: {args.lon}")
        print(f"  Alt: {args.alt}m")
        print(f"  Rate: {args.rate} Hz")
        print("\nPress Ctrl+C to stop.\n")

        heartbeat_interval = 1.0  # Send heartbeat every 1 second
        last_heartbeat = 0
        update_interval = 1.0 / args.rate

        while True:
            now = time.time()

            # Send heartbeat periodically
            if now - last_heartbeat >= heartbeat_interval:
                injector.send_heartbeat()
                last_heartbeat = now

            # Generate and send traffic
            beacon = generate_sample_traffic(args.lat, args.lon, args.alt)

            injector.send_adsb_vehicle(beacon)
            print(
                f"  Sent: {beacon.callsign} @ ({beacon.latitude:.6f}, "
                f"{beacon.longitude:.6f}, {beacon.altitude_msl:.0f}m) "
                f"HDG={beacon.heading:.0f}° SPD={beacon.speed_horizontal:.1f}m/s"
            )

            time.sleep(update_interval)

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        injector.disconnect()


if __name__ == "__main__":
    main()
