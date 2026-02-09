"""ADS-B related data structures and utilities."""

import hashlib
from dataclasses import dataclass

from simulator.helpers.rid import RIDData


@dataclass
class ADSBBeacon:
    """Semantic ADS-B state (not MAVLink)."""

    icao: int
    lat_deg: float
    lon_deg: float
    alt_m: float

    heading_deg: float
    hor_speed_mps: float
    ver_speed_mps: float

    callsign: str


def sysid_to_icao(sysid: int) -> int:
    """Deterministic 24-bit ICAO address from sysid."""
    h = hashlib.sha1(f"uav-{sysid}".encode()).digest()
    return int.from_bytes(h[:3], "big") | 0x100


def rid_to_adsb_beacon(rid: RIDData) -> ADSBBeacon:
    """Convert Remote ID data to ADS-B semantic beacon."""
    # --- position ---
    lat = rid.gra_pos.lat
    lon = rid.gra_pos.lon
    alt = rid.gra_pos.alt  # meters MSL

    # --- heading ---
    # ADS-B expects course-over-ground in degrees
    heading = rid.cog % 360.0

    # --- velocity ---
    # Horizontal speed is already provided
    hor_speed = rid.speed

    # Vertical speed from ENU (Up positive)
    ver_speed = rid.enu_vel.z

    return ADSBBeacon(
        icao=sysid_to_icao(rid.sysid),
        lat_deg=lat,
        lon_deg=lon,
        alt_m=alt,
        heading_deg=heading,
        hor_speed_mps=hor_speed,
        ver_speed_mps=ver_speed,
        callsign=f"UAV{rid.sysid}",
    )
