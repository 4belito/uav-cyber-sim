"""ADS-B semantic data models."""

from dataclasses import dataclass


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
