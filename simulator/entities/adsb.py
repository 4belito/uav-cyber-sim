"""ADS-B semantic data models."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class ADSBBeacon:
    """
    Semantic ADS-B state for one aircraft (unit-suffixed fields, not MAVLink).

    - `icao` — the aircraft's 24-bit ICAO address (its unique transponder id).
    - `callsign` — the flight / tail identifier broadcast in the message.

    The rest are position and motion in their named units (`_deg` degrees,
    `_m` metres, `_mps` metres per second).
    """

    icao: int
    lat_deg: float
    lon_deg: float
    alt_m: float
    heading_deg: float
    hor_speed_mps: float
    ver_speed_mps: float
    callsign: str
