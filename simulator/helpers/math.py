"""Distance and bearing utilities for ENU coordinate arithmetic."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from simulator.helpers.coordinates import ENU


def manhattan_distance(
    x: NDArray[np.float64], y: NDArray[np.float64]
) -> float | NDArray[np.float64]:
    """Manhattan distance: two vectors → float, two arrays → array of floats."""
    return np.sum(np.abs(x - y), axis=-1).squeeze()


def heading_to_yaw(heading_deg: float) -> float:
    """Convert compass heading (deg) to yaw (rad)."""
    return -math.radians(heading_deg)


def enu_bearing(p1: ENU, p2: ENU) -> float | None:
    """Compass bearing from p1 to p2 in ENU space (degrees, 0-360), or None if coincident."""
    dx = p2.x - p1.x
    dy = p2.y - p1.y
    if not (dx or dy):
        return None
    return math.degrees(math.atan2(dx, dy)) % 360


def connection_id(sysid: int) -> int:
    """Convert a system ID to a connection ID."""
    return (sysid - 1) % 255 + 1
