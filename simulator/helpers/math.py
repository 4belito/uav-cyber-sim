"""Distance utilities for computing Manhattan distances between vectors or arrays."""

import math

import numpy as np
from numpy.typing import NDArray


def manhattan_distance(
    x: NDArray[np.float64], y: NDArray[np.float64]
) -> float | NDArray[np.float64]:
    """
    Compute the Manhattan distance between:
    - Two vectors → returns a float
    - Two arrays → returns an array of floats.
    """
    return np.sum(np.abs(x - y), axis=-1).squeeze()


def heading_to_yaw(heading_deg: float) -> float:
    """Convert compass heading (deg) to yaw (rad)."""
    return -math.radians(heading_deg)


def connection_id(sysid: int) -> int:
    """Convert a system ID to a connection ID."""
    return (sysid - 1) % 255 + 1
