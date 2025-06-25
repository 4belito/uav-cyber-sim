"""Distance utilities for computing Manhattan distances between vectors or arrays."""

import math

import numpy as np
from numpy.typing import NDArray

from mavlink.customtypes.location import XY


def manhattan_distance(
    x: NDArray[np.float64], y: NDArray[np.float64]
) -> float | NDArray[np.float64]:
    """
    Compute the Manhattan distance between:
    - Two vectors → returns a float
    - Two arrays → returns an array of floats.
    """
    return np.sum(np.abs(x - y), axis=-1).squeeze()


def rotate_mapcoord(point: XY, angle_deg: float) -> XY:
    """
    Rotate a single (x, y) point counter-clockwise by angle_deg degrees.

    Args:
        point: (x, y) coordinate
        angle_deg: Rotation angle in degrees

    Returns:
        Rotated (x, y) coordinate

    """
    x, y = point
    theta = math.radians(angle_deg)
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    x_rot = x * cos_t - y * sin_t
    y_rot = x * sin_t + y * cos_t
    return XY(x_rot, y_rot)
