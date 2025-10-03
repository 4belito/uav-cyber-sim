"""Utility functions for coordinate transformations."""

import math
from math import cos, radians, sin
from typing import TypeVar, overload

import folium
from matplotlib.axes import Axes
from pymap3d import geodetic2enu  # type: ignore
from pymavlink import mavextra  # type: ignore

from config import Color
from helpers.connections.mavlink.customtypes.location import (
    ENU,
    GRA,
    NED,
    XY,
    ENUPose,
    ENUPoses,
    ENUs,
    GRAPose,
    GRAPoses,
    GRAs,
)
from helpers.math import rotate_mapcoord

T = TypeVar("T", ENU, ENUPose, GRAPose)
G = TypeVar("G", GRA, GRAPose)


def poses(origin: T, points: ENUs | ENUPoses) -> list[T]:
    """
    Convert a list of ENU/ENuPoses (with heading) into ENU/GRA
    depending on the orign type.
    """
    return [pose(origin, point) for point in points]


def pose(origin: T, point: ENU | ENUPose) -> T:
    """
    Convert a relative pose into an absolute one in the ENU/GRA frame.
    The position and heading are both rotated and translated.
    """
    if isinstance(point, ENU):
        x, y, z = point
        h = 0.0
    else:
        x, y, z, h = point
    if isinstance(origin, ENU):
        x0, y0, z0 = origin
        h0 = 0
    else:
        x0, y0, z0, h0 = origin

    # Rotate relative position by origin heading
    x_rot, y_rot = rotate_mapcoord(XY(x, y), h0)

    # Translate xy
    if isinstance(origin, GRAPose):
        x_abs, y_abs = mavextra.gps_offset(x0, y0, x_rot, y_rot)  # type: ignore
    else:
        x_abs = x0 + x_rot
        y_abs = y0 + y_rot

    z_abs = z0 + z
    h_abs = (h + h0) % 360

    if isinstance(origin, ENU):
        return ENU(x_abs, y_abs, z_abs)
    else:
        return origin.__class__(x_abs, y_abs, z_abs, h_abs)


def GRAs_to_ENUs(origin: GRA, points: GRAs) -> ENUs:
    """
    Convert a list of GRA points to ENU points relative to a
    GRA origin.
    """
    return [GRA_to_ENU(origin, point) for point in points]


def GRAPoses_to_ENUPoses(origin: GRAPose, points: GRAPoses) -> ENUPoses:
    """
    Convert a list of GRAPose points to ENUPose relative to a
    GRAPose origin.
    """
    return [GRA_to_ENU(origin, point) for point in points]


def GLOBAL_INT_to_GRA(lat: float, lon: float, alt: float) -> GRA:
    """Convert GLOBAL_POSITION_INT coordinates to GRA."""
    lat = lat / 1e7
    lon = lon / 1e7
    alt = alt / 1000
    return GRA(lat, lon, alt)


@overload
def GRA_to_ENU(origin: GRA, point: GRA) -> ENU: ...
@overload
def GRA_to_ENU(origin: GRAPose, point: GRAPose) -> ENUPose: ...


# Implementation (no decorator here)
def GRA_to_ENU(origin: GRA | GRAPose, point: GRA | GRAPose) -> ENU | ENUPose:
    """
    Convert a point from GRA or GRAPose to ENU or ENUPose relative to a
    GRA/GRAPose origin.
    """
    # Unpack coordinates
    lat, lon, alt = point[:3]
    lat0, lon0, alt0 = origin[:3]
    x, y, z = map(float, geodetic2enu(lat, lon, alt, lat0, lon0, alt0))  # type: ignore

    # Return appropriate pose
    if isinstance(origin, GRAPose) and isinstance(point, GRAPose):
        heading = (point[3] - origin[3]) % 360  # type: ignore
        return ENUPose(x, y, z, heading)
    if isinstance(origin, GRA) and isinstance(point, GRA):
        return ENU(x, y, z)
    raise TypeError("Origin and point must both be GRA or both GRAPose")


def heading_to_yaw(heading_deg: float) -> float:
    """Convert compass heading (deg) to yaw (rad)."""
    return -math.radians(heading_deg)


def ENU_to_NED(pos: ENU) -> NED:
    """Thransform from ENU to NED cooredinates."""
    x, y, z = pos
    return NED(y, x, -z)


def NED_to_ENU(pos: NED) -> ENU:
    """Thransform from NED to ENU cooredinates."""
    x, y, z = pos
    return ENU(y, x, -z)


def draw_enupose(ax: Axes, pose: ENUPose, label: str, color: str, alpha: float = 1.0):
    """Draws an ENUPose on a matplotlib Axes with an arrow and label."""
    x, y, _, h = pose
    arrow_scale = 2  # in meters

    dx = cos(radians(h)) * arrow_scale
    dy = sin(radians(h)) * arrow_scale

    ax.arrow(  # type: ignore
        x,
        y,
        dx,
        dy,
        head_width=arrow_scale * 0.5,
        color=color,
        alpha=alpha,
        length_includes_head=True,
    )
    ax.text(x, y, label, color=color, alpha=alpha)  # type: ignore


def draw_grapose(map_obj: folium.Map, pose: GRA | GRAPose, label: str, color: Color):
    """Draws a GRAPose as a marker on a folium map."""
    lat, lon = pose[:2]
    folium.Marker(
        location=[lat, lon], popup=label, icon=folium.Icon(color=color)
    ).add_to(map_obj)
