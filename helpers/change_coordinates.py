"""Utility functions for coordinate transformations."""

import math
from typing import Type, TypeVar

from pymavlink import mavextra  # type: ignore

from helpers.math import rotate_mapcoord
from mavlink.customtypes.location import (
    ENU,
    NED,
    XY,
    XYZ,
    ENUPose,
    ENUPoses,
    ENUs,
    GRAPose,
    GRAPoses,
)


def ENUs_to_GRAs(origin: GRAPose, spawns: ENUs | ENUPoses) -> GRAPoses:
    """
    Convert a list of ENU spawns (with heading) into GLOBAL_RELATIVE_ALT coordinates
    (with global heading).
    """
    return [ENU_to_GRA(origin, spawn) for spawn in spawns]


def ENU_to_GRA(
    origin: GRAPose,
    point: ENU | ENUPose,
) -> GRAPose:
    """
    Convert a single ENU spawn (with heading) into GLOBAL_RELATIVE_ALT coordinates
    (with global heading).
    """
    if isinstance(point, ENU):
        x, y, z = point
        heading = 0.0
    else:
        x, y, z, heading = point

    lat, lon, alt, origin_heading = origin
    x_rot, y_rot = rotate_mapcoord(XY(x, y), origin_heading)
    lat, lon = mavextra.gps_offset(lat, lon, x_rot, y_rot)  # type: ignore
    alt = alt + z

    # Rotate heading relative to origin
    global_heading = (origin_heading + heading) % 360
    return GRAPose(lat, lon, alt, global_heading)


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


# def local2global(positions: NDArray[np.float64], home: Position
# ) -> NDArray[np.float64]:
#     """Convert local NED positions to global frame."""
#     return positions + np.asarray(home)


# def local2global_batch(
#     positions: NDArray[np.float64], homes: NDArray[np.float64]
# ) -> NDArray[np.float64]:
#     """Convert batches of local NED positions to global frame."""
#     return positions[None, :, :] + homes[:, None, :]


Vect = TypeVar("Vect", XYZ, ENU, NED)


def abs_to_rel(pos: Vect, home: Vect, cls: Type[Vect]) -> Vect:
    """Convert an absolute position to a position relative to a home position."""
    x, y, z = pos
    hx, hy, hz = home
    return cls(x - hx, y - hy, z - hz)


def rel_to_abs(pos: Vect, home: Vect, cls: Type[Vect]) -> Vect:
    """Convert an absolute position to a postion relative to a home posiiton."""
    x, y, z = pos
    hx, hy, hz = home
    return cls(x + hx, y + hy, z + hz)


# def global2local_batch(
#     positions: NDArray[np.float64], homes: NDArray[np.float64]
# ) -> NDArray[np.float64]:
#     """Convert batches of global positions to local NED frame."""
#     return positions[None, :, :] - homes[:, None, :]


# ## Taken from sim_vehicle.py
# def find_spawns(loc: Offset, homes: Offsets) -> Offsets:
#     """Compute GPS spawn locations from local NED offsets."""
#     lat, lon, alt, _ = loc
#     spawns: list[Offset] = []
#     for x, y, z, head in homes:
#         g = mavextra.gps_offset(lat, lon, x, y)
# # type: ignore[reportUnknownMemberType]
#         offset = Offset(g[0], g[1], alt + z, head)
#         spawns.append(offset)
#     return spawns


## Taken from sim_vehicle.py
# def ned_to_global_rel_alt(locals: Positions, origin: Position) -> Offsets:
#     """Convert a list of local NED waypoints to GLOBAL_RELATIVE_ALT frame."""
#     lat0, lon0, alt0 = origin
#     global_path: list[Position] = []

#     for x, y, z in locals:
#         # Converts local x/y to lat/lon
#         lat, lon = mavextra.gps_offset(lat0, lon0, x, y)  # type: ignore
#         # NED z is down → up is negative → subtract to get altitude above home
#         global_path.append(Offset(lat, lon, alt0 - z))

#     return global_path


# def enu_to_ned(pos: Position) -> Position:
#     """Convert a position from ENU (East-North-Up) to NED (North-East-Down) frame."""
#     x, y, z = pos
#     return Position(x, y, -z)


# def localned_to_global_relative(
#     pos_local: Position,
#     home_gps: Position,
# ) -> Position:
#     """
#     Convert local NED (meters) to global relative coordinates in
#     MAV_FRAME_GLOBAL_RELATIVE_ALT_INT frame.
#     Returns (lat_int, lon_int, alt_m).
#     """
#     x, y, z = pos_local
#     home_lat, home_lon, _ = home_gps

#     # Convert x/y offset to GPS lat/lon
#     lat, lon = mavextra.gps_offset(home_lat, home_lon, x, y)  # type: ignore

#     # z is down in NED, so negate it to get up (positive altitude)
#     alt = -z

#     return Position(lat, lon, alt)


# def gps_float2int(lat: float, lon: float):
#     """
#     Convert the first two elements (lat, lon) in a tuple from float degrees
#     to int (scaled by 1e7).
#     """
#     return int(lat * 1e7), int(lon * 1e7)
