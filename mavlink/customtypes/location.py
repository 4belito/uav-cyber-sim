"""Location types."""

from typing import NamedTuple, Self, Type, TypeVar
from geopy import distance
import numpy as np

T = TypeVar("T")


# === Base coordinate and pose types ===


class XY(NamedTuple):
    """Abstract 2D map coordinate (e.g., on mission map)."""

    x: float
    y: float

    @classmethod
    def list(cls: Type[Self], data: list[tuple[float, float]]) -> list[Self]:
        """Create a list of XY instances from a list of (x, y) tuples."""
        return [cls(*pair) for pair in data]


class XYZ(NamedTuple):
    """3D vector in a local Cartesian frame (e.g., ENU or NED)."""

    x: float
    y: float
    z: float

    @classmethod
    def list(cls: Type[Self], data: list[tuple[float, float, float]]) -> list[Self]:
        """Create a list of XYZ instances from a list of (x, y, z) tuples."""
        return [cls(*pair) for pair in data]


class LLA(NamedTuple):
    """Geographic position: latitude, longitude, altitude (WGS84, meters)."""

    lat: float
    lon: float
    alt: float

    @classmethod
    def list(cls: Type[Self], data: list[tuple[float, float, float]]) -> list[Self]:
        """Create a list of LLA instances from a list of (x, y, z) tuples."""
        return [cls(*pair) for pair in data]

    @classmethod
    def distance(cls, a: Self, b: Self) -> float:
        """Calculate the distance between two LLA points."""
        return np.sqrt(
            distance.geodesic((a.lat, a.lon), (b.lat, b.lon)).m ** 2  # type: ignore
            + (a.alt - b.alt) ** 2
        )


class PoseXYZ(NamedTuple):
    """3D pose in a local frame: position + heading (yaw, degrees)."""

    x: float
    y: float
    z: float
    heading: float = 0.0

    @classmethod
    def list(
        cls: Type[Self], data: list[tuple[float, float, float, float]]
    ) -> list[Self]:
        """Create a list of PoseXYZ instances from a list of (x, y, z,h) tuples."""
        return [cls(*pair) for pair in data]


class PoseLLA(NamedTuple):
    """Geographic pose: lat, lon, alt + heading (degrees from North)."""

    lat: float
    lon: float
    alt: float
    heading: float = 0.0

    @classmethod
    def list(cls, data: list[tuple[float, float, float, float]]) -> list[Self]:
        """Create a list of PoseLLA instances from (lat, lon, alt, heading) tuples."""
        return [cls(*pair) for pair in data]


class XYZRPY(NamedTuple):
    """6-DOF pose in local frame: x, y, z + roll, pitch, yaw (degrees)."""

    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float

    @classmethod
    def list(
        cls,
        data: list[tuple[float, float, float, float, float, float]],
    ) -> list[Self]:
        """Create a list of XYZRPY instances from (x, y, z, roll, pitch, yaw) tuples."""
        return [cls(*pair) for pair in data]

    def __str__(self) -> str:
        return f"{self.x} {self.y} {self.z} {self.roll} {self.pitch} {self.yaw}"


# === Specialized frames ===


class ENU(XYZ):
    """ENU vector (x=East, y=North, z=Up)."""


class NED(XYZ):
    """NED vector (x=North, y=East, z=Down)."""


class GRA(LLA):
    """Global relative position (MAV_FRAME_GLOBAL_RELATIVE_ALT)."""


class ENUPose(PoseXYZ):
    """Pose in ENU frame: position + heading."""


class NEDPose(PoseXYZ):
    """Pose in NED frame: position + heading."""


class GRAPose(PoseLLA):
    """Pose in GLOBAL_RELATIVE_ALT frame: lat/lon/alt + heading."""


# === Type aliases for grouped data ===

XYs = list[XY]
XYZs = list[XYZ]
LLAs = list[LLA]

PoseXYZs = list[PoseXYZ]
PoseLLAs = list[PoseLLA]
XYZRPYs = list[XYZRPY]

ENUs = list[ENU]
ENUPoses = list[ENUPose]
NEDs = list[NED]
NEDPoses = list[NEDPose]
GRAs = list[GRA]
GRAPoses = list[GRAPose]
