"""Location types."""

from typing import NamedTuple

# === Base coordinate and pose types ===


class XY(NamedTuple):
    """Abstract 2D map coordinate (e.g., on mission map)."""

    x: float
    y: float


class XYZ(NamedTuple):
    """3D vector in a local Cartesian frame (e.g., ENU or NED)."""

    x: float
    y: float
    z: float


class LLA(NamedTuple):
    """Geographic position: latitude, longitude, altitude (WGS84, meters)."""

    lat: float
    lon: float
    alt: float


class PoseXYZ(NamedTuple):
    """3D pose in a local frame: position + heading (yaw, degrees)."""

    x: float
    y: float
    z: float
    heading: float = 0.0


class PoseLLA(NamedTuple):
    """Geographic pose: lat, lon, alt + heading (degrees from North)."""

    lat: float
    lon: float
    alt: float
    heading: float = 0.0


class XYZRPY(NamedTuple):
    """6-DOF pose in local frame: x, y, z + roll, pitch, yaw (degrees)."""

    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float

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
