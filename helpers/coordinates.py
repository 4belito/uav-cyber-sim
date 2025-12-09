"""Location types."""

from __future__ import annotations

import math
from typing import (
    Generator,
    Iterable,
    Iterator,
    NamedTuple,
    Self,
    Type,
)

import folium
import numpy as np
from geopy import distance
from matplotlib.axes import Axes
from pymap3d import enu2geodetic, geodetic2enu  # type: ignore

from config import Color
from helpers.connections.mavlink.customtypes.mavconn import MAVConnection

# TODO: Check repetitions of similar methods across classes


def float_nans(n: int) -> Generator[float, None, None]:
    """Create a stream of NaN floats of length n."""
    return (float("nan") for _ in range(n))


# === Base coordinate and pose types ===
class XY(NamedTuple):
    """Abstract 2D Cartesian coordinate (e.g., on a mission map)."""

    x: float
    y: float

    @classmethod
    def list(cls: Type[Self], data: list[tuple[float, float]]) -> list[Self]:
        """Create XY instances from a list of (x, y) tuples."""
        return [cls(*pair) for pair in data]

    @classmethod
    def nan(cls) -> Self:
        """Return a NaN XY instance."""
        return cls(*float_nans(2))

    @classmethod
    def add(cls, a: Self, b: Self) -> Self:
        """Add two XY vectors."""
        return cls(a.x + b.x, a.y + b.y)

    @classmethod
    def sub(cls, a: Self, b: Self) -> Self:
        """Subtract two XY vectors (a - b)."""
        return cls(a.x - b.x, a.y - b.y)

    @classmethod
    def dot(cls, a: Self, b: Self) -> float:
        """Dot product of two XY vectors."""
        return a.x * b.x + a.y * b.y

    def scale(self, factor: float) -> Self:
        """Scale the XY vector by a factor."""
        return self.__class__(self.x * factor, self.y * factor)

    def rotate(self, angle_deg: float) -> Self:
        """Rotate counter-clockwise by angle_deg degrees."""
        theta = math.radians(angle_deg)
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        x_rot = self.x * cos_t - self.y * sin_t
        y_rot = self.x * sin_t + self.y * cos_t
        return self.__class__(x_rot, y_rot)

    def norm(self) -> float:
        """Return the Euclidean norm of the XY vector."""
        return math.sqrt(self.x**2 + self.y**2)

    @classmethod
    def div(cls, a: Self, b: Self) -> Self:
        """Element-wise division of two XY vectors (a / b)."""
        assert b.x != 0 and b.y != 0, "Division by zero in XY.div"
        return cls(a.x / b.x, a.y / b.y)


class XYZ(NamedTuple):
    """3D vector in a local Cartesian frame (e.g., ENU or NED)."""

    x: float
    y: float
    z: float

    @classmethod
    def list(cls: Type[Self], data: list[tuple[float, float, float]]) -> list[Self]:
        """Create XYZ instances from a list of (x, y, z) tuples."""
        return [cls(*pair) for pair in data]

    @classmethod
    def nan(cls) -> Self:
        """Return a NaN XYZ instance."""
        return cls(*float_nans(3))

    @classmethod
    def add(cls, a: Self, b: Self) -> Self:
        """Add two XYZ vectors."""
        return cls(a.x + b.x, a.y + b.y, a.z + b.z)

    @classmethod
    def sub(cls, a: Self, b: Self) -> Self:
        """Subtract two XYZ vectors (a - b)."""
        return cls(a.x - b.x, a.y - b.y, a.z - b.z)

    @classmethod
    def dot(cls, a: Self, b: Self) -> float:
        """Dot product of two XYZ vectors."""
        return a.x * b.x + a.y * b.y + a.z * b.z

    def scale(self, factor: float) -> Self:
        """Scale the XYZ vector by a factor."""
        return self.__class__(self.x * factor, self.y * factor, self.z * factor)

    def pose(self, heading: float = 0.0) -> XYZPose:
        """Convert this XYZ point into an XYZPose with the given heading."""
        return XYZPose(self.x, self.y, self.z, heading)


class LLA(NamedTuple):
    """Geographic position: latitude, longitude, altitude (WGS-84, meters)."""

    lat: float
    lon: float
    alt: float

    @classmethod
    def list(cls: Type[Self], data: list[tuple[float, float, float]]) -> list[Self]:
        """Create LLA instances from a list of (lat, lon, alt) tuples."""
        return [cls(*pair) for pair in data]

    def pose(self, heading: float = 0.0) -> LLAPose:
        """Convert this LLA point into an LLAPose with the given heading."""
        return LLAPose(self.lat, self.lon, self.alt, heading)

    @classmethod
    def distance(cls, a: Self, b: Self) -> float:
        """
        3D distance (meters) between two LLA points, including altitude
        difference.
        """
        return np.sqrt(
            distance.geodesic((a.lat, a.lon), (b.lat, b.lon)).m ** 2  # type: ignore
            + (a.alt - b.alt) ** 2
        )

    @classmethod
    def nan(cls) -> Self:
        """Return a NaN LLA instance."""
        return cls(*float_nans(3))


class XYZPose(NamedTuple):
    """3D pose in a local frame: (x, y, z) + heading (yaw in degrees)."""

    x: float
    y: float
    z: float
    heading: float = 0.0

    @classmethod
    def list(
        cls: Type[Self], data: list[tuple[float, float, float, float]]
    ) -> list[Self]:
        """Create XYZPose instances from (x, y, z, heading) tuples."""
        return [cls(*pair) for pair in data]

    def unpose(self) -> XYZ:
        """Drop heading and return the XYZ point."""
        return XYZ(self.x, self.y, self.z)

    @classmethod
    def nan(cls) -> Self:
        """Return a NaN XYZPose instance."""
        return cls(*float_nans(4))

    @classmethod
    def add(cls, a: Self, b: Self) -> Self:
        """Add two XYZPose poses (heading wrapped to [0, 360))."""
        return cls(a.x + b.x, a.y + b.y, a.z + b.z, (a.heading + b.heading) % 360)

    @classmethod
    def sub(cls, a: Self, b: Self) -> Self:
        """Subtract two XYZPose poses (a - b; heading wrapped to [0, 360))."""
        return cls(a.x - b.x, a.y - b.y, a.z - b.z, (a.heading - b.heading) % 360)


class LLAPose(NamedTuple):
    """Geographic pose: (lat, lon, alt) + heading (degrees from North, clockwise)."""

    lat: float
    lon: float
    alt: float
    heading: float = 0.0

    @classmethod
    def list(cls, data: list[tuple[float, float, float, float]]) -> list[Self]:
        """Create LLAPose instances from (lat, lon, alt, heading) tuples."""
        return [cls(*pair) for pair in data]

    def unpose(self) -> LLA:
        """Drop heading and return the LLA point."""
        return LLA(self.lat, self.lon, self.alt)

    @classmethod
    def nan(cls) -> Self:
        """Return a NaN LLAPose instance."""
        return cls(*float_nans(4))


class XYZRPY(NamedTuple):
    """6-DOF local pose: x, y, z + roll, pitch, yaw (degrees)."""

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
        """Create XYZRPY instances from (x, y, z, roll, pitch, yaw) tuples."""
        return [cls(*pair) for pair in data]

    def __str__(self) -> str:
        return f"{self.x} {self.y} {self.z} {self.roll} {self.pitch} {self.yaw}"

    @classmethod
    def nan(cls) -> Self:
        """Return a NaN XYZRPY instance."""
        return cls(*float_nans(6))


def format_component(component: float, decimal_places: int) -> float:
    """Format a single coordinate component with rounding if needed."""
    if isinstance(component, int) or component.is_integer():
        return int(component)
    else:
        return round(component, decimal_places)


# === Specialized frames ===
class ENU(XYZ):
    """ENU vector (x=East, y=North, z=Up)."""

    @classmethod
    def distance(cls, a: Self, b: Self) -> float:
        """Euclidean distance between two ENU points."""
        return np.sqrt(cls.distance_squared(a, b))

    @classmethod
    def distance_squared(cls, a: Self, b: Self) -> float:
        """Squared Euclidean distance between two ENU points."""
        return (a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2

    def norm(self) -> float:
        """Return the Euclidean norm of the ENU vector."""
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

    def to_abs(self, rel_p: ENU | ENUPose) -> ENU:
        """Compose this ENU (as origin) with a relative ENU/ENUPose."""
        if isinstance(rel_p, ENUPose):
            rel_p = rel_p.unpose()
        return ENU.add(self, rel_p)

    def to_rel(self, abs_p: ENU | ENUPose) -> ENU:
        """Absolute ENU/ENUPose → relative ENU w.r.t. this origin."""
        if isinstance(abs_p, ENUPose):
            abs_p = abs_p.unpose()
        return ENU.sub(abs_p, self)

    def pose(self, heading: float = 0.0) -> ENUPose:
        """Return an ENUPose with this position and the given heading."""
        return ENUPose(self.x, self.y, self.z, heading)

    def to_ned(self) -> NED:
        """Convert ENU → NED coordinates."""
        return NED(self.y, self.x, -self.z)

    @staticmethod
    def from_ned(x: float, y: float, z: float) -> ENU:
        """Convert a NED point into an ENU point."""
        return ENU(y, x, -z)

    # ---- Batch helpers (ENU) ----
    def to_abs_all(self, rels: Iterable[ENU | ENUPose]) -> list[ENU]:
        """Vectorize `to_abs` over a sequence of relative ENU/ENUPose."""
        return [self.to_abs(p) for p in rels]

    def to_rel_all(self, abss: Iterable[ENU | ENUPose]) -> list[ENU]:
        """Vectorize `to_rel` over a sequence of absolute ENU/ENUPose."""
        return [self.to_rel(p) for p in abss]

    def to_abs_iter(self, rels: Iterable[ENU | ENUPose]) -> Iterator[ENU]:
        """Iterate relative ENU/ENUPose → absolute ENU."""
        for p in rels:
            yield self.to_abs(p)

    def to_rel_iter(self, abss: Iterable[ENU | ENUPose]) -> Iterator[ENU]:
        """Iterate absolute ENU/ENUPose → relative ENU."""
        for p in abss:
            yield self.to_rel(p)

    @classmethod
    def get_rel_position(cls, conn: MAVConnection) -> ENU | None:
        """Request and return the UAV's current local NED position."""
        ## Check this to make blocking optional parameter
        msg = conn.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=0.001)
        if msg:
            return cls.from_ned(msg.x, msg.y, msg.z)
        return None

    def get_position(self, conn: MAVConnection) -> ENU | None:
        """Alias for get_rel_position."""
        rel_pos = self.get_rel_position(conn)
        if rel_pos is None:
            return None
        return self.to_abs(rel_pos)

    def short(self, decimal_places: int = 2) -> ENU:
        """Return a copy of this ENU with coordinates rounded if needed."""
        return ENU(*(format_component(component, decimal_places) for component in self))


class NED(XYZ):
    """NED vector (x=North, y=East, z=Down)."""

    def to_enu(self) -> ENU:
        """Convert NED → ENU coordinates."""
        return ENU(self.y, self.x, -self.z)


class GRA(LLA):
    """Global relative position (MAV_FRAME_GLOBAL_RELATIVE_ALT)."""

    def to_rel(self, p: GRA | GRAPose) -> ENU:
        """Relative ENU of `p` w.r.t. this origin (drops heading if GRAPose)."""
        if isinstance(p, GRAPose):
            p = p.unpose()
        e, n, u = map(float, geodetic2enu(*p, *self, deg=True))  # type: ignore
        return ENU(e, n, u)

    def to_abs(self, point: ENU | ENUPose) -> GRA:
        """Absolute GRA obtained by applying ENU/ENUPose `point` to this origin."""
        if isinstance(point, ENUPose):
            point = point.unpose()
        lat, lon, alt = map(float, enu2geodetic(*point, *self, deg=True))  # type: ignore
        return GRA(lat, lon, alt)

    def pose(self, heading: float = 0.0) -> GRAPose:
        """Return a GRAPose with this position and the given heading."""
        return GRAPose(self.lat, self.lon, self.alt, heading)

    def to_str(self):
        """Return"""
        return ",".join(map(str, self))

    @staticmethod
    def from_global_int(lat_e7: int, lon_e7: int, alt_mm: int) -> GRA:
        """Create a GRA from MAVLink GLOBAL_POSITION_INT message fields."""
        lat = lat_e7 / 1e7
        lon = lon_e7 / 1e7
        alt = alt_mm / 1e3
        return GRA(lat, lon, alt)

    def to_global_int(self) -> tuple[int, int, int]:
        """
        Convert a GRA to MAVLink GLOBAL_POSITION_INT message fields.
        with alt in mm (it agrees with relative altitude).
        """
        lat = int(self.lat * 1e7)
        lon = int(self.lon * 1e7)
        alt = int(self.alt * 1e3)
        return lat, lon, alt

    def to_global_int_alt_in_meters(self) -> tuple[int, int, float]:
        """
        Convert a GRA to MAVLink GLOBAL_POSITION_INT message fields
        with alt in meters (it agrees with absolute altitude).
        """
        lat = int(self.lat * 1e7)
        lon = int(self.lon * 1e7)
        alt = self.alt  # Altitude in meters
        return lat, lon, alt

    @staticmethod
    def from_msn_item_int(lat_e7: int, lon_e7: int, alt_mm: int) -> GRA:
        """Create a GRA from MAVLink GLOBAL_POSITION_INT message fields."""
        lat = lat_e7 / 1e7
        lon = lon_e7 / 1e7
        alt = alt_mm
        return GRA(lat, lon, alt)

    # ---- Batch helpers (GRA) ----
    def to_rel_all(self, points: Iterable[GRA | GRAPose]) -> list[ENU]:
        """Vectorize `to_rel` over a sequence of GRA/GRAPose points."""
        return [self.to_rel(p) for p in points]

    def to_abs_all(self, offsets: Iterable[ENU | ENUPose]) -> list[GRA]:
        """Vectorize `to_abs` over a sequence of ENU/ENUPose offsets."""
        return [self.to_abs(v) for v in offsets]

    def to_rel_iter(self, points: Iterable[GRA | GRAPose]) -> Iterator[ENU]:
        """Iterate GRA/GRAPose → ENU relative to this origin."""
        for p in points:
            yield self.to_rel(p)

    def to_abs_iter(self, offsets: Iterable[ENU | ENUPose]) -> Iterator[GRA]:
        """Iterate ENU/ENUPose → GRA absolute from this origin."""
        for v in offsets:
            yield self.to_abs(v)

    def draw(self, map_obj: folium.Map, label: str, color: Color):
        """Draws a GRAPose as a marker on a folium map."""
        folium.Marker(
            location=[self.lat, self.lon], popup=label, icon=folium.Icon(color=color)
        ).add_to(map_obj)

    @classmethod
    def get_position(cls, conn: MAVConnection) -> GRA | None:
        """
        Request and return the UAV's current global position.
        It requires GLOBAL_POSITION_INT mesages to be emited from ardupilot.
        """
        msg = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=0.001)
        if msg:
            return cls.from_global_int(msg.lat, msg.lon, msg.alt)  # type: ignore
        return None

    def get_enu_position(self, conn: MAVConnection) -> ENU | None:
        """Get the ENU position of the UAV relative to this GRA origin."""
        gra_pos = GRA.get_position(conn)
        if gra_pos is None:
            return None
        return self.to_rel(gra_pos)

    def short(self, decimal_places: int = 6, alt_decimal_places: int = 2) -> GRA:
        """Return a copy of this GRA with coordinates rounded if needed."""
        lat = format_component(self.lat, decimal_places)
        lon = format_component(self.lon, decimal_places)
        alt = format_component(self.alt, alt_decimal_places)
        return GRA(lat, lon, alt)


class ENUPose(XYZPose):
    """Pose in ENU frame: position + heading."""

    def unpose(self) -> ENU:
        """Drop heading and return the ENU point."""
        return ENU(self.x, self.y, self.z)

    def to_abs(self, point: ENU | ENUPose) -> ENUPose:
        """Compose this ENUPose (as origin) with a relative ENU/ENUPose."""
        if isinstance(point, ENU):
            point = point.pose()
        x, y, z, h = point
        x_rot, y_rot = XY(x, y).rotate(self.heading)
        return ENUPose.add(self, ENUPose(x_rot, y_rot, z, h))

    def to_rel(self, point: ENU | ENUPose) -> ENUPose:
        """Absolute ENU/ENUPose → relative ENUPose w.r.t. this origin."""
        if isinstance(point, ENU):
            point = point.pose()
        p = ENUPose.sub(point, self)
        x_rot, y_rot = XY(p.x, p.y).rotate(-self.heading)
        return ENUPose(x_rot, y_rot, p.z, p.heading)

    def to_str(self) -> str:
        """Return a string representation of the ENUPose."""
        return ",".join(map(str, self))

    # ---- Batch helpers (ENUPose) ----
    def to_rel_all(self, points: Iterable[ENU | ENUPose]) -> list[ENUPose]:
        """Vectorize `to_rel` over a sequence of ENU/ENUPose."""
        return [self.to_rel(p) for p in points]

    def to_abs_all(self, offsets: Iterable[ENU | ENUPose]) -> list[ENUPose]:
        """Vectorize `to_abs` over a sequence of ENU/ENUPose offsets."""
        return [self.to_abs(v) for v in offsets]

    def to_rel_iter(self, points: Iterable[ENU | ENUPose]) -> Iterator[ENUPose]:
        """Iterate ENU/ENUPose → relative ENUPose."""
        for p in points:
            yield self.to_rel(p)

    def to_abs_iter(self, offsets: Iterable[ENU | ENUPose]) -> Iterator[ENUPose]:
        """Iterate ENU/ENUPose → absolute ENUPose."""
        for v in offsets:
            yield self.to_abs(v)

    @staticmethod
    def unpose_all(poses: Iterable[ENUPose]) -> list[ENU]:
        """Vectorize `unpose` over a sequence of ENUPose."""
        return [p.unpose() for p in poses]

    def draw(self, ax: Axes, label: str, color: str, alpha: float = 1.0):
        """Draws an ENUPose on a matplotlib Axes with an arrow and label."""
        arrow_scale = 2  # in meters

        dx = math.cos(math.radians(self.heading)) * arrow_scale
        dy = math.sin(math.radians(self.heading)) * arrow_scale

        ax.arrow(  # type: ignore
            self.x,
            self.y,
            dx,
            dy,
            head_width=arrow_scale * 0.5,
            color=color,
            alpha=alpha,
            length_includes_head=True,
        )
        ax.text(self.x, self.y, label, color=color, alpha=alpha)  # type: ignore


class NEDPose(XYZPose):
    """Pose in NED frame: position + heading."""


class GRAPose(LLAPose):
    """Pose in GLOBAL_RELATIVE_ALT frame: lat, lon, alt + heading."""

    def unpose(self) -> GRA:
        """Drop heading and return the GRA point."""
        return GRA(self.lat, self.lon, self.alt)

    def to_abs(self, p: ENU | ENUPose) -> GRAPose:
        """Compose this GRAPose (as origin) with a relative ENU/ENUPose."""
        if isinstance(p, ENU):
            p = p.pose()
        # rotate local offset by origin heading, then apply on globe
        x_rot, y_rot = XY(p.x, p.y).rotate(self.heading)
        lat, lon, alt = map(
            float,
            enu2geodetic(  # type: ignore
                x_rot,
                y_rot,
                p.z,
                self.lat,
                self.lon,
                self.alt,
                deg=True,
            ),
        )
        h_abs = (self.heading + p.heading) % 360
        return GRAPose(lat, lon, alt, h_abs)

    def to_rel(self, p: GRA | GRAPose) -> ENUPose:
        """Absolute GRA/GRAPose → relative ENUPose w.r.t. this GRAPose."""
        if isinstance(p, GRA):
            p = p.pose()
        # world ENU from self to p
        e, n, u = map(
            float,
            geodetic2enu(  # type: ignore
                p.lat,
                p.lon,
                p.alt,
                self.lat,
                self.lon,
                self.alt,
                deg=True,
            ),
        )
        # rotate back into the origin's local frame
        xl, yl = XY(e, n).rotate(-self.heading)
        h_rel = (p.heading - self.heading) % 360
        return ENUPose(xl, yl, u, h_rel)

    def to_str(self) -> str:
        """Return a string representation of the GRAPose."""
        return ",".join(map(str, self))

    # ---- Batch helpers (GRAPose) ----
    def to_abs_all(self, rels: Iterable[ENU | ENUPose]) -> GRAPoses:
        """Vectorize `to_abs` over a sequence of ENU/ENUPose offsets."""
        return [self.to_abs(v) for v in rels]

    def to_rel_all(self, points: Iterable[GRA | GRAPose]) -> ENUPoses:
        """Vectorize `to_rel` over a sequence of GRA/GRAPose points."""
        return [self.to_rel(p) for p in points]

    def to_abs_iter(self, rels: Iterable[ENU | ENUPose]) -> Iterator[GRAPose]:
        """Iterate ENU/ENUPose → absolute GRAPose."""
        for v in rels:
            yield self.to_abs(v)

    def to_rel_iter(self, points: Iterable[GRA | GRAPose]) -> Iterator[ENUPose]:
        """Iterate GRA/GRAPose → relative ENUPose."""
        for p in points:
            yield self.to_rel(p)

    @staticmethod
    def unpose_all(poses: Iterable[GRAPose]) -> GRAs:
        """Vectorize `unpose` over a sequence of GRAPose."""
        return [p.unpose() for p in poses]


# === Type aliases for grouped data (inputs are usually Iterable) ===

XYs = list[XY]
XYZs = list[XYZ]
LLAs = list[LLA]

XYZPoses = list[XYZPose]
LLAPoses = list[LLAPose]
XYZRPYs = list[XYZRPY]

ENUs = list[ENU]
ENUPoses = Iterable[ENUPose]
NEDs = list[NED]
NEDPoses = list[NEDPose]
GRAs = list[GRA]
GRAPoses = list[GRAPose]
