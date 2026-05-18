"""Auto plan module for UAV missions."""

from __future__ import annotations

from typing import Any, Literal, Self

from pymavlink.dialects.v20.ardupilotmega import MAVLink_mission_item_message as ItemMsg

from simulator.helpers.connections.mavlink.customtypes.mission import MissionLoader
from simulator.helpers.connections.mavlink.enums import (
    CmdNav,
    Frame,
)  # Cmd,
from simulator.helpers.coordinates import ENUPose, ENUs, GRAPose, GRAs
from simulator.planner.actions import (
    make_monitoring,
    make_start_mission,
    make_upload_mission,
)
from simulator.planner.plan import Plan, PlanSpec


@Plan.register("AutoPlan")
class AutoPlan(Plan):
    """A UAV plan in auto mode to execute a mission file."""

    def __init__(
        self,
        name: str,
        mission_path: str,
        firmware: Literal["ArduPlane", "ArduCopter"],
        navigation_speed: float = 5.0,
    ):
        super().__init__(name=name)
        self.wps: GRAs
        self.mission_path = mission_path
        item_count = MissionLoader().load(mission_path)
        self.add(make_upload_mission(mission_path=self.mission_path))
        self.extend(
            Plan.arm(
                navigation_speed=navigation_speed,
                firmware=firmware,
            )
        )
        self.add(make_start_mission())
        self.add(make_monitoring(item_count - 1, firmware=firmware))

        self._spec = PlanSpec(
            plan_class="AutoPlan",
            kwargs={
                "name": name,
                "mission_path": mission_path,
                "navigation_speed": navigation_speed,
                "firmware": firmware,
            },
        )

    @classmethod
    def from_spec(cls, **kwargs: Any) -> AutoPlan:
        """Create AutoPlan from specification dictionary."""
        missing = {"name", "mission_path"} - kwargs.keys()
        if missing:
            raise ValueError(f"Missing spec fields: {sorted(missing)}")

        return cls(
            name=kwargs["name"],
            mission_path=kwargs["mission_path"],
            firmware=kwargs["firmware"],
            navigation_speed=kwargs.get("navigation_speed", 5),
        )

    @classmethod
    def rectangle_traj(
        cls,
        xlen: float,
        ylen: float,
        alt: float,
        gra_origin: GRAPose,
        mission_path: str,
        relative_home: ENUPose,
        name: str = "auto_rectangle_plan",
        sysid: int = 1,
        clockwise: bool = True,
        navigation_speed: float = 5.0,
        land: bool = True,
    ) -> Self:
        """Create a rectangular auto plan from relative waypoints."""
        relative_path = Plan.create_rectangle_path(
            xlen=xlen,
            ylen=ylen,
            alt=alt,
            clockwise=clockwise,
        )
        return cls.from_relative_path(
            name=name,
            sysid=sysid,
            gra_origin=gra_origin,
            relative_home=relative_home,
            relative_path=relative_path,
            mission_path=mission_path,
            navigation_speed=navigation_speed,
            land=land,
        )

    @classmethod
    def square_traj(
        cls,
        side_len: float,
        alt: float,
        gra_origin: GRAPose,
        mission_path: str,
        relative_home: ENUPose,
        name: str = "auto_square_plan",
        sysid: int = 1,
        clockwise: bool = True,
        navigation_speed: float = 5.0,
        land: bool = True,
    ) -> Self:
        """Create a square auto plan from relative waypoints."""
        return cls.rectangle_traj(
            xlen=side_len,
            ylen=side_len,
            alt=alt,
            gra_origin=gra_origin,
            mission_path=mission_path,
            relative_home=relative_home,
            name=name,
            sysid=sysid,
            clockwise=clockwise,
            navigation_speed=navigation_speed,
            land=land,
        )

    @classmethod
    def from_path(
        cls,
        name: str,
        sysid: int,
        gra_wps: GRAs,
        mission_path: str,
        navigation_speed: float = 5.0,
        land: bool = True,
        firmware: Literal["ArduPlane", "ArduCopter"] = "ArduCopter",
    ) -> Self:
        """Create and save a basic mission to file."""
        AutoPlan.save_basic_mission(
            mission_path,
            sysid,
            gra_wps,
            land,  # navigation_speed
        )
        plan = cls(
            name=name,
            mission_path=str(mission_path),
            navigation_speed=navigation_speed,
            firmware=firmware,
        )
        return plan

    @classmethod
    def from_relative_path(
        cls,
        name: str,
        sysid: int,
        gra_origin: GRAPose,
        relative_home: ENUPose,
        relative_path: ENUs,
        mission_path: str,
        navigation_speed: float = 5.0,
        land: bool = True,
        firmware: Literal["ArduPlane", "ArduCopter"] = "ArduCopter",
    ) -> Self:
        """Create and save a basic mission from relative waypoints to file."""
        AutoPlan.save_basic_mission_from_relative(
            mission_path,
            sysid,
            gra_origin,
            relative_home,
            relative_path,
            land,
        )

        plan = cls(
            name=name,
            mission_path=str(mission_path),
            navigation_speed=navigation_speed,
            firmware=firmware,
        )

        return plan

    @staticmethod
    def save_basic_mission(
        mission_path: str,
        sysid: int,
        gra_wps: GRAs,
        land: bool = True,
        # speed: float = 5.0,
        takeoff_alt: float | None = None,
    ):
        """Save the mission to file and returns number of items."""
        wps = gra_wps
        mission_loader = MissionLoader(sysid, target_component=0)
        mission_loader.add_latlonalt(
            lat=wps[0].lat,
            lon=wps[0].lon,
            altitude=0,
            terrain_alt=False,
        )
        # takeoff_alt lets the TAKEOFF command finish below cruise altitude so
        # TECS handles the final climb at controlled speed — prevents the abrupt
        # full-throttle → cruise transition that causes phugoid oscillation.
        tk_alt = takeoff_alt if takeoff_alt is not None else wps[1].alt
        mission_loader.add(
            ItemMsg(
                sysid,
                0,
                0,
                Frame.GLOBAL_RELATIVE_ALT,
                CmdNav.TAKEOFF,
                0,
                0,
                0,
                0,
                0,
                0,
                wps[1].lat,
                wps[1].lon,
                tk_alt,
            )
        )
        for wp in wps[1:-1]:
            mission_loader.add_latlonalt(
                lat=wp.lat,
                lon=wp.lon,
                altitude=wp.alt,
                terrain_alt=False,
            )
        if land:
            mission_loader.add(
                ItemMsg(
                    sysid,
                    0,
                    0,
                    Frame.GLOBAL_RELATIVE_ALT,
                    CmdNav.LAND,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    wps[-1].lat,
                    wps[-1].lon,
                    0,
                )
            )
        mission_loader.save(mission_path)

    @staticmethod
    def save_basic_mission_from_relative(
        mission_path: str,
        sysid: int,
        gra_origin: GRAPose,
        relative_home: ENUPose,
        relative_path: ENUs,
        land: bool = True,
        # speed: float = 5.0,
    ):
        """Convert ENU waypoints to GRAs and save the mission to file."""
        gra_wps = GRAPose.resolve_path(gra_origin, relative_home, relative_path)
        AutoPlan.save_basic_mission(
            mission_path,
            sysid,
            gra_wps,
            land,  # speed
        )
