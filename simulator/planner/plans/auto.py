"""Auto plan module for UAV missions."""

from __future__ import annotations

from typing import Any, Self

from pymavlink.dialects.v20.ardupilotmega import MAVLink_mission_item_message as ItemMsg

from simulator.config import DATA_PATH
from simulator.helpers.connections.mavlink.customtypes.mission import MissionLoader
from simulator.helpers.connections.mavlink.enums import Cmd, CmdNav, Frame
from simulator.helpers.coordinates import ENUPose, ENUs, GRAPose, GRAs
from simulator.planner.actions import make_start_mission, make_upload_mission
from simulator.planner.actions.monitoring import make_monitoring
from simulator.planner.plan import Plan, PlanSpec


@Plan.register("AutoPlan")
class AutoPlan(Plan):
    """A UAV plan in auto mode to execute a mission file."""

    def __init__(
        self,
        name: str,
        mission_path: str,
        navigation_speed: float = 5.0,
    ):
        super().__init__(name=name)
        self.wps: GRAs
        self.mission_path = mission_path

        self.add(make_upload_mission(self.mission_path))
        self.extend(Plan.arm(navigation_speed=navigation_speed))
        self.add(make_start_mission())
        self.add(make_monitoring())

        self._spec = PlanSpec(
            plan_class="AutoPlan",
            kwargs={
                "name": name,
                "mission_path": mission_path,
                "navigation_speed": navigation_speed,
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
            navigation_speed=kwargs.get("navigation_speed", 5),
        )

    @classmethod
    def rectangle_traj(
        cls,
        xlen: float,
        ylen: float,
        alt: float,
        gra_origin: GRAPose,
        relative_home: ENUPose = ENUPose(0, 0, 0),
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
            navigation_speed=navigation_speed,
            land=land,
        )

    @classmethod
    def square_traj(
        cls,
        side_len: float,
        alt: float,
        gra_origin: GRAPose,
        relative_home: ENUPose = ENUPose(0, 0, 0),
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
        navigation_speed: float = 5.0,
        land: bool = True,
    ) -> Self:
        """Create and save a basic mission to file."""
        mission_path = DATA_PATH / f"mission_{sysid}.waypoints"
        plan = cls(
            name=name,
            mission_path=str(mission_path),
            navigation_speed=navigation_speed,
        )
        plan.save_basic_mission(sysid, gra_wps, land, navigation_speed)
        return plan

    @classmethod
    def from_relative_path(
        cls,
        name: str,
        sysid: int,
        gra_origin: GRAPose,
        relative_home: ENUPose,
        relative_path: ENUs,
        navigation_speed: float = 5.0,
        land: bool = True,
    ) -> Self:
        """Create and save a basic mission from relative waypoints to file."""
        mission_path = DATA_PATH / f"mission_{sysid}.waypoints"
        plan = cls(
            name=name,
            mission_path=str(mission_path),
            navigation_speed=navigation_speed,
        )
        plan.save_basic_mission_from_relative(
            sysid,
            gra_origin,
            relative_home,
            relative_path,
            land,
            navigation_speed,
        )
        return plan

    def save_basic_mission(
        self, sysid: int, gra_wps: GRAs, land: bool = True, speed: float = 5.0
    ) -> None:
        """Save the mission to file."""
        self.wps = gra_wps
        mission_loader = MissionLoader(sysid, target_component=0)
        mission_loader.add_latlonalt(
            lat=self.wps[0].lat,
            lon=self.wps[0].lon,
            altitude=0,
            terrain_alt=False,
        )
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
                *self.wps[0],
            )
        )
        if speed != 5.0:
            # speed_type = 0 → airspeed, 1 → ground speed, 2 → climb rate
            # speed = target speed (in m/s)
            # throttle = throttle (usually -1 = unchanged)
            speed_type = 1
            throttle = -1
            mission_loader.add(
                ItemMsg(
                    sysid,
                    0,
                    0,
                    Frame.GLOBAL_RELATIVE_ALT,
                    Cmd.DO_CHANGE_SPEED,
                    0,
                    0,
                    speed_type,
                    speed,
                    throttle,
                    0,
                    0,
                    0,
                    0,
                )
            )
        for wp in self.wps[1:]:
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
                    self.wps[-1].lat,
                    self.wps[-1].lon,
                    0,
                )
            )
        mission_loader.save(self.mission_path)

    def save_basic_mission_from_relative(
        self,
        sysid: int,
        gra_origin: GRAPose,
        relative_home: ENUPose,
        relative_path: ENUs,
        land: bool = True,
        speed: float = 5.0,
    ) -> None:
        """Convert ENU waypoints to GRAs and save the mission to file."""
        gra_home = gra_origin.to_abs(relative_home)
        grapose_wps = gra_home.to_abs_all(relative_path)
        gra_wps = GRAPose.unpose_all(grapose_wps)
        self.save_basic_mission(sysid, gra_wps, land, speed)
