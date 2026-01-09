"""Auto plan module for UAV missions."""

from pymavlink.dialects.v20.ardupilotmega import MAVLink_mission_item_message as ItemMsg

from simulator.helpers.connections.mavlink.customtypes.mission import MissionLoader
from simulator.helpers.connections.mavlink.enums import Cmd, CmdNav, Frame
from simulator.helpers.coordinates import ENUPose, ENUs, GRAPose, GRAs
from simulator.planner.actions import make_start_mission, make_upload_mission
from simulator.planner.actions.monitoring import make_monitoring
from simulator.planner.plan import Plan


class AutoPlan(Plan):
    """A UAV plan in auto mode to execute a mission file."""

    def __init__(
        self,
        name: str,
        mission_path: str,
    ):
        super().__init__(name=name)
        self.wps: GRAs
        self.mission_path = mission_path

        self.add(make_upload_mission(self.mission_path))
        self.extend(Plan.arm())
        self.add(make_start_mission())
        self.add(make_monitoring())

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
