from typing import List, Tuple
from vehicle_logic import VehicleLogic
from plan import Plan
import subprocess
import platform

from config import ARDUPILOT_VEHICLE_PATH


class SimName:
    NONE = "none"
    QGROUND = "qgroundcontrol"
    GAZEBO = "gazebo"


class Simulator:
    def __init__(
        self,
        name: SimName = SimName.NONE,
        offsets: List[Tuple] = None,
        plans: List[Plan] = None,
    ):
        self.name = name
        self.info = {}
        self.offsets = [(0, 0, 0, 0)] if offsets is None else offsets
        self.n_uavs = len(offsets)
        self.ardu_path = ARDUPILOT_VEHICLE_PATH
        self.plans = [Plan.basic()] if plans is None else plans

    def _add_vehicle_cmd_fn(self, i):
        """To be implemented in subclass."""
        raise NotImplementedError("This method should be implemented in a subclass")

    def _launch_application(self):
        print("ℹ️  Running without a simulator.")

    def add_info(self, key, value):
        self.info[key] = value

    def launch_vehicles(self):
        for i in range(self.n_uavs):
            vehicle_cmd = f"python3 {self.ardu_path} -v ArduCopter -I{i} --sysid {i+1} --no-rebuild"
            vehicle_cmd += self._add_vehicle_cmd_fn(i)
            p = subprocess.Popen(
                [
                    "gnome-terminal",
                    "--",
                    "bash",
                    "-c",
                    f"{vehicle_cmd}; exit",
                ]  # , exec bash"
            )
            print(f"🚀 Vehicle {i+1} launched (PID {p.pid})")

    def launch_logics(self, visible=True):
        """
        Launch one MAVLink proxy process per UAV.

        Args:
            n_uavs (int): Number of UAVs
            visible (bool): If True, open each proxy in a new terminal window
        """
        procs = []
        for i in range(self.n_uavs):
            sysid = i + 1
            command = f"python3 proxy.py --sysid {sysid}"

            if visible:
                if platform.system() == "Linux":
                    p = subprocess.Popen(
                        [
                            "gnome-terminal",
                            "--",
                            "bash",
                            "-c",
                            f"{command};  exec bash",
                        ]  # exit"
                    )
                else:
                    raise OSError("Unsupported OS for visible terminal mode.")
            else:
                # Background mode (silent, attachable)
                p = subprocess.Popen(command.split())

            print(f"🚀 Vehicle {sysid} logic launched (PID {p.pid})")
            procs.append(p)
        return procs

    # def create_VehicleLogics(self, verbose: int = 1):
    #     uavs = []
    #     for i, (offset, plan) in enumerate(zip(self.offsets, self.plans)):
    #         uavs.append(
    #             VehicleLogic(sys_id=i + 1, home=offset[:3], plan=plan, verbose=verbose)
    #         )
    #     return uavs

    def launch(self) -> List[VehicleLogic]:
        self.launch_vehicles()
        self.launch_logics(self.n_uavs)
        self._launch_application()
        # return self.create_VehicleLogics(verbose)

    def __repr__(self):
        return f"SimulatorInfo(name='{self.name}', offsets ={self.offsets}, info={self.info})"
