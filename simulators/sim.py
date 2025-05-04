from typing import List, Tuple
from config import ARDUPILOT_VEHICLE_PATH
from vehicle_logic import VehicleLogic
from plan import Plan
import subprocess
import platform
from config import GCS_BASE_PORT


class SimName:
    NONE = "none"
    QGROUND = "qgroundcontrol"
    GAZEBO = "gazebo"


class Simulator:
    def __init__(
        self,
        name: SimName = SimName.NONE,
        offsets: List[Tuple] = [(0, 0, 0, 0)],
        plans: List[Plan] = [Plan.basic()],
    ):
        self.name = name
        self.info = {}
        self.offsets = offsets
        self.n_uavs = len(offsets)
        self.ardu_path = ARDUPILOT_VEHICLE_PATH
        self.plans = plans

    def _add_vehicle_cmd_fn(self, i):
        return ""

    def _launch_application(self):
        print("ℹ️  Running without a simulator.")

    def add_info(self, key, value):
        self.info[key] = value

    def launch_vehicles(self):
        for i in range(self.n_uavs):
            vehicle_cmd = f"python3 {self.ardu_path} -v ArduCopter -I{i} --sysid {i+1} --no-rebuild"
            vehicle_cmd += self._add_vehicle_cmd_fn(i)
            subprocess.Popen(
                ["gnome-terminal", "--", "bash", "-c", f"{vehicle_cmd}; exec bash"]
            )

    def launch_proxies(self, n_uavs, visible=True):
        """
        Launch one MAVLink proxy process per UAV.

        Args:
            n_uavs (int): Number of UAVs
            visible (bool): If True, open each proxy in a new terminal window
        """
        procs = []
        for i in range(n_uavs):
            sysid = i + 1
            command = f"python3 proxy.py --sysid {sysid}"

            if visible:
                if platform.system() == "Darwin":  # macOS
                    p = subprocess.Popen(
                        [
                            "osascript",
                            "-e",
                            f'tell app "Terminal" to do script "{command}"',
                        ]
                    )
                elif platform.system() == "Linux":
                    p = subprocess.Popen(
                        ["gnome-terminal", "--", "bash", "-c", f"{command}; exec bash"]
                    )
                else:
                    raise OSError("Unsupported OS for visible terminal mode.")
            else:
                # Background mode (silent, attachable)
                p = subprocess.Popen(command.split())

            print(f"🚀 Started proxy for UAV {sysid} (PID {p.pid})")
            procs.append(p)
        return procs

    # def launch_logic(self):
    #     for i in range(self.n_uavs):
    #         cmd = f"python rebroadcast.py --sysid {i+1}"
    #         subprocess.Popen(
    #             ["gnome-terminal", "--", "bash", "-c", f"{cmd}; exec bash"]
    #         )

    def create_VehicleLogics(self, verbose: int = 1):
        uavs = []
        for i, (offset, plan) in enumerate(zip(self.offsets, self.plans)):
            uavs.append(
                VehicleLogic(sys_id=i + 1, home=offset[:3], plan=plan, verbose=verbose)
            )
        return uavs

    def launch(self, verbose: int = 1) -> List[VehicleLogic]:
        self.launch_vehicles()
        self.launch_proxies(self.n_uavs)
        self._launch_application()
        return self.create_VehicleLogics(verbose)

    def __repr__(self):
        return f"SimulatorInfo(name='{self.name}', offsets ={self.offsets}, info={self.info})"
