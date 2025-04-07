
from typing import List,Tuple
from config import ARDUPILOT_VEHICLE_PATH
from vehicle_logic import VehicleLogic
from plan import Plan
import subprocess


class SimName:
    NONE = "none"
    QGROUND = "qgroundcontrol"
    GAZEBO = "gazebo"

class Simulator:
    def __init__(self, name: SimName=SimName.NONE, offsets:List[Tuple]= [(0, 0, 0, 0)],plans:List[Plan]=[Plan.basic()]):
        self.name = name
        self.info = {} 
        self.offsets = offsets 
        self.n_uavs = len(offsets)
        self.plans=plans
        self.ardu_path = ARDUPILOT_VEHICLE_PATH

    
    def _add_vehicle_cmd_fn(self, i):
        return ""
    
    def _launch_application(self):
        print("ℹ️  Running without a simulator.")
       
    def add_info(self,key,value):
        self.info[key]=value

    def launch_vehicles(self):
        for i in range(self.n_uavs):
            vehicle_cmd = f"python3 {self.ardu_path} -v ArduCopter -I{i} --sysid {i+1} --no-rebuild"
            vehicle_cmd += self._add_vehicle_cmd_fn(i)
            subprocess.Popen(["gnome-terminal", "--", "bash", "-c", f"{vehicle_cmd}; exec bash"])

    def create_VehicleLogics(self):
        uavs=[]
        for i,(plan,offset) in enumerate(zip(self.plans,self.offsets)): 
            uavs.append(VehicleLogic(sys_id=i+1,
                        offset=offset,
                        plan= plan))
        return uavs

    def launch(self): 
        self.launch_vehicles()
        self._launch_application()
        return self.create_VehicleLogics()


    def __repr__(self):
        return f"SimulatorInfo(name='{self.name}', offsets ={self.offsets}, info={self.info})"
    

        
        
    


# def launch_simulator(simulator:SimInfo):
#     if simulator.name==SimName.QGROUND:
#         delete_all_qgc_links()
#         add_qgc_links(n=simulator.n_uavs)
#         sim_cmd=get_qgc_sim_cmd()
#     elif simulator.name==SimName.GAZEBO:
#         sim_cmd=get_gazebo_sim_cmd(simulator.offsets,simulator.info['markers'])
#     else:
#         sim_cmd=[]
#         print('No simulator launched')
#     if sim_cmd:
#         subprocess.Popen(
#             sim_cmd,
#             stdout=subprocess.DEVNULL,  # Suppress standard output
#             stderr=subprocess.DEVNULL,  # Suppress error output
#             shell=False  # Ensure safety when passing arguments
#             )
        
# def launch_vehicle(simulator:SimInfo):      
#     for i in range(simulator.n_uavs):
#         vehicle_cmd = f"python3 {simulator.ardu_path} -v ArduCopter -I{i} --sysid {i+1} --no-rebuild"
#         if simulator == 'QGroundControl':
#             vehicle_cmd = get_qgc_vehicle_cmd(vehicle_cmd,simulator.info['spawns'][i])
#         else:
#             vehicle_cmd = get_gazebo_vehicle_cmd(vehicle_cmd)
#         subprocess.Popen(["gnome-terminal", "--", "bash", "-c", f"{vehicle_cmd}; exec bash"])