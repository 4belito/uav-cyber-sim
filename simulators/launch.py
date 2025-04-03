
# import subprocess

# from simulators.QGroundControl.qgc import get_qgc_sim_cmd,get_qgc_vehicle_cmd,add_qgc_links,delete_all_qgc_links
# from simulators.gazebo.gazebo import get_gazebo_vehicle_cmd,get_gazebo_sim_cmd

# from simulators.sim import SimInfo,SimName

    

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
