# uav-cyber-sim


## Installation

1. [Install ArduPilot](installation/Installing_Ardupilot_20_04.md)
2. [Install QGroundControl](installation/installing_qgc.md)  
2. [Install Gazebo](installation/installing_gazebo_arduplugin.md)
4. If the installation was not done in the home directory, modify `config.py` accordingly.  
5. Install Python dependencies:
   ```bash
   conda create -n uav-cyber-sim python=3.11
   conda activate uav-cyber-sim
   pip install numpy pymavlink plotly nbformat
