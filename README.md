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

## Docker Image

A Docker image with preinstalled dependencies is available on Docker Hub as [dalbick/uav-cyber-sim](https://hub.docker.com/r/dalbick/uav-cyber-sim).

To enable running GUI applications (e.g. QGroundControl, Gazebo) inside Docker on Linux systems, launch the container with the following command:
```shell
docker run -u ubuntu --env="DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="/dev/dri:/dev/dri:ro" --name <container name> dalbick/uav-cyber-sim
```

VS Code's 'Dev Containers' extension can be used to attach to a running container in order to run the example Jupyter notebooks. 

**If you plan on using the QGroundControl simulator, run it once manually:**
```shell
~/QGroundControl.AppImage --appimage-extract-and-run
```

**If you encounter "cannot connect to display" error, run the following on your host system:**
```shell
xhost +local:root
```