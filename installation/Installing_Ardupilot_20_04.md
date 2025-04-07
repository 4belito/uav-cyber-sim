# Installing Ardupilot and MAVProxy Ubuntu 20.04

## Adapted from [Intelligent Quads](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_gazebo_arduplugin.md)

Video Tutorial at https://youtu.be/1FpJvUVPxL0

## Clone ArduPilot

In home directory:
```
cd ~
sudo apt install git
git clone https://github.com/4belito/ardupilot.git
cd ardupilot
```

## Install dependencies:
```
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

reload profile
```
. ~/.profile
```
## If the next step "git submodule update" fails
```
git config --global url.https://.insteadOf git://
```


<!-- ## Checkout Latest Copter Build
```
git checkout Copter-4.5.7
git submodule update --init --recursive
``` -->

Run SITL (Software In The Loop) once to set params:
```
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w
```

