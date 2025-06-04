FROM ubuntu:20.04

SHELL ["/bin/bash", "--login", "-c"]

# SYSTEM SETUP
# disable interactive prompts
ENV DEBIAN_FRONTEND=noninteractive
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections
RUN apt-get update && \
    apt-get install -y sudo git locales gnome-terminal
# add sudo user
RUN useradd ubuntu
RUN echo "ubuntu ALL=(ALL:ALL) NOPASSWD: ALL" >> /etc/sudoers
USER ubuntu
ENV USER=ubuntu
WORKDIR /home/ubuntu
# set time and locale
RUN sudo ln -sf /usr/share/zoneinfo/EST /etc/localtime
RUN sudo sed -i '/en_US.UTF-8/s/^# //g' /etc/locale.gen && \
    sudo locale-gen
ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en \
    LC_ALL=en_US.UTF-8

# INSTALL ARDUPILOT
RUN git clone https://github.com/4belito/ardupilot.git --recurse-submodules
RUN ./ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh -y
# add local python modules to path
RUN echo "export PATH=~/.local/bin:$PATH" >> ~/.profile
# test run
RUN cd ardupilot/ArduCopter && sim_vehicle.py -w

# INSTALL QGROUNDCONTROL
RUN sudo usermod -a -G dialout ubuntu
RUN sudo apt-get remove -y modemmanager
RUN sudo apt-get install -y gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl libfuse2 libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev '^libxcb.*-dev'
RUN wget https://github.com/mavlink/qgroundcontrol/releases/download/v4.4.4/QGroundControl.AppImage
RUN chmod +x ./QGroundControl.AppImage
# docker appimage fix
RUN dd if=/dev/zero bs=1 count=3 seek=8 conv=notrunc of=./QGroundControl.AppImage
# test run
RUN ./QGroundControl.AppImage --appimage-extract-and-run; exit 0

# INSTALL GAZEBO
RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN sudo apt-get update && \
    sudo apt-get install -y gazebo11 libgazebo11-dev
RUN git clone https://github.com/4belito/ardupilot_gazebo.git
RUN cd ardupilot_gazebo && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && \
    sudo make install
RUN echo 'source /usr/share/gazebo/setup.sh' >> ~/.profile
RUN echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models' >> ~/.profile

# INSTALL MINICONDA
RUN mkdir ~/miniconda3 && \
    wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh && \
    bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3 && \
    rm ~/miniconda3/miniconda.sh && \
    source ~/miniconda3/bin/activate && \
    conda init

# ENVIRONMENT SETUP
RUN . ~/.bashrc && \
    conda create -n uav-cyber-sim python=3.10 && \
    conda activate uav-cyber-sim && \
    pip install numpy pymavlink plotly nbformat notebook

# CLONE THE REPO
RUN git clone https://github.com/4belito/uav-cyber-sim.git
WORKDIR /home/ubuntu/uav-cyber-sim

# ENTRYPOINT [ "tail", "-f", "/dev/null" ]
CMD ["/bin/bash", "--login"]
