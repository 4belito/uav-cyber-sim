FROM ubuntu:22.04 AS main

SHELL ["/bin/bash", "--login", "-c"]
ARG DEBIAN_FRONTEND=noninteractive 

# install general packages
RUN apt-get update && apt-get install --yes --no-install-recommends \
    build-essential \
    ca-certificates \
    dbus-x11 \
    git \
    gnome-terminal \
    locales \
    sudo \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# set time and locale
RUN ln -sf /usr/share/zoneinfo/EST /etc/localtime && \
    sed -i '/en_US.UTF-8/s/^# //g' /etc/locale.gen && \
    locale-gen

# install QGroundControl dependencies
RUN apt-get update && apt-get install --yes --no-install-recommends \
    apt-utils \
    binutils \
    ccache \
    checkinstall \
    cmake \
    curl \
    espeak \
    fuse \
    g++ \
    gcc \
    git \
    gosu \
    kmod \
    libespeak-dev \
    libfontconfig1 \
    libfuse2 \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer1.0-0 \
    libgstreamer1.0-dev \
    libsdl2-dev \
    libssl-dev \
    libudev-dev \
    locales \
    make \
    ninja-build \
    openssh-client \
    openssl \
    patchelf \
    pkg-config \
    rsync \
    speech-dispatcher \
    wget \
    xvfb \
    zlib1g-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# install Ardupilot Gazebo plugin dependencies
RUN apt-get update && apt-get install --yes --no-install-recommends \
    gnupg \
    lsb-release \
    && apt-get clean && rm -rf /var/lib/apt/lists/*
RUN wget -qO- https://packages.osrfoundation.org/gazebo.key | gpg --dearmor | tee /usr/share/keyrings/gazebo-archive-keyring.gpg > /dev/null && \
    echo "deb [signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list 
RUN apt-get update && apt-get install --yes --no-install-recommends \
    gazebo \
    libgazebo-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# add sudo user (required for some install scripts to run)
RUN useradd ubuntu --create-home --home-dir /home/ubuntu --shell /bin/bash && \
    echo "ubuntu ALL=(ALL:ALL) NOPASSWD: ALL" >> /etc/sudoers
USER ubuntu
ENV USER=ubuntu
ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en \
    LC_ALL=en_US.UTF-8
WORKDIR /home/ubuntu

# INSTALL ARDUPILOT
WORKDIR /home/ubuntu
RUN git clone https://github.com/4belito/ardupilot.git --recurse-submodules
RUN ./ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh -y
# add local python modules to path
RUN echo 'export PATH=~/.local/bin:$PATH' >> ~/.profile
# test run
RUN cd ardupilot/ArduCopter && sim_vehicle.py -w
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*

# INSTALL QGROUNDCONTROL
ENV QT_VERSION=5.15.2 \
    DISPLAY=:99 \
    QMAKESPEC=linux-g++-64 \
    QT_PATH=/opt/Qt \
    QT_DESKTOP=$QT_PATH/${QT_VERSION}/gcc_64
RUN echo 'export PATH=/usr/lib/ccache:/opt/Qt/5.15.2/gcc_64/bin:$PATH' >> ~/.profile
RUN git clone https://github.com/4belito/qgroundcontrol.git --recurse-submodules && \
    cd qgroundcontrol && \
    git checkout v4.4.0 && \
    git submodule update --init --recursive
ARG PIP_BREAK_SYSTEM_PACKAGES=1
RUN cd qgroundcontrol && \
    sudo ./deploy/docker/install-qt-linux.sh && \
    mkdir build && \
    cd build && \
    qmake .. && \
    make -j$(nproc) && \
    cd .. && \
    ./deploy/create_linux_appimage.sh . ./build/staging && \
    mv QGroundControl.AppImage ~ && \
    cd && \
    rm -rf qgroundcontrol && \
    chmod a+x QGroundControl.AppImage && \
    cd /home/ubuntu/.config && \
    mkdir QGroundControl.org && \
    touch "QGroundControl.org/QGroundControl Daily.ini"

# INSTALL GAZEBO
RUN git clone https://github.com/4belito/ardupilot_gazebo.git && \
    cd ardupilot_gazebo && \
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
RUN source ~/miniconda3/bin/activate && \
    conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main && \
    conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r && \
    conda create -n uav-cyber-sim python=3.11 && \
    conda activate uav-cyber-sim && \
    pip install \
    folium \
    geopy \
    matplotlib \
    nbformat \
    notebook \
    numpy \
    plotly \
    pymap3d \
    pymavlink

# CLONE THE REPO
RUN git clone https://github.com/4belito/uav-cyber-sim.git


# INSTALL ULI NET SIM
FROM uli-net-sim:latest AS netsim
# collect packages explicitly installed in uli-net-sim image
RUN mkdir -p /deps && apt-mark showmanual | sort -u > /deps/packages.txt
FROM main AS final
# install the same packages
COPY --from=netsim /deps/packages.txt /tmp/packages.txt
USER root
RUN apt-get update && \
    xargs -r -a /tmp/packages.txt apt-get install --yes --no-install-recommends && \
    rm -rf /var/lib/apt/lists/* /tmp/manual.txt
USER ubuntu
# copy over uli-net-sim source and binaries
COPY --from=netsim --chown=1005:1005 /usr/uli-net-sim /usr/uli-net-sim


WORKDIR /home/ubuntu/uav-cyber-sim

ENTRYPOINT [ "tail", "-f", "/dev/null" ]
# CMD ["/bin/bash", "--login"]
