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
    socat \
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

# CLONE THE REPO FIRST
RUN git clone --branch remove-sim_vehicle --recurse-submodules \
    https://github.com/4belito/uav-cyber-sim.git
WORKDIR /home/ubuntu/uav-cyber-sim

# INIT SUBMODULES
RUN git submodule update --init --recursive

# INSTALL QGroundControl INSIDE THE REPO
RUN mkdir -p QGroundControl && \
    wget https://github.com/mavlink/qgroundcontrol/releases/download/v4.4.0/QGroundControl.AppImage \
    -O QGroundControl/QGroundControl.AppImage && \
    chmod +x QGroundControl/QGroundControl.AppImage


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
    pymavlink \
    pyserial


ENV PATH="/home/ubuntu/.local/bin:${PATH}"
RUN ./ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh -y
RUN cd ardupilot/Tools/autotest && ./sim_vehicle.py -w -v ArduCopter || true



# Install build tools for ardupilot_gazebo
USER root
RUN apt-get update && apt-get install --yes --no-install-recommends \
    cmake \
    make \
    g++ \
    && apt-get clean && rm -rf /var/lib/apt/lists/*


RUN cd ardupilot_gazebo && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && \
    sudo make install

# INSTALL ULI NET SIM
FROM uli-net-sim:latest AS netsim
RUN mkdir -p /deps && apt-mark showmanual | sort -u > /deps/packages.txt

FROM main AS final

COPY --from=netsim /deps/packages.txt /tmp/packages.txt
USER root
RUN apt-get update && \
    xargs -r -a /tmp/packages.txt apt-get install --yes --no-install-recommends && \
    rm -rf /var/lib/apt/lists/* /tmp/manual.txt

# --- Make UID/GID configurable and align the runtime user ---
ARG UID=1001
ARG GID=1001

# Ensure group/user IDs match the args
RUN groupmod -o -g ${GID} ubuntu && \
    usermod  -o -u ${UID} -g ${GID} ubuntu && \
    # IMPORTANT: fix ownership of files created earlier under the old UID/GID
    chown -R ${UID}:${GID} /home/ubuntu

# Copy uli-net-sim content owned by the runtime user
COPY --from=netsim --chown=${UID}:${GID} /usr/uli-net-sim /usr/uli-net-sim

USER ubuntu
WORKDIR /home/ubuntu/uav-cyber-sim

ENTRYPOINT ["tail","-f","/dev/null"]

