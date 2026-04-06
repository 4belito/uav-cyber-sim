FROM ubuntu:22.04 AS main

SHELL ["/bin/bash", "--login", "-c"]
ARG DEBIAN_FRONTEND=noninteractive 

# =========================
# SYSTEM + BUILD + QGC DEPS
# =========================
RUN apt-get update && apt-get install --yes --no-install-recommends \
    build-essential \
    ca-certificates \
    dbus-x11 \
    git \
    gnome-terminal \
    locales \
    socat \
    sudo \
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
    gnupg \
    lsb-release \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# =========================
# TIME + LOCALE
# =========================
RUN ln -sf /usr/share/zoneinfo/EST /etc/localtime && \
    sed -i '/en_US.UTF-8/s/^# //g' /etc/locale.gen && \
    locale-gen

# =========================
# GAZEBO REPO + INSTALL
# =========================
RUN wget -qO- https://packages.osrfoundation.org/gazebo.key | gpg --dearmor | tee /usr/share/keyrings/gazebo-archive-keyring.gpg > /dev/null && \
    echo "deb [signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install --yes --no-install-recommends \
    gazebo \
    libgazebo-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# =========================
# USER SETUP
# =========================
RUN useradd ubuntu --create-home --home-dir /home/ubuntu --shell /bin/bash && \
    echo "ubuntu ALL=(ALL:ALL) NOPASSWD: ALL" >> /etc/sudoers

USER ubuntu
ENV USER=ubuntu
ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en \
    LC_ALL=en_US.UTF-8

WORKDIR /home/ubuntu

# =========================
# CLONE REPO (with submodules)
# =========================
RUN git clone --branch remove-sim_vehicle --recurse-submodules \
    https://github.com/4belito/uav-cyber-sim.git

WORKDIR /home/ubuntu/uav-cyber-sim

# =========================
# QGroundControl
# =========================
RUN mkdir -p QGroundControl && \
    wget https://github.com/mavlink/qgroundcontrol/releases/download/v4.4.0/QGroundControl.AppImage \
    -O QGroundControl/QGroundControl.AppImage && \
    chmod +x QGroundControl/QGroundControl.AppImage

# =========================
# ARDUPILOT PREREQS 
# =========================
RUN ./ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh -y

# =========================
# MINICONDA
# =========================
RUN mkdir ~/miniconda3 && \
    wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh && \
    bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3 && \
    rm ~/miniconda3/miniconda.sh && \
    ~/miniconda3/bin/conda init

# =========================
# PYTHON ENV
# =========================
RUN ~/miniconda3/bin/conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main && \
    ~/miniconda3/bin/conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r && \
    ~/miniconda3/bin/conda create -n uav-cyber-sim python=3.11 -y && \
    ~/miniconda3/bin/conda run -n uav-cyber-sim pip install \
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

# =========================
# BUILD ARDUPILOT GAZEBO PLUGIN
# =========================
USER root
RUN cd /home/ubuntu/uav-cyber-sim/ardupilot_gazebo && \
    mkdir -p build && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install

# =========================
# OPTIONAL: SIM_VEHICLE 
# =========================
USER ubuntu
RUN cd ardupilot/Tools/autotest && ./sim_vehicle.py -w -v ArduCopter || true

# =========================
# ULI NET SIM
# =========================
FROM uli-net-sim:latest AS netsim
RUN mkdir -p /deps && apt-mark showmanual | sort -u > /deps/packages.txt

FROM main AS final

COPY --from=netsim /deps/packages.txt /tmp/packages.txt

USER root
RUN apt-get update && \
    xargs -r -a /tmp/packages.txt apt-get install --yes --no-install-recommends && \
    rm -rf /var/lib/apt/lists/* /tmp/packages.txt

# =========================
# UID/GID FIX
# =========================
ARG UID=1001
ARG GID=1001

RUN groupmod -o -g ${GID} ubuntu && \
    usermod  -o -u ${UID} -g ${GID} ubuntu && \
    chown -R ${UID}:${GID} /home/ubuntu

COPY --from=netsim --chown=${UID}:${GID} /usr/uli-net-sim /usr/uli-net-sim

USER ubuntu
WORKDIR /home/ubuntu/uav-cyber-sim

ENTRYPOINT ["tail","-f","/dev/null"]