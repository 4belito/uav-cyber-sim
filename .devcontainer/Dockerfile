# Declared globally so it can be used in a FROM instruction (Dockerfile spec requirement).
# Override with --build-arg NETSIM_IMAGE=uli-net-sim:latest to include the net-sim layer.
ARG NETSIM_IMAGE=ubuntu:22.04
FROM ubuntu:22.04 AS main

SHELL ["/bin/bash", "--login", "-c"]
ARG DEBIAN_FRONTEND=noninteractive 


# =========
# SYSTEM
# =========
RUN apt-get update && apt-get install --yes --no-install-recommends \
    build-essential \
    ca-certificates \
    dbus-x11 \
    git \
    gnome-terminal \
    locales \
    psmisc \
    sudo \
    socat \
    xterm \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# =========================
# TIME + LOCALE
# =========================
RUN ln -sf /usr/share/zoneinfo/EST /etc/localtime && \
    sed -i '/en_US.UTF-8/s/^# //g' /etc/locale.gen && \
    locale-gen


# =========================
# QGroundControl
# =========================
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
    && apt-get clean && rm -rf /var/lib/apt/lists/*



# =========================
# GAZEBO REPO + INSTALL
# =========================
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

# =========================
# USER SETUP
# =========================
ARG UID=1000
ARG GID=1000
RUN groupadd -o --gid ${GID} ubuntu && \
    useradd -o --uid ${UID} --gid ${GID} -m ubuntu && \
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
ARG REPO_BRANCH=remove-sim_vehicle
RUN git clone --branch ${REPO_BRANCH} --recurse-submodules \
    https://github.com/4belito/uav-cyber-sim.git

WORKDIR /home/ubuntu/uav-cyber-sim

# =========================
# QGroundControl
# =========================
# We place it in the repo folder so it persists and is easy to find
RUN mkdir -p QGroundControl && \
    wget https://github.com/mavlink/qgroundcontrol/releases/download/v4.4.0/QGroundControl.AppImage \
    -O QGroundControl/QGroundControl.AppImage && \
    chmod +x QGroundControl/QGroundControl.AppImage && \
    # Standard Linux config path (stays in home)
    mkdir -p /home/ubuntu/.config/QGroundControl.org && \
    touch "/home/ubuntu/.config/QGroundControl.org/QGroundControl Daily.ini"

# =========================
# ARDUPILOT PREREQS 
# =========================
RUN ./ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh -y
ENV PATH="/home/ubuntu/.local/bin:${PATH}"


# =========================
# UV + PYTHON ENV
# =========================
RUN curl -LsSf https://astral.sh/uv/install.sh | sh
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


# This version handles the variable even if it is currently empty/undefined
ENV GAZEBO_MODEL_PATH=""
ENV GAZEBO_MODEL_PATH="/home/ubuntu/uav-cyber-sim/ardupilot_gazebo/models:${GAZEBO_MODEL_PATH:-}"


# =========================
# ULI NET SIM (optional)
# Pass --build-arg NETSIM_IMAGE=uli-net-sim:latest to enable.
# Defaults to a plain ubuntu image so the build works without the local image.
# =========================
FROM ${NETSIM_IMAGE:-ubuntu:22.04} AS netsim
USER root
RUN mkdir -p /deps /usr/uli-net-sim && \
    if [ -d /usr/uli-net-sim-real ]; then \
    apt-mark showmanual | sort -u > /deps/packages.txt; \
    else \
    touch /deps/packages.txt; \
    fi


# =========================
# FINAL STAGE
# =========================
FROM main AS final


# Re-declare ARGs so they are available in this stage
ARG UID=1000
ARG GID=1000

COPY --from=netsim /deps/packages.txt /tmp/packages.txt

USER root
RUN apt-get update && \
    if [ -s /tmp/packages.txt ]; then \
    xargs -r -a /tmp/packages.txt apt-get install --yes --no-install-recommends; \
    fi && \
    rm -rf /var/lib/apt/lists/* /tmp/packages.txt

COPY --from=netsim /usr/uli-net-sim /usr/uli-net-sim

# --- CRITICAL FOR PORTABILITY ---
# Re-align the 'ubuntu' user one last time and fix ownership of the cloned repo
RUN groupmod -o -g ${GID} ubuntu && \
    usermod  -o -u ${UID} -g ${GID} ubuntu && \
    chown -R ubuntu:ubuntu /home/ubuntu /usr/uli-net-sim

USER ubuntu
WORKDIR /home/ubuntu/uav-cyber-sim

ENV SHELL=/bin/bash
CMD ["/bin/bash"]
