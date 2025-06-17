FROM ubuntu:20.04

SHELL ["/bin/bash", "--login", "-c"]

# SYSTEM SETUP
# disable interactive prompts
ENV DEBIAN_FRONTEND=noninteractive
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections
RUN apt-get update && \
    apt-get install -y sudo git locales gnome-terminal && \
    sudo apt-get -y autoremove && \
    sudo apt-get clean autoclean && \
    rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*
# add sudo user
RUN useradd ubuntu --create-home --home-dir /home/ubuntu --shell /bin/bash && \
	echo "ubuntu ALL=(ALL:ALL) NOPASSWD: ALL" >> /etc/sudoers
USER ubuntu
ENV USER=ubuntu
WORKDIR /home/ubuntu
# set time and locale
RUN sudo ln -sf /usr/share/zoneinfo/EST /etc/localtime && \
	sudo sed -i '/en_US.UTF-8/s/^# //g' /etc/locale.gen && \
    sudo locale-gen
ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en \
    LC_ALL=en_US.UTF-8

# INSTALL ARDUPILOT
RUN git clone https://github.com/4belito/ardupilot.git --recurse-submodules && \
	./ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh -y && \
    sudo apt-get -y autoremove && \
    sudo apt-get clean autoclean && \
    rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/* && \
# add local python modules to path
	echo 'export PATH=~/.local/bin:$PATH' >> ~/.profile
# test run
RUN cd ardupilot/ArduCopter && sim_vehicle.py -w

# INSTALL QGROUNDCONTROL
ENV QT_VERSION=5.15.2 \
    DISPLAY=:99 \
    QMAKESPEC=linux-g++-64 \
    QT_PATH=/opt/Qt \
    QT_DESKTOP=$QT_PATH/${QT_VERSION}/gcc_64
RUN echo 'export PATH=/usr/lib/ccache:/opt/Qt/5.15.2/gcc_64/bin:$PATH' >> ~/.profile
RUN sudo apt-get update && \
    sudo apt-get -y --quiet --no-install-recommends install \
		apt-utils \
		binutils \
		build-essential \
		ca-certificates \
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
		zlib1g-dev && \
    sudo apt-get -y autoremove && \
    sudo apt-get clean autoclean && \
    rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*
RUN git clone https://github.com/4belito/qgroundcontrol.git --recurse-submodules && \
    cd qgroundcontrol && \
	git checkout v4.4.0 && \
    git submodule update --init --recursive && \
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
	chmod a+x QGroundControl.AppImage

# INSTALL GAZEBO
RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
	wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN sudo apt-get update && \
    sudo apt-get install -y gazebo11 libgazebo11-dev && \
    sudo apt-get -y autoremove && \
    sudo apt-get clean autoclean && \
    rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*
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
    conda create -n uav-cyber-sim python=3.11 && \
    conda activate uav-cyber-sim && \
    pip install numpy pymavlink plotly nbformat notebook

# CLONE THE REPO
RUN git clone https://github.com/4belito/uav-cyber-sim.git && \
	cd uav-cyber-sim && \
	git checkout parallel-simulation
WORKDIR /home/ubuntu/uav-cyber-sim

ENTRYPOINT [ "tail", "-f", "/dev/null" ]
# CMD ["/bin/bash", "--login"]
