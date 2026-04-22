# Variables
IMAGE_NAME=uav-cyber-sim
DOCKER_USER=4bel
UID=$(shell id -u)
GID=$(shell id -g)
DOCKERFILE_PATH=.devcontainer/Dockerfile

XAUTH_DOCKER := /tmp/.docker.xauth.$(shell id -u)


# 1. Download the pre-built image from your account
pull:
	docker pull $(DOCKER_USER)/$(IMAGE_NAME):latest

stop:
	@docker rm -f $(IMAGE_NAME)_container 2>/dev/null || true

# 2. Run the environment (GUI enabled for ArduPilot/Gazebo/QGC)
# This uses the version from your Docker Hub account
run: stop
	@touch $(XAUTH_DOCKER)
	@xauth nlist "$$DISPLAY" 2>/dev/null | sed -e 's/^..../ffff/' | xauth -f $(XAUTH_DOCKER) nmerge - 2>/dev/null || true
	@chmod 600 $(XAUTH_DOCKER)
	@if [ -z "$$SSH_CLIENT" ] && [ -z "$$SSH_TTY" ]; then xhost +local:docker > /dev/null 2>&1 || true; fi
	docker run -it --rm \
		--net=host \
		-u ubuntu \
		--env="DISPLAY=$(DISPLAY)" \
		--env="XAUTHORITY=/tmp/.docker.xauth" \
		--volume="$(XAUTH_DOCKER):/tmp/.docker.xauth:rw" \
		$(if $(wildcard /tmp/.X11-unix),--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw",) \
		--volume="/dev/dri:/dev/dri:ro" \
		--name $(IMAGE_NAME)_container \
		$(DOCKER_USER)/$(IMAGE_NAME):latest


# 3. Run targeting a specific VNC display (default :4)
# Use when connecting via TurboVNC from a remote machine.
# Run this from any terminal on the host — no need to be inside the VNC session.
# Override the display with: make vnc-run VNC_DISPLAY=:5
VNC_DISPLAY ?= :4
vnc-run: stop
	@touch $(XAUTH_DOCKER)
	@# Merge cookies from both the VNC xauth file and the default xauth search
	@xauth -f "$$HOME/.vnc/`hostname`$(VNC_DISPLAY).xauth" nlist 2>/dev/null | sed -e 's/^..../ffff/' | xauth -f $(XAUTH_DOCKER) nmerge - 2>/dev/null || true
	@xauth nlist "$(VNC_DISPLAY)" 2>/dev/null | sed -e 's/^..../ffff/' | xauth -f $(XAUTH_DOCKER) nmerge - 2>/dev/null || true
	@chmod 600 $(XAUTH_DOCKER)
	@xhost +local:docker > /dev/null 2>&1 || true
	docker run -it --rm \
		--net=host \
		--ipc=host \
		-u ubuntu \
		--env="DISPLAY=$(VNC_DISPLAY)" \
		--env="XAUTHORITY=/tmp/.docker.xauth" \
		--env="LIBGL_ALWAYS_SOFTWARE=1" \
		--env="GALLIUM_DRIVER=llvmpipe" \
		--env="XDG_RUNTIME_DIR=/tmp/runtime-ubuntu" \
		--env="NO_AT_BRIDGE=1" \
		--env="DBUS_FATAL_WARNINGS=0" \
		--volume="$(XAUTH_DOCKER):/tmp/.docker.xauth:rw" \
		--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
		--name $(IMAGE_NAME)_container \
		$(DOCKER_USER)/$(IMAGE_NAME):latest \
		bash -c 'mkdir -p /tmp/runtime-ubuntu && chmod 700 /tmp/runtime-ubuntu && export DBUS_SESSION_BUS_ADDRESS=$$(dbus-daemon --session --print-address --fork) && exec bash'


# --- Maintenance Commands ---
login:
	docker login -u $(DOCKER_USER)

build:
	docker build --no-cache \
		-f $(DOCKERFILE_PATH) \
		--build-arg UID=$(UID) \
		--build-arg GID=$(GID) \
		-t $(DOCKER_USER)/$(IMAGE_NAME):latest .

push:
	docker push $(DOCKER_USER)/$(IMAGE_NAME):latest