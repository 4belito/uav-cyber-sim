# Variables
IMAGE_NAME=uav-cyber-sim
DOCKER_USER=4bel
UID=$(shell id -u)
GID=$(shell id -g)
DOCKERFILE_PATH=.devcontainer/Dockerfile

# 1. Download the pre-built image from your account
pull:
	docker pull $(DOCKER_USER)/$(IMAGE_NAME):latest

# 2. Run the environment (GUI enabled for ArduPilot/Gazebo/QGC)
# This uses the version from your Docker Hub account
run:
	docker run -it --rm \
		-u ubuntu \
		--env="DISPLAY" \
		--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
		--volume="/dev/dri:/dev/dri:ro" \
		--name $(IMAGE_NAME)_container \
		$(DOCKER_USER)/$(IMAGE_NAME):latest


# --- Maintenance Commands ---
login:
	docker login -u $(DOCKER_USER)

build:
	docker build --no-cache \
		-f $(DOCKERFILE_PATH) \
		--build-arg UID=$(UID) \
		--build-arg GID=$(GID) \
		-t $(IMAGE_NAME) .

push:
	docker tag $(IMAGE_NAME) $(DOCKER_USER)/$(IMAGE_NAME):latest
	docker push $(DOCKER_USER)/$(IMAGE_NAME):latest