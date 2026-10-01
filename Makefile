# RBSF Racing container workflow (ROS 2 Jazzy / Ubuntu 24.04).
#
# Every variable can be overridden from the command line or the environment, e.g.
#   make build FLAVOR=robot
#   make run ROS_DOMAIN_ID=42 CMD="ros2 launch ..."

# dev on workstations, robot on the car's on-board computer (arm64).
FLAVOR        ?= $(if $(filter aarch64 arm64,$(shell uname -m)),robot,dev)
IMAGE         ?= rbsf-racing
TAG           ?= jazzy-$(FLAVOR)
CONTAINER     ?= rbsf-racing-$(FLAVOR)
# Use e.g. nvidia/cuda:12.9.1-runtime-ubuntu24.04 when nodes need the CUDA toolkit.
BASE_IMAGE    ?= ubuntu:24.04

# Must match on the car and the compute node in distributed mode.
ROS_DOMAIN_ID ?= 0
RMW_IMPLEMENTATION ?=
GPU           ?= $(if $(shell command -v nvidia-smi 2>/dev/null),1,0)
AUTO_BUILD    ?= 1
CMD           ?= bash

RUN_FLAGS := \
	--rm -it \
	--name $(CONTAINER) \
	--net host \
	--ipc host \
	--privileged \
	-v /dev:/dev \
	-v /run/udev:/run/udev:ro \
	-v $(CURDIR):/ros2_ws \
	-e HOST_UID=$(shell id -u) \
	-e HOST_GID=$(shell id -g) \
	-e ROS_DOMAIN_ID=$(ROS_DOMAIN_ID) \
	-e AUTO_BUILD=$(AUTO_BUILD) \
	$(if $(RMW_IMPLEMENTATION),-e RMW_IMPLEMENTATION=$(RMW_IMPLEMENTATION)) \
	$(if $(DISPLAY),-e DISPLAY=$(DISPLAY) -v /tmp/.X11-unix:/tmp/.X11-unix) \
	$(if $(filter 1,$(GPU)),--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all)

.DEFAULT_GOAL := help
.PHONY: help build rebuild run attach stop clean

help: ## Show this help
	@grep -E '^[a-z-]+:.*## ' $(MAKEFILE_LIST) | awk -F':.*## ' '{printf "  \033[1m%-10s\033[0m %s\n", $$1, $$2}'
	@echo ""
	@echo "  FLAVOR=$(FLAVOR) IMAGE=$(IMAGE):$(TAG) ROS_DOMAIN_ID=$(ROS_DOMAIN_ID) GPU=$(GPU)"

build: ## Build the image (incremental, reuses cached layers)
	docker build -f docker/Dockerfile \
		--build-arg BASE_IMAGE=$(BASE_IMAGE) \
		--build-arg FLAVOR=$(FLAVOR) \
		-t $(IMAGE):$(TAG) .

rebuild: ## Build from scratch, pulling fresh base images
	docker build -f docker/Dockerfile --pull --no-cache \
		--build-arg BASE_IMAGE=$(BASE_IMAGE) \
		--build-arg FLAVOR=$(FLAVOR) \
		-t $(IMAGE):$(TAG) .

run: ## Start the container (builds the workspace on start when AUTO_BUILD=1)
	@if [ -n "$(DISPLAY)" ] && command -v xhost > /dev/null; then xhost +SI:localuser:$(shell id -un) > /dev/null; fi
	docker run $(RUN_FLAGS) $(IMAGE):$(TAG) $(CMD)

attach: ## Open another shell in the running container
	docker exec -it -u roboracer $(CONTAINER) bash

stop: ## Stop the running container
	docker stop $(CONTAINER)

clean: ## Remove colcon build artifacts
	rm -rf build/ install/ log/
