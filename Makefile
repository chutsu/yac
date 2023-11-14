SHELL:=/bin/bash
PREFIX=/opt/yac
BUILD_DIR=build
CATKIN_WS=${HOME}/yac_ws
YAC_PATH=${CATKIN_WS}/src/yac
ROS_VERSION="noetic"
NUM_CPU=2

.PHONY: help deps lib_debug lib debug release download_test_data tests

define compile_yac
	@cd ${BUILD_DIR} \
		&& cmake ../yac \
		-DCMAKE_BUILD_TYPE=$(1) \
		-DCMAKE_INSTALL_PREFIX=${PREFIX} \
		&& time sudo make install -s -j${NUM_CPU}
endef

define compile_yac_ros
  @cd ${CATKIN_WS} \
	&& . /opt/ros/${ROS_VERSION}/setup.sh \
	&& catkin build yac_ros -DCMAKE_BUILD_TYPE=Release -j${NUM_CPU}
endef

help:
	@echo -e "\033[1;34m[make targets]:\033[0m"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) \
		| awk 'BEGIN {FS = ":.*?## "}; \
		{printf "\033[1;36m%-20s\033[0m%s\n", $$1, $$2}'

${BUILD_DIR}:
	@sudo mkdir -p ${BUILD_DIR}

${PREFIX}:
	@sudo mkdir -p ${PREFIX}

${CATKIN_WS}:
	@mkdir -p ${CATKIN_WS}/src; catkin init

${YAC_PATH}:
	ln -sf ${PWD} ${CATKIN_WS}/src/yac

deps: ## Install dependencies
	@echo "[Installing Dependencies]"
	@make -s -C deps

lib_debug: ${BUILD_DIR} ${PREFIX} ## Build libyac in debug mode
	$(call compile_yac,Debug)

lib_relwithdeb: ${BUILD_DIR} ${PREFIX} ## Build libyac in release with debug info
	$(call compile_yac,RelWithDebInfo)

lib: ${BUILD_DIR} ${PREFIX} ## Build libyac in release mode
	$(call compile_yac,Release)

download_test_data: ## Download test data
	@bash ./scripts/download_test_data.bash

tests: ## Build and run tests
	@cd build \
		&& ./test_util_config \
		&& ./test_util_cv \
		&& ./test_util_data \
		&& ./test_util_fs \
		&& ./test_util_net \
		&& ./test_util_timeline \
		&& ./test_calib_camera \
		&& ./test_calib_data \
		&& ./test_calib_mocap \
		&& ./test_calib_vi

debug: ${CATKIN_WS} ${YAC_PATH} ## Build libyac and yac_ros in debug mode
	$(call compile_yac_ros,Debug)

relwithdeb: ${CATKIN_WS} ${YAC_PATH} ## Build libyac and yac_ros in release with debug info
	$(call compile_yac_ros,RelWithDebInfo)

release: ${CATKIN_WS} ${YAC_PATH} ## Build libyac and yac_ros in release mode
	$(call compile_yac_ros,Release)

build_docker:
	sudo rm -rf build \
		&& docker build -f Dockerfile -t chutsu/yac .

run_docker:
	@xhost +local:docker
	@docker run -e DISPLAY \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		-v /data:/data \
		--network="host" \
		-it \
		--rm chutsu/yac

