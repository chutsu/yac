SHELL:=/bin/bash
PREFIX=/opt/yac
BUILD_DIR=build
CATKIN_WS=${HOME}/yac_ws
YAC_PATH=${CATKIN_WS}/src/yac
ROS_VERSION="noetic"
NUM_CPU=4

.PHONY: help deps lib_debug lib debug release download_test_data tests

help:
	@echo -e "\033[1;34m[make targets]:\033[0m"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) \
		| awk 'BEGIN {FS = ":.*?## "}; \
			{printf "\033[1;36m%-20s\033[0m%s\n", $$1, $$2}'

${BUILD_DIR}:
	@sudo mkdir -p ${BUILD_DIR}

${PREFIX}:
	@sudo mkdir -p ${PREFIX}

${YAC_PATH}:
	ln -sf ${PWD} ${CATKIN_WS}/src/yac

${CATKIN_WS}:
	@mkdir -p ${CATKIN_WS}/src; catkin init

deps: ## Install dependencies
	@echo "[Installing Dependencies]"
	@make -s -C deps

lib_debug: ${BUILD_DIR} ${PREFIX} ## Build libyac in debug mode
	@cd ${BUILD_DIR} \
		&& cmake ../yac \
			-DCMAKE_BUILD_TYPE=Debug \
			-DCMAKE_INSTALL_PREFIX=${PREFIX} \
		&& time sudo make install -s -j${NUM_CPU}

lib_relwithdeb: ${BUILD_DIR} ${PREFIX} ## Build libyac in release with debug mode
	@cd ${BUILD_DIR} \
		&& cmake ../yac \
			-DCMAKE_BUILD_TYPE=RelWtihDebInfo \
			-DCMAKE_INSTALL_PREFIX=${PREFIX} \
		&& time sudo make install -s -j${NUM_CPU}

lib: ${BUILD_DIR} ${PREFIX} ## Build libyac in release mode
	@cd ${BUILD_DIR} \
		&& cmake ../yac \
			-DCMAKE_BUILD_TYPE=Release \
			-DCMAKE_INSTALL_PREFIX=${PREFIX} \
		&& time sudo make install -s -j${NUM_CPU}

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
		&& ./test_calib_data \
		&& ./test_calib_camera \
		&& ./test_calib_vi

debug: ${CATKIN_WS} ${YAC_PATH} ## Build libyac and yac_ros in debug mode
	@cd ${CATKIN_WS} \
		&& . /opt/ros/${ROS_VERSION}/setup.sh \
		&& catkin build yac_ros -DCMAKE_BUILD_TYPE=Debug -j${NUM_CPU}

relwithdeb: ${CATKIN_WS} ${YAC_PATH} ## Build libyac and yac_ros in release with debug mode
	@cd ${CATKIN_WS} \
		&& . /opt/ros/${ROS_VERSION}/setup.sh \
		&& catkin build yac_ros -DCMAKE_BUILD_TYPE=RelWithDebInfo -j${NUM_CPU}

release: ${CATKIN_WS} ${YAC_PATH} ## Build libyac and yac_ros in release mode
	cd ${CATKIN_WS} \
		&& . /opt/ros/${ROS_VERSION}/setup.sh \
		&& catkin build yac_ros -DCMAKE_BUILD_TYPE=Release -j${NUM_CPU}
