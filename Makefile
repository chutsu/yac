SHELL:=/bin/bash
CATKIN_WS=${HOME}/catkin_ws
YAC_PATH=${CATKIN_WS}/src/yac
ROS_VERSION="noetic"
NUM_CPU=4

.PHONY: help deps lib_debug lib debug release download_test_data tests

help:
	@echo -e "\033[1;34m[make targets]:\033[0m"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) \
		| awk 'BEGIN {FS = ":.*?## "}; \
			{printf "\033[1;36m%-20s\033[0m%s\n", $$1, $$2}'

${YAC_PATH}:
	ln -sf ${PWD} ${CATKIN_WS}/src/yac

${CATKIN_WS}:
	@mkdir -p ${CATKIN_WS}/src; catkin init

deps: ## Install dependencies
	@echo "[Installing Dependencies]"
	@make -s -C deps

lib_debug: ## Build libyac in debug mode
	@mkdir -p build \
		&& cd build \
		&& cmake ../yac -DCMAKE_BUILD_TYPE=Debug \
		&& make -j${NUM_CPU}

lib_relwithdeb: ## Build libyac in release with debug mode
	@mkdir -p build \
		&& cd build \
		&& cmake ../yac -DCMAKE_BUILD_TYPE=RelWtihDebInfo \
		&& make -j${NUM_CPU}

lib: ## Build libyac in release mode
	@mkdir -p build \
		&& cd build \
		&& cmake ../yac -DCMAKE_BUILD_TYPE=Release \
		&& make -j${NUM_CPU}

download_test_data: ## Download test data
	@bash ./scripts/download_test_data.bash

tests: ## Build and run tests
	@. /opt/ros/${ROS_VERSION}/setup.sh\
		&& source ${CATKIN_WS}/devel/setup.bash \
		&& rosrun yac test_util_aprilgrid \
		&& rosrun yac test_util_config \
		&& rosrun yac test_util_cv \
		&& rosrun yac test_util_data \
		&& rosrun yac test_util_fs \
		&& rosrun yac test_util_net \
		&& rosrun yac test_util_timeline \
		&& rosrun yac test_calib_data \
		&& rosrun yac test_calib_camera \
		&& rosrun yac test_calib_vi

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
