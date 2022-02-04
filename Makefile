SHELL:=/bin/bash
CATKIN_WS=${HOME}/catkin_ws
YAC_PATH=${CATKIN_WS}/src/yac
ROS_VERSION="noetic"

.PHONY: help deps lib_debug lib debug release download_test_data tests

help:
	@echo -e "\033[1;34m[make targets]:\033[0m"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) \
		| awk 'BEGIN {FS = ":.*?## "}; \
			{printf "\033[1;36m%-20s\033[0m%s\n", $$1, $$2}'

${YAC_PATH}:
	ln -sf . ${CATKIN_WS}/src/yac

${CATKIN_WS}:
	@mkdir -p ${CATKIN_WS}/src; catkin init

deps: ## Install dependencies
	@echo "[Installing Dependencies]"
	@sudo bash ./scripts/deps/install.bash
	@make -s -C deps

lib_debug: ${CATKIN_WS} ${YAC_PATH}  ## Build libyac in debug mode
	@cd ${CATKIN_WS} && \
		. /opt/ros/${ROS_VERSION}/setup.sh && \
		catkin build yac -DCMAKE_BUILD_TYPE=RelWithDebInfo -j4

lib_relwithdeb: ${CATKIN_WS} ${YAC_PATH}  ## Build libyac in release with debug mode
	@cd ${CATKIN_WS} && \
		. /opt/ros/${ROS_VERSION}/setup.sh && \
		catkin build yac -DCMAKE_BUILD_TYPE=RelWithDebInfo -j4

lib: ${CATKIN_WS} ${YAC_PATH}  ## Build libyac in release mode
	@cd ${CATKIN_WS} && \
		. /opt/ros/${ROS_VERSION}/setup.sh && \
		catkin build yac -DCMAKE_BUILD_TYPE=Release -j4

debug: ${CATKIN_WS} ${YAC_PATH} ## Build libyac and yac_ros in debug mode
	@cd ${CATKIN_WS} && \
		. /opt/ros/${ROS_VERSION}/setup.sh && \
		catkin build yac yac_ros -DCMAKE_BUILD_TYPE=Debug -j4

relwithdeb: ${CATKIN_WS} ${YAC_PATH} ## Build libyac and yac_ros in release with debug mode
	@cd ${CATKIN_WS} && \
		. /opt/ros/${ROS_VERSION}/setup.sh && \
		catkin build yac yac_ros -DCMAKE_BUILD_TYPE=RelWithDebInfo -j4

release: ${CATKIN_WS} ${YAC_PATH} ## Build libyac and yac_ros in release mode
	cd ${CATKIN_WS} && \
		. /opt/ros/${ROS_VERSION}/setup.sh && \
		catkin build yac yac_ros -DCMAKE_BUILD_TYPE=Release -j4

download_test_data: ## Download test data
	@bash ./scripts/download_test_data.bash

tests: ## Build and run tests
	cd ${CATKIN_WS} && \
		. /opt/ros/${ROS_VERSION}/setup.sh\
		&& source ${CATKIN_WS}/devel/setup.bash \
		&& rosrun yac test_util_aprilgrid \
		&& rosrun yac test_util_config \
		&& rosrun yac test_util_cv \
		&& rosrun yac test_util_data \
		&& rosrun yac test_util_fs \
		&& rosrun yac test_util_net \
		&& rosrun yac test_util_timeline \
		&& rosrun yac test_calib_data \
		&& rosrun yac test_calib_camera
