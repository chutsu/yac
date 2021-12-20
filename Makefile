SHELL:=/bin/bash
CATKIN_WS=${HOME}/catkin_ws
YAC_PATH=${CATKIN_WS}/src/yac

default: release
.PHONY: deps release debug

${YAC_PATH}:
	ln -sf ${PWD} ${CATKIN_WS}/src/yac

${CATKIN_WS}:
	@mkdir -p ${CATKIN_WS}/src; catkin init

deps:
	@echo "[Installing Dependencies]"
	@sudo bash ./scripts/deps/install.bash
	@make -s -C deps

lib_debug: ${CATKIN_WS} ${YAC_PATH}
	@cd ${CATKIN_WS} && \
		. /opt/ros/melodic/setup.sh && \
		catkin build yac -DCMAKE_BUILD_TYPE=RelWithDebInfo -j4

lib: ${CATKIN_WS} ${YAC_PATH}
	@cd ${CATKIN_WS} && \
		. /opt/ros/melodic/setup.sh && \
		catkin build yac -DCMAKE_BUILD_TYPE=Release -j4

debug: ${CATKIN_WS} ${YAC_PATH}
	@cd ${CATKIN_WS} && \
		. /opt/ros/melodic/setup.sh && \
		catkin build yac yac_ros -DCMAKE_BUILD_TYPE=Debug -j4

release: ${CATKIN_WS} ${YAC_PATH}
	@cd ${CATKIN_WS} && \
		. /opt/ros/melodic/setup.sh && \
		catkin build yac yac_ros -DCMAKE_BUILD_TYPE=Release -j4

download_test_data:
	@bash ./scripts/download_test_data.bash

tests:
	@. /opt/ros/melodic/setup.sh && \
		source ${CATKIN_WS}/devel/setup.bash && \
		rosrun yac test_calib_data
