default: release

deps:
	@echo "[Installing Dependencies]"
	@sudo bash ./scripts/deps/install.bash

release:
	@mkdir -p ${HOME}/catkin_ws/src; \
		ln -sf ${PWD}/yac ${HOME}/catkin_ws/src/yac; \
		ln -sf ${PWD}/yac_ros ${HOME}/catkin_ws/src/yac_ros; \
		cd ${HOME}/catkin_ws; \
		catkin build -DCMAKE_BUILD_TYPE=Release -j2

debug:
	@mkdir -p ${HOME}/catkin_ws/src; \
		ln -sf ${PWD}/yac ${HOME}/catkin_ws/src/yac; \
		ln -sf ${PWD}/yac_ros ${HOME}/catkin_ws/src/yac_ros; \
		cd ${HOME}/catkin_ws; \
		catkin build -DCMAKE_BUILD_TYPE=Debug -j2
