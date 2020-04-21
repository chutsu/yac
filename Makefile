default: deps build
.PHONY: deps build debug
CATKIN_WS="~/catkin_ws"

deps:
	@echo "Installing deps..."
	@sudo apt-get install -qqq -y libopencv-*
	@sudo apt-get install -qqq -y libceres1 libceres-dev
	@sudo apt-get install -qqq -y libyaml-cpp-dev
	@echo "Done!"

build:
	cd $CATKIN_WS && catkin build yac yac_ros -DCMAKE_BUILD_TYPE=Release -j2

debug:
	cd $CATKIN_WS && catkin build yac yac_ros -DCMAKE_BUILD_TYPE=Debug -j2
