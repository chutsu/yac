#!/bin/bash
set -e  # exit on first error

ROS_VERSION="melodic"
ROS_BASH="/opt/ros/$ROS_VERSION/setup.bash"
ROS_PACKAGES_URL='http://packages.ros.org/ros/ubuntu'
APT_KEYS_URL='http://packages.ros.org/ros.key'
APT_TARGETS="$(lsb_release -sc) main"
SOURCES_LIST_TARGET='/etc/apt/sources.list.d/ros-latest.list'

install() {
	# Update sources.list and add apt-keys
	echo "deb $ROS_PACKAGES_URL $APT_TARGETS" > $SOURCES_LIST_TARGET
	wget $APT_KEYS_URL -O - | apt-key add -

	# Update apt and install ros
	apt-get update -qq  # -qq makes apt-get less noisy
	apt-get install -y ros-$ROS_VERSION-desktop-full

	# Initialize rosdep
	rosdep init
	rosdep update

	# Env setup
	echo "source $ROS_BASH" >> "$HOME/.bashrc"

	# Install ros
	apt-get install -y python-rosinstall python-catkin-tools

	# Fix rosdep permissions
	rosdep fix-permissions
}

# RUN
install
