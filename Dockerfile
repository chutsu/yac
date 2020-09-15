FROM ros:melodic-robot

SHELL ["/bin/bash", "-c"]
ENV HOME /root

# Install dependencies
RUN apt-get update -yq
RUN apt-get update && apt-get install -qq -y \
  build-essential \
  git \
  cmake \
  vim \
  openssh-server \
  libopencv-* \
  ros-melodic-cv-bridge \
  ros-melodic-rviz \
  python-catkin-tools \
  libeigen3-dev \
  libceres-dev \
  libgtest-dev

# Copy ros packages over
WORKDIR $HOME/catkin_ws/src
RUN mkdir -p yac
COPY ./ yac
# COPY Makefile yac/Makefile
# COPY README.md yac/README.md
# COPY scripts yac/scripts
# COPY yac yac/yac
# COPY yac_ros yac/yac_ros

# Initialize catkin_ws
RUN mkdir -p $HOME/catkin_ws/src
WORKDIR $HOME/catkin_ws
RUN catkin init

# Build yac
WORKDIR $HOME/catkin_ws
RUN ls $HOME/catkin_ws/src/yac
RUN /bin/bash -c ". /opt/ros/melodic/setup.bash; cd ~/catkin_ws/src/yac; make deps; make release"
