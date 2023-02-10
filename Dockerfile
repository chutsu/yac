FROM ros:noetic-perception-focal

SHELL ["/bin/bash", "-c"]
ENV HOME /root

# Install basic things
RUN apt-get update -yq
RUN apt-get update && apt-get install -qq -y \
  sudo \
  lsb-release \
  build-essential \
  git \
  cmake \
  vim \
  python3-catkin-tools \
  python3-osrf-pycommon

# yac over to home
WORKDIR $HOME
RUN mkdir -p yac
COPY ./ yac

# build yac
WORKDIR $HOME/yac
RUN make deps
RUN make lib
RUN source /opt/ros/noetic/setup.bash && make release
