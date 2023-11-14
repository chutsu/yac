FROM ros:noetic-perception-focal
SHELL ["/bin/bash", "-c"]
ENV HOME /root

# Permissions
RUN apt-get update && apt-get install -y sudo
ARG USER=docker
ARG PASS=docker
ARG UID=1000
ARG GID=1000
RUN useradd -m ${USER} --uid=${UID} && echo "${USER}:${PASS}" | chpasswd
RUN adduser ${USER} sudo
ENV HOME /home/$USER

# Install basic things
RUN apt-get update -yq
RUN apt-get update && apt-get install -qq -y \
  sudo \
  lsb-release \
  build-essential \
  git \
  cmake \
  vim \
  wget

# Install catkin-tools
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
  > /etc/apt/sources.list.d/ros-latest.list'
RUN wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
RUN apt-get update -yq
RUN apt-get install -qq -y python3-catkin-tools

# Build yac
WORKDIR /usr/local/src
RUN git clone https://github.com/chutsu/yac
WORKDIR /usr/local/src/yac
RUN git checkout iros2023
RUN make deps
RUN make lib

# Switch to $USER
USER $USER
RUN echo 'export PS1="[\u@docker] \W # "' >> $HOME/.bashrc

# Update environment variables
ENV LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/opt/yac/lib"
ENV PATH="${PATH}:/opt/yac/bin"

# Create catkin_ws
WORKDIR $HOME
RUN mkdir -p $HOME/catkin_ws/src/
RUN git clone https://github.com/chutsu/rs4se
RUN ln -s /usr/local/src/yac/yac_ros $HOME/catkin_ws/src/yac_ros
RUN . /opt/ros/noetic/setup.sh \
  && cd $HOME/catkin_ws \
  && catkin init \
  && catkin build
