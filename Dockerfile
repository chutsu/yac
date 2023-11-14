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
  vim

# Build yac
WORKDIR /usr/local/src
RUN git clone https://github.com/chutsu/yac
WORKDIR /usr/local/src/yac
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
RUN . /opt/ros/noetic/setup.sh
RUN mkdir -p $HOME/catkin_ws/src/
RUN cd $HOME/catkin_ws/src && ln -s /usr/local/yac/yac_ros .
RUN catkin build
