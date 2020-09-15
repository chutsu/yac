FROM ubuntu:18.04

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
  vim

# yac over to home
WORKDIR $HOME
RUN mkdir -p yac
COPY ./ yac

# build yac
WORKDIR $HOME/yac
RUN make deps
RUN make release
