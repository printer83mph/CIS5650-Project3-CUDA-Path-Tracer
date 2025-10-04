FROM nvidia/cuda:13.0.1-devel-ubuntu24.04


ENV DEBIAN_FRONTEND=noninteractive

# Update system
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y \
  software-properties-common build-essential wget curl

RUN add-apt-repository ppa:git-core/ppa && apt-get update

# Install tooling
RUN apt-get install -y \
  cuda-toolkit \
  ninja-build git cmake=3.28.3-1build7 clangd-20 \
  libglfw3-dev libglew-dev

RUN update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-20 100

# Clean up
RUN apt-get autoremove -y && \
  apt-get clean && \
  rm -rf /var/lib/apt/lists/*
