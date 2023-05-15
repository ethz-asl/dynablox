FROM osrf/ros:noetic-desktop-full
LABEL maintainer="Kin Zhang <qingwen@kth.se>"

# Just in case we need it
ENV DEBIAN_FRONTEND noninteractive

# install zsh
RUN apt update && apt install -y wget git zsh tmux vim g++
RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.1.5/zsh-in-docker.sh)" -- \
  -t robbyrussell \
  -p git \
  -p ssh-agent \
  -p https://github.com/agkozak/zsh-z \
  -p https://github.com/zsh-users/zsh-autosuggestions \
  -p https://github.com/zsh-users/zsh-completions \
  -p https://github.com/zsh-users/zsh-syntax-highlighting

RUN echo "source /opt/ros/noetic/setup.zsh" >> ~/.zshrc
RUN echo "source /opt/ros/noetic/setup.bashrc" >> ~/.bashrc

RUN mkdir -p /workspace/dynablox_ws
RUN git clone --recurse-submodules https://github.com/ethz-asl/dynablox.git /workspace/dynablox_ws/src/dynablox
RUN apt-get install -y python3-vcstool python3-catkin-tools ros-noetic-cmake-modules protobuf-compiler autoconf rsync libtool
# dynablox dependencies
RUN cd /workspace/dynablox_ws/src && vcs import . < ./dynablox/https.rosinstall --recursive
# ouster driver dependencies
RUN cd /workspace && git clone https://github.com/gabime/spdlog.git && cd spdlog && mkdir build && cd build  && cmake .. -DCMAKE_CXX_FLAGS=-fPIC && make -j && make install

WORKDIR /workspace/dynablox_ws