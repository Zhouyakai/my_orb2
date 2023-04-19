# 基础镜像
FROM nvidia/cuda:10.2-devel-ubuntu18.04

# 安装必要的依赖项
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libopencv-dev \
    libeigen3-dev \
    libglew-dev \
    libsuitesparse-dev \
    libboost-all-dev \
    libx11-dev \
    libgtk-3-dev \
    libopenni2-dev \
    libqhull-dev \
    libgeographic-dev \
    libpcl-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libprotobuf-dev \
    protobuf-compiler \
    pkg-config \
    wget

# 安装Pangolin库
RUN git clone https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j8 && \
    make install

# 复制ORB-SLAM2源代码并构建
COPY . /ORB_SLAM2
WORKDIR /ORB_SLAM2
RUN chmod +x build.sh && \
    ./build.sh

