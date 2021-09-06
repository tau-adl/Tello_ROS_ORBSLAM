FROM osrf/ros:melodic-desktop-full
# from https://hub.docker.com/layers/osrf/ros/melodic-desktop-full/images/sha256-c06dcfd97338602211247b347832c70928b30775e365c6e5869e45b89cb58c99?context=explore
# Dockerfile to build https://github.com/samlaf/Tello_ROS_ORBSLAM?organization=samlaf&organization=samlaf

# Not sure if I can have two froms... see https://docs.docker.com/develop/develop-images/multistage-build/
# FROM nvidia/cuda:11.0-base
# Instead we copy it all at the bottom of this file

RUN apt-get update && apt-get install --no-install-recommends -y \
        # dependencies for building packages
        python-rosinstall-generator \
        python-wstool \
        # prerequisites
        python-catkin-tools \
        libeigen3-dev \
        ffmpeg \
        ros-melodic-joystick-drivers \
        python-imaging-tk \
        # for pangolin
        libgl1-mesa-dev \
        libglew-dev \
        libxkbcommon-dev \
        libcanberra-gtk3-module \
        && rm -rf /var/lib/apt/lists/*

COPY . /root/Tello_ROS_ORBSLAM

# Install Pangolin
RUN mkdir /root/Tello_ROS_ORBSLAM/Pangolin/build \
    && cd /root/Tello_ROS_ORBSLAM/Pangolin/build \
    && cmake .. \
    && cmake --build .

# Install h264decoder
RUN mkdir /root/Tello_ROS_ORBSLAM/h264decoder/build \
    && cd /root/Tello_ROS_ORBSLAM/h264decoder/build \
    && cmake .. \
    && cmake --build . \
    && cp libh264decoder.so /usr/local/lib/python2.7/dist-packages

# Install TelloPy & orbslam2
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
    && cd /root/Tello_ROS_ORBSLAM/TelloPy \
    && sudo python setup.py install \
    && cd ../ROS/tello_catkin_ws \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y \
    && catkin init && catkin clean -y && catkin build \
    && echo "source $PWD/devel/setup.bash" >> ~/.bashrc

####################  NVIDIA RELATED #################################################
#
#
# see https://gitlab.com/nvidia/container-images/cuda/blob/master/dist/11.2.2/ubuntu18.04-x86_64/base/Dockerfile
#
#
######################################################################################
RUN apt-get update && apt-get install -y --no-install-recommends \
    gnupg2 curl ca-certificates && \
    curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/7fa2af80.pub | apt-key add - && \
    echo "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64 /" > /etc/apt/sources.list.d/cuda.list && \
    echo "deb https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1804/x86_64 /" > /etc/apt/sources.list.d/nvidia-ml.list && \
    apt-get purge -y curl \
    && rm -rf /var/lib/apt/lists/*

ENV CUDA_VERSION 11.2.2

# For libraries in the cuda-compat-* package: https://docs.nvidia.com/cuda/eula/index.html#attachment-a
RUN apt-get update && apt-get install -y --no-install-recommends \
    cuda-cudart-11-2=11.2.152-1 \
    cuda-compat-11-2 \
    && ln -s cuda-11.2 /usr/local/cuda && \
    rm -rf /var/lib/apt/lists/*

# Required for nvidia-docker v1
RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf \
    && echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf

ENV PATH /usr/local/nvidia/bin:/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES all
ENV NVIDIA_REQUIRE_CUDA "cuda>=11.2 brand=tesla,driver>=418,driver<419 brand=tesla,driver>=440,driver<441 driver>=450,driver<451"
