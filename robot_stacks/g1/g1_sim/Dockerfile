# g1_sim and G1_bridge in one container: ROS 2 Humble, MuJoCo, offscreen GL (EGL, OSMesa).
FROM docker.io/library/ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-numpy \
        python3-pip \
        python3-pytest \
        python3-yaml \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-rosidl-generator-dds-idl \
        libegl1 \
        libgl1 \
        libosmesa6 \
        iproute2 \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir mujoco==3.3.6 pyzmq opencv-python-headless==4.10.0.84

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Mesa's EGL renders without a GPU; with one passed in, it uses it.
ENV MUJOCO_GL=egl
