# robot_sim and the bridges in one container (image robot-sim): ROS 2 Humble, MuJoCo, offscreen GL.
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
# For tools/g1-release-lowcmd.py against robot_sim's motion switcher. --no-deps: its opencv-python
# would overwrite the headless cv2.
RUN pip3 install --no-cache-dir cyclonedds==0.10.2 \
    && pip3 install --no-cache-dir --no-deps \
        https://github.com/unitreerobotics/unitree_sdk2_python/archive/814556d15970dd2ecf1c9984e845ca02ab07e206.tar.gz

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Mesa's EGL renders without a GPU; with one passed in, it uses it.
ENV MUJOCO_GL=egl
