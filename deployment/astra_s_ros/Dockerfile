FROM ros:noetic-ros-base-focal

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG ASTRA_CAMERA_COMMIT=3759de789bba025e500f9b344104afadfcde83e5

RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        build-essential \
        git \
        libgflags-dev \
        libgoogle-glog-dev \
        libudev-dev \
        libusb-1.0-0-dev \
        libuvc-dev \
        ros-noetic-camera-info-manager \
        ros-noetic-cv-bridge \
        ros-noetic-image-geometry \
        ros-noetic-image-transport \
        ros-noetic-rgbd-launch \
    && mkdir -p /opt/astra_ws/src \
    && git clone https://github.com/orbbec/ros_astra_camera.git /opt/astra_ws/src/astra_camera \
    && git -C /opt/astra_ws/src/astra_camera checkout "${ASTRA_CAMERA_COMMIT}" \
    && source /opt/ros/noetic/setup.bash \
    && cd /opt/astra_ws \
    && catkin_make -DCMAKE_BUILD_TYPE=Release \
    && rm -rf /opt/astra_ws/src/astra_camera/.git /var/lib/apt/lists/*

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["roslaunch", "astra_camera", "astra.launch"]
