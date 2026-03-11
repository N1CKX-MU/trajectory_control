FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive
ENV TURTLEBOT3_MODEL=burger

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    python3-rosdep \
    gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-simulations \
    ros-humble-nav-msgs \
    ros-humble-geometry-msgs \
    ros-humble-visualization-msgs \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-rviz2 \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-rosbag2 \
    ros-humble-rosbag2-storage-default-plugins \
    ros-humble-ament-cmake-gtest \
    python3-matplotlib \
    python3-numpy \
    libnvidia-gl-550 \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

COPY src/ src/

RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON"
    
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash 2>/dev/null || true" >> /root/.bashrc && \
    echo "export TURTLEBOT3_MODEL=burger" >> /root/.bashrc && \
    echo "export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:/opt/ros/humble/share/turtlebot3_gazebo/models" >> /root/.bashrc && \
    echo "export GAZEBO_MODEL_DATABASE_URI=" >> /root/.bashrc

ENV TURTLEBOT3_MODEL=burger
ENV GAZEBO_MODEL_DATABASE_URI=""
ENV GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:/opt/ros/humble/share/turtlebot3_gazebo/models
ENV XDG_RUNTIME_DIR=/tmp/runtime-root
# COPY Makefile.workspace /ros2_ws/Makefile
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh



ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
