#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:/opt/ros/humble/share/turtlebot3_gazebo/models
export XDG_RUNTIME_DIR=/tmp/runtime-root

mkdir -p /tmp/runtime-root
chmod 700 /tmp/runtime-root
if [ -d /ros2_ws/src ]; then
    cd /ros2_ws
    rosdep install --from-paths src --ignore-src -r -y -q
    colcon build --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --event-handlers console_cohesion+
    source /ros2_ws/install/setup.bash
fi

exec "$@"
