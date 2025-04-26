#!/bin/bash
set -e

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash

# Execute command passed to docker run
exec "$@"