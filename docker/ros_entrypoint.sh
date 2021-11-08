#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/melodic/setup.bash"
export ROS_MASTER_URI=http://localhost:11311/
exec "$@"
