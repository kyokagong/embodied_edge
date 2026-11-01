#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /opt/q1_ros/install/setup.bash
exec python3 -m q1_edge "$@"
