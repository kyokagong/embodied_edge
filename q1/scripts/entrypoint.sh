#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /opt/q1_ros/install/setup.bash

if [ "$#" -eq 0 ] || [[ "$1" == -* ]]; then
    set -- python3 -m q1_edge "$@"
fi
exec "$@"
