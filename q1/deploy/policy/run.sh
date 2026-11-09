#!/usr/bin/env bash
set -eo pipefail
policy_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# The SDK remains installed on Q1; only this task's libraries are prepended.
source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH="$policy_root/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export AMENT_PREFIX_PATH="$policy_root${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export ORT_DYLIB_PATH="$policy_root/lib/libonnxruntime.so.1.23.2"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
program="${1:?Select q1-policy or q1-rt}"; shift
case "$program" in q1-policy|q1-rt) ;; *) exit 2;; esac
exec "$policy_root/bin/$program" "$@"
