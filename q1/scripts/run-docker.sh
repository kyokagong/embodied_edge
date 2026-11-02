#!/bin/bash
set -euo pipefail

q1_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

exec docker run --rm -it \
    --name embodied-q1 \
    --network host \
    --mount "type=bind,source=${q1_dir},target=/opt/q1" \
    --workdir /opt/q1 \
    --env ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    --env ROS_LOCALHOST_ONLY=0 \
    embodied-q1:humble "$@"
