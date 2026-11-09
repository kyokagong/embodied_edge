#!/usr/bin/env bash
set -euo pipefail

q1_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
image="${Q1_RUST_IMAGE:-embodied-q1:rust}"
docker build --file "$q1_root/rust/Dockerfile" --tag "$image" "$@" "$q1_root"
printf 'Built %s for the current Docker host architecture.\n' "$image"
printf 'Run mock with docker run --rm --network none %s --diagnose\n' "$image"
