#!/usr/bin/env bash
set -euo pipefail
policy_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
model_root="${1:?Verified model package directory required}"
state_root="${2:-/home/run/q1-policy}"
exec python3 "$policy_root/tools/manage.py" --runtime "$policy_root" --model "$model_root" --base "$state_root"
