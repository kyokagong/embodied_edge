#!/usr/bin/env bash
set -euo pipefail
q1_edge_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
model_root="${1:?Model path required}"
guard_root="${2:?Existing guard path required}"
mkdir -p "$q1_edge_root/state"
test -f "$model_root/release.json"
test -S "$guard_root/state/bridge.sock"
test -f "$q1_edge_root/private/operator-key"
if [[ -f "$q1_edge_root/state/edge.pid" ]]; then
    pid="$(cat "$q1_edge_root/state/edge.pid")"
    if sudo -n kill -0 "$pid" 2>/dev/null; then
        printf 'Onboard actor is already running; inspect it before restarting.\n' >&2
        exit 2
    fi
fi
nohup sudo -n bash -c 'echo $$ > "$1"; shift; exec "$@"' onboard \
    "$q1_edge_root/state/edge.pid" taskset -c 4,5 "$q1_edge_root/run" \
    --backend ros2 --controller supermimic --shadow-only --ort-threads 1 \
    --model-dir "$model_root" --hardware-profile "$guard_root/config/hardware.json" \
    --bridge-socket "$guard_root/state/bridge.sock" \
    --operator-key-file "$q1_edge_root/private/operator-key" --host 0.0.0.0 \
    --allow-host 192.168.54.41 --allow-host 192.168.54.49 --port 8768 \
    > "$q1_edge_root/state/edge.log" 2>&1 < /dev/null &
printf 'Started Q1 local inference and console; verify state/edge.log.\n'
