#!/usr/bin/env bash
set -euo pipefail
stand_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
model_root="${1:?Model directory required}"
mkdir -p "$stand_root/state"
mkdir -p "$stand_root/private"
chmod 700 "$stand_root/private"
for role in rt stand; do
    if [[ -f "$stand_root/state/$role.pid" ]]; then
        pid="$(cat "$stand_root/state/$role.pid")"
        if sudo -n kill -0 "$pid" 2>/dev/null; then printf '%s already running\n' "$role" >&2; exit 2; fi
    fi
done
nohup sudo -n bash -c 'echo $$ > "$1"; shift; exec "$@"' stand \
    "$stand_root/state/rt.pid" "$stand_root/run" q1-rt \
    --hardware-profile "$stand_root/config/hardware.json" --socket "$stand_root/state/bridge.sock" \
    --enable-hardware --stand-trial --rt-priority 20 --publish-hz "${2:-50}" \
    > "$stand_root/state/rt.log" 2>&1 < /dev/null &
sleep 1
sudo -n taskset -pc 7 "$(cat "$stand_root/state/rt.pid")" >/dev/null
nohup sudo -n bash -c 'echo $$ > "$1"; shift; exec "$@"' stand \
    "$stand_root/state/stand.pid" taskset -c 4,5 "$stand_root/run" q1-stand \
    --model-dir "$model_root" --hardware-profile "$stand_root/config/hardware.json" \
    --socket "$stand_root/state/bridge.sock" \
    --private-trace-file "$stand_root/private/model-trace.json" \
    --port 8768 > "$stand_root/state/stand.log" 2>&1 < /dev/null &
printf 'Q1 real stand stack started, unarmed; explicit UI stand command required (no password).\n'
