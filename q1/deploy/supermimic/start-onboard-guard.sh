#!/usr/bin/env bash
set -euo pipefail
q1_guard_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mode="${1:-readonly}"
mini_ip="${2:?Mini PC IP is required}"
guard_bind="${3:-0.0.0.0:18766}"
case "$mode" in readonly) hardware=();; air) hardware=(--enable-hardware --commissioning);; *) exit 2;; esac
mkdir -p "$q1_guard_root/state"
test -f "$q1_guard_root/config/hardware.json"
test -f "$q1_guard_root/private/network-key"
for role in rt net; do
    if [[ -f "$q1_guard_root/state/$role.pid" ]]; then
        pid="$(cat "$q1_guard_root/state/$role.pid")"
        if sudo -n kill -0 "$pid" 2>/dev/null; then
            printf 'Guard %s is already running; inspect it before restarting.\n' "$role" >&2
            exit 2
        fi
    fi
done
nohup sudo -n bash -c 'echo $$ > "$1"; shift; exec "$@"' guard \
    "$q1_guard_root/state/rt.pid" "$q1_guard_root/run" q1-rt \
    --hardware-profile "$q1_guard_root/config/hardware.json" \
    --socket "$q1_guard_root/state/bridge.sock" --rt-priority 20 "${hardware[@]}" \
    > "$q1_guard_root/state/rt.log" 2>&1 < /dev/null &
nohup sudo -n bash -c 'echo $$ > "$1"; shift; exec "$@"' guard \
    "$q1_guard_root/state/net.pid" "$q1_guard_root/run" q1-guard-net --mode server --bind "$guard_bind" \
    --allowed-ip "$mini_ip" --socket "$q1_guard_root/state/bridge.sock" \
    --hardware-profile "$q1_guard_root/config/hardware.json" \
    --key-file "$q1_guard_root/private/network-key" \
    > "$q1_guard_root/state/net.log" 2>&1 < /dev/null &

printf 'Started guarded %s session; check state/rt.log and state/net.log.\n' "$mode"
