#!/usr/bin/env bash
set -euo pipefail
q1_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
if [[ -n "${Q1_SETTINGS_FILE:-}" ]]; then
    # Operator-owned shell assignments; never populated from HTTP input.
    source "$Q1_SETTINGS_FILE"
fi
export Q1_MODEL_DIR="${Q1_MODEL_DIR:-$q1_root/artifacts/supermimic/iter22100-onnx}"
export Q1_PROFILE="${Q1_PROFILE:-$q1_root/config/supermimic-unconfirmed.json}"
export Q1_RUN_DIR="${Q1_RUN_DIR:-$q1_root/deploy/supermimic/private/run}"
export Q1_OPERATOR_KEY="${Q1_OPERATOR_KEY:-$q1_root/deploy/supermimic/private/operator-key}"
export Q1_NETWORK_KEY="${Q1_NETWORK_KEY:-$q1_root/deploy/supermimic/private/network-key}"
export Q1_UI_HOST="${Q1_UI_HOST:-127.0.0.1}"
mkdir -p "$Q1_RUN_DIR" "$(dirname "$Q1_OPERATOR_KEY")"
chmod 700 "$q1_root/deploy/supermimic/private" "$Q1_RUN_DIR"
for key_path in "$Q1_OPERATOR_KEY" "$Q1_NETWORK_KEY"; do
    if [[ ! -e "$key_path" ]]; then
        (umask 077; head -c 32 /dev/urandom | od -An -tx1 | tr -d ' \n' > "$key_path")
    fi
done
test -f "$Q1_MODEL_DIR/release.json"
test -f "$Q1_PROFILE"
compose_file="$q1_root/deploy/supermimic/compose.yaml"
if [[ "${Q1_DEPLOYMENT_MODE:-mini}" == "onboard" ]]; then
    compose_file="$q1_root/deploy/supermimic/compose.onboard.yaml"
fi
if docker compose version >/dev/null 2>&1; then
    exec docker compose --project-name q1-supermimic --file "$compose_file" "$@"
fi
q1_compose="$q1_root/deploy/supermimic/private/tools/usr/libexec/docker/cli-plugins/docker-compose"
if [[ ! -x "$q1_compose" ]]; then
    printf 'Docker Compose V2 is required. Install it or unpack it under the project private/tools directory.\n' >&2
    exit 2
fi
exec "$q1_compose" --project-name q1-supermimic --file "$compose_file" "$@"
