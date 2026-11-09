#!/usr/bin/env bash
set -euo pipefail
# Run inside the project's Humble build container after the native release build.
q1_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
q1_stage="$q1_root/artifacts/supermimic/mini-runtime"
q1_archives="$q1_root/artifacts/supermimic/mini-packages"
q1_ort="${Q1_ORT_SOURCE:-/opt/ort}"
for binary in q1-edge q1-guard-net q1-rt q1-air-check; do
    test -x "$q1_root/rust/target/release/$binary"
done
archive="$("$q1_root/scripts/package-rust.sh" "$q1_root/rust/target/release/q1-edge" /opt/q1_ros/install/aimdk_msgs "$q1_archives")"
mkdir -p "$q1_stage/ort"
tar -xzf "$archive" --strip-components=1 -C "$q1_stage"
cp "$q1_ort/libonnxruntime.so.1.23.2" "$q1_ort/libonnxruntime_providers_shared.so" "$q1_stage/ort/"
(
    cd "$q1_stage"
    find . -type f ! -name SHA256SUMS -print0 | sort -z | xargs -0 sha256sum > SHA256SUMS
    sha256sum -c SHA256SUMS >/dev/null
)
printf 'Verified Mini PC runtime: %s\n' "$q1_stage"
