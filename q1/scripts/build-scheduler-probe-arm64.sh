#!/usr/bin/env bash
set -euo pipefail
q1_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$q1_root/artifacts/scheduler-probe/bin"
cd "$q1_root/rust"
rustc --edition 2021 --target aarch64-unknown-linux-gnu -C opt-level=3 \
    -C linker=aarch64-linux-gnu-gcc tools/scheduler_probe.rs \
    -o "$q1_root/artifacts/scheduler-probe/bin/scheduler-probe"
(
    cd "$q1_root/artifacts/scheduler-probe"
    sha256sum bin/scheduler-probe > SHA256SUMS
)
