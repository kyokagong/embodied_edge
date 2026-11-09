#!/usr/bin/env bash
set -euo pipefail
q1_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$q1_root/rust"
export CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER=aarch64-linux-gnu-gcc
cargo build --release --locked --target aarch64-unknown-linux-gnu --features supermimic --bin q1-model-check
printf 'Built read-only model checker: %s\n' "$q1_root/rust/target/aarch64-unknown-linux-gnu/release/q1-model-check"
