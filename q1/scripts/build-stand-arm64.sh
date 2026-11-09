#!/usr/bin/env bash
set -euo pipefail
q1_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Reuse the checked Q1 SDK cross-build environment; never compile on the robot.
source "$q1_root/scripts/build-onboard-edge-arm64.sh"
cd "$q1_root/rust"
cargo build --release --locked --target aarch64-unknown-linux-gnu --features ros2,supermimic --bin q1-rt --bin q1-stand
stand_package="$q1_root/artifacts/stand/arm64"
mkdir -p "$stand_package/bin" "$stand_package/lib" "$stand_package/config" "$stand_package/notices"
cp target/aarch64-unknown-linux-gnu/release/q1-rt target/aarch64-unknown-linux-gnu/release/q1-stand "$stand_package/bin/"
cp "$q1_root"/cross/aimdk_msgs/lib/*.so "$stand_package/lib/"
cp -a "$q1_root/cross/aimdk_msgs/share" "$stand_package/"
cp "$q1_root"/artifacts/guardian/ort-arm64/*.so* "$stand_package/lib/"
cp "$q1_root/config/supermimic-stand-trial.json" "$stand_package/config/hardware.json"
cp Cargo.lock "$stand_package/notices/"
cp "$q1_root/artifacts/guardian/ort-arm64/provenance.json" "$stand_package/notices/ort-provenance.json"
cp "$q1_root/rust/reports/stand-hardware-map.json" "$stand_package/notices/"
cp "$q1_root/deploy/supermimic/start-stand-onboard.sh" "$stand_package/start"
cat > "$stand_package/run" <<'RUN'
#!/usr/bin/env bash
set -eo pipefail
stand_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH="$stand_root/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export AMENT_PREFIX_PATH="$stand_root${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export ORT_DYLIB_PATH="$stand_root/lib/libonnxruntime.so.1.23.2"
program="${1:?Select q1-rt or q1-stand}"; shift
case "$program" in q1-rt|q1-stand) ;; *) exit 2;; esac
exec "$stand_root/bin/$program" "$@"
RUN
chmod +x "$stand_package/run" "$stand_package/start"
(cd "$stand_package"; find bin lib config notices share run start -type f -print0 | sort -z | xargs -0 sha256sum > SHA256SUMS; sha256sum -c SHA256SUMS >/dev/null)
printf 'Real stand package: %s\n' "$stand_package"
