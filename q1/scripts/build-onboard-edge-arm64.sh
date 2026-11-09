#!/usr/bin/env bash
set -euo pipefail
q1_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
arm_ros="$q1_root/cross/rootfs/opt/ros/humble"
arm_sdk="$q1_root/cross/aimdk_msgs"
test -f "$arm_ros/lib/librcl.so"
python3 "$q1_root/scripts/build_arm64_sdk.py"
set +u
source /opt/ros/humble/setup.bash
source /opt/q1_ros/install/setup.bash
set -u
export R2R_HOST_PREFIX_PATH="$AMENT_PREFIX_PATH"
export R2R_TARGET_PREFIX_PATH="$arm_sdk:$arm_ros"
export IDL_PACKAGE_FILTER="aimdk_msgs;builtin_interfaces;std_msgs;geometry_msgs;sensor_msgs;nav_msgs;std_srvs;rcl_interfaces;rosgraph_msgs;action_msgs;unique_identifier_msgs"
export ROS_DISTRO=humble
export AMENT_PREFIX_PATH="$arm_sdk:$arm_ros"
export CMAKE_PREFIX_PATH="$AMENT_PREFIX_PATH"
export CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER=aarch64-linux-gnu-gcc
export CC_aarch64_unknown_linux_gnu=aarch64-linux-gnu-gcc
export CXX_aarch64_unknown_linux_gnu=aarch64-linux-gnu-g++
export BINDGEN_EXTRA_CLANG_ARGS_aarch64_unknown_linux_gnu='--sysroot=/usr/aarch64-linux-gnu'
export CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_RUSTFLAGS="-C link-arg=-Wl,-rpath-link,$arm_ros/lib -C link-arg=-Wl,-rpath-link,$arm_sdk/lib"
cd "$q1_root/rust"
cargo build --release --locked --target aarch64-unknown-linux-gnu --features ros2,supermimic --bin q1-edge
package="$q1_root/artifacts/onboard/arm64"
mkdir -p "$package/bin" "$package/lib" "$package/notices"
cp target/aarch64-unknown-linux-gnu/release/q1-edge "$package/bin/"
cp "$arm_sdk"/lib/libaimdk_msgs*.so "$package/lib/"
cp -a "$arm_sdk/share" "$package/"
cp "$q1_root"/artifacts/guardian/ort-arm64/*.so* "$package/lib/"
cp "$q1_root/artifacts/guardian/ort-arm64/provenance.json" "$package/notices/ort-provenance.json"
cp Cargo.lock "$package/notices/"
cp "$q1_root/deploy/supermimic/start-onboard-edge.sh" "$package/start"
cat > "$package/run" <<'RUN'
#!/usr/bin/env bash
set -eo pipefail
q1_edge_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH="$q1_edge_root/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export AMENT_PREFIX_PATH="$q1_edge_root${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export ORT_DYLIB_PATH="$q1_edge_root/lib/libonnxruntime.so.1.23.2"
exec "$q1_edge_root/bin/q1-edge" "$@"
RUN
chmod +x "$package/run" "$package/start"
(
    cd "$package"
    find bin lib notices share run start -type f -print0 | sort -z | xargs -0 sha256sum > SHA256SUMS
    sha256sum -c SHA256SUMS >/dev/null
)
printf 'Onboard actor package: %s\n' "$package"
