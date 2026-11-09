#!/usr/bin/env bash
set -euo pipefail
# Run in the project development container after copying the Q1 ROS sysroot.
q1_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
arm_ros="$q1_root/cross/rootfs/opt/ros/humble"
arm_sdk="$q1_root/cross/aimdk_msgs"
test -f "$arm_ros/lib/librcl.so"
# Linker-only sysroot shim: resolve Q1 system dependencies without changing Q1.
for library in "$q1_root"/cross/rootfs/usr/lib/aarch64-linux-gnu/*.so*; do
    [[ -e "$library" ]] || continue
    [[ -e "$arm_ros/lib/$(basename "$library")" ]] || ln -s "$library" "$arm_ros/lib/$(basename "$library")"
done
command -v aarch64-linux-gnu-gcc
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
cargo build --release --locked --target aarch64-unknown-linux-gnu --features ros2,supermimic --bin q1-rt --bin q1-guard-net --bin q1-air-check
package="$q1_root/artifacts/guardian/arm64"
mkdir -p "$package/bin" "$package/lib" "$package/notices"
cp target/aarch64-unknown-linux-gnu/release/q1-rt target/aarch64-unknown-linux-gnu/release/q1-guard-net target/aarch64-unknown-linux-gnu/release/q1-air-check "$package/bin/"
cp "$arm_sdk"/lib/libaimdk_msgs*.so "$package/lib/"
cp -a "$arm_sdk/share" "$package/"
cp "$q1_root/deploy/supermimic/start-onboard-guard.sh" "$package/start"
chmod +x "$package/start"
cp Cargo.lock "$package/notices/"
cp "$q1_root/vendor/primebot_sdk/provenance.json" "$package/notices/"
cp "$q1_root/vendor/r2r_common_cross/CROSS_PATCH.json" "$package/notices/"
cat > "$package/run" <<'RUN'
#!/usr/bin/env bash
set -eo pipefail
q1_guard_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
set +u
source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH="$q1_guard_root/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export AMENT_PREFIX_PATH="$q1_guard_root${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
q1_program="${1:?Select q1-rt, q1-guard-net or q1-air-check}"
shift
case "$q1_program" in q1-rt|q1-guard-net|q1-air-check) ;; *) exit 2;; esac
exec "$q1_guard_root/bin/$q1_program" "$@"
RUN
chmod +x "$package/run"
for f in "$package"/bin/* "$package"/lib/*; do
    aarch64-linux-gnu-readelf -h "$f" | grep -q AArch64
done
(
    cd "$package"
    find bin lib notices share run start -type f -print0 | sort -z | xargs -0 sha256sum > SHA256SUMS
)
printf 'ARM64 package: %s\n' "$package"
