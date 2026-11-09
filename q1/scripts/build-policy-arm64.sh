#!/usr/bin/env bash
set -euo pipefail
q1_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
package_only=false
package="$q1_root/artifacts/policy/arm64"
while (($#)); do
    case "$1" in
        --package-only) package_only=true; shift ;;
        --output) package="${2:?Output directory required}"; shift 2 ;;
        --help|-h) printf 'Usage: %s [--package-only] [--output DIRECTORY]\nBuild only q1-policy and q1-rt, audit ARM64 ELF closure, and package locally. No robot access.\n' "$0"; exit 0 ;;
        *) printf 'Unknown argument: %s\n' "$1" >&2; exit 2 ;;
    esac
done
arm_ros="$q1_root/cross/rootfs/opt/ros/humble"
arm_sdk="$q1_root/cross/aimdk_msgs"
if ! "$package_only"; then
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
    (cd "$q1_root/rust"; cargo build --release --locked --target aarch64-unknown-linux-gnu --features ros2,supermimic --bin q1-rt --bin q1-policy)
fi
mkdir -p "$(dirname "$package")"
stage="$(mktemp -d "$(dirname "$package")/.policy-package.XXXXXX")"
trap 'rm -rf -- "$stage"' EXIT
mkdir -p "$stage/bin" "$stage/lib" "$stage/config" "$stage/notices" "$stage/tools"
cp "$q1_root/rust/target/aarch64-unknown-linux-gnu/release/q1-rt" "$q1_root/rust/target/aarch64-unknown-linux-gnu/release/q1-policy" "$stage/bin/"
cp "$arm_sdk"/lib/*.so "$stage/lib/"
cp -rL "$arm_sdk/share" "$stage/"
cp "$q1_root"/artifacts/guardian/ort-arm64/*.so* "$stage/lib/"
cp "$q1_root/config/supermimic-stand-trial.json" "$stage/config/hardware.json"
cp "$q1_root/rust/reports/stand-hardware-map.json" "$stage/notices/"
cp "$q1_root/rust/Cargo.lock" "$stage/notices/"
cp "$q1_root/artifacts/guardian/ort-arm64/provenance.json" "$stage/notices/ort-provenance.json"
cp "$q1_root/vendor/primebot_sdk/provenance.json" "$stage/notices/sdk-provenance.json"
cp "$q1_root/vendor/r2r_common_cross/CROSS_PATCH.json" "$stage/notices/"
cp "$q1_root/deploy/policy/manage.py" "$stage/tools/manage.py"
cp "$q1_root/deploy/policy/run.sh" "$stage/run"
cp "$q1_root/deploy/policy/start.sh" "$stage/start"
chmod +x "$stage/run" "$stage/start"
python3 "$q1_root/deploy/policy/validate_arm64.py" --package "$stage" --sysroot "$q1_root/cross/rootfs" --report "$stage/notices/arm64-dependency-closure.json"
python3 - "$q1_root" "$stage" "$package" <<'PY'
from pathlib import Path
import shutil,sys
sys.path.insert(0,str(Path(sys.argv[1])/'deploy/policy'))
from manage import seal,verify_package,sha,atomic_json
stage,output=map(Path,sys.argv[2:])
atomic_json(stage/'notices/runtime.json', {
    'schema':'q1-policy-runtime-v1', 'architecture':'aarch64', 'policy_protocol':2,
    'programs':{name:sha(stage/'bin'/name) for name in ['q1-policy','q1-rt']},
    'profile_sha256':sha(stage/'config/hardware.json'),
    'hardware_map_sha256':sha(stage/'notices/stand-hardware-map.json'),
    'cargo_lock_sha256':sha(stage/'notices/Cargo.lock'),
    'model_abi':{'inputs':{'obs_history':[1,10,78],'goal':[1,594]},'outputs':{'actions':[1,22]},'dtype':'float32'},
    'model_packaging':'independent-immutable-package', 'restart_enabled':False,
    'hardware_qualified':False,
})
identity=seal(stage)
# An existing output is a replaceable local build artifact; remotely installed
# packages use immutable manifest-digest directories via install_immutable.
if output.exists():
    if output.is_symlink() or not output.is_dir():
        raise ValueError('runtime output must be a directory')
    shutil.rmtree(output)
stage.rename(output)
verify_package(output)
print('Policy ARM64 runtime: '+str(output.resolve()))
print('Runtime manifest SHA256: '+identity)
PY
