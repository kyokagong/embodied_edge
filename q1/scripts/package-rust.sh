#!/usr/bin/env bash
set -euo pipefail

q1_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
binary="${1:-$q1_root/rust/target/release/q1-edge}"
sdk_prefix="${2:-/opt/q1_ros/install/aimdk_msgs}"
output="${3:-$q1_root/rust/dist}"
if [[ ! -x "$binary" || ! -d "$sdk_prefix/lib" ]]; then
    printf 'Usage: %s [release-binary] [aimdk_msgs-install-prefix] [output-directory]\n' "$0" >&2
    printf 'Build with the ros2 feature and supply the matching compiled message package.\n' >&2
    exit 2
fi
python3 - "$binary" <<'PY'
import json
import subprocess
import sys

diagnostic = json.loads(subprocess.check_output([sys.argv[1], "--backend", "ros2", "--diagnose"], text=True))
if diagnostic.get("ros2_compiled") is not True:
    raise SystemExit("Release packaging requires a binary built with --features ros2")
PY
mkdir -p "$output"
output="$(cd "$output" && pwd)"
package="$(mktemp -d "$output/q1-edge-XXXXXXXX")"
mkdir -p "$package/bin" "$package/config" "$package/notices" "$package/ros/aimdk_msgs/lib"
cp "$binary" "$package/bin/q1-edge"
for companion in q1-rt q1-guard-net q1-air-check q1-model-check q1-console-proxy; do
    if [[ -x "$(dirname "$binary")/$companion" ]]; then
        cp "$(dirname "$binary")/$companion" "$package/bin/$companion"
    fi
done
if [[ -f "$q1_root/config/supermimic-unconfirmed.json" ]]; then
    cp "$q1_root/config/supermimic-unconfirmed.json" "$package/config/"
fi
cp "$q1_root/config/default.json" "$package/config/default.json"
cp "$q1_root/vendor/primebot_sdk/provenance.json" "$package/notices/sdk-provenance.json"
cp "$q1_root/vendor/primebot_sdk/友好用户使用声明.md" "$package/notices/"
cp "$q1_root/assets/LICENSE.Apache-2.0" "$package/notices/"
cp "$q1_root/rust/Cargo.lock" "$package/notices/Cargo.lock"
cp -a "$sdk_prefix/share" "$package/ros/aimdk_msgs/share"
for library in "$sdk_prefix"/lib/libaimdk_msgs*.so; do
    # Python bindings are build-time provenance only. The service uses C type support.
    [[ "$library" == *generator_py.so ]] && continue
    cp -L "$library" "$package/ros/aimdk_msgs/lib/"
done
cat > "$package/run" <<'RUN'
#!/usr/bin/env bash
set -eo pipefail
q1_release="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ros_setup="${Q1_ROS_SETUP:-/opt/ros/humble/setup.bash}"
if [[ ! -f "$ros_setup" ]]; then
    printf 'ROS 2 Humble setup is missing: %s\n' "$ros_setup" >&2
    exit 2
fi
source "$ros_setup"
export AMENT_PREFIX_PATH="$q1_release/ros/aimdk_msgs${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export LD_LIBRARY_PATH="$q1_release/ros/aimdk_msgs/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
if [[ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" && -z "${FASTDDS_DEFAULT_PROFILES_FILE:-}" ]]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$q1_release/ros/aimdk_msgs/share/aimdk_msgs/config/fastdds_profiles.xml"
    export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
fi
q1_executable="q1-edge"
case "${1:-}" in
    q1-rt|q1-guard-net|q1-air-check|q1-model-check|q1-console-proxy) q1_executable="$1"; shift;;
esac
exec "$q1_release/bin/$q1_executable" "$@"
RUN
chmod +x "$package/run"
python3 - "$q1_root" "$package" <<'PY'
import hashlib
import json
import pathlib
import platform
import subprocess
import sys

root, release = map(pathlib.Path, sys.argv[1:])
def version(command):
    try:
        return subprocess.check_output(command, text=True, stderr=subprocess.DEVNULL).strip()
    except (OSError, subprocess.CalledProcessError):
        return None
baseline = ["config/default.json", "q1_edge/core.py", "q1_edge/backends.py",
            "q1_edge/config.py", "q1_edge/server.py", "q1_edge/web/app.js",
            "tests/test_core.py", "tests/test_backends.py", "scripts/check_ros2_loopback.py"]
manifest = {
    "architecture": platform.machine(),
    "libc": platform.libc_ver(),
    "rustc": version(["rustc", "--version"]),
    "cargo": version(["cargo", "--version"]),
    "ros_distro": "humble",
    "build_features": ["ros2"] + (["supermimic"] if (release / "bin/q1-guard-net").exists() else []),
    "sdk_commit": json.loads((root / "vendor/primebot_sdk/provenance.json").read_text())["commit"],
    "python_baseline_sha256": {name: hashlib.sha256((root / name).read_bytes()).hexdigest() for name in baseline},
    "system_runtime_packages": version(["dpkg-query", "-W", "-f=${Package}=${Version}\n", "ros-humble-*"]),
    "runtime_requirements": "Matching Ubuntu 22.04/glibc and ROS 2 Humble with rmw_fastrtps_cpp, geometry_msgs, sensor_msgs and nav_msgs. Load through ./run.",
}
(release / "manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n")
PY
(
    cd "$package"
    find . -type f ! -name SHA256SUMS -print0 | sort -z | xargs -0 sha256sum > SHA256SUMS
)
archive="$package.tar.gz"
tar -czf "$archive" -C "$output" "$(basename "$package")"
printf '%s\n' "$archive"
