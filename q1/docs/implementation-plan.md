# Q1 edge implementation plan

Goal: runnable UI+mock and real SDK/Pico adapters, ready for next-day hardware integration, no git commits.
Architecture: shared pure control logic, nonblocking native-MC adapter, bounded upper-body IK, local HTTP dashboard.
Tech: Python3.10+, numpy, optional ROS2 Humble/aimdk_msgs and xrobotoolkit_sdk.
Deployment: host-native Ubuntu22.04 / ROS2 Humble for all physical interaction; Docker only for offline verification.
Spec: design.md; interfaces: interfaces.md.

## Global constraints
- Requested destination /data/kk/embodied_edge/q1. Stage under /tmp only because destination writes require sandbox escalation; synchronize preserving existing files.
- Preserve native MC; no HAL commands or motor calibration in first version.
- No physical actuation tests without devices; mock default, explicit hardware mode, no auto-enable.
- User has asked to proceed with sensible choices despite unavailable devices; no further preference questions or approval round.

## Review focus
- Old/repeated/invalid input must not keep motion alive; reconnect must require explicit re-enable.
- ROS mode/service requests cannot block watchdog or zero velocity; validate both RPC and business status.
- Upper teleop must not output unfilled joints, jump on calibration, or accept unconfirmed hardware profiles.
- Q1 actual IMU/limits and Pico service availability cannot be fabricated; surface missing prerequisites.
- Browser APIs require lease, same-origin JSON requests, stop on blur/release; real backend rejects simulator input.

## Tasks
- [x] Core: first failing unittest for explicit enable+neutral calibration+deadman; implement state transitions, input validation/lease/watchdog; add regression cases before fixes.
- [x] SDK: schema boundary tests first; mock backend and nonblocking ROS2 bridge, vendor pinned official files with hashes and notices; verify full tests.
- [x] Pico: identity/relative-motion/stale/unreachable tests first; model-based FK/IK and XRobo adapter; explicit measured-pose calibration.
- [x] UI: local buildless dashboard wired to exact API; genuine browser end-to-end tests cover enable/calibration/walk/deadman/stop/teleop.
- [x] Integrate and document: CLI/config, HTTP server security, dependency diagnostics, Docker/run instructions, hardware checklist and SuperMimic boundary.
- [x] Full verification and fresh independent review; fix material findings with failing regression tests; synchronize to requested directory, verify there, no commit.
