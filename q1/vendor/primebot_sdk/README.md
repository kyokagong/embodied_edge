# Pinned official Q1 SDK references

Repository: https://github.com/PrimebotTech/primebot-q1-sdk

Commit: `7310314eee058323a2c15ab1f4f947df4112573e`.

This directory contains the complete official `aimdk_msgs` ROS2 package, three
exact upstream examples and the original usage notice. `provenance.json` records
Git blob hashes verified against GitHub and SHA-256 hashes for every source file.
The backend does not import or execute these synchronous upstream examples.
`aimdk_msgs/package.xml` declares Apache License2.0. No root LICENSE was found at
this revision; the upstream notice is preserved.

Build the vendored message package in ROS2 Humble (Ubuntu22.04). These commands
only build interfaces and do not connect to or command the robot:

```bash
source /opt/ros/humble/setup.bash
mkdir -p /data/kk/embodied_edge/q1/.ros_ws
cd /data/kk/embodied_edge/q1/.ros_ws
colcon build --base-paths ../vendor/primebot_sdk/aimdk_msgs --packages-select aimdk_msgs
source install/setup.bash
python3 -c 'from aimdk_msgs.msg import McLocomotionVelocity, McCustomJointCommand; from aimdk_msgs.srv import SetMcAction, GetMcAction, SetMcInputSource, GetCurrentInputSource; print("SDK interfaces ready")'
```

Install ROS2 Humble, `colcon`, `rosidl_default_generators`, the standard ROS
message packages and `rmw_fastrtps_cpp` first. Use the Ubuntu system Python3.10
with Humble; a Conda Python3.12 environment cannot load Humble's binary bindings.
Sourcing the official package installs its default FastDDS profile through the
upstream environment hook. Use the same ROS_DOMAIN_ID as the robot and its LAN.
The full package build and localhost DDS loopback have been verified inside the project Docker image; robot LAN discovery remains unverified.

Behavioral decisions follow the pinned
[interface documentation](https://github.com/PrimebotTech/primebot-q1-sdk/blob/7310314eee058323a2c15ab1f4f947df4112573e/%E6%8E%A5%E5%8F%A3%E8%AF%B4%E6%98%8E.md):

- `BIPED_STAND_DEFAULT` has no active balance. UI stand/stop therefore use native
  `BIPED_WALK_RUN` with zero velocity, and exact action status `RUNNING` is required.
- Adding an input source immediately enables arbitration. Connect is read-only;
  only explicit local enable may add `edge_q1` at priority30, below native rc80.
  An existing same-name source is never silently deleted or taken over.
- Current-owner feedback can still name a source whose commands have expired.
  Independent local command, full-joint feedback and owner-feedback watchdogs are
  all required; current owner alone is not evidence of fresh commands.
- Locomotion publishes Reliable at50Hz; custom upper publishes BestEffort,
  Volatile, depth10 at100Hz and always fills all10 joints with SDK gains20/3.
  Only the HAL **state** topic is subscribed; no HAL command or MC-disable exists.
- Joint-state headers must advance and be recent in the ROS clock. Synchronize
  PC/robot clocks. Empty, zero, duplicate, delayed or nonfinite feedback cannot
  authorize motion. Local receipt ages and request deadlines use monotonic time.
- Software stop freezes the last upper command. Native walk hold is requested
  only near official HOME (all10 measured errors <=0.15rad), avoiding an implicit
  arm jump. Away from HOME the source is retained and a visible fault requests
  native remote recovery. A bounded close cannot prove release safe in this case
  or during unknown RPC outcomes; it reports that limit and does not delete the
  source. This recovery needs physical validation before routine teleoperation.

No robot, gamepad, headset, firmware behavior, was available for physical verification. ROS middleware was tested only against a fake peer in an isolated Docker network. Unit tests instantiate complete fields directly from
these schemas and replace only unavailable ROS transport.
