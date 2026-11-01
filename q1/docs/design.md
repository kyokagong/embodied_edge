# Q1 edge interaction design

User requested /data/kk/embodied_edge/q1 implementation through Pico4Ultra upper-body teleoperation, a local UI, gamepad enabling/calibration/stand/walk, and later SuperMimic. User explicitly asks to proceed without physical devices, using official SDK and existing implementations. No git commits.

Approach: retain native MC for balance. Stand/walk and Pico CUSTOM_UPPER are separate exclusive modes; zero velocity and confirm stand before switches. SuperMimic is documented as a future mutually-exclusive HAL backend, not implemented or presented as hardware-ready.

Build a small Python package with numpy only for the offline default. Stdlib HTTP server serves a buildless local dashboard. Default mock backend makes no ROS calls. Explicit --backend ros2 --enable-hardware activates the ROS2 adapter in a Humble workspace with official aimdk_msgs. Browser Gamepad API provides normalized axes/buttons; no separate Linux joydev service is needed. Pico4Ultra uses official XRoboToolkit PC Service and optional xrobotoolkit_sdk; simulator is conspicuously labeled, and simulator data must not drive a ROS backend.

Freshness, strict finite-value validation, button edges, explicit enable after reconnect, neutral calibration, held deadman, asynchronous bounded mode transitions, source ownership checks, and software stop form the controller contract. Software stop is not a physical emergency stop. Upper commands always contain all10 joints and start at measured pose. Hardware upper motion requires confirmed limits/zero-direction profile because SDK limits conflict with official model. No motor-zero writes are exposed. Head/controller calibration maps relative hand translations into robot frame; Q1 arms have4DOF, prioritize hand position and neutral posture, not exact6DOF wrist orientation.

No devices are present: acceptance is deterministic core/kinematics/SDK schema tests, HTTP security and lease tests, simulated input through real browser, published-message traces and real SDK/ROS dependency checks where available. Hardware results remain unverified.

Deployment decision (2026-10-30): physical interaction runs natively on Ubuntu22.04 / ROS2 Humble using a system-Python3.10 venv with --system-site-packages. Q1 Edge and Pico PC Service run on the host. Docker is retained only for offline software verification, not hardware deployment.
