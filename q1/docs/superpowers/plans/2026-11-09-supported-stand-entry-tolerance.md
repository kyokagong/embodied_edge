# Supported stand entry tolerance implementation plan

> Execute inline using superpowers:executing-plans. No Git commits, as requested by the user.

**Goal:** Allow modest manually posed variation when entering the supported squat-to-stand reference.

**Architecture:** Set a separate controller entry error cap of 0.60 rad, rather than reusing the hardware's 0.50 rad motor-command correction cap. Keep the original reference, phase matching, native ONNX inputs and all RT limits. Expose the entry limit and name the worst mismatched joint in rejection messages.

**Tech stack:** Existing Rust native policy, integration tests, immutable ARM64 deployment.

**Spec:** User request to relax standing preparation requirements; previous delegated implementation decisions remain applicable.

## Constraints and alternatives

Raising the shared hardware cap would change the model's trained execution wrapper and torque correction behavior; do not do that. Removing the entry gate would admit large mismatches; retain a bounded gate. A separate 0.60 rad entry cap is the chosen implementation decision, based on observed 0.532–0.556 rad rejections. Keep gravity, feedback freshness/skew, temperature, effort, velocity, mechanical ranges and prone-getup restrictions unchanged. This is an experimental entry allowance, not hardware qualification. No automatic stand or dance request.

## Work

- [x] Add a regression probe: 0.556 and 0.599 rad single-joint deviations enter the real supplied reference, 0.601 rad is rejected. Retain tilt and stationary checks.
- [x] Confirm failing test, then separate entry cap from hardware correction cap, improve rejection details, expose limit in read-only state.
- [x] Run Rust and Python regression suites and actual model golden validation; build and check ARM64 bundle. 56 Rust tests, 134 Python tests, 55 native golden probes and 63 ARM64 dependency objects passed.
- [x] Record sensor-skew fault and inspect feedback. Subsequent 40 read-only samples had no fault, max skew 2.517 ms; final check showed STAND/enabled=true, so no remote mutation was attempted.
- [x] Initially deferred at user's request to continue testing, then deployed after explicit authorization and a fresh released/disabled check. Saved prior deployment report; new immutable runtime e3793e81c0eeeb92ddf00a33b0a37b9bcc261312845474f8fb9734655201aa82, RT PID 30330, policy PID 30341. The real API exposes 0.60 rad admission; model/profile remain unchanged. Observed 100 samples over 50.306 s with zero faults, released zero-stiffness throughout and approximately 50 Hz policy/HAL. No hardware task requested; no Git commit.
