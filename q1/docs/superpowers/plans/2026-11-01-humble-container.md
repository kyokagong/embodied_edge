# Q1 Humble Container Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans for inline execution. Keep changes in the user-selected q1 checkout; do not create a worktree or commit.

**Goal:** Build a usable ROS2 Humble development image with the vendored SDK, basic tools, a code bind mount and host networking.

**Architecture:** Compile aimdk_msgs in /opt/q1_ros, separate from the code mounted at /opt/q1. Reuse the existing Dockerfile and entrypoint; add one Docker CLI launcher because Compose is not installed.

**Tech Stack:** Ubuntu22.04, ROS2 Humble, Python3.10, Docker Engine on Linux.

**Spec:** User request in this chat and docs/design.md deployment update.

## Global Constraints

- Work in /home/kyoka/code/embodied_edge/q1, as confirmed by the user.
- Use the official multi-architecture ros:humble-ros-base-jammy image.
- Compile the complete vendored aimdk_msgs package; it is the SDK dependency used by this Python application.
- Preserve existing application flags and default Mock behavior; permit ordinary commands and interactive Bash.
- Mount the current q1 directory, share host networking, use FastDDS, and pass ROS_DOMAIN_ID.
- Keep WITH_PICO optional. Do not start a hardware backend during verification.

## Review Focus

- Mounting code must not hide the compiled SDK or its environment.
- ROS_DOMAIN_ID must reach the container; ROS_LOCALHOST_ONLY must allow off-host communication.
- Interactive Bash must load ROS and aimdk_msgs, including via docker exec.
- Default startup and explicit shell commands must preserve arguments and exit codes.
- Build and smoke tests must work on the current ARM64 host without changing Docker daemon configuration.

## Task 1: Container and launcher

- [x] Add tests/test_entrypoint.py: ordinary Bash command preserves stdout/exit code; existing --backend mock --diagnose flags still work.
- [x] Run the tests and confirm the shell case fails before changing the entrypoint.
- [x] Update Dockerfile: explicit build/ROS/network tools, vendored SDK build, Python requirements, optional Pico dependency and interactive Bash setup.
- [x] Update scripts/entrypoint.sh to dispatch commands and preserve Q1 flags. Add scripts/run-docker.sh for the q1 mount and host networking. Update .dockerignore.
- [x] Verify the entrypoint tests and bash -n on both scripts.

## Task 2: Document and validate

- [x] Document build, Mock, shell, read-only ROS diagnostics, optional Pico and proxy build arguments in README; align hardware/design/deployment references.
- [x] Build embodied-q1:humble using host networking and existing terminal proxy build arguments.
- [x] Run the image with the code bind mount; verify Humble, SDK imports, command dispatch, host networking and Mock HTTP state.
- [x] Run the Python test suite in the image and the isolated DDS loopback with --network none.
- [x] Review the diff and record actual validation in docs/verification.md. No robot actuation or deployment claims.


## Completion evidence

ARM64 image built successfully. Container test suite:75/75. Isolated real-DDS loopback:passed. Launcher, mounted SDK visibility, Mock HTTP endpoint, host network namespace and docker exec environment:passed. Independent code review:no findings. No Q1/Pico actuation, no commit, no Docker daemon changes. Optional Pico installation and x86_64 runtime remain untested.
