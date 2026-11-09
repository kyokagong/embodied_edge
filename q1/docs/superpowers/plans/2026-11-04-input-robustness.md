# Input link robustness implementation plan

> Execute inline under the user's existing direct-execution authorization. Use systematic debugging, TDD and verification-before-completion.

**Goal:** Reduce avoidable heartbeat delay and expose the cause of real link stalls without extending motion safety deadlines.
**Architecture:** Keep one in-flight input, latest-sample semantics and current same-origin/token/sequence gates. Add compact ACKs, HTTP/1.1 connection reuse, response-completion wakeups, 5Hz dashboard polling and explicit link telemetry. No dependency or transport rewrite.
**Tech Stack:** existing Python stdlib HTTP/threads, JavaScript fetch, ROS2 Humble.

## Constraints and review focus

- Keep 250ms input timeout, ROS feedback/command watchdogs, release-to-zero, focus/disconnect stop and explicit recovery. Never replay a queued stale movement packet or auto-enable.
- No parallel movement POSTs, no old epoch response restoring state, no client timestamps trusted for safety freshness.
- HTTP1.1 error replies close the connection; bound idle sockets and request backlog. Compact replies must still perform full input validation and safety processing.
- Client telemetry is optional, finite, whitelisted diagnostic data; cannot grant control or refresh input independently.
- Current live robot will remain stopped/unenabled during edits and deployment. Validate jitter with mock/isolated DDS, no autonomous physical motion in this task.

## Tasks

1. Add failing API/core tests: compact input avoids constructing robot state, persists sequence and validation; safe keepalive; monotonic gaps and exact timeout diagnostics.
2. Implement compact ACK and keepalive, per-client input gap/RTT summaries and stop context. Run Python suite.
3. Add failing browser tests for compact requests, delayed response immediate latest-sample wakeup, timeout counters, epoch/stop isolation and five-Hz polling. Implement without overlapping/queued inputs; display RTT/timeouts and retain safety UX.
4. Add backend component ages and worker/controller scheduling-gap logs. Run Python/UI regressions and controlled jitter exercise proving tolerable bounded delays and >250ms faults with zero/no auto-recovery.
5. Independent review, fix important findings with regressions, run final suite and isolated DDS, build and deploy. Read-only live status and log checks; report residual network uncertainty.

## Decisions

- Reuse current mounted checkout; existing uncommitted changes belong to the ongoing robot integration and are preserved.
- Start with lower-risk HTTP optimizations rather than a new websocket stack; RTT serialization is reduced but cannot eliminate Wi-Fi/SSH outages. Do not claim field robustness before measurement.

## User steering: RB latch and body yaw

User explicitly replaces continuous physical RB hold with a continuous 1-second long press to enable/latch, another separate 1-second hold to disable; a single uninterrupted hold may fire once only. Calibration and neutral axes still required to arm. Stop/B, source changes, disconnect, focus loss and stale input clear the latch. Connection restoration never re-enables. Simulator keeps its current hold/Space behavior. Start/UI enable remains an explicit alternative; physical enable also latches movement permission. Arming starts native stand/zero, X selects walking. Right-stick horizontal sends whole-body angular_velocity, no direct waist/HAL or advanced-mode change.

Add deterministic server-clock tests for short/long/held press, neutral requirement, B same-frame priority, stale input interruption and right-stick signed yaw. Expose latch and long-press progress in state/UI; update on-screen mapping so it never instructs physical RB hold-to-run after this change. Keep 250ms input/feedback/command deadlines.

## Execution evidence

- RED: compact-state construction/HTTP reuse tests, long-press/yaw and diagnostic tests failed before their implementation. Stop fence regression reproduced delayed Back+Start rearming; independent review reproduced post-toggle-off RB+Start rearming. Both now pass with calibration invalidation on stop and neutral-release edge fencing (disable preserves calibration).
- GREEN: 110 Python tests; Node VM compact/catch-up/timeout/epoch/latch regressions and JS syntax; independent 6-case safety review. Both isolated DDS joint formats pass physical latch cycles and both signed body-yaw commands. No physical nonzero commands this round.
- Bounded jitter: deterministic 100 frames at 50/120/180/200ms pass; 251ms stops and does not automatically re-enable on restored input.
- Local isolated HTTP benchmark, 200 requests each with one reused socket: full response 1701 bytes mean, compact 38 bytes; p50/p95 RTT full 0.348/0.522ms, compact 0.309/0.393ms. Not a measurement of the operator's Wi-Fi/SSH path.
- Image built successfully as embodied-q1:humble (ec163e5117ca); deployment and read-only live status verification follow. Do not interpret loopback as physical turning validation.
- Deployment complete: embodied-q1 runs image ec163e5117ca on host network, original stopped container preserved as embodied-q1-before-rb-latch-20261104. Four read-only live snapshots over 6s confirmed ROS2 connected, all 10 joints fresh, BIPED_WALK_RUN, fault=null, enabled=false, latch=false and all commanded velocities zero. Log health summaries remain healthy. Browser needs refresh for new script and session token.
