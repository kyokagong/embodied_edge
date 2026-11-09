# Q1 physical gamepad repair implementation plan

> **For agentic workers:** Use superpowers:executing-plans inline, as the user requested direct execution.

**Goal:** Connect the existing physical gamepad input pipeline to the real Q1 ROS backend with valid feedback.
**Architecture:** Reuse Gamepad API and Controller input ownership. Accept both SDK and standard joint feedback through one validation path; special-case only the observed read-only source query response.
**Tech Stack:** JavaScript, Python 3.10, ROS 2 Humble, Docker.
**Spec:** ../specs/2026-11-04-real-gamepad-design.md

## Global constraints

Keep explicit calibration/enable, RB hold, stale-feedback and command watchdogs. Keep Set service business-state validation. Never move the real robot during automated verification. Retain all ten measured joints; do not fill missing joints with zero.

## Review focus

- Backend changes while browser input is active must reset controls and stop.
- Mismatched, duplicate, nonfinite, stale or repeated standard joints must not refresh feedback.
- UNKNOWN success compatibility must not permit failed Set responses or ownership takeover.
- Holding buttons while connecting must not trigger mode transitions.
- Release RB, disconnected pad and stale input must end in zero commands.

## Task 1: Feedback and source query compatibility

Files: q1_edge/backends.py, tests/test_backends.py.
- [x] Add standard JointState and read-only UNKNOWN response tests and run them to observe failures.
- [x] Subscribe /upper_body_joint_states and normalize into the existing validation path. Preserve SDK subscriber and full pose/time checks.
- [x] Accept UNKNOWN only for GetCurrentInputSource with code=0/reason=0. Verify rejects for failures, nonzero reasons and Set UNKNOWN.
- [x] Run the Python suite offline.

## Task 2: Hardware input source selection

Files: q1_edge/web/app.js, q1_edge/web/index.html, tests/check_ui.cjs.
- [x] Add failing UI tests for automatic physical source, disabled simulator, visible hardware diagnostics and switch-stop behavior.
- [x] On hardware state, disable simulator option and select browser_gamepad if necessary; stop/reset when switching. Preserve mock input selection.
- [x] Run front-end regression tests with Node.js.

## Task 3: End-to-end verification and deployment

Files: scripts/check_ros2_loopback.py, README.md.
- [x] Extend isolated DDS peer to publish actual sensor_msgs/JointState and observed read-only UNKNOWN response.
- [x] Use real Controller gamepad frames for calibrate, enable, walk, deadman-held velocity, RB release and disconnection verification.
- [x] Retain existing source arbitration, upper-stream and watchdog checks.
- [x] Review changes, run Python/UI/isolated DDS tests, build embodied-q1:humble, restart ROS backend, verify fresh live feedback and disabled state.

## Execution notes

- User: direct execution; no further design/plan approval needed.
- Ruling: reuse current checkout because the live development container binds this exact directory; avoid a separate worktree that would not be served.
- No commits or integration requested; leave reviewable changes in workspace.

## Verification ledger

- Backend RED: standard subscription absent and UNKNOWN query rejected (2 failures); GREEN: 79 Python tests passed after fixes.
- Frontend RED: hardware retained simulator selection; GREEN: front-end regression passed after auto-selection fix.
- DDS: both SDK and standard JointState runs passed actual Humble DDS with fake peer under --network none; covered controller gamepad frames, Back/Start/X/RB, positive velocity, RB release zero, pad disconnect zero/source release, upper stream and watchdog.
- Read-only live deployment: connected=true, all ten joints, ~19ms feedback, enabled=false; discovered a transient startup GetMcAction timeout that stayed latched after RPC recovery.
- Startup retry RED: new recovery test failed. Fix only clears the exact query timeout after successful retry before any ownership; active faults stay latched. Full suite and standard DDS rerun required.
- Independent review: no Critical/Important findings. Deferred minor: UI automated harness does not separately cover initial ROS2 page load/no-stop and initially held action buttons; existing held-start backend and release-gating code remain, actual browser input still needs operator confirmation.
- Deployment choice: reuse current bound checkout; cost if unsuitable is that future changes in this directory also affect the development container. No separate worktree, commits or PR created.
- Physical device evidence: Linux sees MACHENIKE G3V2 at js0/event13, 8 axes/15 raw buttons. Chrome's own standard mapping/actual pressed controls require operator confirmation before enabling; kernel mapping alone does not prove browser mapping.

- Final verification: 81/81 Python tests passed; UI regression passed; standard DDS rerun passed after startup retry fix. Image 80bc3e1e9ad3 built and deployed as embodied-q1.
- Final live verification: eight successful samples; backend=ros2, connected=true, fault=null, joint_count=10, last feedback_age_ms=0.8, action=BIPED_WALK_RUN, enabled=false, source_active=false.
- Operational acceptance remains with the operator: browser physical key mapping and real Q1 Set responses/motion have not been exercised by the agent.

## Follow-up: real source registration and diagnostic logger

- User reported enable failure. Event history: first source ADD code=0/state=0/reason=0 was wrongly rejected; later ADDs returned code=1. SDK source-management examples judge header.code==0. Compatibility extended only to source-management UNKNOWN+reason0 responses, retaining nonzero-code/explicit-failure rejection and strict action validation.
- TDD: real source registration compatibility failed before fix; registration/release, nonzero RPC, UNKNOWN-with-reason and UNKNOWN-action rejection tests passed after fix.
- Live cleanup: DELETE edge_q1 code0/state0; then ADD edge_q1 priority30 timeout1000 and immediate DELETE both returned code0/state0. No publishers or action commands were created during this verification.
- User requested more logger. Added queued stdout/rotating-file logger, INFO input/action/source/RPC/fault/velocity records, 5-second health summaries, DEBUG Get RPCs and 1-second input samples, safe API-rejection messages without token/body. TDD logger tests failed before implementation and passed afterwards. Mock subprocess confirmed stdout/file delivery and clean listener shutdown.
- Review found malformed URL logging could repeat a parsing exception. Reproduced as RemoteDisconnected, fixed by safely logging the raw request target with repr; regression now returns HTTP400. Updated spec to firmware evidence.
- Final validation: 88/88 Python tests, UI tests and standard-format isolated DDS pipeline passed; git diff --check passed.
- Deployment: image 7fd00ef793d5, container embodied-q1, --log-level INFO --log-file logs/q1-console.log. File 5MiB plus3 backups; Docker json-file 10MiB plus2 backups. Existing controller container was absent at deployment; recreated after confirming port8766 free.
- Fresh live result: ros2 backend, idle, enabled=false, connected=true, fault=null, source_active=false, 10 joints, feedback age~12ms. Operator must calibrate/enable again; no real motion exercised by agent.

## Follow-up: source acquisition deadlock and authorized physical verification

- User requested actual short forward/reverse, confirmed standing/protection, clear area and onsite estop, and clarified Basic developer mode. Pinned official quickstart says Basic preserves native business/safety and supports external high-level commands; no Advanced mode, MC shutdown or HAL command was used.
- Live failure timeline: source ADD code0/state0 at 11:41:58Z; zero heartbeat ~50Hz; owner remained empty; acquisition timed out at 11:42:01Z. Native mc_ros2_node subscribes the correct velocity type with compatible QoS. Zero-only ADD/ENABLE and reliable/best-effort probes all retained empty attribution.
- Direct SDK test 11:50:14Z: ±0.2m/s for <=0.6s each; first nonzero established owner. Odometry heading projection forward +1.955cm, reverse -1.936cm. Zero + DELETE confirmed. Actual records logs/bounded-motion-20261104.jsonl.
- Subsequent Get retained the deleted diagnostic source name, as documented by official interface. One application preflight correctly aborted with zero packets due strict foreign-name gate; another preflight rejected not-yet-fresh backend feedback. They did not actuate. Diagnostic harness then used the same backend ROS context and awaited fresh feedback.
- Implemented one bounded explicit walk+RB+nonzero acquisition, cap0.2 each component, deadline=min(0.25s,command_timeout,feedback_timeout), measured only from first motion intent. Standby no longer acquires/times out. Established ownership loss stays latched, never automatically reacquires. Native MC priority arbitration remains authoritative because cached attribution cannot identify live foreign commands; priority stays below RC80.
- Every new ADD uses configured base name plus secrets.token_hex(6) suffix; prevents old cached SELF attribution from confirming a new registration epoch. No other sources are deleted. Node name stays stable.
- Independent review found late first ACK could bypass timeout and non-WALK startup could deadlock. Both reproduced RED then fixed: reject late ACK before adopting ownership; initial unowned enable requires native WALK RUNNING. Longer standby test also reproduced old mode timer expiry, fixed by no transition timer if already RUNNING walk.
- RED→GREEN covered first nonzero acquisition, idle without timeout, finite cap/deadline/release, active RC arbitration rejection, expired foreign attribution handover, cached self epoch isolation, late ACK, initial mode refusal, and Controller walk+RB path. 96/96 Python tests passed; UI tests and standard DDS peer requiring nonzero + retaining deleted attribution passed. git diff --check passed.
- Final actual Ros2Backend test at 12:06:55Z–12:06:59Z (20:06 Beijing): unique source edge_q1_appcheck_dc725337a689, priority30, timeout250ms; ±0.2m/s for <=0.6s per direction; continuous full feedback, odometry/tilt/displacement guard and owner check. First motion acquired source, both directions completed, zero+DELETE confirmed, fault=null. Heading projection forward +1.45cm, reverse -1.25cm; XY changes 2.14cm and2.52cm. Precise step count not controlled. Evidence logs/bounded-app-backend-20261104.jsonl.
- Built image2fc48ceb3669 and restored embodied-q1 with rotating INFO logs. Fresh live state: ros2, idle, enabled=false, connected=true, fault=null, source_active=false, ten joints, feedback~23ms. Cached owner still names deleted appcheck source; actual registration false at startup. No further autonomous movement.
