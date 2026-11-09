# Q1 Edge interfaces

Project: /data/kk/embodied_edge/q1. Python3.10+, numpy, optional ROS2 Humble and XRoboToolkit. No git commits.

## Backend (backends.py; SDK agent)
`MockBackend()` / `Ros2Backend(config:dict)` expose:
- `enable()` -> None: explicit source registration/permission intent; no positive velocity.
- `request_mode(mode: str)` -> None: stand/walk/teleop; nonblocking.
- `set_velocity(forward:float,lateral:float,angular:float)` -> None: target update, finite values.
- `set_upper(positions:dict[str,float])` -> None: exact UPPER_JOINT_NAMES set; finite, all10.
- `stop()` -> None: zero velocity, cancel pending unsafe transitions, maintain native balance, request stand if safe; no HAL.
- `close()` -> None: bounded best-effort zero and safe source release; cannot prove physical stop. Away from HOME or after takeover, release may remain unconfirmed.
- `snapshot()` -> dict: {connected:bool,action:str,action_running:bool,joints:dict[str,float],joints_age_s:float|None,source_active:bool,fault:str|None,velocity:{forward,lateral,angular}}. Mock returns fresh joints, modes immediate and connected; Ros2 starts a background rclpy executor, no rclpy import at module import; bounded async transitions and continuous feedback. Ros2 MUST independently watchdog stale set_velocity/set_upper and source loss. Use monotonic time for ages. SDK bridge config lives under config['ros2']; input source default edge_q1 unique, priority30 (<native rc80).
- `UPPER_JOINT_NAMES` tuple: waist_roll_joint,waist_yaw_joint,left_shoulder_pitch_joint,left_shoulder_roll_joint,left_shoulder_yaw_joint,left_elbow_joint,right_shoulder_pitch_joint,right_shoulder_roll_joint,right_shoulder_yaw_joint,right_elbow_joint. `UPPER_HOME` dict from official SDK upper_body_control example, not training home. These constants owned in backends.py; pico may copy internal names to avoid import dependence before file arrives.

## Pico (pico.py; Pico agent)
Frame: {timestamp_ns:int,head:[x,y,z,qx,qy,qz,qw],left:[7],right:[7],left_grip:float,right_grip:float,valid:bool}. Must validate finite/unit-quaternion, source timestamp progression; parent controller handles receipt freshness and mode gating.
`PicoRetargeter(asset_path: str|Path|None=None, limits:dict|None=None)`
- `calibrate(frame:dict,joints:dict[str,float])` -> None: measured joints (all10), fixed neutral VR→robot frame calibration.
- `retarget(frame:dict,dt:float=0.01)` -> dict[str,float]: exact all10; bounded relative motion,4DOF arm IK, no exact6DOF orientation promise; signal unreachable/tracking jumps with ValueError (no dangerous zero targets).
- `status` dict property: {calibrated:bool,error_m:float, ...} JSON-safe.
`XRoboSource()` with `poll()->dict|None` returns only advancing native frames (not stale cached time); `close()`. Actual SDK lazy import. Optional JSON normalize helper if useful. Asset assets/q1_upper.xml contains the pinned official kinematic tree; provenance JSON and Apache license are included.

## Core/API (core.py,server.py,__main__.py; parent)
`Controller(backend,config,retargeter=None)`, `.input(payload,now=None,compact=False)`, `.pico_frame(frame,source='simulator',now=None)`, `.action(name)`, `.tick(now=None)`, `.state()` JSON dict. Controller lock for all mutations.
GET /api/state => state; POST /api/action {client_id:str (required except stop/disable),action:'enable'|'disable'|'calibrate'|'stand'|'walk'|'teleop'|'stop'|'clear_fault'}; POST /api/input {client_id:str,source:'simulator'|'browser_gamepad'|'linux_gamepad',sequence:int,axes:[4 floats -1..1],buttons:[bool up to17],connected:bool}; POST /api/pico same frame plus source:'simulator'|'xrobotoolkit' (HTTP endpoint simulator only; native source local process). Responses {ok:bool,state:object,error?:str}; GET /api/state directly state.
State fields: backend:'mock'|'ros2',enabled,calibrated,mode:'idle'|'stand'|'walk'|'teleop'|'fault',status,robot:{...backend.snapshot()},input:{source,age_ms,deadman,client_id},pico:{connected,calibrated,age_ms,source,frame,ik_error_m},command:{velocity:{forward,lateral,angular},upper:dict},events:[{time,event}],supermimic:{available:false,reason:str}. Null ages instead of Infinity.
Input default standard gamepad axes0 leftX,1 leftY,2 rightX,3 rightY; buttons0 A stand,1 B stop,2 X walk,3 Y teleop,5 RB long-press toggle for physical sources (legacy held deadman for simulator),8 Back calibration,9 Start enable. Startup init button edges MUST NOT enable from held buttons. Calibration requires neutral axes and NOT held deadman, then explicit enable; UI can calibrate first. Stale input250ms disables/latches; Pico active stale250ms disables/latches; only UI show capture examples if no device. Two grips>=.7 required for Pico motion plus explicit enable; Pico standalone may supply control heartbeat separate browser gamepad. Mock simulator pane must heartbeat inputs only when selected.

## UI (web/index.html,web/app.js,web/style.css; UI agent)
Use exact API above. Source toggles simulator / browser_gamepad; conspicuous MOCK and SIMULATED labels; client_id random per tab; sequential input sequence, 20–50Hz heartbeat; connected false when browser pad absent; stop on blur/visibility/gamepaddisconnected. Buttons disabled logically; enable/calibrate/stand/walk/teleop/stop; simulator held-deadman; Pico simulator controls head yaw/roll and hand XYZ offsets with grips, track source timestamps; default head identity [0,1.6,0,0,0,0,1], left[-.25,1.2,-.35,0,0,0,1],right[.25,1.2,-.35,0,0,0,1]. Reset sliders neutral before calibration. Joint visual bars/SVG, events, prerequisites, SuperMimic pending; no external CDN/build frameworks. Backend controls 100Hz output; UI polls state5Hz. GET includes api_token? parent may include meta token; UI read meta[name=api-token] if present and send X-Q1-Token header. JSON only, same-origin. No browser camera requirements.

## Firmware acquisition behavior (verified 2026-11-04)

On the tested Q1 Basic-mode firmware, zero velocities do not establish current-source attribution. Enable registers an idle source; only explicit Controller walk+latched physical enable (or legacy held RB)+nonzero axes may initiate a bounded acquisition. The pre-ownership velocity components are capped at 0.2 and deadline is min(0.25s, command_timeout_s, feedback_timeout_s), measured from the first motion intent without resetting. First owner replies processed after this deadline are rejected. Timeout outputs zero and deletes only the program's registered source. Native WALK RUNNING and fresh full feedback are required before first enable.

GetCurrentInputSource may retain attribution after expiry/deletion. Native arbitration remains authoritative for a bounded first attempt at priority<80; established ownership loss still latches and does not auto-reacquire. Every fresh ADD uses configured source_name as a prefix plus a random epoch suffix so old self-attribution cannot confirm a new registration. Node name remains stable.

Ros2 snapshot adds source_name, source_owner, source_can_acquire and source_acquiring. Controller consumes source_can_acquire only with explicit walk+latched physical enable (or legacy held RB)+nonzero input; no automatic movement occurs on enable. Logs add source_acquisition_start plus actual epoch and observed owner.


### Physical gamepad latch and link diagnostics

`gamepad.rb_toggle_enabled=true`, `gamepad.rb_hold_s=1.0` (validated 1–3 seconds). Browser/Linux physical RB uses accepted server-monotonic frames to measure a continuous hold; one hold fires once. Separate release plus new hold toggles again. Calibration/neutral axes remain required; enable starts stand with zero command, X selects walk. Right-stick horizontal axis2 maps to negative angular velocity (left stick deflection turns left), using the same native MC locomotion topic, never direct waist control. Start/UI enable is an explicit alternative and also latches. Simulator and standalone Pico retain prior controls. Disable keeps calibration but requires a fresh neutral release before enabling again. Stop/B clears calibration and the latch. Disconnect, source switch, stale input, backend fault, browser blur/hidden all stop; no automatic recovery.

State adds `controls:{rb_toggle_enabled,rb_hold_s}`, `input:{motion_latched,rb_hold_ms,link:{last_gap_ms,max_gap_ms,client,last_stop}}`. `deadman` represents effective motion permission; raw RB remains in input buttons. Optional input `telemetry` only accepts finite nonnegative numeric rtt_ms, max_rtt_ms, timeouts, errors, send_gap_ms, skipped, solely for diagnostics. `last_stop` captures first fault's input age, sequence, maximum gap, component feedback ages and last telemetry.

Authenticated `POST /api/input` with `X-Q1-Compact: 1` returns `{ok:true,ack:{sequence:int}}`, after the same validation/arbitration/safety processing. Legacy full-state input response stays available. HTTP/1.1 responses include length; error responses close connections, idle connections time out after 3s. Browser keeps at most one input request; on slow completion resamples current controls rather than replaying payloads. Completion and state polling are fenced by stop epoch; state polls at 5Hz. RPC/control workers log scheduling gaps >50ms, rate-limited to 1Hz; safety deadlines are unchanged.
