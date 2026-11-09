# Q1 BlinkMimic policy deployment

User authorized direct implementation on 2026-11-09; implementation decisions delegated. No git commits, no robot connection or motion in this phase. Work in the current checkout because the existing implementation is uncommitted here.

## Goals and architecture

Use the delivered native FP32 ONNX: obs_history[1,10,78], goal[1,594], actions[1,22]. Rename the active controller q1-policy. Keep existing ROS topics, feedback validation, HAL fixed slots, PD and effort/speed limits. Extend the local RT protocol once to carry the current aligned reference rotation and report the raw action selected at a successful HAL publish. A versioned handshake rejects old RT instances. Future same-protocol models are versioned data packages, independently checked and switched without recompiling ROS or controller binaries. No imitation of training delay/reuse on hardware.

Model packages include ONNX, deployment contract, references, golden vectors and checksums. Updating a bare ONNX without new checksums and independently generated golden outputs is not supported; the user operation is one model-package update. Never pin checkpoint identity or training iteration in executable code. Require the same validated ABI, observation meanings, execution profile and hardware parameters.

## Actions

Standing acquisition is a pose-matched supported squat-to-stand reference task; its subsequent hold is zero-command policy. Sitting (floor_fold), lying and dances use the same model/output path. Public prone recovery is unavailable. Remove the old 3+20+3 second home preparation. A short bounded measured-pose handshake establishes ownership, then policy targets use the same slew and RT constraints for all tasks. No teleport, physical reset, fabricated sensors, history reset on normal mode switches, or external reference-skip feedforward.

Freeze reference yaw alignment on each action start; the actor and RT use the same aligned current Rref. Compute future f+1..f+10 features, clamp at the active phase boundary. Sit holds at floor_fold hold_end until a return request restores the return segment. Lie holds its endpoint; returning from lie uses getup, never instantly selects upright command. Dance cancellation follows the supplied return phase. Preserve history, isolate predictions with epochs, ignore duplicate requests and stale predictions. Explicit hold/release remain independent of task cancellation.

The shipped 48k snapshot is experimental (getup 0/60). Expose qualification honestly. Getup admission checks measured orientation/joints/velocities against the prone reference and stable samples. Root height and valid root linear speed are unavailable on this deployment: never claim the complete simulator getup-ready predicate. The retained internal prone-recovery code is not available through the public controller; prohibit automatic getup chains on missing measurements. Model completion alone is not physical standing verification.

## Deployment

Prepare ARM64 runtime bundle and separate immutable model bundle locally. Preflight does not create ROS or IPC writers. It validates hashes, actual ONNX ABI, all supplied golden outputs, runtime hardware compatibility, references and task metadata. First deployment replaces only this task's old processes after checking hardware identity and released state. Future updates retain RT, preflight the new model before stopping policy, change the active model reference atomically and restore the old policy/model on failure. Restart disabled; motion requires a new explicit UI action. Report model hashes and runtime identities.

## Alternatives

Native dual input is selected by the user. The legacy1376 wrapper preserves tensor layout but cannot fix published-action history, reference tilt or task control and offers little deployment simplification. Replacing the ROS driver stack is unnecessary and outside scope.

## Verification

TDD for task transitions, protocol checks, input history and publication semantics, update failures and rollback. Real delivered package golden checks via Rust ONNX, full Rust/Python regression suites, JS checks, ARM64 ELF dependency closure. No simulated robot feedback is used to claim hardware acceptance. Robot-only timing, ownership, mapping identity, IMU mounting and physical motion remain the next phase.

## User clarification: supported squat-to-stand

The requested standing action starts disabled in a hoist-supported squat with both feet already on the ground. It is distinct from prone recovery. Cold stand/start_policy must match a current measured joint/IMU pose in floor_fold return_squat_start..return_end, align the reference yaw, then run the same native motion pipeline, ending with measured quiet upright admission to zero-command hold. Existing tracking/tilt limits bound entry mismatch. No20-second home interpolation, no physical reset, and no official_getUp fallback. Feet/hoist conditions are the user-confirmed setup, not sensor claims. Public prone getup is unavailable in this revision; sit/lie/dance require initial standing control first.
