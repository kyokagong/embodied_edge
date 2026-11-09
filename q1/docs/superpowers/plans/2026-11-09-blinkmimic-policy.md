# BlinkMimic Policy Implementation Plan

**Goal:** Prepare a native q1-policy ARM64 deployment and independent model update workflow.
**Architecture:** Native model adapter, pure reference task state machine, q1-policy actor, stable q1-rt hardware bridge, versioned model updater.
**Spec:** ../specs/2026-11-09-blinkmimic-policy-design.md
**Constraints:** no commits; no robot access; preserve prior unrelated edits; current checkout; actual shipped 48k vectors; ROS/HAL ABI unchanged.

## Review Focus

- Missing/future/stale feedback cannot authorize motion or reset timeouts.
- Sitting/lying endpoints clamp all future features and cannot silently select upright standing.
- Published raw history follows successful HAL output, including reuse and initial takeover.
- New model failure leaves active service untouched or restores it; unchanged RT is verifiable.
- Same shape with different protocol/execution/hardware semantics is refused.

## Tasks

- [x] 1. Native package/observation/reference/model modules. Tests first: delivered package, golden vectors, mismatched protocol, future-frame/endpoint and published raw layout. Module APIs specified in agent task.
- [x] 2. Pure task transitions and new action UI. Tests first: sit hold/return, lie hold/getup admission, dance cancel/return, duplicate/no reset, epoch boundaries.
- [x] 3. Controller rename/integration and RT extension. Tests first: protocol capabilities, matching reference-relative gravity, successfully-published raw. Add q1-policy --check-only with no ROS/IPC writes.
- [x] 4. Offline packaging/build/deployment/model-update tools. Tests first: malformed paths/digests, incompatible runtime, staging and rollback, release requirements. Build ARM64 bundle with no automatic robot execution.
- [x] 5. Integration verification, independent review, corrections and deployment runbook. Record commands/results and limitations.

## Execution ledger

Baseline: supplied 86 checksums valid; both models 55 golden probes pass ORT1.23.2; package 5 tests pass; existing Rust lib suite 20 tests pass.
Ruling: preserve historical q1-stand source and scripts as explicit legacy tools; active binary/scripts/UI are q1-policy. This avoids silently changing old model tools while clearly separating the new deployment.
Ruling: model update means complete versioned runtime model package, not an unverified raw file; matching golden/contract/checksums travel with weights.

Final review rulings: preserve frozen phase endpoint semantics (dance marks start of return, return marks its endpoint). Getup terminal transition additionally requires measured quiet upright posture; failure enters existing protection hold. Initial ownership must complete within250ms and with a live operator lease. Health must reject faults/blockers and require advancing damping publications. Cold boot is a separate explicit mode and may not kill unknown processes.

Completion evidence: native Rust suite48 tests (34lib+3actor+11integration) PASS; actual55 native golden probes max error2.1457672119140625e-6; Python132/132 PASS; UI syntax and real no-RT HTTP refusal checks PASS; ARM64 optimized build PASS and63 ELF dependency objects resolved. Runtime/model manifests228/42 files verified. Cold deployment dry-run only, no SSH/sudo/robot execution.
Independent review: three actionable findings fixed with RED→GREEN regression tests; no unresolved critical/important findings. Runtime/profile retained on model updates; experimental capability qualification unchanged.

## Supported standing clarification

- [x] Correct task interpretation to disabled, hoist-supported squat with feet on ground; exclude prone recovery from public entry.
- [x] Add pose-matched floor_fold squat-return standing entry with no extra slow home interpolation.
- [x] Unit regressions for entry/duplicates/refusal/end condition and native reference snapshot integration.
- [x] Refresh ARM64 runtime, run full Rust suite, independently review, update preparation report.

Supported-standing correction verified:54 Rust tests pass, including actual supplied reference snapshot entry and endpoint checks; ARM64 build and63-object closure pass; JS syntax pass; independent follow-up review found no important issue. No robot connection, model bytes unchanged, no commits.
