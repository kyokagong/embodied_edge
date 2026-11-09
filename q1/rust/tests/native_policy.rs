#![cfg(feature = "supermimic")]
use ndarray::{Array2, Array3};
use ndarray_npy::NpzReader;
use q1_edge::{
    lowlevel::types::Sensors,
    policy::{
        model::Model,
        observation::{command_goal, Observation},
        package::Package,
        reference,
    },
    supermimic::contract::Hardware,
};
use serde_json::Value;
use std::path::PathBuf;

fn root() -> PathBuf {
    if let Some(p) = std::env::var_os("BLINKMIMIC_PACKAGE") {
        return p.into();
    }
    let artifacts = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../artifacts/policy/models");
    let mut prepared = std::fs::read_dir(artifacts)
        .into_iter()
        .flatten()
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| p.join("model_package.json").is_file())
        .collect::<Vec<_>>();
    prepared.sort();
    if let Some(prepared) = prepared.pop() {
        return prepared;
    }
    let supplied = PathBuf::from("/home/kyokagong/models/blinkmimic/q1_blinkmimic_48000_20261109");
    if supplied.join("deployment_contract.json").is_file() {
        return supplied;
    }
    PathBuf::from("/tmp/q1-blink-native-package")
}
fn package() -> Package {
    Package::load(&root()).expect("real delivered BlinkMimic package")
}
fn close(a: &[f32], b: &[f32], atol: f32, rtol: f32) {
    assert_eq!(a.len(), b.len());
    for (i, (&x, &y)) in a.iter().zip(b).enumerate() {
        assert!(
            x.is_finite() && y.is_finite() && (x - y).abs() <= atol + rtol * y.abs(),
            "index {i}: {x} != {y}"
        );
    }
}
fn hardware(p: &Package) -> Hardware {
    Hardware::load(&p.root.join("provenance/hardware/deployed_hardware.json")).unwrap()
}

#[test]
fn delivered_package_and_actual_native_model_pass_all_golden_probes() {
    let p = package();
    assert_eq!(p.iteration, 48000);
    p.validate_hardware(&hardware(&p)).unwrap();
    let mut m = Model::load(&p, 2).unwrap();
    let report = m.golden(&p).unwrap();
    assert_eq!(report["samples"], 55);
    assert_eq!(report["hardware_command_sent"], false);
    assert!(m.infer(&[0.; 779], &[0.; 594]).is_err());
    let mut bad = [0.; 780];
    bad[0] = f32::NAN;
    assert!(m.infer(&bad, &[0.; 594]).is_err());
}

#[test]
fn rebuilt_motion_and_command_goals_match_all_supplied_vectors() {
    let p = package();
    let refs = reference::load(&p).unwrap();
    assert_eq!(refs.len(), 13);
    let cases: Vec<Value> =
        serde_json::from_slice(&std::fs::read(p.root.join("validation/cases.json")).unwrap())
            .unwrap();
    let mut npz =
        NpzReader::new(std::fs::File::open(p.root.join("validation/golden.npz")).unwrap()).unwrap();
    let goals: Array2<f32> = npz.by_name("goal").unwrap();
    for (i, case) in cases.iter().enumerate() {
        let actual = if case["mode"] == "velocity" {
            let command: [f32; 3] = serde_json::from_value(case["command"].clone()).unwrap();
            command_goal(&p.home, command).unwrap()
        } else {
            let clip = &refs[case["name"].as_str().unwrap()];
            clip.goal(case["frame"].as_u64().unwrap() as usize, clip.len() - 1)
                .unwrap()
        };
        close(&actual, goals.row(i).as_slice().unwrap(), 2e-5, 2e-5);
    }
}

#[test]
fn reference_future_starts_at_next_frame_and_clamps_before_return_phase() {
    let p = package();
    let refs = reference::load(&p).unwrap();
    let c = &refs["floor_fold"];
    let end = 600; // Authoritative 12 second hold_end at 50 Hz.
    let g = c.goal(end - 1, end).unwrap();
    close(&g[15..37], c.q.row(end).as_slice().unwrap(), 0., 0.);
    for k in 1..10 {
        close(&g[k * 59..(k + 1) * 59], &g[..59], 0., 0.);
    }
    let held = c.goal(end, end).unwrap();
    assert_eq!(&held[..3], &[0., 0., 0.]);
    assert!(c.goal(end + 1, end).is_err());
    assert!(c.goal(0, c.len()).is_err());
    assert!(command_goal(&p.home, [0.61, 0., 0.]).is_err());
    assert!(command_goal(&p.home, [0., f32::NAN, 0.]).is_err());
}

#[test]
fn observation_preserves_published_raw_history_and_native_zero_bias_sign() {
    let mut p = package();
    p.offset[0] = 0.1;
    let mut h = hardware(&p);
    h.encoder_bias[0] = -0.1;
    let mut s = Sensors {
        q: p.home,
        qd: [2.; 22],
        omega: [1., 2., 3.],
        quaternion_xyzw: [0., 0., 0., 1.],
        complete: true,
        imu_valid: true,
        imu_frame: h.imu_frame.clone(),
        ..Default::default()
    };
    s.q[0] += 0.3;
    let mut o = Observation::default();
    let a = o.build(&s, &h, &p, [0.25; 22], None).unwrap();
    for frame in a.chunks_exact(78) {
        assert!((frame[0] - 0.2).abs() < 1e-6);
        assert_eq!(&frame[22..44], &[2.; 22]);
        assert_eq!(&frame[44..47], &[1., 2., 3.]);
        assert_eq!(&frame[47..50], &[0., 0., -1.]);
        assert_eq!(&frame[50..72], &[0.25; 22]);
        assert_eq!(&frame[72..78], &[1., 0., 0., 1., 0., 0.]);
    }
    let ref_r = [[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]];
    let b = o.build(&s, &h, &p, [-0.5; 22], Some(ref_r)).unwrap();
    assert_eq!(&b[..702], &a[78..]);
    assert_eq!(&b[752..774], &[-0.5; 22]);
    assert_eq!(&b[774..], &[1., 0., 0., 0., 0., 1.]);
    s.complete = false;
    assert!(o.build(&s, &h, &p, [0.; 22], None).is_err());
    s.complete = true;
    s.q[0] = f32::NAN;
    assert!(o.build(&s, &h, &p, [0.; 22], None).is_err());
    s.q[0] = p.home[0];
    o.reset();
    let reset = o.build(&s, &h, &p, [0.; 22], None).unwrap();
    assert_eq!(&reset[..78], &reset[702..]);
}

#[test]
fn actual_reference_observations_rebuild_motion_golden_history() {
    let p = package();
    let refs = reference::load(&p).unwrap();
    let h = hardware(&p);
    let cases: Vec<Value> =
        serde_json::from_slice(&std::fs::read(p.root.join("validation/cases.json")).unwrap())
            .unwrap();
    let mut npz =
        NpzReader::new(std::fs::File::open(p.root.join("validation/golden.npz")).unwrap()).unwrap();
    let history: Array3<f32> = npz.by_name("obs_history").unwrap();
    for (i, case) in cases
        .iter()
        .enumerate()
        .filter(|(_, c)| c["mode"] == "motion")
    {
        let c = &refs[case["name"].as_str().unwrap()];
        let f = case["frame"].as_u64().unwrap() as usize;
        let mut observations = Observation::default();
        let mut actual = [0.; 780];
        for k in 0..10 {
            let u = f.saturating_sub(9 - k);
            let r = c.rotation(u);
            // Matrix-to-quaternion conversion is independent of the observation builder.
            let s = Sensors {
                q: std::array::from_fn(|j| c.q[[u, j]]),
                qd: std::array::from_fn(|j| c.qd[[u, j]]),
                omega: std::array::from_fn(|j| c.root_w_local[[u, j]]),
                quaternion_xyzw: matrix_quat(r),
                complete: true,
                imu_valid: true,
                imu_frame: h.imu_frame.clone(),
                ..Default::default()
            };
            let previous = std::array::from_fn(|j| history[[i, k, 50 + j]]);
            actual = observations.build(&s, &h, &p, previous, Some(r)).unwrap();
        }
        close(
            &actual,
            history.index_axis(ndarray::Axis(0), i).as_slice().unwrap(),
            2e-5,
            2e-5,
        );
    }
}
fn matrix_quat(m: [[f32; 3]; 3]) -> [f32; 4] {
    let trace = m[0][0] + m[1][1] + m[2][2];
    if trace > 0. {
        let s = (trace + 1.).sqrt() * 2.;
        return [
            (m[2][1] - m[1][2]) / s,
            (m[0][2] - m[2][0]) / s,
            (m[1][0] - m[0][1]) / s,
            s / 4.,
        ];
    }
    let i = (0..3).max_by(|&a, &b| m[a][a].total_cmp(&m[b][b])).unwrap();
    let j = (i + 1) % 3;
    let k = (i + 2) % 3;
    let s = (1. + m[i][i] - m[j][j] - m[k][k]).sqrt() * 2.;
    let mut q = [0.; 4];
    q[i] = s / 4.;
    q[j] = (m[j][i] + m[i][j]) / s;
    q[k] = (m[k][i] + m[i][k]) / s;
    q[3] = (m[k][j] - m[j][k]) / s;
    q
}

struct Fixture(PathBuf);
impl Fixture {
    fn new() -> Self {
        use std::sync::atomic::{AtomicU64, Ordering};
        static NEXT: AtomicU64 = AtomicU64::new(0);
        let dir = std::env::temp_dir().join(format!(
            "q1-native-{}-{}",
            std::process::id(),
            NEXT.fetch_add(1, Ordering::Relaxed)
        ));
        std::fs::create_dir_all(&dir).unwrap();
        let sums = std::fs::read_to_string(root().join("SHA256SUMS")).unwrap();
        for line in sums.lines() {
            let (_, name) = line.split_once("  ").unwrap();
            let target = dir.join(name);
            std::fs::create_dir_all(target.parent().unwrap()).unwrap();
            if name == "deployment_contract.json" {
                std::fs::copy(root().join(name), target).unwrap();
            } else {
                std::fs::hard_link(root().join(name), &target)
                    .or_else(|_| std::fs::copy(root().join(name), &target).map(|_| ()))
                    .unwrap();
            }
        }
        std::fs::write(dir.join("SHA256SUMS"), sums).unwrap();
        Self(dir)
    }
    fn contract(&self, change: impl FnOnce(&mut Value)) {
        self.rewrite_json("deployment_contract.json", change);
    }
    fn rewrite_json(&self, name: &str, change: impl FnOnce(&mut Value)) {
        use sha2::{Digest, Sha256};
        let path = self.0.join(name);
        let mut c: Value = serde_json::from_slice(&std::fs::read(&path).unwrap()).unwrap();
        change(&mut c);
        let bytes = serde_json::to_vec_pretty(&c).unwrap();
        let staging = path.with_extension("fixture-next");
        std::fs::write(&staging, &bytes).unwrap();
        std::fs::rename(staging, &path).unwrap();
        let hash = format!("{:x}", Sha256::digest(&bytes));
        let sums = std::fs::read_to_string(self.0.join("SHA256SUMS"))
            .unwrap()
            .lines()
            .map(|line| {
                let (_, entry) = line.split_once("  ").unwrap();
                if entry == name {
                    format!("{hash}  {entry}")
                } else {
                    line.to_owned()
                }
            })
            .collect::<Vec<_>>()
            .join("\n")
            + "\n";
        std::fs::write(self.0.join("SHA256SUMS"), sums).unwrap();
    }
}
impl Drop for Fixture {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

#[test]
fn compatible_model_update_is_not_pinned_to_training_iteration_or_checkpoint() {
    let f = Fixture::new();
    f.contract(|c| {
        c["iteration"] = 49000.into();
        c["checkpoint_sha256"] =
            "1234567890abcdef1234567890abcdef1234567890abcdef1234567890abcdef".into();
    });
    assert_eq!(Package::load(&f.0).unwrap().iteration, 49000);
}

#[test]
fn matching_shapes_cannot_hide_incompatible_input_or_execution_semantics() {
    for change in [
        "previous_action",
        "goal_tail",
        "tilt_guard",
        "external_normalization",
    ] {
        let f = Fixture::new();
        f.contract(|c| match change {
            "previous_action" => c["input"][change] = "latest predicted raw".into(),
            "goal_tail" => c["input"][change] = "[motion,vx,vy,wz]".into(),
            "tilt_guard" => c["controller"]["execution_profile"][change] = "absolute".into(),
            _ => c["normalization"][change] = true.into(),
        });
        assert!(
            Package::load(&f.0).is_err(),
            "accepted incompatible {change}"
        );
    }
}

#[test]
fn manifest_requires_runtime_hashes_and_rejects_escaping_or_duplicate_entries() {
    for mutation in ["omit", "mismatch", "escape", "duplicate"] {
        let f = Fixture::new();
        let path = f.0.join("SHA256SUMS");
        let old = std::fs::read_to_string(&path).unwrap();
        let sums = match mutation {
            "omit" => old
                .lines()
                .filter(|l| !l.ends_with("model/policy_native.onnx"))
                .collect::<Vec<_>>()
                .join("\n"),
            "mismatch" => old
                .lines()
                .map(|l| {
                    if l.ends_with("model/policy_native.onnx") {
                        format!("{}  model/policy_native.onnx", "0".repeat(64))
                    } else {
                        l.to_owned()
                    }
                })
                .collect::<Vec<_>>()
                .join("\n"),
            "escape" => old + &format!("{}  ../outside\n", "0".repeat(64)),
            _ => old.clone() + old.lines().next().unwrap() + "\n",
        };
        std::fs::write(path, sums).unwrap();
        assert!(Package::load(&f.0).is_err(), "accepted {mutation}");
    }
}

#[test]
fn old_hardware_ankle_damping_and_encoder_bias_are_refused() {
    let p = package();
    let mut h = hardware(&p);
    h.kd[4] = 0.8;
    assert!(p.validate_hardware(&h).is_err());
    h = hardware(&p);
    h.encoder_bias[0] = 0.1;
    assert!(p.validate_hardware(&h).is_err());
    h = hardware(&p);
    h.stand_trial.as_mut().unwrap().effort_limit[14] = 21.;
    assert!(p.validate_hardware(&h).is_err());
}

#[test]
fn actual_graph_dtype_is_verified_independently_of_contract_claims() {
    use sha2::{Digest, Sha256};
    let fixture = Fixture::new();
    let mut p = Package::load(&fixture.0).unwrap();
    let bytes: &[u8] = &[
        8, 10, 58, 202, 2, 10, 227, 1, 18, 7, 97, 99, 116, 105, 111, 110, 115, 34, 8, 67, 111, 110,
        115, 116, 97, 110, 116, 42, 205, 1, 10, 5, 118, 97, 108, 117, 101, 42, 192, 1, 8, 1, 8, 22,
        16, 11, 66, 5, 118, 97, 108, 117, 101, 82, 176, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 160, 1, 4, 18, 11, 119, 114, 111, 110, 103, 95, 100,
        116, 121, 112, 101, 90, 33, 10, 11, 111, 98, 115, 95, 104, 105, 115, 116, 111, 114, 121,
        18, 18, 10, 16, 8, 11, 18, 12, 10, 2, 8, 1, 10, 2, 8, 10, 10, 2, 8, 78, 90, 23, 10, 4, 103,
        111, 97, 108, 18, 15, 10, 13, 8, 11, 18, 9, 10, 2, 8, 1, 10, 3, 8, 210, 4, 98, 25, 10, 7,
        97, 99, 116, 105, 111, 110, 115, 18, 14, 10, 12, 8, 11, 18, 8, 10, 2, 8, 1, 10, 2, 8, 22,
        66, 4, 10, 0, 16, 18,
    ];
    p.model_path = fixture.0.join("wrong_dtype.onnx");
    std::fs::write(&p.model_path, bytes).unwrap();
    p.model_sha256 = format!("{:x}", Sha256::digest(bytes));
    assert!(Model::load(&p, 2).err().unwrap().contains("ABI"));
}

#[test]
fn reference_manifest_cannot_omit_task_return_or_change_timebase_paths_and_identity() {
    for change in [
        "return",
        "phase_boundary",
        "fps",
        "path",
        "hash",
        "joint_order",
    ] {
        let f = Fixture::new();
        f.rewrite_json("references/manifest.json", |m| match change {
            "return" => {
                m["clips"]["dance1_subject1"]["phases"]
                    .as_object_mut()
                    .unwrap()
                    .remove("return");
            }
            "phase_boundary" => m["clips"]["floor_fold"]["phases"]["hold_end"] = 50.into(),
            "fps" => m["clips"]["floor_fold"]["npz_fps"] = 30.into(),
            "path" => m["clips"]["floor_fold"]["file"] = "../model/policy_native.onnx".into(),
            "hash" => m["clips"]["floor_fold"]["npz_sha256"] = "0".repeat(64).into(),
            _ => m["joint_names"][0] = "wrong_joint".into(),
        });
        assert!(
            Package::load(&f.0).is_err(),
            "accepted bad reference {change}"
        );
    }
}

#[test]
fn supported_standing_uses_real_squat_return_reference_then_zero_command() {
    // Offline reference-state probe; this does not stand in for a hardware test.
    let p = package();
    let refs = reference::load(&p).unwrap();
    let h = hardware(&p);
    let clip = &refs["floor_fold"];
    let frame = 1200; // 24s, within the supplied feet-supported squat return (22..28s).
    let mut sensors = Sensors {
        q: std::array::from_fn(|i| clip.q[[frame, i]]),
        quaternion_xyzw: matrix_quat(clip.rotation(frame)),
        complete: true,
        imu_valid: true,
        ..Default::default()
    };
    let mut task = q1_edge::policy::task::TaskState::new();
    task.request_for_control(false, "start_policy", None, &refs, &sensors, &h)
        .unwrap();
    assert_eq!(task.mode, "stand_up");
    assert_eq!(task.clip.as_deref(), Some("floor_fold"));
    assert_eq!(task.frame, frame);
    assert!(task.stand_entry.as_ref().unwrap().joint_max_rad < 1e-6);
    let goal = clip.goal(task.frame, task.end).unwrap();
    close(
        &goal[15..37],
        clip.q.row(frame + 1).as_slice().unwrap(),
        0.,
        0.,
    );
    assert_eq!(&goal[590..], &[0.; 4]);
    task.frame = task.end;
    sensors.q = std::array::from_fn(|i| clip.q[[task.end, i]]);
    sensors.quaternion_xyzw = matrix_quat(clip.rotation(task.end));
    task.advance(&refs, &sensors, &h).unwrap();
    assert_eq!(task.mode, "stand");
    assert!(task.clip.is_none());
    assert_eq!(command_goal(&p.home, [0.; 3]).unwrap()[590], 1.);
}

#[test]
fn supported_standing_allows_modest_entry_variation_but_keeps_a_bounded_gate() {
    let p = package();
    let refs = reference::load(&p).unwrap();
    let h = hardware(&p);
    let clip = &refs["floor_fold"];
    let frame = 1200;
    for (deviation, allowed) in [(0.556, true), (0.599, true), (0.601, false)] {
        let mut sensors = Sensors {
            q: std::array::from_fn(|i| clip.q[[frame, i]]),
            quaternion_xyzw: matrix_quat(clip.rotation(frame)),
            complete: true,
            imu_valid: true,
            ..Default::default()
        };
        // Hip roll is constant across the eligible segment, so phase search cannot hide this error.
        sensors.q[7] += deviation;
        let mut task = q1_edge::policy::task::TaskState::new();
        let result = task.request_for_control(false, "stand", None, &refs, &sensors, &h);
        assert_eq!(result.is_ok(), allowed, "deviation {deviation}: {result:?}");
        if allowed {
            assert_eq!(task.mode, "stand_up");
            assert_eq!(task.clip.as_deref(), Some("floor_fold"));
            assert!((task.stand_entry.as_ref().unwrap().joint_max_rad - deviation).abs() < 1e-5);
            let goal = clip.goal(task.frame, task.end).unwrap();
            close(
                &goal[15..37],
                clip.q.row(task.frame + 1).as_slice().unwrap(),
                0.,
                0.,
            );
        } else {
            assert!(result.unwrap_err().contains("right_hip_roll_joint"));
            assert!(task.clip.is_none());
        }
    }
    // An increased entry tolerance must not mutate the hardware's trained execution wrapper.
    p.validate_hardware(&h).unwrap();
}

#[test]
fn supported_standing_entry_still_requires_gravity_match_and_stationary_feedback() {
    let p = package();
    let refs = reference::load(&p).unwrap();
    let h = hardware(&p);
    let clip = &refs["floor_fold"];
    let frame = 1400;
    let base = Sensors {
        q: std::array::from_fn(|i| clip.q[[frame, i]]),
        quaternion_xyzw: matrix_quat(clip.rotation(frame)),
        complete: true,
        imu_valid: true,
        ..Default::default()
    };
    for bad in ["tilt", "joint_speed", "angular_speed", "imu"] {
        let mut sensors = base.clone();
        match bad {
            "tilt" => sensors.quaternion_xyzw = [(0.25_f32).sin(), 0., 0., (0.25_f32).cos()],
            "joint_speed" => sensors.qd[0] = 0.301,
            "angular_speed" => sensors.omega[0] = 0.251,
            _ => sensors.imu_valid = false,
        }
        let mut task = q1_edge::policy::task::TaskState::new();
        assert!(
            task.request_for_control(false, "stand", None, &refs, &sensors, &h)
                .is_err(),
            "accepted {bad}"
        );
    }
}

#[test]
fn package_must_declare_supported_stand_entry_segment() {
    let f = Fixture::new();
    f.rewrite_json("references/manifest.json", |m| {
        m["clips"]["floor_fold"]["phases"]
            .as_object_mut()
            .unwrap()
            .remove("return_squat_start");
    });
    assert!(Package::load(&f.0).is_err());
}
