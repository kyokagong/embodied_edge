//! Immutable data package and runtime protocol validation. Model identity is data.
use crate::supermimic::contract::{hash, Hardware, NAMES};
use serde_json::{json, Value};
use std::{
    collections::BTreeMap,
    path::{Component, Path, PathBuf},
};

#[derive(Clone)]
pub struct Package {
    pub root: PathBuf,
    pub contract: Value,
    pub home: [f32; 22],
    pub offset: [f32; 22],
    pub iteration: u64,
    pub model_sha256: String,
    pub contract_sha256: String,
    pub model_path: PathBuf,
    hashes: BTreeMap<String, String>,
    hardware_config_sha256: String,
}

pub fn contained(root: &Path, name: &str) -> Result<PathBuf, String> {
    if name.is_empty()
        || name.contains('\\')
        || !Path::new(name)
            .components()
            .all(|c| matches!(c, Component::Normal(_)))
    {
        return Err(format!("Invalid package path: {name}"));
    }
    let p = root
        .join(name)
        .canonicalize()
        .map_err(|e| format!("{name}: {e}"))?;
    if !p.starts_with(root) || !p.is_file() {
        return Err(format!(
            "Package path escapes root or is not a file: {name}"
        ));
    }
    Ok(p)
}
pub fn read_json(path: &Path) -> Result<Value, String> {
    serde_json::from_slice(&std::fs::read(path).map_err(|e| e.to_string())?)
        .map_err(|e| e.to_string())
}
fn digest(s: &str) -> bool {
    s.len() == 64
        && s.bytes()
            .all(|b| b.is_ascii_hexdigit() && !b.is_ascii_uppercase())
}
fn equal(c: &Value, path: &str, want: Value) -> Result<(), String> {
    if c.pointer(path) != Some(&want) {
        return Err(format!("Unsupported deployment protocol field {path}"));
    }
    Ok(())
}
fn finite22(v: &Value, label: &str) -> Result<[f32; 22], String> {
    let a: [f32; 22] =
        serde_json::from_value(v.clone()).map_err(|e| format!("Invalid {label}: {e}"))?;
    if !a.iter().all(|v| v.is_finite()) {
        return Err(format!("Nonfinite {label}"));
    }
    Ok(a)
}

impl Package {
    pub fn load(root: &Path) -> Result<Self, String> {
        let root = root.canonicalize().map_err(|e| e.to_string())?;
        let sums =
            std::fs::read_to_string(contained(&root, "SHA256SUMS")?).map_err(|e| e.to_string())?;
        let mut hashes: BTreeMap<String, String> = BTreeMap::new();
        for line in sums.lines() {
            let (expected, name) = line.split_once("  ").ok_or("Malformed SHA256SUMS entry")?;
            if !digest(expected) || hashes.insert(name.into(), expected.into()).is_some() {
                return Err("Invalid or duplicate SHA256SUMS digest/path".into());
            }
            let p = contained(&root, name)?;
            if hash(&p)? != expected {
                return Err(format!("Package digest mismatch: {name}"));
            }
        }
        for required in [
            "deployment_contract.json",
            "model/policy_native.onnx",
            "references/manifest.json",
            "validation/golden.npz",
            "validation/cases.json",
            "provenance/execution_profile.json",
            "provenance/hardware/deployed_hardware.json",
        ] {
            if !hashes.contains_key(required) {
                return Err(format!("Missing required checksum: {required}"));
            }
        }
        let contract = read_json(&contained(&root, "deployment_contract.json")?)?;
        validate_contract(&contract)?;
        let profile = read_json(&contained(&root, "provenance/execution_profile.json")?)?;
        if profile != contract["controller"]["execution_profile"]
            || profile != contract["controller"]["identity"]["execution_profile"]
        {
            return Err("Execution profile does not match controller identity".into());
        }
        let source_hardware = read_json(&contained(
            &root,
            "provenance/hardware/deployed_hardware.json",
        )?)?;
        if contract["controller"]["identity"]["profile_sha256"]
            != hashes["provenance/hardware/deployed_hardware.json"]
        {
            return Err("Hardware profile file identity differs from contract".into());
        }
        // This nested identity is the installed MotorModule YAML, not the enclosing profile JSON.
        let hardware_config_sha256 = source_hardware["stand_trial"]["hardware_config_sha256"]
            .as_str()
            .filter(|s| digest(s))
            .ok_or("Invalid installed hardware configuration identity")?
            .to_owned();
        let home = finite22(&contract["action"]["home"], "home")?;
        let offset = if let Some(v) = contract["action"].get("offset") {
            finite22(v, "offset")?
        } else {
            [0.; 22]
        };
        let iteration = contract["iteration"]
            .as_u64()
            .filter(|v| *v > 0)
            .ok_or("Invalid iteration")?;
        let model_sha256 = hashes["model/policy_native.onnx"].clone();
        let model_path = contained(&root, "model/policy_native.onnx")?;
        if contract["models"]["policy_native.onnx"]["sha256"] != model_sha256
            || contract["models"]["policy_native.onnx"]["bytes"].as_u64()
                != Some(
                    std::fs::metadata(&model_path)
                        .map_err(|e| e.to_string())?
                        .len(),
                )
        {
            return Err("Native model identity does not match deployment contract".into());
        }
        let contract_sha256 = hashes["deployment_contract.json"].clone();
        let package = Self {
            root,
            contract,
            home,
            offset,
            iteration,
            model_sha256,
            contract_sha256,
            model_path,
            hashes,
            hardware_config_sha256,
        };
        // Reference paths, metadata, shape and phase validity are part of package admission.
        super::reference::load(&package)?;
        Ok(package)
    }
    pub fn verified_file(&self, name: &str) -> Result<PathBuf, String> {
        let expected = self
            .hashes
            .get(name)
            .ok_or_else(|| format!("Missing required checksum: {name}"))?;
        let p = contained(&self.root, name)?;
        if hash(&p)? != *expected {
            return Err(format!("Package file changed: {name}"));
        }
        Ok(p)
    }
    pub fn validate_hardware(&self, h: &Hardware) -> Result<(), String> {
        h.validate()?;
        let t = h.stand_trial.as_ref().ok_or("Missing controller limits")?;
        let c = &self.contract["controller"];
        if h.firmware
            != c["identity"]["firmware"]
                .as_str()
                .ok_or("Missing firmware identity")?
            || t.hardware_config_sha256 != self.hardware_config_sha256
            || !t.soft_limits
            || h.signs != [1.; 22]
            || h.zeros != [0.; 22]
        {
            return Err("Hardware identity, native coordinates or soft limits differ".into());
        }
        for (i, j) in c["joints"]
            .as_array()
            .ok_or("Missing controller joints")?
            .iter()
            .enumerate()
        {
            for (actual, expected, name) in [
                (h.kp[i], j["kp"].as_f64(), "kp"),
                (h.kd[i], j["kd"].as_f64(), "kd"),
                (
                    h.lower[i],
                    j["command_position_range_rad"][0].as_f64(),
                    "lower",
                ),
                (
                    h.upper[i],
                    j["command_position_range_rad"][1].as_f64(),
                    "upper",
                ),
                (
                    t.effort_limit[i],
                    j["software_effort_limit_nm"].as_f64(),
                    "effort",
                ),
                (
                    t.policy_velocity_limit[i],
                    j["configured_policy_slew_rad_s"].as_f64(),
                    "policy velocity",
                ),
                (
                    t.measured_velocity_limit[i],
                    j["configured_measured_speed_soft_limit_rad_s"].as_f64(),
                    "measured velocity",
                ),
                (t.home[i], Some(self.home[i] as f64), "home"),
                (
                    h.encoder_bias[i],
                    Some(-self.offset[i] as f64),
                    "encoder offset",
                ),
            ] {
                if !actual.is_finite() || expected.is_none_or(|v| (actual as f64 - v).abs() > 1e-5)
                {
                    return Err(format!("Hardware joint {i} {name} differs from package"));
                }
            }
        }
        let w = &c["runtime_wrapper"];
        for (actual, expected, name) in [
            (
                h.state_timeout_s,
                w["state_timeout_s"].as_f64(),
                "state timeout",
            ),
            (
                h.target_timeout_s,
                w["target_timeout_s"].as_f64(),
                "target timeout",
            ),
            (h.sensor_skew_s, w["sensor_skew_s"].as_f64(), "sensor skew"),
            (
                h.max_step_rad as f64,
                w["policy_target_max_step_rad"].as_f64(),
                "step limit",
            ),
            (
                h.max_tilt_rad as f64,
                w["tilt_limit_rad"].as_f64(),
                "tilt limit",
            ),
            (
                h.max_tracking_error_rad as f64,
                w["tracking_error_correction_limit_rad"].as_f64(),
                "tracking limit",
            ),
            (
                h.max_temperature as f64,
                w["temperature_limit"].as_f64(),
                "temperature limit",
            ),
        ] {
            if !actual.is_finite() || expected.is_none_or(|v| (actual - v).abs() > 1e-5) {
                return Err(format!("Hardware {name} differs from package"));
            }
        }
        Ok(())
    }
}

fn validate_contract(c: &Value) -> Result<(), String> {
    let schema = c["schema"].as_str().ok_or("Missing deployment schema")?;
    if !schema.starts_with("q1-blinkmimic-") || !schema.ends_with("-deployment-v1") {
        return Err("Unsupported deployment schema".into());
    }
    if !c["checkpoint_sha256"].as_str().is_some_and(digest) {
        return Err("Invalid checkpoint identity".into());
    }
    for (path,want) in [
        ("/batch_size",json!(1)),("/dtype",json!("float32")),("/control_dt",json!(0.02)),("/policy_hz",json!(50)),("/training_physics_dt",json!(0.002)),
        ("/models/policy_native.onnx/inputs",json!([{"name":"obs_history","shape":[1,10,78],"dtype":"float32"},{"name":"goal","shape":[1,594],"dtype":"float32"}])),
        ("/models/policy_native.onnx/outputs",json!([{"name":"actions","shape":[1,22],"dtype":"float32"}])),
        ("/action/shape",json!([1,22])),("/action/scale",json!(0.25)),("/action/raw_clip",json!(20.0)),
        ("/action/kind",json!("raw position-offset action, not torque or absolute joint target")),
        ("/action/joint_names",json!(NAMES)),
        ("/action/ref_skip",json!("included in graph: network correction + (q_ref[f+1]-home)/0.25")),
        ("/action/base_target",json!("home + 0.25*clip(raw,-20,20) + offset; further controller constraints apply")),
        ("/action/hal",json!("factory direction/offset already resolved in URDF coordinates; do not apply twice")),
        ("/normalization/location",json!("inside both exported graphs")),("/normalization/external_normalization",json!(false)),
        ("/controller/execution_profile/schema",json!("q1-reference-tilt-v1")),("/controller/execution_profile/tilt_guard",json!("reference_relative")),
        ("/controller/identity/schema",json!("q1-body-aligned-controller-v1")),
        ("/controller/identity/joint_names",json!(NAMES)),
        ("/controller/delay",json!("measured-chain delay/reuse simulated in training; do not add it again to the physical chain")),
        ("/reference_subset/manifest",json!("references/manifest.json")),
    ] {equal(c,path,want)?;}
    let expected_input: Value = serde_json::from_str(INPUT_PROTOCOL).map_err(|e| e.to_string())?;
    for (key, value) in expected_input.as_object().unwrap() {
        equal(c, &format!("/input/{key}"), value.clone())?;
    }
    let wrapper = &c["controller"]["runtime_wrapper"];
    for (key, want) in [
        ("soft_limits", json!(true)),
        (
            "tilt_scale",
            json!("clamp(max_tilt_rad / max(tilt,max_tilt_rad), 0.2, 1)"),
        ),
        (
            "tracking_rule",
            json!("qcmd=clamp(qcmd,q_measured-0.5*tilt_scale,q_measured+0.5*tilt_scale)"),
        ),
        (
            "torque_rule",
            json!("clip(Kp*(qcmd-q)-Kd*qd, +/-effort_limit); convert back to position"),
        ),
        (
            "speed_rule",
            json!(
                "When torque*qd>0, multiply torque by clamp((v_limit-abs(qd))/(0.2*v_limit),0,1)"
            ),
        ),
    ] {
        if wrapper[key] != want {
            return Err(format!("Unsupported controller execution rule {key}"));
        }
    }
    let joints = c["controller"]["joints"]
        .as_array()
        .filter(|a| a.len() == 22)
        .ok_or("Expected 22 controller joints")?;
    let home = finite22(&c["action"]["home"], "home")?;
    for (i, j) in joints.iter().enumerate() {
        if j["name"] != NAMES[i]
            || j["home_rad"]
                .as_f64()
                .is_none_or(|v| (v - home[i] as f64).abs() > 1e-6)
        {
            return Err(format!("Controller joint identity differs at {i}"));
        }
        for key in [
            "kp",
            "kd",
            "software_effort_limit_nm",
            "configured_policy_slew_rad_s",
            "configured_measured_speed_soft_limit_rad_s",
        ] {
            if !j[key].as_f64().is_some_and(|v| v.is_finite() && v > 0.) {
                return Err(format!("Invalid controller joint {i} {key}"));
            }
        }
        let lo = j["command_position_range_rad"][0]
            .as_f64()
            .ok_or("Missing lower limit")?;
        let hi = j["command_position_range_rad"][1]
            .as_f64()
            .ok_or("Missing upper limit")?;
        if lo >= hi || (home[i] as f64) < lo || (home[i] as f64) > hi {
            return Err("Invalid command range/home".into());
        }
    }
    Ok(())
}
// These meanings form the stable native protocol; weights/checkpoint and qualification do not.
const INPUT_PROTOCOL: &str = r#"{
  "history_shape": [
    1,
    10,
    78
  ],
  "history_order": "oldest-to-newest, frame-major",
  "reset": "repeat first actual full observation10 times; previous raw zero only for a genuine cold start with no previously published action; takeover must match actual published state",
  "mode_switch": "preserve history and physical state",
  "actor_privileged_velocity": false,
  "frame_slices": {
    "joint_pos": [
      0,
      22
    ],
    "joint_vel": [
      22,
      44
    ],
    "body_ang_vel": [
      44,
      47
    ],
    "projected_gravity": [
      47,
      50
    ],
    "last_published_raw": [
      50,
      72
    ],
    "relative_orientation": [
      72,
      78
    ]
  },
  "joint_pos": "q_measured - home - offset",
  "joint_vel": "rad/s, no 0.1 multiplier",
  "body_ang_vel": "pelvis-local rad/s: R.T @ omega_world",
  "projected_gravity": "R.T@[0,0,-1], unit direction, not m/s^2",
  "rotation6": "first two columns, row-flattened: M00,M01,M10,M11,M20,M21",
  "relative_orientation": "motion: R.T@Rref[f]; velocity/stand: R.T@yaw(R)",
  "previous_action": "last actually published clipped raw, after simulated delay/reuse selection, before mechanical/slew/RT mapping",
  "goal_shape": [
    1,
    594
  ],
  "goal_tail": "[is_velocity,vx,vy,wz]; velocity=1, motion=0",
  "goal_motion": "10 future frames f+1..f+10 (clamp at clip end); then four zeros",
  "goal_frame_layout": "dp_heading3, rot_heading6, root_v_local3, root_w_local3, q22, qd22*0.1",
  "goal_velocity": "repeat static frame10 times: dp/v/w/qd zero, rot6 identity, q=home; append [1,vx,vy,wz]",
  "command_units": [
    "m/s",
    "m/s",
    "rad/s"
  ],
  "explicit_gait_phase": false,
  "command_coordinates": "current heading-plane for vx/vy (yaw only, no roll/pitch); world-Z yaw rate wz",
  "command_limits": {
    "vx": [
      -0.6,
      0.6
    ],
    "vy": [
      -0.35,
      0.35
    ],
    "wz": [
      -1.0,
      1.0
    ],
    "scope": "recorded training curriculum and preview bounds, not whole-domain capability qualification"
  },
  "reference_alignment": "on motion start use the frozen player _align convention: align reference yaw to current actual yaw; use the same aligned current Rref for actor orientation and reference-relative tilt guard; no physical state reset",
  "reference_phase_control": "motion_goal helper plays full NPZ; hold phases require truncating the active reference before computing future goals; return restores full reference and sets its frame; not part of ONNX"
}"#;
