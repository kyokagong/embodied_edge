use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
};
pub const N: usize = 22;
pub const INPUT: usize = 1376;
pub const NAMES: [&str; N] = [
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_roll_joint",
    "waist_yaw_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
];
#[derive(Clone, Deserialize, Serialize)]
pub struct Joint {
    pub index: usize,
    pub name: String,
    pub home: f32,
    pub kp: f32,
    pub kd: f32,
    pub model_range_rad: [f32; 2],
}
#[derive(Clone, Deserialize, Serialize)]
pub struct Action {
    pub home: [f32; N],
    pub scale: f32,
    pub clip: f32,
}
#[derive(Clone, Deserialize, Serialize)]
pub struct Contract {
    pub model_kind: String,
    pub policy_hz: u32,
    pub physics_hz: u32,
    pub gait_frequency_hz: f32,
    pub action: Action,
    pub joints: Vec<Joint>,
}
pub fn hash(path: &Path) -> Result<String, String> {
    use std::io::Read;
    let mut f = std::fs::File::open(path).map_err(|e| e.to_string())?;
    let mut h = Sha256::new();
    let mut b = [0; 65536];
    loop {
        let n = f.read(&mut b).map_err(|e| e.to_string())?;
        if n == 0 {
            break;
        }
        h.update(&b[..n]);
    }
    Ok(format!("{:x}", h.finalize()))
}
pub fn contained(root: &Path, name: &str) -> Result<PathBuf, String> {
    let p = root.join(name).canonicalize().map_err(|e| e.to_string())?;
    if !p.starts_with(root) {
        return Err("Artifact path escapes release".into());
    }
    Ok(p)
}
pub fn load(root: &Path) -> Result<Contract, String> {
    let release: serde_json::Value = serde_json::from_slice(
        &std::fs::read(root.join("release.json")).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())?;
    if release["iteration"] != 22100
        || release["source_jit_sha256"]
            != "8b9ba29c505501fbf3dfafd7cc8ceb19f331c030e8c6532a8d4f6e39cb6fddba"
    {
        return Err("Unexpected model identity".into());
    }
    let files: BTreeMap<String, String> =
        serde_json::from_value(release["files"].clone()).map_err(|e| e.to_string())?;
    for required in [
        "policy.onnx",
        "deployment_contract.json",
        "dance_catalog.json",
        "source_snapshot.json",
        "golden.json",
    ] {
        if !files.contains_key(required) {
            return Err(format!("Missing manifest entry {required}"));
        }
    }
    for (name, expected) in files {
        if hash(&contained(root, &name)?)? != expected {
            return Err(format!("Artifact digest mismatch: {name}"));
        }
    }
    let c: Contract = serde_json::from_slice(
        &std::fs::read(root.join("deployment_contract.json")).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())?;
    if c.model_kind != "q1_unified_shared_decoder"
        || c.policy_hz != 50
        || c.physics_hz != 500
        || c.action.scale != 0.25
        || c.action.clip != 20.
        || c.joints.len() != N
    {
        return Err("Unsupported deployment contract".into());
    }
    for (i, j) in c.joints.iter().enumerate() {
        if j.index != i || j.name != NAMES[i] || j.home != c.action.home[i] {
            return Err("Joint identity differs".into());
        }
    }
    Ok(c)
}
#[derive(Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Hardware {
    #[serde(default)]
    pub stand_trial: Option<StandTrial>,
    pub firmware: String,
    pub evidence: String,
    pub confirmed: bool,
    pub mc_exclusive: bool,
    pub watchdog_ms: u32,
    pub fault_damping_confirmed: bool,
    pub supported_motion: Vec<String>,
    pub signs: [f32; N],
    pub zeros: [f32; N],
    pub encoder_bias: [f32; N],
    pub lower: [f32; N],
    pub upper: [f32; N],
    pub kp: [f32; N],
    pub kd: [f32; N],
    pub fault_kd: [f32; N],
    pub imu_to_pelvis: [[f32; 3]; 3],
    pub imu_frame: String,
    pub state_timeout_s: f64,
    pub sensor_skew_s: f64,
    pub target_timeout_s: f64,
    pub max_start_error_rad: f32,
    pub max_step_rad: f32,
    pub max_temperature: f32,
    pub max_tilt_rad: f32,
    pub max_tracking_error_rad: f32,
    pub joint_topic: String,
    pub imu_topic: String,
    pub command_topic: String,
}
#[derive(Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TrialTuningLimits {
    pub source: String,
    pub max_effort: [f32; N],
    pub max_velocity: [f32; N],
    pub max_prepare_velocity: f32,
    pub max_step_rad: f32,
}
impl Default for TrialTuningLimits {
    fn default() -> Self {
        Self {
            source: "legacy supported-trial envelope".into(),
            max_effort: std::array::from_fn(|i| {
                if i < 4 || (6..10).contains(&i) {
                    18.
                } else {
                    10.
                }
            }),
            max_velocity: [314. / 100.; N],
            max_prepare_velocity: 0.15,
            max_step_rad: 0.0628,
        }
    }
}
#[derive(Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StandTrial {
    #[serde(default)]
    pub tuning_limits: TrialTuningLimits,
    pub soft_limits: bool,
    pub home: [f32; N],
    pub effort_limit: [f32; N],
    pub velocity_limit: [f32; N],
    pub policy_velocity_limit: [f32; N],
    pub measured_velocity_limit: [f32; N],
    /// Read old profiles, but never use the legacy standing envelope for policy actions.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub policy_residual: Option<[f32; N]>,
    pub prepare_seconds: f64,
    pub hardware_config_sha256: String,
}
impl Hardware {
    pub fn stand_trial_blockers(&self) -> Vec<String> {
        let Some(t) = &self.stand_trial else {
            return vec!["缺少受限站立试运行配置".into()];
        };
        let limits = &t.tuning_limits;
        let valid = !limits.source.is_empty()
            && limits.max_prepare_velocity.is_finite()
            && limits.max_prepare_velocity > 0.
            && limits.max_step_rad.is_finite()
            && limits.max_step_rad > 0.
            && self.supported_motion == ["stand_trial"]
            && self.mc_exclusive
            && self.signs.iter().all(|v| *v == 1.)
            && self
                .zeros
                .iter()
                .chain(self.encoder_bias.iter())
                .all(|v| *v == 0.)
            && !self.evidence.is_empty()
            && t.hardware_config_sha256.len() == 64
            && (10. ..=40.).contains(&t.prepare_seconds)
            && self.max_step_rad <= limits.max_step_rad
            && self.max_start_error_rad <= 0.02
            && (0..N).all(|i| {
                t.home[i].is_finite()
                    && t.home[i] >= self.lower[i]
                    && t.home[i] <= self.upper[i]
                    && t.effort_limit[i].is_finite()
                    && t.effort_limit[i] > 0.
                    && limits.max_effort[i].is_finite()
                    && limits.max_effort[i] > 0.
                    && t.effort_limit[i] <= limits.max_effort[i]
                    && limits.max_velocity[i].is_finite()
                    && limits.max_velocity[i] > 0.
                    && t.velocity_limit[i].is_finite()
                    && t.velocity_limit[i] > 0.
                    && t.velocity_limit[i] <= limits.max_prepare_velocity
                    && t.policy_velocity_limit[i].is_finite()
                    && t.policy_velocity_limit[i] > 0.
                    && t.policy_velocity_limit[i] <= limits.max_velocity[i]
                    && t.measured_velocity_limit[i].is_finite()
                    && t.measured_velocity_limit[i] >= t.policy_velocity_limit[i]
                    && t.measured_velocity_limit[i] <= limits.max_velocity[i]
                    && self.kp[i] > 0.
                    && self.kp[i] <= 80.
                    && self.kd[i] > 0.
                    && self.kd[i] <= 3.5
            });
        if valid {
            vec![]
        } else {
            vec!["站立试运行超出固定位置、速度、增益或软件力矩限制".into()]
        }
    }
    /// Explicit air-only commissioning envelope; never authorizes a learned policy.
    pub fn commissioning_blockers(&self) -> Vec<String> {
        let safe = self.supported_motion == ["air_hold"]
            && self.mc_exclusive
            && !self.evidence.is_empty()
            && self.max_step_rad <= 0.01
            && self.max_start_error_rad <= 0.01
            && self.max_tracking_error_rad <= 0.1
            && (0..22).all(|i| {
                i == 17 || (self.kp[i] == 0. && self.kd[i] == 0. && self.fault_kd[i] == 0.)
            })
            && self.kp.iter().all(|v| *v <= 2.)
            && self.kp.iter().sum::<f32>() <= 2.
            && self
                .kd
                .iter()
                .chain(self.fault_kd.iter())
                .all(|v| *v <= 0.2)
            && self.kd.iter().sum::<f32>() <= 0.2
            && self.fault_kd.iter().sum::<f32>() <= 0.2
            && (0..22).all(|i| self.upper[i] - self.lower[i] <= 0.061);
        if safe {
            Vec::new()
        } else {
            vec!["吊装调试配置超出固定低增益/小范围限制".into()]
        }
    }
    pub fn load(p: &Path) -> Result<Self, String> {
        let h: Self = serde_json::from_slice(&std::fs::read(p).map_err(|e| e.to_string())?)
            .map_err(|e| e.to_string())?;
        h.validate()?;
        Ok(h)
    }
    pub fn validate(&self) -> Result<(), String> {
        if !self.signs.iter().all(|s| *s == 1. || *s == -1.) {
            return Err("Joint sign must be +/-1".into());
        }
        for i in 0..N {
            if ![
                self.zeros[i],
                self.encoder_bias[i],
                self.lower[i],
                self.upper[i],
                self.kp[i],
                self.kd[i],
                self.fault_kd[i],
            ]
            .iter()
            .all(|x| x.is_finite())
                || self.lower[i] >= self.upper[i]
                || self.kp[i] < 0.
                || self.kd[i] < 0.
                || self.fault_kd[i] < 0.
            {
                return Err("Invalid calibration/PD profile".into());
            }
        }
        for (v, max) in [
            (self.state_timeout_s, 0.1),
            (self.sensor_skew_s, 0.1),
            (
                self.target_timeout_s,
                if self.supported_motion == ["stand_trial"] {
                    0.12
                } else {
                    0.06
                },
            ),
        ] {
            if !v.is_finite() || v <= 0. || v > max {
                return Err("Invalid low-level timeout".into());
            }
        }
        if ![
            self.max_start_error_rad,
            self.max_step_rad,
            self.max_temperature,
            self.max_tilt_rad,
            self.max_tracking_error_rad,
        ]
        .iter()
        .all(|x| x.is_finite() && *x > 0.)
            || self.max_tilt_rad > 1.2
            || self.max_tracking_error_rad > 0.5
        {
            return Err("Invalid safety thresholds".into());
        }
        for i in 0..3 {
            for j in 0..3 {
                let dot = (0..3)
                    .map(|k| self.imu_to_pelvis[i][k] * self.imu_to_pelvis[j][k])
                    .sum::<f32>();
                if !dot.is_finite() || (dot - if i == j { 1. } else { 0. }).abs() > 1e-4 {
                    return Err("Invalid IMU installation rotation".into());
                }
            }
        }
        let r = self.imu_to_pelvis;
        let det = r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
            - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
            + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]);
        if (det - 1.).abs() > 1e-4 {
            return Err("IMU transform must be a proper rotation".into());
        }
        Ok(())
    }
    pub fn blockers(&self) -> Vec<String> {
        let mut b = vec![];
        if !self.confirmed || self.firmware.is_empty() || self.evidence.is_empty() {
            b.push("固件、关节/IMU映射和限位尚未确认".into());
        }
        if !self.mc_exclusive {
            b.push("原厂MC与HAL独占交接未确认".into());
        }
        if self.watchdog_ms == 0 || !self.fault_damping_confirmed {
            b.push("本体HAL断联保护与故障阻尼未确认".into());
        }
        if !self.supported_motion.iter().any(|s| s == "stand") {
            b.push("当前模型站立闭环尚未验收".into());
        }
        b
    }
}
