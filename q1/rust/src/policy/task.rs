//! Pure reference-task transitions. Advancing a task never resets robot or observation state.
use super::reference::Clip;
use crate::lowlevel::types::Sensors;
use crate::supermimic::contract::Hardware;
use crate::supermimic::observation::{multiply, pelvis, transpose, yaw, Rotation};
use serde::Serialize;
use std::collections::BTreeMap;

const IDENTITY: Rotation = [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]];
const SETTLE_SECONDS: f64 = 0.75;
/// Pose admission only. RT motor-command correction remains defined by the hardware profile.
pub const SUPPORTED_STAND_ENTRY_JOINT_ERROR_RAD: f32 = 0.60;

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct PendingTask {
    pub action: String,
    pub clip: Option<String>,
}
#[derive(Clone, Debug)]
struct Sample {
    joint_frames: u64,
    imu_frames: u64,
    at: f64,
}
#[derive(Clone, Debug, Serialize)]
pub struct StandEntry {
    pub reference_start_frame: usize,
    pub joint_rms_rad: f32,
    pub joint_max_rad: f32,
    pub gravity_error_rad: f32,
}
#[derive(Clone, Debug, Serialize)]
pub struct TaskState {
    pub mode: String,
    pub clip: Option<String>,
    pub frame: usize,
    /// Inclusive phase boundary; every future goal frame must clamp here.
    pub end: usize,
    pub returning: bool,
    pub epoch: u64,
    /// Frozen yaw transform applied to an immutable shared reference.
    pub alignment: Rotation,
    pub queued: Option<PendingTask>,
    pub stand_entry: Option<StandEntry>,
    /// Available measured pose/velocity checks only, not the full simulator predicate.
    pub measured_prone_ready: bool,
    pub getup_ready: bool,
    pub getup_reason: String,
    pub stable_samples: u32,
    pub settle_seconds: f64,
    pub missing_measurements: Vec<String>,
    #[serde(skip)]
    last_sample: Option<Sample>,
}
impl Default for TaskState {
    fn default() -> Self {
        Self {
            mode: "stand".into(),
            clip: None,
            frame: 0,
            end: 0,
            returning: false,
            epoch: 0,
            alignment: IDENTITY,
            queued: None,
            stand_entry: None,
            measured_prone_ready: false,
            getup_ready: false,
            getup_reason: "unmeasured".into(),
            stable_samples: 0,
            settle_seconds: 0.,
            missing_measurements: vec!["root_height".into(), "root_linear_velocity".into()],
            last_sample: None,
        }
    }
}
fn phase(clip: &Clip, key: &str) -> Option<f64> {
    clip.metadata
        .get("phases")?
        .get(key)?
        .as_f64()
        .filter(|v| v.is_finite() && *v >= 0.)
}
fn clip_at<'a>(clips: &'a BTreeMap<String, Clip>, name: &str) -> Result<&'a Clip, String> {
    let clip = clips
        .get(name)
        .ok_or_else(|| format!("Unknown reference clip: {name}"))?;
    if clip.len() < 2 {
        return Err(format!("Reference clip has no usable segment: {name}"));
    }
    Ok(clip)
}
fn normalize(
    action: &str,
    clip: Option<&str>,
    clips: &BTreeMap<String, Clip>,
) -> Result<PendingTask, String> {
    let (mode, name) = match action {
        "stand" | "cancel_motion" => ("stand", None),
        "sit" | "fold" => ("sit", Some("floor_fold")),
        "lie" => ("lie", Some("official_lie")),
        "getup" => ("getup", Some("official_getUp")),
        "play_motion" | "dance" => {
            let name = clip.ok_or("A dance clip is required")?;
            let c = clip_at(clips, name)?;
            if c.metadata.get("group").and_then(|v| v.as_str()) != Some("lafan1")
                && !name.starts_with("dance")
            {
                return Err("play_motion accepts dance references only; use the explicit sit/lie/getup actions".into());
            }
            ("dance", Some(name))
        }
        _ => return Err(format!("Unsupported task action: {action}")),
    };
    if let Some(name) = name {
        clip_at(clips, name)?;
    }
    Ok(PendingTask {
        action: mode.into(),
        clip: name.map(str::to_owned),
    })
}
fn measured_rotation(s: &Sensors, h: &Hardware) -> Result<Rotation, String> {
    if !s.complete || !s.imu_valid {
        return Err("Complete joint and valid IMU feedback are required".into());
    }
    pelvis(s, h)
}
impl TaskState {
    pub fn new() -> Self {
        Self::default()
    }
    /// Enter only the feet-supported squat-to-stand part of floor_fold.
    /// Feet contact and hoist support are the operator's stated setup, not measured sensors.
    pub fn stand_from_supported_pose(
        &mut self,
        clips: &BTreeMap<String, Clip>,
        s: &Sensors,
        h: &Hardware,
    ) -> Result<(), String> {
        let actual = measured_rotation(s, h)?;
        if !s
            .q
            .iter()
            .chain(s.qd.iter())
            .chain(s.omega.iter())
            .all(|v| v.is_finite())
            || s.qd.iter().any(|v| v.abs() > 0.3)
            || s.omega.iter().map(|v| v * v).sum::<f32>().sqrt() > 0.25
        {
            return Err("站立接入要求完整、有限且静止的实测关节和IMU".into());
        }
        let clip = clip_at(clips, "floor_fold")?;
        let begin = phase(clip, "return_squat_start")
            .ok_or("Missing supported squat-to-stand phase return_squat_start")?;
        let finish =
            phase(clip, "return_end").ok_or("Missing supported squat-to-stand phase return_end")?;
        let first = (begin * 50.).round() as usize;
        let last = (finish * 50.).round() as usize;
        if first >= last || last >= clip.len() {
            return Err("Invalid supported squat-to-stand reference range".into());
        }
        let q: [f32; 22] =
            std::array::from_fn(|i| h.signs[i] * (s.q[i] - h.zeros[i]) + h.encoder_bias[i]);
        let metric = |frame: usize| {
            let errors: [f32; 22] = std::array::from_fn(|i| clip.q[[frame, i]] - q[i]);
            let rms = (errors.iter().map(|v| v * v).sum::<f32>() / 22.).sqrt();
            let max = errors.iter().map(|v| v.abs()).fold(0., f32::max);
            let angle = crate::lowlevel::policy::tilt(actual, clip.rotation(frame));
            (rms, max, angle)
        };
        let rank = |a: &(usize, (f32, f32, f32)), b: &(usize, (f32, f32, f32))| {
            (a.1 .0 * a.1 .0 + a.1 .2 * a.1 .2).total_cmp(&(b.1 .0 * b.1 .0 + b.1 .2 * b.1 .2))
        };
        let (frame, (rms, max, angle)) = (first..=last)
            .map(|f| (f, metric(f)))
            .filter(|(_, (_, max, angle))| {
                *max <= SUPPORTED_STAND_ENTRY_JOINT_ERROR_RAD && *angle <= h.max_tilt_rad
            })
            .min_by(rank)
            .ok_or_else(|| {
                let (frame, (_, max, angle)) = (first..=last)
                    .map(|f| (f, metric(f)))
                    .min_by(rank)
                    .unwrap();
                let joint = (0..22)
                    .max_by(|&a, &b| {
                        (clip.q[[frame, a]] - q[a]).abs()
                            .total_cmp(&(clip.q[[frame, b]] - q[b]).abs())
                    })
                    .unwrap();
                let name = crate::supermimic::contract::NAMES[joint];
                format!("吊装蹲坐姿态与站立参考不匹配：最大关节差{max:.3}rad（限值{SUPPORTED_STAND_ENTRY_JOINT_ERROR_RAD:.3}rad，{name}），重力方向差{angle:.3}rad（限值{:.3}rad）；需核对实测起始姿态", h.max_tilt_rad)
            })?;
        let alignment = yaw(multiply(actual, transpose(clip.rotation(frame))));
        self.mode = "stand_up".into();
        self.clip = Some("floor_fold".into());
        self.frame = frame;
        self.end = clip.len() - 1;
        self.returning = true;
        self.alignment = alignment;
        self.queued = None;
        self.epoch = self.epoch.wrapping_add(1);
        self.stand_entry = Some(StandEntry {
            reference_start_frame: frame,
            joint_rms_rad: rms,
            joint_max_rad: max,
            gravity_error_rad: angle,
        });
        self.clear_readiness("supported_stand_not_prone_getup");
        Ok(())
    }
    /// Public controller semantics distinguish supported standing from prone recovery.
    pub fn request_for_control(
        &mut self,
        enabled: bool,
        action: &str,
        clip: Option<&str>,
        clips: &BTreeMap<String, Clip>,
        s: &Sensors,
        h: &Hardware,
    ) -> Result<(), String> {
        if action == "getup" {
            return Err("趴姿起身暂不开放；站立入口用于双脚着地、吊杆支撑的蹲坐姿态".into());
        }
        if !enabled {
            if matches!(action, "start_policy" | "stand") {
                return self.stand_from_supported_pose(clips, s, h);
            }
            if action == "cancel_motion" {
                return Ok(());
            }
            return Err("请先从吊杆支撑、双脚着地的蹲坐姿态启动站立，再选择其他动作".into());
        }
        self.request(
            if action == "start_policy" {
                "stand"
            } else {
                action
            },
            clip,
            clips,
            s,
            h,
            false,
        )
    }
    /// Requesting another task never edits sensors, reference arrays, or policy history.
    pub fn request(
        &mut self,
        action: &str,
        clip: Option<&str>,
        clips: &BTreeMap<String, Clip>,
        s: &Sensors,
        h: &Hardware,
        operator_confirmed: bool,
    ) -> Result<(), String> {
        let pending = normalize(action, clip, clips)?;
        if self.mode == pending.action && self.clip == pending.clip {
            return Ok(());
        }
        if pending.action == "getup" {
            if !matches!(self.mode.as_str(), "stand" | "lie") {
                return Err(
                    "Getup entry requires an idle or lying task and measured prone feedback".into(),
                );
            }
            self.update_readiness(clips, s, h)?;
            if !operator_confirmed {
                return Err("Getup requires explicit operator confirmation because root height and complete root motion checks are unavailable".into());
            }
            if !self.measured_prone_ready {
                return Err(format!(
                    "Getup measured-pose admission refused: {}",
                    self.getup_reason
                ));
            }
            return self.start(pending, clips, s, h);
        }
        if self.mode == "lie" {
            return Err(
                "Lie holds its endpoint; explicitly confirm getup to return from prone".into(),
            );
        }
        if let Some(name) = self.clip.clone() {
            if self.returning || self.mode == "getup" {
                self.queued = Some(pending);
                return Ok(());
            }
            let active = clip_at(clips, &name)?;
            let return_start = phase(active, "return_start")
                .or_else(|| phase(active, "hold_end"))
                .or_else(|| phase(active, "dance"));
            let Some(return_start) = return_start else {
                self.queued = Some(pending);
                return Ok(());
            };
            let actual = measured_rotation(s, h)?;
            let full_end = active.len() - 1;
            let mut frame = ((return_start * 50.).round() as usize).min(full_end - 1);
            let hold_start = phase(active, "hold_start").unwrap_or(return_start);
            if self.mode == "sit" && self.frame < (hold_start * 50.).round() as usize {
                // Root height is unavailable; report that omission rather than inventing it.
                let q: [f32; 22] =
                    std::array::from_fn(|i| h.signs[i] * (s.q[i] - h.zeros[i]) + h.encoder_bias[i]);
                if !q.iter().all(|v| v.is_finite()) {
                    return Err("Nonfinite measured joints during sit return".into());
                }
                let cost = |candidate: usize| {
                    let joint = (0..22)
                        .map(|i| (active.q[[candidate, i]] - q[i]).powi(2))
                        .sum::<f32>()
                        / 22.;
                    let reference = active.rotation(candidate);
                    joint
                        + (0..3)
                            .map(|i| (reference[2][i] - actual[2][i]).powi(2))
                            .sum::<f32>()
                            / 3.
                };
                frame = (frame..full_end)
                    .min_by(|a, b| cost(*a).total_cmp(&cost(*b)))
                    .ok_or("No sit return candidate")?;
            }
            let alignment = yaw(multiply(actual, transpose(active.rotation(frame))));
            self.queued = Some(pending);
            self.end = full_end;
            self.frame = frame;
            self.alignment = alignment;
            self.returning = true;
            self.epoch = self.epoch.wrapping_add(1);
            return Ok(());
        }
        self.start(pending, clips, s, h)
    }
    fn start(
        &mut self,
        pending: PendingTask,
        clips: &BTreeMap<String, Clip>,
        s: &Sensors,
        h: &Hardware,
    ) -> Result<(), String> {
        let mut end = 0;
        let mut alignment = IDENTITY;
        if let Some(name) = pending.clip.as_deref() {
            let clip = clip_at(clips, name)?;
            end = clip.len() - 1;
            if pending.action == "sit" {
                let boundary = phase(clip, "hold_end")
                    .or_else(|| phase(clip, "return_start"))
                    .ok_or("Sit reference requires hold_end or return_start")?;
                let boundary = (boundary * 50.).round() as usize;
                if boundary == 0 {
                    return Err("Sit hold boundary must be positive".into());
                }
                end = end.min(boundary - 1);
            }
            alignment = yaw(multiply(
                measured_rotation(s, h)?,
                transpose(clip.rotation(0)),
            ));
        }
        self.mode = pending.action;
        self.clip = pending.clip;
        self.frame = 0;
        self.end = end;
        self.returning = false;
        self.alignment = alignment;
        self.queued = None;
        self.stand_entry = None;
        self.epoch = self.epoch.wrapping_add(1);
        self.clear_readiness("unmeasured");
        Ok(())
    }
    /// Called once per enabled 50 Hz policy sampling tick, never by UI or RT.
    pub fn advance(
        &mut self,
        clips: &BTreeMap<String, Clip>,
        s: &Sensors,
        h: &Hardware,
    ) -> Result<(), String> {
        if self.clip.is_some() {
            if self.frame < self.end {
                self.frame += 1;
            } else if !matches!(self.mode.as_str(), "sit" | "lie") || self.returning {
                if matches!(self.mode.as_str(), "getup" | "stand_up")
                    || (self.mode == "sit" && self.returning)
                {
                    let actual = measured_rotation(s, h)?;
                    let terminal = clip_at(clips, self.clip.as_deref().unwrap())?;
                    let errors: [f32; 22] = std::array::from_fn(|i| {
                        h.signs[i] * (s.q[i] - h.zeros[i]) + h.encoder_bias[i]
                            - terminal.q[[self.end, i]]
                    });
                    let matched = actual[2][2] > 0.94
                        && (errors.iter().map(|v| v * v).sum::<f32>() / 22.).sqrt() <= 0.15
                        && errors.iter().all(|v| v.abs() <= 0.35);
                    let quiet = s.omega.iter().map(|v| v * v).sum::<f32>().sqrt() <= 0.25
                        && s.qd.iter().all(|v| v.is_finite() && v.abs() <= 0.8)
                        && (s.qd.iter().map(|v| v * v).sum::<f32>() / 22.).sqrt() <= 0.35;
                    if !matched || !quiet {
                        return Err("Standing return reference ended without a measured quiet upright endpoint; protection hold required".into());
                    }
                }
                let pending = self.queued.clone().unwrap_or(PendingTask {
                    action: "stand".into(),
                    clip: None,
                });
                self.start(pending, clips, s, h)?;
            }
        }
        self.update_readiness(clips, s, h)
    }
    pub fn rotation(&self, clip: &Clip) -> Rotation {
        multiply(
            self.alignment,
            clip.rotation(self.frame.min(self.end).min(clip.len().saturating_sub(1))),
        )
    }
    fn clear_readiness(&mut self, reason: &str) {
        self.measured_prone_ready = false;
        self.getup_ready = false;
        self.getup_reason = reason.into();
        self.stable_samples = 0;
        self.settle_seconds = 0.;
        self.last_sample = None;
    }
    /// Also usable while disabled: inspects measured feedback without advancing tasks.
    /// The actor must validate feedback freshness before calling this method.
    pub fn update_readiness(
        &mut self,
        clips: &BTreeMap<String, Clip>,
        s: &Sensors,
        h: &Hardware,
    ) -> Result<(), String> {
        self.missing_measurements = vec!["root_height".into()];
        let sample_at = s.joints_at.min(s.imu_at);
        let root_v = s.root_velocity.filter(|v| {
            v.iter().all(|v| v.is_finite())
                && s.root_velocity_at > 0.
                && (0. ..=h.state_timeout_s).contains(&(sample_at - s.root_velocity_at))
        });
        if root_v.is_none() {
            self.missing_measurements
                .push("root_linear_velocity".into());
        }
        if !matches!(self.mode.as_str(), "stand" | "lie") {
            self.clear_readiness("unmeasured");
            return Ok(());
        }
        let Some(prone) = clips.get("official_lie") else {
            self.clear_readiness("feedback_unavailable");
            return Ok(());
        };
        let hold = phase(prone, "hold_start")
            .map(|v| (v * 50. - 1e-6).ceil() as usize)
            .unwrap_or(prone.len().saturating_sub(1));
        if prone.len() == 0 || hold >= prone.len() {
            return Err("Prone hold phase is outside its reference clip".into());
        }
        if self.mode == "lie" && self.frame < hold {
            self.clear_readiness("lowering");
            return Ok(());
        }
        if !s.complete
            || !s.imu_valid
            || !sample_at.is_finite()
            || sample_at <= 0.
            || !s
                .q
                .iter()
                .chain(s.qd.iter())
                .chain(s.omega.iter())
                .all(|v| v.is_finite())
        {
            self.clear_readiness("feedback_unavailable");
            return Ok(());
        }
        let actual = match measured_rotation(s, h) {
            Ok(r) => r,
            Err(_) => {
                self.clear_readiness("feedback_unavailable");
                return Ok(());
            }
        };
        let aligned = if self.mode == "lie" {
            multiply(self.alignment, prone.rotation(hold))
        } else {
            let reference = prone.rotation(hold);
            multiply(yaw(multiply(actual, transpose(reference))), reference)
        };
        let delta = multiply(actual, transpose(aligned));
        let angle = ((delta[0][0] + delta[1][1] + delta[2][2] - 1.) * 0.5)
            .clamp(-1., 1.)
            .acos();
        let errors: [f32; 22] = std::array::from_fn(|i| {
            h.signs[i] * (s.q[i] - h.zeros[i]) + h.encoder_bias[i] - prone.q[[hold, i]]
        });
        let matched = angle <= 0.21
            && (errors.iter().map(|v| v * v).sum::<f32>() / 22.).sqrt() <= 0.15
            && errors.iter().all(|v| v.abs() <= 0.35);
        let quiet = s.omega.iter().map(|v| v * v).sum::<f32>().sqrt() <= 0.25
            && (s.qd.iter().map(|v| v * v).sum::<f32>() / 22.).sqrt() <= 0.35
            && s.qd.iter().all(|v| v.abs() <= 0.8)
            && root_v.is_none_or(|v| v.iter().map(|v| v * v).sum::<f32>().sqrt() <= 0.08);
        if !matched || !quiet {
            self.clear_readiness(if !matched { "pose_mismatch" } else { "moving" });
            return Ok(());
        }
        if self.last_sample.as_ref().is_some_and(|last| {
            s.joint_frames < last.joint_frames
                || s.imu_frames < last.imu_frames
                || sample_at < last.at
        }) {
            self.clear_readiness("feedback_regressed");
            return Ok(());
        }
        let new_sample = self.last_sample.as_ref().is_none_or(|last| {
            s.joint_frames > last.joint_frames
                && s.imu_frames > last.imu_frames
                && sample_at > last.at
        });
        if new_sample {
            if let Some(last) = &self.last_sample {
                let dt = sample_at - last.at;
                if dt > h.state_timeout_s {
                    self.settle_seconds = 0.;
                    self.stable_samples = 0;
                } else {
                    self.settle_seconds = (self.settle_seconds + dt.min(0.04)).min(SETTLE_SECONDS);
                }
            }
            self.stable_samples = self.stable_samples.saturating_add(1);
            self.last_sample = Some(Sample {
                joint_frames: s.joint_frames,
                imu_frames: s.imu_frames,
                at: sample_at,
            });
        }
        self.measured_prone_ready =
            self.settle_seconds >= SETTLE_SECONDS - 1e-9 && self.stable_samples >= 38;
        // Root height is not measured on this deployment, so never report full readiness.
        self.getup_ready = false;
        self.getup_reason = if self.measured_prone_ready {
            "missing_root_measurements"
        } else {
            "settling"
        }
        .into();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ndarray::{Array2, Array3};
    use serde_json::json;

    fn fixture(name: &str, frames: usize, phases: serde_json::Value) -> Clip {
        Clip {
            metadata: json!({"id":name,"phases":phases}),
            rotations: Array3::from_shape_fn(
                (frames, 3, 3),
                |(_, i, j)| if i == j { 1. } else { 0. },
            ),
            q: Array2::zeros((frames, 22)),
            root_p: Array2::zeros((frames, 3)),
            root_v_local: Array2::zeros((frames, 3)),
            root_w_local: Array2::zeros((frames, 3)),
            qd: Array2::zeros((frames, 22)),
        }
    }
    fn clips() -> BTreeMap<String, Clip> {
        BTreeMap::from([
            (
                "floor_fold".into(),
                fixture(
                    "floor_fold",
                    1551,
                    json!({"hold_start":9.,"hold_end":12.,"return_start":12.}),
                ),
            ),
            (
                "official_lie".into(),
                fixture(
                    "official_lie",
                    1121,
                    json!({"hold_start":12.4,"hold_end":22.4}),
                ),
            ),
            (
                "official_getUp".into(),
                fixture(
                    "official_getUp",
                    2226,
                    json!({"rise_start":1.,"stand_start":14.5}),
                ),
            ),
            (
                "dance1_subject1".into(),
                fixture(
                    "dance1_subject1",
                    1001,
                    json!({"dance":15.,"return":18.,"stand":20.}),
                ),
            ),
        ])
    }
    fn hardware() -> Hardware {
        serde_json::from_str(include_str!("../../../config/supermimic-unconfirmed.json")).unwrap()
    }
    fn sensors(h: &Hardware) -> Sensors {
        Sensors {
            q: std::array::from_fn(|i| h.zeros[i] - h.encoder_bias[i] / h.signs[i]),
            quaternion_xyzw: [0., 0., 0., 1.],
            complete: true,
            imu_valid: true,
            ..Sensors::default()
        }
    }
    fn sample(s: &mut Sensors, n: u64) {
        s.joint_frames = n;
        s.imu_frames = n;
        s.joints_at = 1. + n as f64 * 0.02;
        s.imu_at = s.joints_at;
    }
    #[test]
    fn control_entry_requires_supported_standing_before_other_tasks_and_disables_prone_recovery() {
        let clips = clips();
        let h = hardware();
        let s = sensors(&h);
        for action in ["play_motion", "sit", "lie", "getup"] {
            let mut t = TaskState::new();
            let before = serde_json::to_value(&t).unwrap();
            assert!(
                t.request_for_control(false, action, Some("dance1_subject1"), &clips, &s, &h)
                    .is_err(),
                "accepted disabled {action}"
            );
            assert_eq!(serde_json::to_value(&t).unwrap(), before);
        }
        let mut t = TaskState::new();
        assert!(t
            .request_for_control(true, "getup", None, &clips, &s, &h)
            .is_err());
        t.request_for_control(true, "sit", None, &clips, &s, &h)
            .unwrap();
        assert_eq!(t.mode, "sit");
    }
    #[test]
    fn supported_squat_stand_enters_matching_upright_return_segment() {
        let mut clips = clips();
        let h = hardware();
        let s = sensors(&h);
        let c = clips.get_mut("floor_fold").unwrap();
        c.metadata["phases"]["return_squat_start"] = json!(22.);
        c.metadata["phases"]["return_end"] = json!(28.);
        c.q.fill(1.);
        c.q.row_mut(1200).fill(0.);
        let mut task = TaskState::new();
        task.stand_from_supported_pose(&clips, &s, &h).unwrap();
        assert_eq!(
            (
                task.mode.as_str(),
                task.clip.as_deref(),
                task.frame,
                task.end
            ),
            ("stand_up", Some("floor_fold"), 1200, 1550)
        );
        assert!(task.returning);
        assert!(!task.measured_prone_ready);
        task.advance(&clips, &s, &h).unwrap();
        let epoch = task.epoch;
        task.request("stand", None, &clips, &s, &h, false).unwrap();
        assert_eq!((task.frame, task.epoch), (1201, epoch));
    }
    #[test]
    fn supported_stand_refuses_prone_moving_and_unmatched_postures_without_changing_task() {
        let mut clips = clips();
        let h = hardware();
        let mut s = sensors(&h);
        let c = clips.get_mut("floor_fold").unwrap();
        c.metadata["phases"]["return_squat_start"] = json!(22.);
        c.metadata["phases"]["return_end"] = json!(28.);
        let task = TaskState::new();
        for scenario in 0..3 {
            s.q = [0.; 22];
            s.qd = [0.; 22];
            s.quaternion_xyzw = [0., 0., 0., 1.];
            if scenario == 0 {
                s.quaternion_xyzw = [
                    0.,
                    std::f32::consts::FRAC_1_SQRT_2,
                    0.,
                    std::f32::consts::FRAC_1_SQRT_2,
                ];
            }
            if scenario == 1 {
                s.qd[3] = 0.5;
            }
            if scenario == 2 {
                s.q = [3.; 22];
            }
            let mut t = task.clone();
            let before = serde_json::to_value(&t).unwrap();
            assert!(t.stand_from_supported_pose(&clips, &s, &h).is_err());
            assert_eq!(serde_json::to_value(t).unwrap(), before);
        }
    }
    #[test]
    fn supported_stand_completion_requires_measured_upright_endpoint() {
        let clips = clips();
        let h = hardware();
        let mut s = sensors(&h);
        s.quaternion_xyzw = [
            0.,
            std::f32::consts::FRAC_1_SQRT_2,
            0.,
            std::f32::consts::FRAC_1_SQRT_2,
        ];
        let mut t = TaskState::new();
        t.mode = "stand_up".into();
        t.clip = Some("floor_fold".into());
        t.frame = 1550;
        t.end = 1550;
        t.returning = true;
        assert!(t.advance(&clips, &s, &h).is_err());
        assert_eq!(t.mode, "stand_up");
        s.quaternion_xyzw = [0., 0., 0., 1.];
        t.advance(&clips, &s, &h).unwrap();
        assert_eq!(t.mode, "stand");
    }
    #[test]
    fn sit_holds_at_boundary_and_return_restores_full_segment() {
        let clips = clips();
        let h = hardware();
        let s = sensors(&h);
        let mut t = TaskState::new();
        t.request("sit", None, &clips, &s, &h, false).unwrap();
        assert_eq!((t.mode.as_str(), t.frame, t.end), ("sit", 0, 599));
        for _ in 0..700 {
            t.advance(&clips, &s, &h).unwrap();
        }
        assert_eq!((t.mode.as_str(), t.frame, t.end), ("sit", 599, 599));
        let held_epoch = t.epoch;
        t.request("stand", None, &clips, &s, &h, false).unwrap();
        assert!(t.returning);
        assert_eq!((t.frame, t.end), (600, 1550));
        assert!(t.epoch > held_epoch);
        for _ in 0..951 {
            t.advance(&clips, &s, &h).unwrap();
        }
        assert_eq!(t.mode, "stand");
        assert!(t.clip.is_none());
    }
    #[test]
    fn repeated_selection_preserves_progress_epoch_and_alignment() {
        let clips = clips();
        let h = hardware();
        let s = sensors(&h);
        let mut t = TaskState::new();
        t.request(
            "play_motion",
            Some("dance1_subject1"),
            &clips,
            &s,
            &h,
            false,
        )
        .unwrap();
        t.advance(&clips, &s, &h).unwrap();
        assert_eq!((t.mode.as_str(), t.frame), ("dance", 1));
        let before = serde_json::to_value(&t).unwrap();
        t.request(
            "play_motion",
            Some("dance1_subject1"),
            &clips,
            &s,
            &h,
            false,
        )
        .unwrap();
        assert_eq!(serde_json::to_value(&t).unwrap(), before);
    }
    #[test]
    fn dance_cancel_uses_return_phase_and_epoch_changes_only_at_boundaries() {
        let clips = clips();
        let h = hardware();
        let s = sensors(&h);
        let mut t = TaskState::new();
        t.request(
            "play_motion",
            Some("dance1_subject1"),
            &clips,
            &s,
            &h,
            false,
        )
        .unwrap();
        let epoch = t.epoch;
        t.advance(&clips, &s, &h).unwrap();
        assert_eq!(t.epoch, epoch);
        t.request("cancel_motion", None, &clips, &s, &h, false)
            .unwrap();
        assert!(t.returning);
        assert_eq!(t.frame, 750);
        assert_eq!(t.end, 1000);
        assert!(t.epoch > epoch);
        let returning_epoch = t.epoch;
        t.request("stand", None, &clips, &s, &h, false).unwrap();
        assert_eq!(t.epoch, returning_epoch);
        for _ in 0..251 {
            t.advance(&clips, &s, &h).unwrap();
        }
        assert_eq!(t.mode, "stand");
        assert!(t.epoch > returning_epoch);
    }
    #[test]
    fn early_sit_cancel_matches_measured_return_pose_without_claiming_root_height() {
        let mut clips = clips();
        let h = hardware();
        let s = sensors(&h);
        let c = clips.get_mut("floor_fold").unwrap();
        c.q.fill(1.);
        c.q.row_mut(900).fill(0.);
        let mut t = TaskState::new();
        t.request("sit", None, &clips, &s, &h, false).unwrap();
        t.request("stand", None, &clips, &s, &h, false).unwrap();
        assert_eq!(t.frame, 900);
        assert!(t.missing_measurements.iter().any(|s| s == "root_height"));
    }
    #[test]
    fn lie_holds_and_never_automatically_selects_upright() {
        let clips = clips();
        let h = hardware();
        let s = sensors(&h);
        let mut t = TaskState::new();
        t.request("lie", None, &clips, &s, &h, false).unwrap();
        for _ in 0..1200 {
            t.advance(&clips, &s, &h).unwrap();
        }
        assert_eq!((t.mode.as_str(), t.frame, t.end), ("lie", 1120, 1120));
        assert!(t.request("stand", None, &clips, &s, &h, false).is_err());
        assert!(t
            .request("cancel_motion", None, &clips, &s, &h, false)
            .is_err());
        assert_eq!(t.mode, "lie");
    }
    #[test]
    fn getup_requires_distinct_stable_samples_and_explicit_confirmation() {
        let clips = clips();
        let h = hardware();
        let mut s = sensors(&h);
        let mut t = TaskState::new();
        t.request("lie", None, &clips, &s, &h, false).unwrap();
        t.frame = t.end;
        sample(&mut s, 1);
        for _ in 0..100 {
            t.advance(&clips, &s, &h).unwrap();
        }
        assert!(!t.measured_prone_ready);
        assert!(t.stable_samples <= 1);
        for n in 2..=40 {
            sample(&mut s, n);
            t.advance(&clips, &s, &h).unwrap();
        }
        assert!(t.measured_prone_ready);
        assert!(!t.getup_ready);
        assert!(t.missing_measurements.iter().any(|s| s == "root_height"));
        assert!(t.request("getup", None, &clips, &s, &h, false).is_err());
        t.request("getup", None, &clips, &s, &h, true).unwrap();
        assert_eq!(t.mode, "getup");
        for _ in 0..2226 {
            t.advance(&clips, &s, &h).unwrap();
        }
        assert_eq!(t.mode, "stand");
    }
    #[test]
    fn failed_getup_cannot_select_upright_command_at_reference_end() {
        let clips = clips();
        let h = hardware();
        let mut s = sensors(&h);
        s.quaternion_xyzw = [
            0.,
            std::f32::consts::FRAC_1_SQRT_2,
            0.,
            std::f32::consts::FRAC_1_SQRT_2,
        ];
        let mut t = TaskState::new();
        t.mode = "getup".into();
        t.clip = Some("official_getUp".into());
        t.frame = 2225;
        t.end = 2225;
        assert!(t.advance(&clips, &s, &h).is_err());
        assert_eq!(t.mode, "getup");
    }
    #[test]
    fn measured_motion_revokes_getup_admission() {
        let clips = clips();
        let h = hardware();
        let mut s = sensors(&h);
        let mut t = TaskState::new();
        t.request("lie", None, &clips, &s, &h, false).unwrap();
        t.frame = t.end;
        for n in 1..=40 {
            sample(&mut s, n);
            t.advance(&clips, &s, &h).unwrap();
        }
        assert!(t.measured_prone_ready);
        s.qd[0] = 1.;
        sample(&mut s, 41);
        assert!(t.request("getup", None, &clips, &s, &h, true).is_err());
        assert_eq!(t.mode, "lie");
    }
    #[test]
    fn regressed_feedback_revokes_previously_stable_getup_admission() {
        let clips = clips();
        let h = hardware();
        let mut s = sensors(&h);
        let mut t = TaskState::new();
        for n in 1..=40 {
            sample(&mut s, n);
            t.update_readiness(&clips, &s, &h).unwrap();
        }
        assert!(t.measured_prone_ready);
        sample(&mut s, 39);
        t.update_readiness(&clips, &s, &h).unwrap();
        assert!(!t.measured_prone_ready);
        assert_eq!(t.stable_samples, 0);
    }
    #[test]
    fn idle_readiness_can_admit_confirmed_getup_without_moving_reference_time() {
        let clips = clips();
        let h = hardware();
        let mut s = sensors(&h);
        let mut t = TaskState::new();
        for n in 1..=40 {
            sample(&mut s, n);
            t.update_readiness(&clips, &s, &h).unwrap();
        }
        assert_eq!((t.frame, t.epoch), (0, 0));
        assert!(t.measured_prone_ready);
        t.request("getup", None, &clips, &s, &h, true).unwrap();
        assert_eq!(t.mode, "getup");
    }
    #[test]
    fn yaw_alignment_is_frozen_without_mutating_reference() {
        let clips = clips();
        let h = hardware();
        let mut s = sensors(&h);
        s.quaternion_xyzw = [
            0.,
            0.,
            (std::f32::consts::FRAC_PI_4).sin(),
            (std::f32::consts::FRAC_PI_4).cos(),
        ];
        let before = clips["floor_fold"].rotations.clone();
        let mut t = TaskState::new();
        t.request("sit", None, &clips, &s, &h, false).unwrap();
        assert!((t.rotation(&clips["floor_fold"])[1][0] - 1.).abs() < 1e-5);
        assert_eq!(clips["floor_fold"].rotations, before);
    }
}
