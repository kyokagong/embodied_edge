//! Q1 high-level locomotion contract and transport-independent safety engine.
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use std::collections::BTreeMap;
use std::sync::OnceLock;
use std::time::Instant;

pub fn monotonic() -> f64 {
    static ORIGIN: OnceLock<Instant> = OnceLock::new();
    ORIGIN.get_or_init(Instant::now).elapsed().as_secs_f64()
}

/// No blocking operations belong in this state machine. ROS owns transport only.
pub struct Engine {
    pub source: String,
    pub enabled: bool,
    pub registered: bool,
    pub stopping: bool,
    pub source_operation: Option<i32>,
    pub fault: Option<String>,
    source_base: String,
    feedback_timeout: f64,
    command_timeout: f64,
    action_name: String,
    running: bool,
    action_at: Option<f64>,
    joints_at: Option<f64>,
    joint_stamp: i64,
    pose: BTreeMap<String, f64>,
    owner_at: Option<f64>,
    owner_name: String,
    source_active: bool,
    ever_owned: bool,
    recovery_inhibited: bool,
    acquisition_at: Option<f64>,
    velocity: Velocity,
    velocity_at: Option<f64>,
    zero_since: Option<f64>,
    release_attempted: bool,
}
impl Engine {
    pub fn new(config: &Value) -> Self {
        let cfg = &config["ros2"];
        let source = cfg["source_name"].as_str().unwrap_or("edge_q1").to_owned();
        Self {
            source: source.clone(),
            source_base: source,
            enabled: false,
            registered: false,
            stopping: false,
            source_operation: None,
            fault: None,
            feedback_timeout: cfg["feedback_timeout_s"].as_f64().unwrap_or(0.25),
            command_timeout: cfg["command_timeout_s"].as_f64().unwrap_or(0.25),
            action_name: String::new(),
            running: false,
            action_at: None,
            joints_at: None,
            joint_stamp: 0,
            pose: BTreeMap::new(),
            owner_at: None,
            owner_name: String::new(),
            source_active: false,
            ever_owned: false,
            recovery_inhibited: false,
            acquisition_at: None,
            velocity: Velocity::default(),
            velocity_at: None,
            zero_since: None,
            release_attempted: false,
        }
    }
    fn fresh(&self, t: Option<f64>, now: f64) -> bool {
        t.is_some_and(|t| (0.0..=self.feedback_timeout).contains(&(now - t)))
    }
    pub fn connected(&self, now: f64) -> bool {
        self.fresh(self.action_at, now) && self.fresh(self.joints_at, now)
    }
    fn ready(&self, now: f64) -> bool {
        self.connected(now) && self.running && self.action_name == WALK
    }
    fn owns(&self, now: f64) -> bool {
        self.registered && self.source_active && self.fresh(self.owner_at, now)
    }
    fn acquisition_limit(&self) -> f64 {
        0.25_f64
            .min(self.command_timeout)
            .min(self.feedback_timeout)
    }
    fn can_acquire(&self, now: f64) -> bool {
        self.enabled
            && self.fault.is_none()
            && self.registered
            && !self.ever_owned
            && !self.stopping
            && !self.recovery_inhibited
            && self.fresh(self.owner_at, now)
            && self.ready(now)
            && self
                .acquisition_at
                .is_none_or(|t| now - t < self.acquisition_limit())
    }
    pub fn enable(&mut self, now: f64) -> Result<(), String> {
        if !self.ready(now) {
            return Err("首次使能需要新鲜的全部关节反馈与 BIPED_WALK_RUN / RUNNING".into());
        }
        if self.stopping || self.source_operation.is_some() {
            return Err("Previous source operation is still finishing".into());
        }
        if self.enabled && self.fault.is_none() {
            return Ok(());
        }
        // Each registration has a distinct epoch, even when MC caches deleted attribution.
        if !self.registered {
            self.source = format!("{}_{:016x}", self.source_base, rand::random::<u64>());
            self.source_active = false;
            self.source_operation = Some(1001);
        }
        self.enabled = true;
        self.fault = None;
        self.ever_owned = false;
        self.recovery_inhibited = false;
        self.acquisition_at = None;
        self.release_attempted = false;
        self.velocity = Velocity::default();
        self.velocity_at = Some(now);
        Ok(())
    }
    pub fn set_velocity(&mut self, v: Velocity, now: f64) -> Result<(), String> {
        v.validate()?;
        if v.moving() {
            if !self.enabled
                || self.fault.is_some()
                || !self.ready(now)
                || !(self.owns(now) || self.can_acquire(now))
            {
                return Err(
                    "Velocity requires owned source and fresh native MC RUNNING feedback".into(),
                );
            }
            if !self.ever_owned && self.acquisition_at.is_none() {
                self.acquisition_at = Some(now);
            }
        }
        self.velocity = v;
        self.velocity_at = Some(now);
        Ok(())
    }
    pub fn stop(&mut self, now: f64) {
        self.enabled = false;
        self.velocity = Velocity::default();
        if (self.registered || self.source_operation.is_some())
            && !self.stopping
            && !self.recovery_inhibited
        {
            self.stopping = true;
            self.zero_since = Some(now);
        }
    }
    pub fn trip(&mut self, message: &str, now: f64, recover: bool) {
        if self.fault.is_none() {
            eprintln!("backend_fault source={} error={message}", self.source);
            self.fault = Some(message.into());
        }
        self.stop(now);
        if !recover {
            self.recovery_inhibited = true;
            self.stopping = false;
        }
    }
    pub fn action(&mut self, name: &str, running: bool, now: f64) {
        if self.action_name != name || self.running != running {
            eprintln!("mc_action action={name} running={running}");
        }
        self.action_name = name.into();
        self.running = running;
        self.action_at = Some(now);
        if self.enabled && (name != WALK || !running) {
            self.trip(
                "Native MC action changed outside requested transition",
                now,
                false,
            );
        }
    }
    pub fn joints(&mut self, pose: BTreeMap<String, f64>, stamp: i64, now: f64) {
        if stamp <= self.joint_stamp
            || pose.len() != 10
            || JOINT_NAMES
                .iter()
                .any(|n| !pose.get(*n).is_some_and(|v| v.is_finite()))
        {
            return;
        }
        self.pose = pose;
        self.joints_at = Some(now);
        self.joint_stamp = stamp;
    }
    pub fn owner(&mut self, name: &str, now: f64) {
        let owned = self.registered && name == self.source;
        if self.owner_name != name {
            eprintln!("source_owner source={} owner={name}", self.source);
        }
        self.owner_name = name.into();
        self.owner_at = Some(now);
        if owned
            && !self.ever_owned
            && self
                .acquisition_at
                .is_some_and(|t| now - t >= self.acquisition_limit())
        {
            self.source_active = false;
            self.trip(
                "MC first ownership acknowledgement arrived after acquisition deadline",
                now,
                true,
            );
            return;
        }
        self.source_active = owned;
        if self.registered && self.ever_owned && !owned {
            self.trip("MC input source ownership lost", now, false);
        }
        self.ever_owned |= owned;
    }
    pub fn source_response(&mut self, operation: i32, now: f64) {
        eprintln!(
            "source_response source={} operation={operation}",
            self.source
        );
        self.source_operation = None;
        self.registered = operation == 1001;
        if !self.registered {
            self.stopping = false;
            self.owner_at = None;
            self.source_active = false;
        } else if !self.enabled {
            self.stop(now);
        }
    }
    pub fn watchdog(&mut self, now: f64) {
        if !self.registered {
            return;
        }
        if !self.connected(now) {
            self.trip("MC or full joint feedback is stale", now, true);
        }
        if self.ever_owned && !self.fresh(self.owner_at, now) {
            self.trip("MC input source ownership feedback is stale", now, false);
        }
        if self.enabled
            && !self.ever_owned
            && self
                .acquisition_at
                .is_some_and(|t| now - t >= self.acquisition_limit())
        {
            self.trip(
                "MC source acquisition timed out after first motion request",
                now,
                true,
            );
        }
        if self.enabled
            && self.velocity.moving()
            && self
                .velocity_at
                .is_none_or(|t| now - t > self.command_timeout)
        {
            self.trip("Velocity command watchdog expired", now, true);
        }
        if self.stopping
            && !self.recovery_inhibited
            && !self.release_attempted
            && self.ready(now)
            && self.source_operation.is_none()
            && self.zero_since.is_some_and(|t| now - t >= 0.08)
        {
            self.release_attempted = true;
            self.source_operation = Some(1003);
        }
    }
    pub fn output(&self, now: f64) -> Velocity {
        if !self.enabled
            || self.fault.is_some()
            || !self.ready(now)
            || !(self.owns(now) || self.can_acquire(now))
        {
            return Velocity::default();
        }
        if self.ever_owned {
            self.velocity
        } else {
            self.velocity.acquisition_cap()
        }
    }
    pub fn snapshot(&self, now: f64) -> Value {
        let age = |t: Option<f64>| t.map(|t| (now - t).max(0.0));
        json!({"connected":self.connected(now),"action":self.action_name,"action_running":self.running&&self.fresh(self.action_at,now),
            "joints":self.pose,"joints_age_s":age(self.joints_at),"action_age_s":age(self.action_at),"owner_age_s":age(self.owner_at),
            "source_active":self.owns(now),"source_name":self.source,"source_owner":self.owner_name,"source_can_acquire":self.can_acquire(now),
            "source_acquiring":self.enabled&&!self.ever_owned&&self.acquisition_at.is_some(),"fault":self.fault,"velocity":self.velocity})
    }
}

pub const WALK: &str = "BIPED_WALK_RUN";
pub const JOINT_NAMES: [&str; 10] = [
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
const HOME: [f64; 10] = [
    -0.01982023485863345,
    -0.00021580613966982196,
    0.017233758101518008,
    0.22511493021261703,
    -0.003530979547924497,
    1.0186741542682025,
    0.017164215358473386,
    -0.22262401216020816,
    0.004204617106736518,
    1.0185279282650144,
];

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct Velocity {
    pub forward: f64,
    pub lateral: f64,
    pub angular: f64,
}
impl Velocity {
    pub fn moving(self) -> bool {
        self != Self::default()
    }
    pub fn validate(self) -> Result<(), String> {
        for (v, limit) in [
            (self.forward, 0.3),
            (self.lateral, 0.2),
            (self.angular, 0.5),
        ] {
            if !v.is_finite() || v.abs() > limit {
                return Err("Velocity must be finite and within 0.3/0.2 m/s and 0.5 rad/s".into());
            }
        }
        Ok(())
    }
    fn acquisition_cap(self) -> Self {
        Self {
            forward: self.forward.clamp(-0.2, 0.2),
            lateral: self.lateral.clamp(-0.2, 0.2),
            angular: self.angular.clamp(-0.2, 0.2),
        }
    }
}

pub trait Backend: Send {
    fn kind(&self) -> &'static str;
    fn enable(&mut self, now: f64) -> Result<(), String>;
    fn request_mode(&mut self, mode: &str, now: f64) -> Result<(), String>;
    fn set_velocity(&mut self, velocity: Velocity, now: f64) -> Result<(), String>;
    fn stop(&mut self, now: f64) -> Result<(), String>;
    fn snapshot(&self, now: f64) -> Value;
}

pub struct MockBackend {
    enabled: bool,
    velocity: Velocity,
}
impl Default for MockBackend {
    fn default() -> Self {
        Self::new()
    }
}
impl MockBackend {
    pub fn new() -> Self {
        Self {
            enabled: false,
            velocity: Velocity::default(),
        }
    }
}
impl Backend for MockBackend {
    fn kind(&self) -> &'static str {
        "mock"
    }
    fn enable(&mut self, _: f64) -> Result<(), String> {
        self.enabled = true;
        Ok(())
    }
    fn request_mode(&mut self, mode: &str, _: f64) -> Result<(), String> {
        if !matches!(mode, "stand" | "walk") {
            return Err("Pico 上肢功能尚未迁移到 Rust".into());
        }
        if !self.enabled {
            return Err("Explicit enable required".into());
        }
        self.velocity = Velocity::default();
        Ok(())
    }
    fn set_velocity(&mut self, v: Velocity, _: f64) -> Result<(), String> {
        v.validate()?;
        if v.moving() && !self.enabled {
            return Err("Explicit enable required".into());
        }
        self.velocity = v;
        Ok(())
    }
    fn stop(&mut self, _: f64) -> Result<(), String> {
        self.enabled = false;
        self.velocity = Velocity::default();
        Ok(())
    }
    fn snapshot(&self, _: f64) -> Value {
        let joints: BTreeMap<_, _> = JOINT_NAMES.into_iter().zip(HOME).collect();
        json!({"connected":true,"action":WALK,"action_running":true,"joints":joints,"joints_age_s":0.0,"source_active":self.enabled,"fault":null,"velocity":self.velocity})
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn initial_acquisition_is_capped_and_late_owner_is_rejected() {
        let config = json!({"ros2":{"source_name":"edge_q1","feedback_timeout_s":0.25,"command_timeout_s":0.25}});
        let mut engine = Engine::new(&config);
        engine.action(WALK, true, 1.0);
        engine.joints(
            JOINT_NAMES
                .into_iter()
                .map(String::from)
                .zip(HOME)
                .collect(),
            1,
            1.0,
        );
        engine.owner("cached_deleted_source", 1.0);
        engine.enable(1.0).unwrap();
        engine.registered = true;
        engine
            .set_velocity(
                Velocity {
                    forward: 0.3,
                    lateral: 0.0,
                    angular: 0.4,
                },
                1.0,
            )
            .unwrap();
        assert_eq!(
            engine.output(1.1),
            Velocity {
                forward: 0.2,
                lateral: 0.0,
                angular: 0.2
            }
        );
        let source = engine.source.clone();
        engine.owner(&source, 1.25);
        assert!(!engine.enabled);
        assert_eq!(engine.output(1.25), Velocity::default());
        assert_eq!(engine.snapshot(1.25)["source_active"], false);
    }
    #[test]
    fn idle_enable_does_not_start_acquisition_timeout() {
        let mut e = Engine::new(&json!({}));
        e.action(WALK, true, 0.0);
        e.joints(
            JOINT_NAMES
                .into_iter()
                .map(String::from)
                .zip(HOME)
                .collect(),
            1,
            0.0,
        );
        e.enable(0.0).unwrap();
        e.registered = true;
        for i in 1..100 {
            let n = i as f64 / 10.0;
            e.action(WALK, true, n);
            e.joints(
                JOINT_NAMES
                    .into_iter()
                    .map(String::from)
                    .zip(HOME)
                    .collect(),
                i,
                n,
            );
            e.owner("", n);
            e.watchdog(n);
        }
        assert!(e.enabled);
        assert!(e.acquisition_at.is_none());
    }
    fn ready_engine() -> Engine {
        let mut e = Engine::new(&json!({}));
        e.action(WALK, true, 1.0);
        e.joints(
            JOINT_NAMES
                .into_iter()
                .map(String::from)
                .zip(HOME)
                .collect(),
            1,
            1.0,
        );
        e.owner("", 1.0);
        e.enable(1.0).unwrap();
        e.source_response(1001, 1.0);
        e
    }
    #[test]
    fn command_watchdog_is_independent_of_owner_and_joint_updates() {
        let mut e = ready_engine();
        e.set_velocity(
            Velocity {
                forward: 0.3,
                ..Velocity::default()
            },
            1.0,
        )
        .unwrap();
        let source = e.source.clone();
        e.owner(&source, 1.01);
        e.action(WALK, true, 1.251);
        e.joints(
            JOINT_NAMES
                .into_iter()
                .map(String::from)
                .zip(HOME)
                .collect(),
            2,
            1.251,
        );
        e.owner(&source, 1.251);
        e.watchdog(1.251);
        assert!(!e.enabled);
        assert_eq!(e.output(1.251), Velocity::default());
        assert!(e.fault.as_deref().unwrap().contains("command watchdog"));
    }
    #[test]
    fn takeover_and_late_owner_do_not_resume_or_delete_during_recovery() {
        let mut e = ready_engine();
        let source = e.source.clone();
        e.set_velocity(
            Velocity {
                forward: 0.3,
                ..Velocity::default()
            },
            1.0,
        )
        .unwrap();
        e.owner(&source, 1.01);
        e.owner("rc", 1.02);
        e.stop(1.03);
        e.owner(&source, 1.04);
        e.watchdog(1.12);
        assert!(!e.enabled);
        assert!(!e.stopping);
        assert!(e.source_operation.is_none());
        assert_eq!(e.output(1.12), Velocity::default());
    }
    #[test]
    fn stop_waits_for_zero_hold_before_deleting_own_source() {
        let mut e = ready_engine();
        e.stop(1.0);
        e.watchdog(1.079);
        assert!(e.source_operation.is_none());
        e.watchdog(1.081);
        assert_eq!(e.source_operation, Some(1003));
    }
    #[test]
    fn replayed_joint_frame_cannot_refresh_feedback_deadline() {
        let mut e = ready_engine();
        e.action(WALK, true, 1.251);
        e.owner("", 1.251);
        e.joints(
            JOINT_NAMES
                .into_iter()
                .map(String::from)
                .zip(HOME)
                .collect(),
            1,
            1.251,
        );
        e.watchdog(1.251);
        assert!(!e.enabled);
        assert_eq!(e.output(1.251), Velocity::default());
    }
}
