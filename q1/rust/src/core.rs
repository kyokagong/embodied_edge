use crate::sdk::{Backend, Velocity, WALK};
use serde_json::{json, Value};
use std::collections::VecDeque;

pub struct Controller {
    backend: Box<dyn Backend>,
    config: Value,
    enabled: bool,
    calibrated: bool,
    mode: String,
    status: String,
    owner: Option<String>,
    source: Option<String>,
    sequence: Option<u64>,
    received: Option<f64>,
    axes: [f64; 4],
    offsets: [f64; 4],
    buttons: [bool; 17],
    buttons_seen: bool,
    motion_latched: bool,
    rb_held_since: Option<f64>,
    rb_toggle_fired: bool,
    rb_wait_release: bool,
    source_was_active: bool,
    velocity: Velocity,
    last_gap_ms: Option<f64>,
    max_gap_ms: f64,
    telemetry: Value,
    last_stop: Option<Value>,
    events: VecDeque<Value>,
}
impl Controller {
    pub fn new(backend: Box<dyn Backend>, config: Value) -> Self {
        let mut core = Self {
            backend,
            config,
            enabled: false,
            calibrated: false,
            mode: "idle".into(),
            status: "等待输入，先校准，再使能".into(),
            owner: None,
            source: None,
            sequence: None,
            received: None,
            axes: [0.; 4],
            offsets: [0.; 4],
            buttons: [false; 17],
            buttons_seen: false,
            motion_latched: false,
            rb_held_since: None,
            rb_toggle_fired: false,
            rb_wait_release: true,
            source_was_active: false,
            velocity: Velocity::default(),
            last_gap_ms: None,
            max_gap_ms: 0.,
            telemetry: json!({}),
            last_stop: None,
            events: VecDeque::new(),
        };
        core.log("启动：默认禁用，尚未校准", 0.);
        core
    }

    fn setting(&self, name: &str) -> f64 {
        self.config[name].as_f64().unwrap()
    }
    fn button(&self, name: &str) -> usize {
        self.config["buttons"][name].as_u64().unwrap() as usize
    }
    fn fresh(&self, now: f64) -> bool {
        self.received
            .is_some_and(|at| now >= at && now - at <= self.setting("input_timeout_s"))
    }
    fn latch_mode(&self) -> bool {
        matches!(
            self.source.as_deref(),
            Some("browser_gamepad" | "linux_gamepad")
        ) && self.config["gamepad"]["rb_toggle_enabled"]
            .as_bool()
            .unwrap()
    }
    fn deadman(&self) -> bool {
        if self.latch_mode() {
            self.motion_latched
        } else {
            self.buttons[self.button("deadman")]
        }
    }
    fn clear_latch(&mut self) {
        self.motion_latched = false;
        self.rb_held_since = None;
        self.rb_toggle_fired = false;
        self.rb_wait_release = true;
    }
    fn log(&mut self, message: &str, now: f64) {
        if self.events.len() == 60 {
            self.events.pop_front();
        }
        self.events
            .push_back(json!({"time":format!("{now:.3}"), "event":message}));
    }
    fn zero(&mut self, now: f64) -> Result<(), String> {
        self.velocity = Velocity::default();
        self.backend.set_velocity(self.velocity, now)
    }
    fn trip(&mut self, message: &str, now: f64) {
        let first = self.enabled || self.mode != "fault";
        if first {
            self.log(message, now);
        }
        self.clear_latch();
        self.buttons_seen = false;
        self.enabled = false;
        self.calibrated = false;
        self.mode = "fault".into();
        self.status = message.into();
        let _ = self.zero(now);
        if let Err(error) = self.backend.stop(now) {
            self.log(&format!("停止通信失败：{error}"), now);
        }
        if first {
            let snapshot = self.backend.snapshot(now);
            self.last_stop = Some(json!({"reason":message,
                "input_age_ms":self.received.map(|at|round_ms((now-at).max(0.)*1000.)),
                "sequence":self.sequence, "last_gap_ms":self.last_gap_ms, "max_gap_ms":self.max_gap_ms,
                "joints_age_s":snapshot["joints_age_s"], "action_age_s":snapshot["action_age_s"],
                "owner_age_s":snapshot["owner_age_s"], "client":self.telemetry}));
        }
    }

    pub fn input(&mut self, payload: &Value, now: f64) -> Result<(), String> {
        if !payload.is_object() {
            return Err("Input must be an object".into());
        }
        let source = payload["source"].as_str().ok_or("Unknown input source")?;
        if !matches!(source, "simulator" | "browser_gamepad" | "linux_gamepad") {
            return Err("Unknown input source".into());
        }
        if self.backend.kind() == "ros2" && source == "simulator" {
            return Err("模拟输入不能控制真实 ROS2 后端".into());
        }
        let client = payload["client_id"]
            .as_str()
            .filter(|s| !s.is_empty() && s.chars().count() <= 80)
            .ok_or("Input requires client_id and nonnegative integer sequence")?;
        let sequence = payload["sequence"]
            .as_u64()
            .ok_or("Input requires client_id and nonnegative integer sequence")?;
        let values = payload["axes"]
            .as_array()
            .filter(|v| v.len() == 4)
            .ok_or("axes must contain 4 finite numbers")?;
        let mut axes = [0.; 4];
        for (target, value) in axes.iter_mut().zip(values) {
            *target = value
                .as_f64()
                .filter(|n| n.is_finite() && n.abs() <= 1.)
                .ok_or("Axes must be finite and in [-1,1]")?;
        }
        let values = payload["buttons"]
            .as_array()
            .filter(|v| v.len() <= 17)
            .ok_or("Buttons must be a boolean array, at most17")?;
        let mut buttons = [false; 17];
        for (target, value) in buttons.iter_mut().zip(values) {
            *target = value
                .as_bool()
                .ok_or("Buttons must be a boolean array, at most17")?;
        }
        let connected = payload["connected"]
            .as_bool()
            .ok_or("connected must be boolean")?;
        let same = self.owner.as_deref() == Some(client) && self.source.as_deref() == Some(source);
        if !connected && !same {
            return Ok(());
        }
        if !same && self.owner.is_some() && self.fresh(now) {
            return Err("另一控制页面正在持有输入，请先停止或等待输入过期".into());
        }
        if same && self.sequence.is_some_and(|seq| sequence <= seq) {
            return Err("重复或倒序输入不会刷新控制时限".into());
        }
        if !same {
            if self.enabled {
                self.trip("控制输入切换，需要重新校准和使能", now);
            }
            self.calibrated = false;
            self.clear_latch();
            self.last_gap_ms = None;
            self.max_gap_ms = 0.;
            self.telemetry = json!({});
            self.buttons_seen = false;
            self.owner = Some(client.into());
            self.source = Some(source.into());
        }
        let previous = self.buttons;
        self.sequence = Some(sequence);
        self.axes = axes;
        self.buttons = buttons;
        if !connected {
            let was_connected = self.received.is_some();
            self.clear_latch();
            self.received = None;
            self.buttons_seen = false;
            self.buttons = [false; 17];
            if was_connected {
                self.calibrated = false;
            }
            if self.enabled && was_connected {
                self.trip("手柄断开，已停止；重连后需重新校准和使能", now);
            }
            return Ok(());
        }
        if let Some(metrics) = payload["telemetry"].as_object() {
            let allowed = [
                "rtt_ms",
                "max_rtt_ms",
                "timeouts",
                "errors",
                "send_gap_ms",
                "skipped",
            ];
            self.telemetry = Value::Object(
                metrics
                    .iter()
                    .filter(|(key, value)| {
                        allowed.contains(&key.as_str())
                            && value
                                .as_f64()
                                .is_some_and(|v| v.is_finite() && (0. ..=1e9).contains(&v))
                    })
                    .map(|(key, value)| (key.clone(), value.clone()))
                    .collect(),
            );
        }
        if let Some(at) = self.received {
            self.last_gap_ms = Some(round_ms((now - at).max(0.) * 1000.));
            self.max_gap_ms = self.max_gap_ms.max(self.last_gap_ms.unwrap());
            if !self.fresh(now) {
                if self.enabled {
                    self.trip("输入曾过期，需重新校准和使能", now);
                } else {
                    self.clear_latch();
                }
            }
        }
        self.received = Some(now);
        if self.buttons_seen {
            let actions: &[&str] = if buttons[self.button("stop")] {
                &["stop"]
            } else {
                &["calibrate", "enable", "stand", "walk", "teleop"]
            };
            for name in actions {
                let index = self.button(name);
                if buttons[index] && !previous[index] {
                    if let Err(error) = self.action(name, Some(client), now) {
                        self.status = error.clone();
                        self.log(&error, now);
                    }
                }
            }
        }
        self.update_rb_toggle(now);
        if !buttons.iter().any(|pressed| *pressed)
            && axes
                .iter()
                .all(|v| v.abs() <= self.setting("neutral_threshold"))
        {
            self.buttons_seen = true;
        }
        if !self.deadman() {
            self.zero(now)?;
        }
        Ok(())
    }

    fn update_rb_toggle(&mut self, now: f64) {
        if !self.latch_mode() {
            return;
        }
        if !self.buttons[self.button("deadman")] {
            self.rb_held_since = None;
            self.rb_toggle_fired = false;
            self.rb_wait_release = false;
            return;
        }
        if self.buttons[self.button("stop")] {
            self.clear_latch();
            return;
        }
        if !self.buttons_seen {
            self.rb_wait_release = true;
        }
        if self.rb_wait_release || self.rb_toggle_fired {
            return;
        }
        let started = *self.rb_held_since.get_or_insert(now);
        if now - started < self.config["gamepad"]["rb_hold_s"].as_f64().unwrap() {
            return;
        }
        self.rb_toggle_fired = true;
        let action = if self.motion_latched {
            "disable"
        } else {
            "enable"
        };
        let owner = self.owner.clone();
        if let Err(error) = self.action(action, owner.as_deref(), now) {
            self.status = error.clone();
            self.log(&error, now);
        }
    }

    pub fn action(&mut self, name: &str, client: Option<&str>, now: f64) -> Result<(), String> {
        if !matches!(
            name,
            "enable"
                | "disable"
                | "calibrate"
                | "stand"
                | "walk"
                | "teleop"
                | "stop"
                | "clear_fault"
        ) {
            return Err("Unknown action".into());
        }
        if !matches!(name, "stop" | "disable")
            && client.is_some()
            && self.owner.is_some()
            && client != self.owner.as_deref()
            && self.fresh(now)
        {
            return Err("操作页面未持有控制输入".into());
        }
        match name {
            "stop" | "disable" => {
                self.clear_latch();
                self.buttons_seen = false;
                if name == "stop" {
                    self.calibrated = false;
                }
                self.enabled = false;
                self.mode = "idle".into();
                self.status = "已停止输出，保留原厂平衡控制".into();
                self.zero(now)?;
                self.backend.stop(now)?;
                self.log("软件停止；不替代物理急停", now);
            }
            "clear_fault" => {
                if self.enabled {
                    return Err("请先停止".into());
                }
                self.mode = "idle".into();
                self.status = "故障已清除，需要重新校准和使能".into();
                self.calibrated = false;
            }
            "calibrate" => {
                if !matches!(self.mode.as_str(), "idle" | "stand") {
                    return Err("先停止运动，再校准".into());
                }
                if !self.fresh(now) {
                    return Err("没有新鲜的手柄或 Pico 输入".into());
                }
                if self.buttons[self.button("deadman")]
                    || self.deadman()
                    || self
                        .axes
                        .iter()
                        .any(|v| v.abs() > self.setting("neutral_threshold"))
                {
                    return Err("校准前请松开使能保持键，并让摇杆回到中位".into());
                }
                self.offsets = self.axes;
                self.calibrated = true;
                self.status = "已校准摇杆中位／Pico 参考坐标；未改动电机零位".into();
            }
            "enable" => {
                if self.enabled {
                    return Ok(());
                }
                if self.latch_mode() && !self.buttons_seen {
                    return Err("请先松开所有按键并让摇杆回中，再重新使能".into());
                }
                if self.mode == "fault" {
                    return Err("先清除故障并重新校准".into());
                }
                if !self.calibrated || !self.fresh(now) {
                    return Err("必须先连接输入并完成校准".into());
                }
                if self.deadman()
                    || self
                        .axes
                        .iter()
                        .zip(self.offsets)
                        .any(|(a, b)| (a - b).abs() > self.setting("neutral_threshold"))
                {
                    return Err("使能前请松开保持键并让摇杆回中".into());
                }
                let snap = self.backend.snapshot(now);
                if snap["connected"] != true
                    || !snap["joints_age_s"].as_f64().is_some_and(|age| {
                        age <= self.config["ros2"]["feedback_timeout_s"].as_f64().unwrap()
                    })
                {
                    return Err("机器人未连接或反馈过期".into());
                }
                self.backend.enable(now)?;
                if let Err(error) = self.backend.request_mode("stand", now) {
                    let _ = self.backend.stop(now);
                    return Err(error);
                }
                self.enabled = true;
                self.mode = "stand".into();
                if self.latch_mode() {
                    self.motion_latched = true;
                    self.rb_held_since = None;
                    self.rb_wait_release = self.buttons[self.button("deadman")];
                }
                self.source_was_active = false;
                self.status = "已请求使能，等待原厂运控就绪".into();
            }
            "teleop" => return Err("Pico 上肢功能尚未迁移到 Rust；请使用 Python 控制服务".into()),
            _ => {
                if !self.enabled {
                    return Err("请先使能".into());
                }
                self.zero(now)?;
                self.backend.request_mode(name, now)?;
                self.mode = name.into();
                self.status = format!("模式切换中：{name}");
            }
        }
        Ok(())
    }

    fn axis_velocity(&self, value: f64, maximum: f64) -> f64 {
        let dead = self.setting("deadzone");
        if value.abs() <= dead {
            return 0.;
        }
        let amount = ((value.abs() - dead) / (1. - dead)).min(1.);
        let minimum = self.config["velocity"]["min_nonzero"].as_f64().unwrap();
        (minimum + amount * (maximum - minimum)).copysign(value)
    }

    pub fn tick(&mut self, now: f64) {
        if !self.enabled {
            return;
        }
        if !self.fresh(now) {
            self.trip("控制输入过期，已停止；需重新校准和使能", now);
            return;
        }
        let snap = self.backend.snapshot(now);
        if snap["connected"] != true
            || !snap["fault"].is_null()
            || !snap["joints_age_s"].as_f64().is_some_and(|age| {
                age <= self.config["ros2"]["feedback_timeout_s"].as_f64().unwrap()
            })
        {
            self.trip(
                &format!(
                    "机器人反馈异常：{}",
                    snap["fault"].as_str().unwrap_or("连接中断")
                ),
                now,
            );
            return;
        }
        let active = snap["source_active"] == true;
        if self.source_was_active && !active {
            self.trip("控制源被抢占，已停止，需重新使能", now);
            return;
        }
        self.source_was_active |= active;
        if snap["action"] != WALK || snap["action_running"] != true {
            if let Err(error) = self.zero(now) {
                self.trip(&format!("控制停止：{error}"), now);
            }
            self.status = format!("等待运控模式确认：{WALK}");
            return;
        }
        self.status = if self.mode == "walk" {
            "行走模式：按住保持键才发送速度"
        } else {
            "平衡站立：零速度"
        }
        .into();
        let velocity = if self.mode == "walk"
            && self.deadman()
            && (active || snap["source_can_acquire"] == true)
        {
            let axes: Vec<_> = self
                .axes
                .iter()
                .zip(self.offsets)
                .map(|(value, offset)| (value - offset).clamp(-1., 1.))
                .collect();
            Velocity {
                forward: self.axis_velocity(
                    -axes[1],
                    self.config["velocity"]["max_forward"].as_f64().unwrap(),
                ),
                lateral: self.axis_velocity(
                    -axes[0],
                    self.config["velocity"]["max_lateral"].as_f64().unwrap(),
                ),
                angular: self.axis_velocity(
                    -axes[2],
                    self.config["velocity"]["max_angular"].as_f64().unwrap(),
                ),
            }
        } else {
            Velocity::default()
        };
        self.velocity = velocity;
        if let Err(error) = self.backend.set_velocity(velocity, now) {
            self.trip(&format!("控制停止：{error}"), now);
        }
    }

    pub fn state(&self, now: f64) -> Value {
        json!({"backend":self.backend.kind(),"enabled":self.enabled,"calibrated":self.calibrated,"mode":self.mode,"status":self.status,
            "robot":self.backend.snapshot(now),
            "input":{"source":self.source,"age_ms":self.received.map(|at|round_ms((now-at).max(0.)*1000.)),
                "deadman":self.deadman(),"client_id":self.owner,"sequence":self.sequence,
                "motion_latched":self.motion_latched,"rb_hold_ms":self.rb_held_since.map_or(0.,|at|round_ms((now-at).max(0.)*1000.)),
                "link":{"last_gap_ms":self.last_gap_ms,"max_gap_ms":self.max_gap_ms,"client":self.telemetry,"last_stop":self.last_stop}},
            "controls":self.config["gamepad"],
            "capabilities":{"pico":false},
            "pico":{"connected":false,"calibrated":false,"age_ms":null,"source":null,"frame":null,"ik_error_m":null,"diagnostic":{}},
            "command":{"velocity":self.velocity,"upper":{}},"events":self.events,
            "limits":{"hardware_upper_confirmed":false,"velocity":self.config["velocity"]},
            "supermimic":{"available":false,"reason":"尚未接入 Rust 控制服务"}})
    }
}

fn round_ms(value: f64) -> f64 {
    (value * 10.).round() / 10.
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdk::MockBackend;
    use serde_json::json;

    fn pad(sequence: u64, buttons: &[usize], axes: [f64; 4]) -> Value {
        json!({"client_id":"test", "source":"browser_gamepad", "sequence":sequence,
            "axes":axes, "buttons":(0..17).map(|i|buttons.contains(&i)).collect::<Vec<_>>(), "connected":true})
    }

    #[test]
    fn long_hold_latches_then_released_rb_can_move_and_timeout_stops() {
        let mut core = Controller::new(
            Box::new(MockBackend::new()),
            crate::config::load(None).unwrap(),
        );
        core.input(&pad(1, &[], [0.; 4]), 0.).unwrap();
        core.action("calibrate", Some("test"), 0.).unwrap();
        core.input(&pad(2, &[5], [0.; 4]), 0.01).unwrap();
        for index in 1..=21 {
            core.input(&pad(index + 2, &[5], [0.; 4]), 0.01 + index as f64 * 0.05)
                .unwrap();
        }
        assert_eq!(core.state(1.06)["enabled"], true);
        assert_eq!(core.state(1.06)["mode"], "stand");
        core.input(&pad(30, &[], [0.; 4]), 1.07).unwrap();
        core.action("walk", Some("test"), 1.07).unwrap();
        core.input(&pad(31, &[], [0., -1., 0., 0.]), 1.08).unwrap();
        core.tick(1.08);
        assert_eq!(core.state(1.08)["command"]["velocity"]["forward"], 0.3);
        core.tick(1.331);
        assert_eq!(core.state(1.331)["enabled"], false);
        assert_eq!(core.state(1.331)["calibrated"], false);
        assert_eq!(core.state(1.331)["command"]["velocity"]["forward"], 0.);
    }

    #[test]
    fn repeated_frame_cannot_refresh_age() {
        let mut core = Controller::new(
            Box::new(MockBackend::new()),
            crate::config::load(None).unwrap(),
        );
        let frame = pad(1, &[], [0.; 4]);
        core.input(&frame, 0.).unwrap();
        assert!(core.input(&frame, 0.2).is_err());
        assert_eq!(core.state(0.2)["input"]["age_ms"], 200.);
    }

    #[test]
    fn stop_frame_wins_over_enable_and_late_action_waits_for_release() {
        let mut core = Controller::new(
            Box::new(MockBackend::new()),
            crate::config::load(None).unwrap(),
        );
        core.input(&pad(1, &[], [0.; 4]), 0.).unwrap();
        core.action("calibrate", Some("test"), 0.).unwrap();
        core.action("enable", Some("test"), 0.).unwrap();
        core.input(&pad(2, &[1, 9], [0.; 4]), 0.01).unwrap();
        assert_eq!(core.state(0.01)["enabled"], false);
        assert_eq!(core.state(0.01)["calibrated"], false);
        assert!(core.action("enable", Some("test"), 0.02).is_err());
        core.input(&pad(3, &[], [0.; 4]), 0.03).unwrap();
        core.action("calibrate", Some("test"), 0.03).unwrap();
        core.action("enable", Some("test"), 0.03).unwrap();
        assert_eq!(core.state(0.03)["enabled"], true);
    }

    #[test]
    fn late_frame_without_prior_tick_cannot_hide_timeout() {
        let mut core = Controller::new(
            Box::new(MockBackend::new()),
            crate::config::load(None).unwrap(),
        );
        core.input(&pad(1, &[], [0.; 4]), 0.).unwrap();
        core.action("calibrate", Some("test"), 0.).unwrap();
        core.action("enable", Some("test"), 0.).unwrap();
        core.input(&pad(2, &[], [0.; 4]), 0.251).unwrap();
        assert_eq!(core.state(0.251)["enabled"], false);
        assert_eq!(core.state(0.251)["calibrated"], false);
        assert_eq!(core.state(0.251)["mode"], "fault");
    }

    #[test]
    fn separate_page_cannot_take_a_fresh_input_owner() {
        let mut core = Controller::new(
            Box::new(MockBackend::new()),
            crate::config::load(None).unwrap(),
        );
        core.input(&pad(1, &[], [0.; 4]), 0.).unwrap();
        let mut other = pad(1, &[], [0.; 4]);
        other["client_id"] = json!("other");
        assert!(core.input(&other, 0.1).is_err());
        assert_eq!(core.state(0.1)["input"]["client_id"], "test");
        core.input(&other, 0.251).unwrap();
        assert_eq!(core.state(0.251)["input"]["client_id"], "other");
        assert_eq!(core.state(0.251)["calibrated"], false);
    }
}
