use super::{
    contract::{self, Hardware},
    model::Model,
    observation::{pelvis, Condition, Observation},
    reference,
};
use crate::lowlevel::{
    ipc::Socket,
    types::{clock, BridgeStatus, Request, Target},
};
use serde_json::{json, Value};
use std::{
    collections::{BTreeMap, VecDeque},
    path::Path,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    thread,
    time::{Duration, Instant},
};
pub struct State {
    pub bridge: BridgeStatus,
    pub mode: String,
    pub enabled: bool,
    pub shadow_only: bool,
    pub preview_running: bool,
    pub feedback_wait: Option<String>,
    pub calibrated: bool,
    pub command: [f32; 3],
    pub queue: VecDeque<String>,
    pub clip: Option<String>,
    pub frame: usize,
    pub epoch: u64,
    pub session: u64,
    pub owner: Option<String>,
    pub input_at: f64,
    pub input_seq: Option<u64>,
    pub axes: [f32; 4],
    pub offsets: [f32; 4],
    pub buttons: [bool; 17],
    pub rb_start: Option<f64>,
    pub rb_fired: bool,
    pub released: bool,
    pub selected: Option<String>,
    pub raw: [f32; 22],
    pub target: [f32; 22],
    pub inference_ms: f64,
    pub fault: Option<String>,
    pub steps: u64,
    pub parity: Value,
    pub events: VecDeque<Value>,
    pub requests: VecDeque<String>,
    pub prepare_at: Option<f64>,
    pub prepare_from: [f32; 22],
    pub stationary_steps: u32,
}
impl State {
    fn event(&mut self, s: &str) {
        if self.events.len() >= 60 {
            self.events.pop_front();
        }
        self.events.push_back(json!({"time":clock(),"event":s}));
    }
    fn stop(&mut self) {
        self.enabled = false;
        self.preview_running = false;
        self.mode = "stand".into();
        self.command = [0.; 3];
        self.queue.clear();
        self.clip = None;
        self.frame = 0;
        self.epoch += 1;
        self.rb_start = None;
        self.event(if self.shadow_only {
            "模型任务已停止；本体保护独立运行"
        } else {
            "请求受控回站；站立执行器控制仍可继续"
        });
    }
}
pub struct Runtime {
    pub state: Arc<Mutex<State>>,
    pub hardware: Hardware,
    pub clips: Arc<BTreeMap<String, reference::Clip>>,
    pub shadow_only: bool,
    pub ort_threads: usize,
    quit: Arc<AtomicBool>,
    worker: Option<thread::JoinHandle<()>>,
}
#[derive(Clone, Copy)]
pub struct RuntimeOptions {
    pub shadow_only: bool,
    pub ort_threads: usize,
}
impl Runtime {
    pub fn new(
        root: &Path,
        profile: Hardware,
        socket: &Path,
        options: RuntimeOptions,
    ) -> Result<Self, String> {
        let root = root.canonicalize().map_err(|e| e.to_string())?;
        let contract = contract::load(&root)?;
        let clips = Arc::new(reference::load(&root)?);
        let mut model = Model::load_with_threads(&root.join("policy.onnx"), options.ort_threads)?;
        let parity = model.golden(&root.join("golden.json"))?;
        let state = Arc::new(Mutex::new(State {
            bridge: BridgeStatus::default(),
            mode: "stand".into(),
            enabled: false,
            shadow_only: options.shadow_only,
            preview_running: false,
            feedback_wait: None,
            calibrated: false,
            command: [0.; 3],
            queue: VecDeque::new(),
            clip: None,
            frame: 0,
            epoch: 0,
            session: rand::random(),
            owner: None,
            input_at: 0.,
            input_seq: None,
            axes: [0.; 4],
            offsets: [0.; 4],
            buttons: [false; 17],
            rb_start: None,
            rb_fired: false,
            released: false,
            selected: None,
            raw: [0.; 22],
            target: [0.; 22],
            inference_ms: 0.,
            fault: None,
            steps: 0,
            parity,
            events: VecDeque::new(),
            requests: VecDeque::new(),
            prepare_at: None,
            prepare_from: [0.; 22],
            stationary_steps: 0,
        }));
        let quit = Arc::new(AtomicBool::new(false));
        let end = quit.clone();
        let shared = state.clone();
        let hw = profile.clone();
        let refs = clips.clone();
        let socket = socket.to_owned();
        let worker = thread::Builder::new()
            .name("q1-policy-50hz".into())
            .spawn(move || {
                let mut connection: Option<Socket> = None;
                let mut history = Observation::default();
                let mut previous = [0.; 22];
                let mut sent: VecDeque<(u64, [f32; 22])> = VecDeque::new();
                let mut seq = 0;
                let mut step = 0;
                let mut deadline = Instant::now();
                let mut next_connect = 0.;
                while !end.load(Ordering::Acquire) {
                    let now = clock();
                    let mut ipc_error = false;
                    if connection.is_none() && now >= next_connect {
                        next_connect = now + 1.;
                        if let Ok(s) = Socket::connect(&socket) {
                            connection = Some(s);
                        }
                    }
                    if let Some(s) = &connection {
                        for _ in 0..64 {
                            match s.receive::<BridgeStatus>() {
                                Ok(Some(b)) => {
                                    let mut a = shared.lock().unwrap();
                                    if a.bridge.session.is_some() && b.session != a.bridge.session {
                                        a.stop();
                                        a.calibrated = false;
                                        history.reset();
                                        previous = [0.; 22];
                                        sent.clear();
                                        step = 0;
                                    }
                                    if let Some(fault) = b.fault.as_ref().filter(|_| !options.shadow_only) {
                                        a.fault.get_or_insert_with(|| fault.clone());
                                        if a.enabled { a.stop(); }
                                    }
                                    a.bridge = b;
                                }
                                Ok(None) => break,
                                Err(_) => {
                                    ipc_error = true;
                                    break;
                                }
                            }
                        }
                    }
                    if ipc_error {
                        connection = None;
                        let mut a = shared.lock().unwrap();
                        a.stop();
                        a.bridge = BridgeStatus::default();
                        a.fault = Some("真实ROS桥连接中断，需要检查后重新建立会话".into());
                    }
                    let mut a = shared.lock().unwrap();
                    if a.enabled && now - a.input_at > 0.25 {
                        a.stop();
                        a.calibrated = false;
                        a.owner = None;
                        a.released = false;
                    }
                    if options.shadow_only && a.mode == "walk" && now - a.input_at > 0.25 {
                        a.command = [0.; 3];
                    }
                    if let Some(accepted) = a.bridge.accepted {
                        while sent.front().is_some_and(|(seq, _)| *seq <= accepted) {
                            previous = sent.pop_front().unwrap().1;
                        }
                    }
                    let error = a
                        .bridge
                        .sensors
                        .validate(now, hw.state_timeout_s, hw.sensor_skew_s)
                        .err();
                    if error.is_none() && a.bridge.sensors.imu_frame == hw.imu_frame {
                        a.feedback_wait = None;
                        let sensor = a.bridge.sensors.clone();
                        let epoch = a.epoch;
                        let session = a.session;
                        let active = !options.shadow_only && a.bridge.session == Some(session) && a.bridge.fault.is_none();
                        let stationary = sensor.root_velocity.is_some_and(|v| {
                            v.iter().map(|x| x * x).sum::<f32>() < 0.08f32.powi(2)
                        }) && (0. ..=hw.state_timeout_s).contains(&(now - sensor.root_velocity_at))
                            && (sensor.root_velocity_stamp - sensor.joint_stamp).abs() < 0.1
                            && sensor.omega.iter().map(|x| x * x).sum::<f32>() < 0.2f32.powi(2)
                            && pelvis(&sensor, &hw).is_ok_and(|r| r[2][2] > 0.95);
                        a.stationary_steps = if stationary { (a.stationary_steps + 1).min(25) } else { 0 };
                        if !options.shadow_only && a.enabled && !active {
                            if let Some(s) = &connection {
                                if s.send(&Request::Arm { session }).is_err() {
                                    a.fault = Some("无法请求HAL执行权".into());
                                    a.stop();
                                }
                            }
                        }
                        let prep = a.prepare_at;
                        let prepare_from = a.prepare_from;
                        let mode = a.mode.clone();
                        let command = if (a.enabled || a.preview_running) && mode == "walk" {
                            a.command
                        } else {
                            [0.; 3]
                        };
                        let clip = a.clip.clone();
                        let frame = a.frame;
                        drop(a);
                        let reference = clip
                            .as_ref()
                            .and_then(|id| refs.get(id))
                            .map(|c| c.frame(frame));
                        let t = Instant::now();
                        let result = history
                            .build(
                                &sensor,
                                &hw,
                                &contract,
                                previous,
                                Condition {
                                    command,
                                    step,
                                    reference,
                                },
                            )
                            .and_then(|x| model.infer(&x));
                        let ms = t.elapsed().as_secs_f64() * 1000.;
                        let completed = clock();
                        let mut a = shared.lock().unwrap();
                        if a.epoch != epoch {
                            continue;
                        }
                        match result {
                            Ok(raw) => {
                                let mut raw = raw.map(|x| x.clamp(-20., 20.));
                                let mut target = std::array::from_fn(|i| {
                                    hw.zeros[i]
                                        + hw.signs[i]
                                            * (contract.action.home[i] + 0.25 * raw[i]
                                                - hw.encoder_bias[i])
                                });
                                if active {
                                    if let Some(start) = prep {
                                        // Preparation is an explicit, supervised trajectory, before policy control.
                                        let alpha = ((now - start) / 5.).clamp(0., 1.) as f32;
                                        for i in 0..22 {
                                            let home = hw.zeros[i]
                                                + hw.signs[i]
                                                    * (contract.action.home[i]
                                                        - hw.encoder_bias[i]);
                                            target[i] =
                                                prepare_from[i] + (home - prepare_from[i]) * alpha;
                                            raw[i] = ((hw.signs[i] * (target[i] - hw.zeros[i])
                                                - contract.action.home[i]
                                                + hw.encoder_bias[i])
                                                / 0.25)
                                                .clamp(-20., 20.);
                                        }
                                        if completed - start > 5.
                                            && (0..22).all(|i| {
                                                (hw.signs[i] * (sensor.q[i] - hw.zeros[i])
                                                    - contract.action.home[i]
                                                    + hw.encoder_bias[i])
                                                    .abs()
                                                    < hw.max_start_error_rad
                                            })
                                        {
                                            a.prepare_at = None;
                                            history.reset();
                                            previous = raw;
                                            step = 0;
                                            a.event(
                                                "初始化姿态通过，使用真实状态重新初始化策略历史",
                                            );
                                        } else if completed - start > 20. {
                                            a.fault = Some("初始化姿态未在期限内收敛".into());
                                            a.stop();
                                        }
                                    }
                                }
                                a.raw = raw;
                                a.target = target;
                                a.inference_ms = ms;
                                a.steps += 1;
                                if ms >= 20. && active {
                                    a.fault = Some("策略超过20ms截止时间".into());
                                    a.stop();
                                }
                                if active && a.fault.is_none() {
                                    seq += 1;
                                    let target = Target {
                                        session,
                                        epoch,
                                        sequence: seq,
                                        generated: now,
                                        observation: sensor.joints_at.min(sensor.imu_at),
                                        expires: now + hw.target_timeout_s,
                                        operator_expires: a.input_at + 0.25,
                                        moving: command.iter().any(|x| *x != 0.)
                                            || clip.is_some()
                                            || prep.is_some(),
                                        q: target,
                                        raw,
                                        gain_scale: 1.,
                                    };
                                    if let Some(s) = &connection {
                                        match s.send(&Request::Target(Box::new(target))) {
                                            Ok(()) => {
                                                sent.push_back((seq, raw));
                                                if sent.len() > 32 {
                                                    a.fault = Some("策略确认积压，停止发送并等待本体保护".into());
                                                }
                                            }
                                            Err(e) => {
                                                a.fault = Some(format!("IPC target send: {e}"));
                                                a.stop();
                                            }
                                        }
                                    }
                                }
                                step += 1;
                                if (active || a.preview_running) && a.fault.is_none() && a.prepare_at.is_none() {
                                    if let Some(id) = &clip {
                                        if frame + 1 >= refs[id].goals.nrows() {
                                            a.mode = "stand".into();
                                            a.command = [0.; 3];
                                            a.clip = None;
                                            a.frame = 0;
                                            a.epoch += 1;
                                            a.stationary_steps = 0;
                                            a.event(if options.shadow_only { "舞蹈推理任务完成，返回站立策略；队列保留" } else { "舞蹈片段结束，已请求回站；保留队列，下一段须重新准入并显式开始" });
                                        } else {
                                            a.frame += 1;
                                        }
                                    }
                                }
                            }
                            Err(e) => {
                                a.fault = Some(e);
                                a.stop();
                            }
                        }
                    } else {
                        a.stationary_steps = 0;
                        if options.shadow_only {
                            // Observation-only tasks keep their reference frame paused.
                            // No inference is performed on stale feedback and no HAL target is sent.
                            a.feedback_wait = error;
                        } else if a.enabled {
                            a.fault = error;
                            a.stop();
                        }
                        drop(a);
                    }
                    deadline += Duration::from_millis(20);
                    let instant = Instant::now();
                    if instant > deadline + Duration::from_millis(20) {
                        deadline = instant;
                    }
                    thread::sleep(deadline.saturating_duration_since(instant));
                }
            })
            .map_err(|e| e.to_string())?;
        Ok(Self {
            state,
            hardware: profile,
            clips,
            shadow_only: options.shadow_only,
            ort_threads: options.ort_threads,
            quit,
            worker: Some(worker),
        })
    }
    pub fn snapshot(&self) -> Value {
        let a = self.state.lock().unwrap();
        let now = clock();
        let mut blockers = self.hardware.blockers();
        blockers.extend(a.bridge.blockers.clone());
        blockers.sort();
        blockers.dedup();
        if let Err(e) = a.bridge.sensors.validate(
            now,
            self.hardware.state_timeout_s,
            self.hardware.sensor_skew_s,
        ) {
            blockers.push(e);
        }
        json!({"controller":"supermimic","backend":"ros2","model":{"iteration":22100,"motion_qualified":false},"mode":a.mode,"enabled":a.enabled,"calibrated":a.calibrated,"shadow_only":self.shadow_only,"preview_running":a.preview_running,"feedback_wait":a.feedback_wait,"model_execution_state":if self.shadow_only {if a.preview_running && a.feedback_wait.is_some() {"WAITING_FEEDBACK"} else if a.preview_running {"OBSERVATION_RUNNING"} else {"OBSERVATION_IDLE"}} else {a.bridge.mode.as_str()},"deployment":{"inference_location":if cfg!(target_arch="aarch64") {"Q1"} else {"Mini PC"},"architecture":std::env::consts::ARCH,"ort_threads":self.ort_threads,"sensor_transport":"local Unix IPC → Rust ROS2"},"model_actuator_active":!self.shadow_only&&a.bridge.session==Some(a.session)&&a.bridge.fault.is_none(),"model_fault":a.fault,"guard_fault":a.bridge.fault,
        "execution_state":a.bridge.mode,"execution_scope":if self.hardware.supported_motion==["air_hold"] {"left_elbow_commissioning"} else {"whole_body"},"actuator_active":a.bridge.session.is_some(),"blockers":blockers,"fault":a.fault.as_ref().or(a.bridge.fault.as_ref()),"bridge":a.bridge,"input_age_ms":(now-a.input_at)*1000.,"command":a.command,"raw_action":a.raw,"target":a.target,"inference_ms":a.inference_ms,"policy_steps":a.steps,
        "dance":{"id":a.clip,"selected":a.selected,"frame":a.frame,"queue":a.queue,"stationary_steps":a.stationary_steps,"entry_ready":a.stationary_steps>=25,"auto_queue":false},"sensor_age_ms":(now-a.bridge.sensors.joints_at.min(a.bridge.sensors.imu_at))*1000.,"parity":a.parity,"events":a.events,"joint_names":contract::NAMES,
        "capabilities":{"hardware":true,"pico":false,"full_body":true,"imu":a.bridge.sensors.imu_valid,"dance":true},"receive_hz":null})
    }
    pub fn input(&self, v: &Value) -> Result<(), String> {
        let now = clock();
        let mut a = self.state.lock().unwrap();
        let id = v["client_id"]
            .as_str()
            .filter(|s| !s.is_empty() && s.len() <= 80)
            .ok_or("Invalid client_id")?;
        if v["source"] != "browser_gamepad" {
            return Err("真实本体只接受浏览器物理手柄输入".into());
        }
        if a.owner.as_deref() != Some(id) {
            if a.owner.is_some() && now - a.input_at < 0.25 {
                return Err("另一操作者持有输入租约".into());
            }
            a.stop();
            a.owner = Some(id.into());
            a.input_seq = None;
            a.calibrated = false;
            a.released = false;
        }
        let seq = v["sequence"].as_u64().ok_or("Missing sequence")?;
        if a.input_seq.is_some_and(|n| seq <= n) {
            return Err("Stale input sequence".into());
        }
        let axes: Vec<f32> =
            serde_json::from_value(v["axes"].clone()).map_err(|e| e.to_string())?;
        let buttons: Vec<bool> =
            serde_json::from_value(v["buttons"].clone()).map_err(|e| e.to_string())?;
        if axes.len() != 4
            || buttons.len() != 17
            || !axes.iter().all(|x| x.is_finite() && x.abs() <= 1.)
        {
            return Err("Invalid gamepad frame".into());
        }
        let old = a.buttons;
        let old_at = a.input_at;
        a.input_at = now;
        a.input_seq = Some(seq);
        a.axes.copy_from_slice(&axes);
        a.buttons.copy_from_slice(&buttons);
        if now - old_at > 0.25 && a.enabled {
            a.stop();
            a.calibrated = false;
        }
        if a.buttons[1] {
            a.stop();
            return Ok(());
        }
        let neutral = axes.iter().all(|x| x.abs() < 0.18);
        if !buttons.iter().any(|x| *x) && neutral {
            a.released = true;
        }
        if a.buttons[8] && !old[8] && !a.enabled && neutral && a.released {
            a.offsets.copy_from_slice(&axes);
            a.calibrated = true;
        }
        let mut action = None;
        if a.buttons[5] {
            let t = *a.rb_start.get_or_insert(now);
            if now - t >= 1. && !a.rb_fired {
                a.rb_fired = true;
                action = Some(if a.enabled || a.preview_running {
                    "stop"
                } else {
                    "enable"
                });
            }
        } else {
            a.rb_start = None;
            a.rb_fired = false;
        }
        for (i, name) in [(9, "enable"), (0, "stand"), (2, "walk"), (3, "dance")] {
            if a.buttons[i] && !old[i] && action != Some("stop") {
                action = Some(name);
            }
        }
        let dead = |x: f32| {
            if x.abs() <= 0.18 {
                0.
            } else {
                x.signum() * ((x.abs() - 0.18) / 0.82).min(1.)
            }
        };
        let forward = dead(-(axes[1] - a.offsets[1]));
        a.command = [
            forward * if forward >= 0. { 0.35 } else { 0.25 },
            -dead(axes[0] - a.offsets[0]) * 0.18,
            -dead(axes[2] - a.offsets[2]) * 0.6,
        ];
        drop(a);
        if let Some(name) = action {
            self.action(name, None)?;
        }
        Ok(())
    }
    pub fn action(&self, name: &str, payload: Option<&Value>) -> Result<(), String> {
        let mut a = self.state.lock().unwrap();
        let now = clock();
        if matches!(name, "stop" | "disable" | "cancel") {
            a.stop();
            return Ok(());
        }
        if name == "clear_fault" {
            if !self.shadow_only && a.bridge.session.is_some() {
                return Err("HAL故障需检查本体后重启桥，不能网页直接清除".into());
            }
            a.fault = None;
            a.calibrated = false;
            return Ok(());
        }
        if !self.shadow_only && now - a.input_at > 0.25 {
            return Err("需要新鲜的物理手柄输入".into());
        }
        match name {
            "calibrate" => {
                if a.enabled || a.axes.iter().any(|x| x.abs() > 0.18) || !a.released {
                    return Err("校准前释放全部按键并回中".into());
                }
                a.offsets = a.axes;
                a.calibrated = true;
            }
            "enable" => {
                if self.shadow_only {
                    a.bridge.sensors.validate(
                        now,
                        self.hardware.state_timeout_s,
                        self.hardware.sensor_skew_s,
                    )?;
                    a.fault = None;
                    a.preview_running = true;
                    a.mode = "stand".into();
                    a.clip = None;
                    a.frame = 0;
                    a.epoch += 1;
                    a.event("Q1真实观测推理任务启动；输出候选目标");
                    return Ok(());
                }
                let blockers = self.hardware.blockers();
                if !blockers.is_empty() {
                    return Err(blockers.join("；"));
                }
                if !a.calibrated || !a.released || a.axes.iter().any(|x| x.abs() > 0.18) {
                    return Err("请先回中、全释放并校准".into());
                }
                if a.fault.is_some() || a.bridge.fault.is_some() {
                    return Err("先处理故障".into());
                }
                a.bridge.sensors.validate(
                    now,
                    self.hardware.state_timeout_s,
                    self.hardware.sensor_skew_s,
                )?;
                if !a.bridge.blockers.is_empty() {
                    return Err(a.bridge.blockers.join("；"));
                }
                if a.bridge.session.is_none() {
                    a.prepare_at = Some(now);
                    a.prepare_from = a.bridge.sensors.q;
                }
                a.enabled = true;
                a.mode = "stand".into();
                a.epoch += 1;
                a.event("请求支撑下初始化/站立，等待HAL确认");
            }
            "stand" => {
                a.mode = "stand".into();
                a.command = [0.; 3];
                a.clip = None;
                a.frame = 0;
                a.epoch += 1;
            }
            "walk" => {
                if self.shadow_only {
                    a.bridge.sensors.validate(
                        now,
                        self.hardware.state_timeout_s,
                        self.hardware.sensor_skew_s,
                    )?;
                }
                if !self.shadow_only
                    && (!a.enabled
                        || a.prepare_at.is_some()
                        || !self.hardware.supported_motion.iter().any(|m| m == "walk"))
                {
                    return Err("行走尚未通过准入或未完成站立初始化".into());
                }
                a.mode = "walk".into();
                if self.shadow_only {
                    a.preview_running = true;
                }
                a.clip = None;
                a.epoch += 1;
            }
            "select" => {
                let id = payload
                    .and_then(|v| v["clip_id"].as_str())
                    .ok_or("Missing clip id")?;
                if !self.clips.contains_key(id) || id == "standing" {
                    return Err("Unknown dance".into());
                }
                a.selected = Some(id.into());
            }
            "queue" => {
                let id = payload
                    .and_then(|v| v["clip_id"].as_str())
                    .ok_or("Missing clip id")?;
                if !self.clips.contains_key(id) || id == "standing" || a.queue.len() >= 8 {
                    return Err("Invalid/full dance queue".into());
                }
                a.queue.push_back(id.into());
            }
            "clear_queue" => a.queue.clear(),
            "remove_queue" => {
                let index = payload
                    .and_then(|v| v["index"].as_u64())
                    .ok_or("Missing queue index")? as usize;
                if a.queue.remove(index).is_none() {
                    return Err("Queue index out of range".into());
                }
            }
            "dance" => {
                if self.shadow_only {
                    a.bridge.sensors.validate(
                        now,
                        self.hardware.state_timeout_s,
                        self.hardware.sensor_skew_s,
                    )?;
                }
                let id = payload
                    .and_then(|v| v["clip_id"].as_str())
                    .map(String::from)
                    .or_else(|| a.selected.clone())
                    .or_else(|| a.queue.front().cloned())
                    .ok_or("请选择舞蹈")?;
                if !self.shadow_only
                    && (!a.enabled
                        || a.prepare_at.is_some()
                        || a.mode != "stand"
                        || !self.hardware.supported_motion.iter().any(|m| m == &id))
                {
                    return Err("该舞蹈及入场/退出尚未完成验收，或尚未站立就绪".into());
                }
                let clip = self.clips.get(&id).ok_or("Unknown dance")?;
                if !self.shadow_only && a.stationary_steps < 25 {
                    return Err(
                        "舞蹈准入需连续25个策略步静止；当前根速度估计或姿态条件未就绪".into(),
                    );
                }
                let r = pelvis(&a.bridge.sensors, &self.hardware)?;
                let rr = clip.frame(0).0;
                let trace = (0..3)
                    .flat_map(|i| (0..3).map(move |j| r[i][j] * rr[i][j]))
                    .sum::<f32>();
                if !self.shadow_only
                    && (trace < 2.97 || a.bridge.sensors.qd.iter().any(|v| v.abs() > 0.08))
                {
                    return Err("参考起始姿态/静止条件不符；禁止任意航向入舞".into());
                }
                if a.queue.front() == Some(&id) {
                    a.queue.pop_front();
                }
                a.clip = Some(id);
                if self.shadow_only {
                    a.preview_running = true;
                    a.fault = None;
                    a.event("已启动舞蹈真实观测推理任务");
                }
                a.frame = 0;
                a.mode = "dance".into();
                a.epoch += 1;
            }
            _ => return Err("Unknown action".into()),
        }
        Ok(())
    }
}
impl Drop for Runtime {
    fn drop(&mut self) {
        self.quit.store(true, Ordering::Release);
        if let Some(t) = self.worker.take() {
            let _ = t.join();
        }
    }
}
