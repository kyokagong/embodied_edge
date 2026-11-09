#![recursion_limit = "256"]
//! Real, support-supervised standing. ROS/HAL writes belong to the independent q1-rt.
use axum::{
    extract::State,
    http::{HeaderMap, StatusCode},
    response::{Html, IntoResponse},
    routing::{get, post},
    Json, Router,
};
use clap::Parser;
use q1_edge::{
    lowlevel::{
        ipc::Socket,
        types::{clock, BridgeStatus, Request, Sensors, Target},
    },
    supermimic::{
        action,
        contract::{self, Hardware},
        flight::Recorder,
        model::Model,
        observation::{pelvis, Condition, Observation},
        reference,
    },
};
use serde::Serialize;
use serde_json::{json, Value};
use std::{
    collections::VecDeque,
    path::PathBuf,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    thread,
    time::{Duration, Instant},
};

#[derive(Parser)]
struct Options {
    #[arg(long)]
    model_dir: PathBuf,
    #[arg(long)]
    hardware_profile: PathBuf,
    #[arg(long)]
    socket: PathBuf,
    #[arg(long)]
    private_trace_file: Option<PathBuf>,
    #[arg(long, default_value_t = 8768)]
    port: u16,
    /// Replay recorded real policy targets; no IPC or hardware commands.
    #[arg(long)]
    audit_recorded_actions: Option<PathBuf>,
}
#[derive(Clone, Serialize)]
struct PolicyContext {
    enabled_at_sampling: bool,
    phase_at_sampling: String,
    mode: u8,
    dance: Option<String>,
    reference_frame: Option<usize>,
    reference_rotation: Option<[[f32; 3]; 3]>,
}
#[derive(Serialize)]
struct TargetStages {
    context: Option<PolicyContext>,
    model_q: Option<[f32; 22]>,
    raw_clip_limited: [bool; 22],
    amplitude_limited: [bool; 22],
    mechanical_limited: [bool; 22],
    desired_q: [f32; 22],
    previous_target_q: [f32; 22],
    step_budget_rad: [f32; 22],
    slew_limited: [bool; 22],
    desired_to_sent_delta_rad: [f32; 22],
    dt_s: f32,
    prediction_reused: bool,
    prediction_age_ms: Option<f64>,
    observation_to_target_ms: f64,
    joint_receive_age_ms: f64,
    imu_receive_age_ms: f64,
}
struct Job {
    built_at: f64,
    context: PolicyContext,
    id: u64,
    x: Vec<f32>,
    observed: f64,
    epoch: u64,
    sensors: Box<Sensors>,
    previous: [f32; 22],
}
struct Prediction {
    context: PolicyContext,
    id: u64,
    raw: [f32; 22],
    observed: f64,
    epoch: u64,
    ms: f64,
    completed: f64,
}
#[derive(Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
enum ActorRecord {
    Input {
        at: f64,
        id: u64,
        epoch: u64,
        observed: f64,
        input: Vec<f32>,
        sensors: Box<Sensors>,
        previous_raw: [f32; 22],
        context: PolicyContext,
    },
    Inference {
        id: u64,
        epoch: u64,
        observed: f64,
        started: f64,
        completed: f64,
        ms: f64,
        candidate_raw: [f32; 22],
        native_action: Box<action::NativeAction>,
        context: PolicyContext,
        queue_wait_ms: f64,
        sensor_to_result_ms: f64,
    },
    Target {
        at: f64,
        observation_id: Option<u64>,
        target: Box<Target>,
        candidate_raw: [f32; 22],
        model_limited: [bool; 22],
        stages: Box<TargetStages>,
        phase: String,
        bridge: Box<BridgeStatus>,
    },
    Event {
        at: f64,
        action: String,
        request_id: String,
        context: Value,
        phase: String,
        error: Option<String>,
    },
    State {
        at: f64,
        phase: String,
        epoch: u64,
        enabled: bool,
        standing_verified: bool,
        dance: Option<String>,
        dance_frame: usize,
        returning_to_stand: bool,
        fault: Option<String>,
        bridge: Box<BridgeStatus>,
    },
}
struct Core {
    bridge: BridgeStatus,
    phase: String,
    enabled: bool,
    epoch: u64,
    session: u64,
    began: f64,
    origin: [f32; 22],
    target: [f32; 22],
    fault: Option<String>,
    requests: VecDeque<String>,
    owner: Option<String>,
    operator_at: f64,
    operator_sequence: Option<u64>,
    input_count: u64,
    model_count: u64,
    inference_ms: f64,
    raw: [f32; 22],
    previous_policy_raw: [f32; 22],
    model_saturation: [bool; 22],
    blend_at: Option<f64>,
    stable_at: Option<f64>,
    verified: bool,
    events: VecDeque<String>,
    parity: Value,
    dances: Vec<Value>,
    dance: Option<String>,
    dance_frame: usize,
    dance_limit: usize,
    stand_pose: [f32; 22],
    reset_history: bool,
}
impl Core {
    fn event(&mut self, message: &str) {
        if self.events.len() >= 40 {
            self.events.pop_front();
        }
        self.events.push_back(message.into());
    }
    fn fail(&mut self, error: String) {
        if self.enabled {
            self.event(&error);
        }
        self.enabled = false;
        self.dance = None;
        self.verified = false;
        self.phase = "FAULT".into();
        self.fault = Some(error);
    }
}
struct App {
    core: Arc<Mutex<Core>>,
    hardware: Hardware,
    profile_path: PathBuf,
    profile_sha256: String,
    requests: Arc<Mutex<VecDeque<Request>>>,
    flight: Recorder<ActorRecord>,
}
fn snapshot(app: &App) -> Value {
    let c = app.core.lock().unwrap();
    let now = clock();
    let home = app.hardware.stand_trial.as_ref().unwrap().home;
    let errors: [f32; 22] = std::array::from_fn(|i| c.bridge.sensors.q[i] - home[i]);
    let tilt = pelvis(&c.bridge.sensors, &app.hardware)
        .ok()
        .map(|r| r[2][2].clamp(-1., 1.).acos());
    json!({"controller":"supermimic_stand","inference_location":"Q1","architecture":std::env::consts::ARCH,
        "phase":c.phase,"enabled":c.enabled,"standing_verified":c.verified,"fault":c.fault,
        "guard_fault":c.bridge.fault,"bridge":c.bridge,"home":home,"home_error":errors,
        "target":c.target,"raw_action":c.raw,"tilt_rad":tilt,"feedback_age_ms":(now-c.bridge.sensors.joints_at.min(c.bridge.sensors.imu_at))*1000.,
        "input_count":c.input_count,"model_count":c.model_count,"input_shape":[1,1376],"history_frames":10,"output_shape":[1,22],
        "inference_ms":c.inference_ms,"joint_names":contract::NAMES,"operator_alive":now-c.operator_at<0.6,
        "events":c.events,"parity":c.parity,"supported_actions":["stand","hold","release","reset","play_dance","cancel_dance"],"trial":true,
        "protection_mode":if app.hardware.stand_trial.as_ref().unwrap().soft_limits {"continuous_saturation"} else {"stop_on_limit"},
        "ground_support_verified":false,"model_saturation":c.model_saturation,
        "stand_elapsed_s":if c.enabled {(now-c.began).max(0.)}else{0.},"prepare_seconds":app.hardware.stand_trial.as_ref().unwrap().prepare_seconds,
        "stand_task_autonomous":true,"dances":c.dances,"dance":c.dance,"dance_frame":c.dance_frame,"dance_elapsed_s":c.dance_frame as f64*0.02,"dance_playing":c.dance.is_some(),"returning_to_stand":false,"stand_pose":c.stand_pose,
        "policy_output_mode":"model_native","policy_hz":50,"policy_ready":c.enabled && c.blend_at.is_some_and(|at| now-at>=3.),"condition_switch":"preserve_history","cancel_behavior":"zero_velocity_condition",
        "policy_velocity_limits_rad_s":app.hardware.stand_trial.as_ref().unwrap().policy_velocity_limit,
        "measured_velocity_limits_rad_s":app.hardware.stand_trial.as_ref().unwrap().measured_velocity_limit,
        "prepare_velocity_limits_rad_s":app.hardware.stand_trial.as_ref().unwrap().velocity_limit,
        "target_step_limit_rad":app.hardware.max_step_rad,
        "effort_limits_nm":app.hardware.stand_trial.as_ref().unwrap().effort_limit,
        "config":{"path":app.profile_path,"sha256":app.profile_sha256,"apply_requires_released":true},
        "flight_log":{"directory":app.flight.directory,"written":app.flight.stats.written.load(Ordering::Relaxed),"dropped":app.flight.stats.dropped.load(Ordering::Relaxed),"failed":app.flight.stats.failed.load(Ordering::Relaxed)}})
}
fn reject(code: StatusCode, error: &str) -> axum::response::Response {
    (code, Json(json!({"ok":false,"error":error}))).into_response()
}
async fn action(
    State(app): State<Arc<App>>,
    headers: HeaderMap,
    Json(v): Json<Value>,
) -> axum::response::Response {
    let host = headers
        .get("host")
        .and_then(|x| x.to_str().ok())
        .unwrap_or("");
    let allowed = host
        .parse::<axum::http::uri::Authority>()
        .ok()
        .is_some_and(|a| matches!(a.host(), "192.168.54.41" | "127.0.0.1" | "localhost"));
    if !allowed {
        return reject(StatusCode::FORBIDDEN, "Host不允许");
    }
    if headers.get("origin").is_some_and(|x| {
        x.to_str().ok() != Some(&format!("http://{host}"))
            && x.to_str().ok() != Some(&format!("https://{host}"))
    }) {
        return reject(StatusCode::FORBIDDEN, "Origin不允许");
    }
    let Some(client) = v["client_id"]
        .as_str()
        .filter(|s| !s.is_empty() && s.len() < 100)
    else {
        return reject(StatusCode::BAD_REQUEST, "client_id缺失");
    };
    let name = v["action"].as_str().unwrap_or("");
    let mut c = app.core.lock().unwrap();
    let now = clock();
    if name == "pair" || name == "heartbeat" {
        if c.owner.as_deref() != Some(client) && now - c.operator_at < 0.6 {
            return reject(StatusCode::CONFLICT, "另一操作者正在控制");
        }
        let Some(seq) = v["sequence"].as_u64() else {
            return reject(StatusCode::BAD_REQUEST, "缺少心跳序号");
        };
        if c.owner.as_deref() != Some(client) {
            c.operator_sequence = None;
        }
        if c.operator_sequence.is_some_and(|last| seq <= last) {
            return reject(StatusCode::BAD_REQUEST, "心跳序号过期");
        }
        c.owner = Some(client.into());
        c.operator_at = now;
        c.operator_sequence = Some(seq);
        return Json(json!({"ok":true})).into_response();
    }
    if !matches!(name, "hold" | "release")
        && (c.owner.as_deref() != Some(client) || now - c.operator_at > 0.6)
    {
        return reject(StatusCode::BAD_REQUEST, "请先连接控制器，保持页面在前台");
    }
    let Some(id) = v["request_id"]
        .as_str()
        .filter(|s| !s.is_empty() && s.len() < 100)
    else {
        return reject(StatusCode::BAD_REQUEST, "缺少request_id");
    };
    if c.requests.iter().any(|old| old == id) {
        return Json(json!({"ok":true,"duplicate":true})).into_response();
    }
    let action_context = json!({"phase_before":c.phase,"dance_before":c.dance,"frame_before":c.dance_frame,"requested_clip":v["clip"],"requested_duration_s":v["duration_s"]});
    let result: Result<(), String> = (|| {
        match name {
            "stand" => {
                if c.enabled {
                    return Ok(());
                }
                if let Some(error) = c.bridge.fault.as_ref() {
                    return Err(format!("先处理故障并点击复位：{error}"));
                }
                if !c.bridge.blockers.is_empty() {
                    return Err(c.bridge.blockers.join("；"));
                }
                c.bridge.sensors.validate(
                    now,
                    app.hardware.state_timeout_s,
                    app.hardware.sensor_skew_s,
                )?;
                if c.bridge
                    .sensors
                    .temperature
                    .iter()
                    .any(|t| *t > app.hardware.max_temperature)
                {
                    return Err("温度超限".into());
                }
                if c.bridge.sensors.qd.iter().any(|v| v.abs() > 0.3) {
                    return Err("当前关节尚未静止".into());
                }
                for i in 0..22 {
                    if c.bridge.sensors.q[i] < app.hardware.lower[i] - 0.01
                        || c.bridge.sensors.q[i] > app.hardware.upper[i] + 0.01
                    {
                        return Err(format!("{} 实测位置超限", contract::NAMES[i]));
                    }
                }
                c.dance = None;
                c.reset_history = true;
                c.epoch += 1;
                c.session = rand::random();
                c.began = now;
                c.origin = c.bridge.sensors.q;
                c.target = std::array::from_fn(|i| {
                    c.origin[i].clamp(app.hardware.lower[i], app.hardware.upper[i])
                });
                c.enabled = true;
                c.phase = "TAKEOVER".into();
                c.fault = None;
                c.blend_at = None;
                c.previous_policy_raw = [0.; 22];
                c.model_saturation = [false; 22];
                c.stable_at = None;
                c.verified = false;
                c.event("开始真实22关节受限接管：实测姿态→模型初始站姿→零速度策略");
            }
            "play_dance" => {
                if !c.enabled
                    || c.blend_at.is_none()
                    || !matches!(c.phase.as_str(), "POLICY_STAND" | "STANDING")
                    || c.dance.is_some()
                    || c.blend_at.is_some_and(|at| now - at < 3.)
                {
                    return Err("请先站立，等待起身完成后播放舞蹈".into());
                }
                let clip = v["clip"].as_str().ok_or("请选择舞蹈")?;
                let meta = c
                    .dances
                    .iter()
                    .find(|m| m["id"].as_str() == Some(clip))
                    .ok_or("舞蹈不存在")?;
                let total = meta["frames_50hz"].as_u64().ok_or("舞蹈帧数无效")? as usize;
                let seconds = v["duration_s"].as_f64().unwrap_or(total as f64 * 0.02);
                if !seconds.is_finite() || seconds <= 0. || seconds > total as f64 * 0.02 + 0.02 {
                    return Err("播放时长无效".into());
                }
                c.verified = false;
                c.stable_at = None;
                c.dance_limit = (seconds / 0.02).round().max(1.) as usize;
                c.stand_pose = c.target;
                c.dance = Some(clip.into());
                c.dance_frame = 0;
                c.epoch += 1;
                c.phase = "DANCING".into();
                c.event(&format!("播放真实舞蹈 {clip}，上限{seconds:.2}秒"));
            }
            "cancel_dance" => {
                if c.dance.take().is_some() {
                    c.epoch += 1;
                    c.verified = false;
                    c.stable_at = None;
                    c.phase = "WAITING_MODEL".into();
                    c.event("取消舞蹈：切换零速度站立条件，保留历史，由同一模型恢复站立");
                }
            }
            "hold" => {
                if let Some(session) = c.bridge.session {
                    app.requests
                        .lock()
                        .unwrap()
                        .push_back(Request::Disarm { session });
                }
                c.enabled = false;
                c.dance = None;
                c.phase = "HOLDING".into();
                c.event("请求保护器捕获当前实测姿态并受限保持");
            }
            "release" => {
                app.requests.lock().unwrap().push_back(Request::Release {
                    session: c.bridge.session,
                });
                c.enabled = false;
                c.verified = false;
                c.fault = None;
                c.dance = None;
                c.phase = "RELEASING".into();
                c.model_saturation = [false; 22];
                c.previous_policy_raw = [0.; 22];
                c.event("解除位置接管：零刚度、低阻尼，等待真实关节自然静止；不切断电机电源");
            }
            "reset" => {
                if c.enabled {
                    return Err("先停止当前任务".into());
                }
                c.bridge.sensors.validate(
                    now,
                    app.hardware.state_timeout_s,
                    app.hardware.sensor_skew_s,
                )?;
                app.requests.lock().unwrap().push_back(Request::Reset {
                    session: c.bridge.session.unwrap_or(0),
                });
                c.fault = None;
                c.phase = "RESETTING".into();
                c.event("已请求静止状态下复位，保护保持继续");
            }
            _ => return Err("支持站立、舞蹈播放/取消、保持、解除接管和复位".into()),
        }
        Ok(())
    })();
    match result {
        Ok(()) => {
            app.flight.record(ActorRecord::Event {
                at: clock(),
                action: name.into(),
                request_id: id.into(),
                context: action_context,
                phase: c.phase.clone(),
                error: None,
            });
            if c.requests.len() >= 64 {
                c.requests.pop_front();
            }
            c.requests.push_back(id.into());
            Json(json!({"ok":true,"phase":c.phase})).into_response()
        }
        Err(e) => {
            app.flight.record(ActorRecord::Event {
                at: clock(),
                action: name.into(),
                request_id: id.into(),
                context: action_context,
                phase: c.phase.clone(),
                error: Some(e.clone()),
            });
            reject(StatusCode::BAD_REQUEST, &e)
        }
    }
}
#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let o = Options::parse();
    let root = o.model_dir.canonicalize()?;
    let contract = contract::load(&root)?;
    let hardware = Hardware::load(&o.hardware_profile)?;
    if hardware.stand_trial.as_ref().map(|t| t.home) != Some(contract.action.home) {
        return Err("Stand home must match the fixed model contract".into());
    }
    let clips = reference::load(&root)?;
    let dances: Vec<Value> = clips
        .values()
        .filter(|v| v.metadata["role"] == "dance")
        .map(|v| v.metadata.clone())
        .collect();
    let blockers = hardware.stand_trial_blockers();
    if !blockers.is_empty() {
        return Err(blockers.join(";").into());
    }
    if let Some(path) = &o.audit_recorded_actions {
        let trace: Value = serde_json::from_slice(&std::fs::read(path)?)?;
        let mut error = 0f32;
        let mut left_elbow_min = f32::INFINITY;
        let mut count = 0;
        let mut corrected_frames = 0;
        let mut raw_error = 0f32;
        for row in trace["series"]
            .as_array()
            .ok_or("Missing real trace series")?
        {
            let q: [f32; 22] = serde_json::from_value(row["model_q"].clone())?;
            let raw =
                std::array::from_fn(|i| (q[i] - contract.action.home[i]) / contract.action.scale);
            let result = action::map(raw, &contract, &hardware);
            for (i, v) in q.iter().enumerate() {
                error = error.max(
                    (result.motor_safe_q[i] - v.clamp(hardware.lower[i], hardware.upper[i])).abs(),
                );
            }
            let mut differs = false;
            for (i, value) in raw.iter().enumerate() {
                raw_error = raw_error.max(
                    (result.policy_raw[i]
                        - value.clamp(-contract.action.clip, contract.action.clip))
                    .abs(),
                );
                let old = ((result.motor_safe_q[i] - contract.action.home[i])
                    / contract.action.scale)
                    .clamp(-contract.action.clip, contract.action.clip);
                differs |= (old - result.policy_raw[i]).abs() > 1e-5;
            }
            corrected_frames += usize::from(differs);
            left_elbow_min = left_elbow_min.min(result.motor_safe_q[17]);
            count += 1;
        }
        if count == 0 || error > 1e-5 || raw_error > 1e-5 {
            return Err("Native mapping replay mismatch".into());
        }
        println!(
            "{}",
            json!({"source":"recorded_real_Q1_model_targets","frames":count,"max_error_rad":error,"max_policy_raw_error":raw_error,"corrected_mechanical_feedback_frames":corrected_frames,"previous_action_semantics_verified":true,"left_elbow_min_rad":left_elbow_min,"extra_standing_amplitude_limit":false,"hardware_writes":false})
        );
        return Ok(());
    }
    let mut model = Model::load_with_threads(&root.join("policy.onnx"), 1)?;
    let parity = model.golden(&root.join("golden.json"))?;
    let flight_base = o
        .private_trace_file
        .as_ref()
        .and_then(|p| p.parent())
        .unwrap_or_else(|| o.socket.parent().unwrap())
        .join("flight");
    let profile_path = o.hardware_profile.clone();
    let profile_sha256 = contract::hash(&profile_path)?;
    let flight = Recorder::<ActorRecord>::open(&flight_base, "actor", 64 * 1024 * 1024)?;
    flight.manifest(&json!({"schema":1,"stage_log_version":2,"source":"real_q1_ros2","model_sha256":contract::hash(&root.join("policy.onnx"))?,"profile_sha256":contract::hash(&o.hardware_profile)?,"hardware":hardware,"contract":contract,"model_parity":parity,"previous_action_semantics":"clip(previous issued model raw, +/- model clip); before mechanical position, slew and torque safety transforms"}))?;
    let core = Arc::new(Mutex::new(Core {
        bridge: BridgeStatus::default(),
        phase: "CONNECTING".into(),
        enabled: false,
        epoch: 0,
        session: 0,
        began: 0.,
        origin: [0.; 22],
        target: [0.; 22],
        fault: None,
        requests: VecDeque::new(),
        owner: None,
        operator_at: 0.,
        operator_sequence: None,
        input_count: 0,
        model_count: 0,
        inference_ms: 0.,
        raw: [0.; 22],
        previous_policy_raw: [0.; 22],
        model_saturation: [false; 22],
        blend_at: None,
        stable_at: None,
        verified: false,
        events: VecDeque::new(),
        parity,
        dances,
        dance: None,
        dance_frame: 0,
        dance_limit: 0,
        stand_pose: contract.action.home,
        reset_history: true,
    }));
    let jobs: Arc<Mutex<Option<Job>>> = Arc::new(Mutex::new(None));
    let predictions: Arc<Mutex<Option<Prediction>>> = Arc::new(Mutex::new(None));
    let requests = Arc::new(Mutex::new(VecDeque::new()));
    let quit = Arc::new(AtomicBool::new(false));
    let (jc, pc, mc, qc) = (
        jobs.clone(),
        predictions.clone(),
        core.clone(),
        quit.clone(),
    );
    let model_flight = flight.clone();
    let action_contract = contract.clone();
    let action_hardware = hardware.clone();
    thread::Builder::new()
        .name("q1-onnx".into())
        .spawn(move || {
            let mut trace_at=0.;
            let mut idle_recorded_at=0.;
            while !qc.load(Ordering::Acquire) {
                let job = jc.lock().unwrap().take();
                if let Some(job) = job {
                    let enabled=mc.lock().unwrap().enabled;
                    let record_full=enabled || clock()-idle_recorded_at>=1.;
                    if record_full {idle_recorded_at=clock();}
                    if record_full {model_flight.record(ActorRecord::Input{at:job.built_at,id:job.id,epoch:job.epoch,observed:job.observed,input:job.x.clone(),sensors:job.sensors.clone(),previous_raw:job.previous,context:job.context.clone()});}
                    let started=clock();
                    let start = Instant::now();
                    match model.infer(&job.x) {
                        Ok(raw) => {
                            let ms = start.elapsed().as_secs_f64() * 1000.;
                            let mut c = mc.lock().unwrap();
                            c.model_count += 1;
                            c.inference_ms = ms;
                            c.raw = raw;
                            let trace = if o.private_trace_file.is_some() && clock()-trace_at>1. {
                                trace_at=clock();
                                Some(json!({"source":"real_q1_ros2","input_observed_at":job.observed,"input":job.x,"raw_action":raw,"requested_q":c.target,"applied_q":c.bridge.commanded_q,"applied_raw":c.bridge.applied_raw,"accepted_sequence":c.bridge.accepted,"measured_q":c.bridge.sensors.q,"model_count":c.model_count,"phase":c.phase}))
                            } else {None};
                            drop(c);
                            let completed=clock();
                            if record_full {model_flight.record(ActorRecord::Inference{id:job.id,epoch:job.epoch,observed:job.observed,started,completed,ms,candidate_raw:raw,native_action:Box::new(action::map(raw,&action_contract,&action_hardware)),context:job.context.clone(),queue_wait_ms:(started-job.built_at)*1000.,sensor_to_result_ms:(completed-job.observed)*1000.});}
                            *pc.lock().unwrap() = Some(Prediction {
                                context:job.context.clone(),
                                id:job.id,
                                raw,
                                observed: job.observed,
                                epoch: job.epoch,
                                ms,
                                completed,
                            });
                            if let (Some(path),Some(trace))=(&o.private_trace_file,trace) {
                                use std::{io::Write,os::unix::fs::OpenOptionsExt};
                                let temporary=path.with_extension("tmp");
                                if let Ok(mut file)=std::fs::OpenOptions::new().write(true).create(true).truncate(true).mode(0o600).open(&temporary) {
                                    if file.write_all(trace.to_string().as_bytes()).is_ok() {let _=std::fs::rename(&temporary,path);}
                                }
                            }
                        }
                        Err(e) => {model_flight.record(ActorRecord::Event{at:clock(),action:"model_error".into(),request_id:String::new(),context:json!({"observation_id":job.id,"epoch":job.epoch,"policy":job.context}),phase:"FAULT".into(),error:Some(e.clone())});mc.lock().unwrap().fail(e)},
                    }
                } else {
                    thread::sleep(Duration::from_millis(1));
                }
            }
        })?;
    let (sc, sj, sp, sr, sq, hw) = (
        core.clone(),
        jobs.clone(),
        predictions.clone(),
        requests.clone(),
        quit.clone(),
        hardware.clone(),
    );
    let control_flight = flight.clone();
    thread::Builder::new()
        .name("q1-stand-50hz".into())
        .spawn(move || {
            let mut socket: Option<Socket> = None;
            let mut next = 0.;
            let mut history = Observation::default();
            let mut seq = 0;
            let mut last_send = clock();
            let mut last_prediction_used=None;
            let mut scheduled = Instant::now();
            let mut observed_epoch = 0;
            let mut logged_phase=String::new();let mut state_logged_at=0.;
            while !sq.load(Ordering::Acquire) {
                let now = clock();
                if socket.is_none() && now >= next {
                    next = now + 1.;
                    if let Ok(s) = Socket::connect(&o.socket) {
                        socket = Some(s);
                    }
                }
                let mut disconnected = false;
                if let Some(s) = &socket {
                    for _ in 0..64 {
                        match s.receive::<BridgeStatus>() {
                            Ok(Some(b)) => sc.lock().unwrap().bridge = b,
                            Ok(None) => break,
                            Err(_) => {
                                disconnected = true;
                                break;
                            }
                        }
                    }
                    for _ in 0..8 {
                        let request = sr.lock().unwrap().pop_front();
                        if let Some(request) = request {
                            if s.send(&request).is_err() {
                                disconnected = true;
                                break;
                            }
                        } else {
                            break;
                        }
                    }
                }
                if disconnected {
                    socket = None;
                    let mut c = sc.lock().unwrap();
                    c.fail("真实HAL保护进程连接中断".into());
                    c.bridge = BridgeStatus::default();
                }
                let mut c = sc.lock().unwrap();
                // Read time after draining IPC and acquiring the snapshot.
                // A just-received timestamp can be newer than the loop start.
                let now = clock();
                if !c.enabled && c.bridge.fault.is_some() {
                    c.phase = "FAULT".into();
                }
                if !c.enabled
                    && c.fault.is_none()
                    && c.bridge.fault.is_none()
                    && c.bridge.blockers.is_empty()
                    && c.bridge.sensors.complete
                {
                    c.phase = if c.bridge.released {
                        "RELEASED"
                    } else if c.bridge.mode == "SAFE_HOLD" {
                        "SAFE_HOLD"
                    } else {
                        "READY"
                    }
                    .into();
                }
                if c.enabled {
                    if let Some(e) = c.bridge.fault.clone() {
                        c.fail(e);
                    }
                }
                let fresh = c
                    .bridge
                    .sensors
                    .validate(now, hw.state_timeout_s, hw.sensor_skew_s);
                if let Err(e) = fresh {
                    if c.enabled {
                        if let Some(session) = c.bridge.session {
                            sr.lock().unwrap().push_back(Request::Disarm { session });
                        }
                        c.fail(e);
                    }
                } else {
                    if observed_epoch != c.epoch {
                        if c.reset_history {history.reset();c.reset_history=false;}
                        observed_epoch = c.epoch;
                    }
                    if c.dance.is_some() && c.dance_frame>=c.dance_limit {
                        c.dance=None;c.epoch+=1;c.event("舞蹈播放完成：切换零速度站立条件，由同一模型恢复站立");
                    }
                    let sensor = c.bridge.sensors.clone();
                    // The guide defines the previous action at the action
                    // manager, before actuator delay, torque caps and slew.
                    let previous = c.previous_policy_raw;
                    match history.build(
                        &sensor,
                        &hw,
                        &contract,
                        previous,
                        Condition {
                            command: [0.; 3],
                            step: c.input_count,
                            reference: c.dance.as_ref().and_then(|id|clips.get(id)).map(|clip|clip.frame(c.dance_frame)),
                        },
                    ) {
                        Ok(x) => {
                            c.input_count += 1;
                            *sj.lock().unwrap() = Some(Job {
                                built_at:clock(),
                                context:PolicyContext {enabled_at_sampling:c.enabled,phase_at_sampling:c.phase.clone(),mode:u8::from(c.dance.is_some()),dance:c.dance.clone(),reference_frame:c.dance.as_ref().map(|_|c.dance_frame),reference_rotation:c.dance.as_ref().and_then(|id|clips.get(id)).map(|clip|clip.frame(c.dance_frame).0)},
                                id:c.input_count,
                                x,
                                observed: sensor.joints_at.min(sensor.imu_at),
                                epoch: c.epoch,
                                sensors:Box::new(sensor.clone()),
                                previous,
                            });
                        }
                        Err(e) => c.fail(e),
                    }
                    if c.enabled {
                        let trial = hw.stand_trial.as_ref().unwrap();
                        let elapsed = now - c.began;
                        if c.bridge.session != Some(c.session) {
                            if let Some(s) = &socket {
                                let _ = s.send(&Request::Arm { session: c.session });
                            }
                        } else {
                            let dt = (now - last_send).clamp(0., 0.05) as f32;
                            let mut wanted = c.target;
                            let mut generated = now;
                            let mut target_observed = sensor.joints_at.min(sensor.imu_at);
                            let mut policy_raw = None;
                            let mut can_send=true;
                            let mut prediction_id=None;
                            let mut candidate_raw=[0.;22];
                            let mut stage_context=None;
                            let mut model_q=None;
                            let mut raw_clip_limited=[false;22];
                            let amplitude_limited=[false;22];
                            let mut mechanical_limited=[false;22];
                            let mut prediction_age_ms=None;
                            // A loaded robot must retain supporting stiffness on handoff.
                            // Match the measured position first; bound torque in q1-rt.
                            let gain = 1.;
                            if elapsed < 3. {
                                c.phase = "TAKEOVER".into();
                                wanted = c.origin;
                            } else if c.blend_at.is_none() {
                                c.phase = "INITIALIZING".into();
                                let alpha =
                                    ((elapsed - 3.) / trial.prepare_seconds).clamp(0., 1.) as f32;
                                let alpha = alpha * alpha * (3. - 2. * alpha);
                                wanted = std::array::from_fn(|i| {
                                    c.origin[i] + (trial.home[i] - c.origin[i]) * alpha
                                });
                                let error = (0..22)
                                    .map(|i| (sensor.q[i] - trial.home[i]).abs())
                                    .fold(0f32, f32::max);
                                let upright = pelvis(&sensor, &hw).is_ok_and(|r| r[2][2] > 0.94);
                                // Loaded PD joints have a steady gravity deflection.
                                // Enter the balancing actor from a quiet supported
                                // posture; it must not already pass final standing.
                                let quiet = sensor.qd.iter().all(|v| v.abs() < 0.3);
                                if elapsed > 3. + trial.prepare_seconds
                                    && (trial.soft_limits
                                        || (error < hw.max_tracking_error_rad && upright && quiet))
                                {
                                    c.blend_at = Some(now);
                                    c.event(&format!("初始过渡完成，实测home误差{error:.3}rad；渐入ONNX零速度策略"));
                                } else if !trial.soft_limits
                                    && elapsed > trial.prepare_seconds + 23.
                                {
                                    c.fail(format!("站姿初始化未收敛，最大home误差{error:.3}rad"));
                                }
                            } else {
                                c.phase = if c.dance.is_some() {"DANCING"} else {"POLICY_STAND"}.into();
                                let prediction = sp.lock().unwrap();
                                if let Some(p) = prediction.as_ref().filter(|p| {
                                    p.epoch == c.epoch
                                        && now - p.observed <= hw.state_timeout_s
                                        && now - p.completed < hw.target_timeout_s
                                }) {
                                    generated = p.completed;
                                    target_observed = p.observed;
                                    prediction_id=Some(p.id);candidate_raw=p.raw;
                                    stage_context=Some(p.context.clone());
                                    prediction_age_ms=Some((now-p.completed)*1000.);
                                    let mapped=action::map(p.raw,&contract,&hw);
                                    model_q=Some(mapped.model_q);
                                    raw_clip_limited=mapped.raw_limited;
                                    mechanical_limited=mapped.mechanical_limited;
                                    policy_raw=Some(mapped.policy_raw);
                                    let blend=((now-c.blend_at.unwrap())/3.).clamp(0.,1.) as f32;
                                    c.model_saturation=std::array::from_fn(|i|mapped.raw_limited[i] || mapped.mechanical_limited[i]);
                                    wanted=std::array::from_fn(|i|contract.action.home[i]+(mapped.motor_safe_q[i]-contract.action.home[i])*blend);
                                    if blend<1. {c.phase="POLICY_HANDOVER".into();}
                                    c.inference_ms = p.ms;
                                    let tilt = pelvis(&sensor, &hw).map(|r| r[2][2]).unwrap_or(0.);
                                    if tilt < 0.9 && !trial.soft_limits {
                                        c.fail("站立策略阶段机身倾角超过26度".into());
                                    }
                                    let quiet = sensor.qd.iter().all(|v| v.abs() < 0.3);
                                    let tracking = (0..22).all(|i| {
                                        (sensor.q[i] - c.bridge.commanded_q[i]).abs() < 0.15
                                    });
                                    if c.dance.is_none() && blend >= 1. && tilt > 0.98 && quiet && tracking {
                                        let at = *c.stable_at.get_or_insert(now);
                                        if now - at > 2. {
                                            c.verified = true;
                                            c.phase = "STANDING".into();
                                        }
                                    } else {
                                        c.stable_at = None;
                                        c.verified = false;
                                    }
                                } else {
                                    // Do not re-stamp an old inference or abort on a
                                    // single missed frame. The independent RT still
                                    // expires the last valid target at its true TTL.
                                    can_send=false;
                                    c.phase="WAITING_MODEL".into();
                                }
                            }
                            if c.enabled && can_send {
                                let previous_target_q=c.target;
                                let mut step_budget_rad=[0.;22];
                                for (i, desired) in wanted.iter().enumerate() {
                                    let velocity = if c.blend_at.is_some() {
                                        trial.policy_velocity_limit[i]
                                    } else {
                                        trial.velocity_limit[i]
                                    };
                                    let step = (velocity * dt).min(hw.max_step_rad);
                                    step_budget_rad[i]=step;
                                    c.target[i] = ((desired.clamp(hw.lower[i], hw.upper[i])
                                        - c.target[i])
                                        .clamp(-step, step)
                                        + c.target[i])
                                        .clamp(hw.lower[i], hw.upper[i]);
                                }
                                let raw = policy_raw.unwrap_or_else(|| {
                                    std::array::from_fn(|i| {
                                        ((c.target[i] - trial.home[i]) / 0.25).clamp(-20., 20.)
                                    })
                                });
                                seq += 1;
                                let t = Target {
                                    session: c.session,
                                    epoch: c.epoch,
                                    sequence: seq,
                                    generated,
                                    observation: target_observed,
                                    expires: generated + hw.target_timeout_s,
                                    operator_expires: c.operator_at + 0.6,
                                    // Explicit one-click stand is an onboard task.
                                    // The UI heartbeat arbitrates new commands; it
                                    // is not a continuous motion authorization.
                                    moving: false,
                                    q: c.target,
                                    raw,
                                    gain_scale: gain,
                                };
                                control_flight.record(ActorRecord::Target{at:clock(),observation_id:prediction_id,target:Box::new(t.clone()),candidate_raw,model_limited:c.model_saturation,stages:Box::new(TargetStages {context:stage_context,model_q,raw_clip_limited,amplitude_limited,mechanical_limited,desired_q:wanted,previous_target_q,step_budget_rad,slew_limited:std::array::from_fn(|i|(wanted[i].clamp(hw.lower[i],hw.upper[i])-c.target[i]).abs()>1e-6),desired_to_sent_delta_rad:std::array::from_fn(|i|wanted[i]-c.target[i]),dt_s:dt,prediction_reused:prediction_id.is_some() && prediction_id==last_prediction_used,prediction_age_ms,observation_to_target_ms:(now-target_observed)*1000.,joint_receive_age_ms:(now-sensor.joints_at)*1000.,imu_receive_age_ms:(now-sensor.imu_at)*1000.}),phase:c.phase.clone(),bridge:Box::new(c.bridge.clone())});
                                if let Some(s) = &socket {
                                    if let Err(e) = s.send(&Request::Target(Box::new(t))) {
                                        c.fail(format!("真实关节目标发送失败：{e}"));
                                    } else {
                                        if prediction_id.is_some(){last_prediction_used=prediction_id;}
                                        if policy_raw.is_some() {c.previous_policy_raw = raw;}
                                        if policy_raw.is_some() && c.dance.is_some(){c.dance_frame+=1;}
                                    }
                                }
                                last_send = now;
                            }
                        }
                    }
                }
                if c.phase!=logged_phase || now-state_logged_at>=1. {
                    control_flight.record(ActorRecord::State{at:clock(),phase:c.phase.clone(),epoch:c.epoch,enabled:c.enabled,standing_verified:c.verified,dance:c.dance.clone(),dance_frame:c.dance_frame,returning_to_stand:false,fault:c.fault.clone(),bridge:Box::new(c.bridge.clone())});
                    logged_phase=c.phase.clone();state_logged_at=now;
                }
                drop(c);
                scheduled += Duration::from_millis(20);
                let t = Instant::now();
                if t > scheduled + Duration::from_millis(20) {
                    scheduled = t;
                }
                thread::sleep(scheduled.saturating_duration_since(t));
            }
        })?;
    let app = Arc::new(App {
        core,
        hardware,
        profile_path,
        profile_sha256,
        requests,
        flight,
    });
    let routes = Router::new()
        .route(
            "/",
            get(|| async { Html(include_str!("../../web/stand.html")) }),
        )
        .route(
            "/stand.js",
            get(|| async {
                (
                    [
                        (
                            axum::http::header::CONTENT_TYPE,
                            "text/javascript; charset=utf-8",
                        ),
                        (axum::http::header::CACHE_CONTROL, "no-store"),
                    ],
                    include_str!("../../web/stand.js"),
                )
            }),
        )
        .route(
            "/supermimic.css",
            get(|| async {
                (
                    [(axum::http::header::CONTENT_TYPE, "text/css")],
                    include_str!("../../web/supermimic.css"),
                )
            }),
        )
        .route(
            "/api/state",
            get(|State(app): State<Arc<App>>| async move { Json(snapshot(&app)) }),
        )
        .route("/api/action", post(action))
        .with_state(app.clone());
    let listener = tokio::net::TcpListener::bind(("0.0.0.0", o.port)).await?;
    println!("Real Q1 standing console listening on {}", o.port);
    axum::serve(listener, routes)
        .with_graceful_shutdown(async move {
            let mut term =
                tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate()).unwrap();
            tokio::select! {_=tokio::signal::ctrl_c()=>{},_=term.recv()=>{}}
            {
                let c = app.core.lock().unwrap();
                if let Some(session) = c.bridge.session {
                    app.requests
                        .lock()
                        .unwrap()
                        .push_back(Request::Disarm { session });
                }
            }
            tokio::time::sleep(Duration::from_millis(100)).await;
            quit.store(true, Ordering::Release);
        })
        .await?;
    Ok(())
}
