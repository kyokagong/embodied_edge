#![recursion_limit = "256"]
//! Native model task controller. All ROS/HAL ownership belongs to q1-rt.
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
        policy::PROTOCOL,
        types::{clock, BridgeStatus, Request, Target},
    },
    policy::{
        model::Model,
        observation::{command_goal, Observation},
        package::Package,
        reference::{self, Clip},
        task::{TaskState, SUPPORTED_STAND_ENTRY_JOINT_ERROR_RAD},
    },
    supermimic::{
        contract::{Hardware, NAMES},
        flight::Recorder,
        observation::{pelvis, yaw},
    },
};
use serde_json::{json, Value};
use std::{
    collections::{BTreeMap, VecDeque},
    path::PathBuf,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    thread,
    time::{Duration, Instant},
};
#[derive(Parser)]
#[command(about = "Q1 native policy controller; ROS/HAL writes belong to q1-rt")]
struct Options {
    #[arg(long)]
    model_dir: PathBuf,
    #[arg(long)]
    hardware_profile: PathBuf,
    #[arg(long)]
    socket: Option<PathBuf>,
    /// Validate package, references, hardware compatibility and real ONNX golden vectors; no IPC/ROS.
    #[arg(long)]
    check_only: bool,
    #[arg(long, default_value_t = 1)]
    ort_threads: usize,
    #[arg(long, default_value = "0.0.0.0")]
    host: String,
    #[arg(long, default_value_t = 8768)]
    port: u16,
    #[arg(long)]
    private_trace_file: Option<PathBuf>,
}
fn pose_raw(q: [f32; 22], home: [f32; 22], offset: [f32; 22]) -> [f32; 22] {
    std::array::from_fn(|i| ((q[i] - home[i] - offset[i]) / 0.25).clamp(-20., 20.))
}
fn model_target(raw: [f32; 22], home: [f32; 22], offset: [f32; 22], h: &Hardware) -> [f32; 22] {
    std::array::from_fn(|i| {
        (home[i] + 0.25 * raw[i].clamp(-20., 20.) + offset[i]).clamp(h.lower[i], h.upper[i])
    })
}
fn takeover_allowed(began: f64, now: f64, operator_at: f64) -> bool {
    [began, now, operator_at].iter().all(|v| v.is_finite())
        && (0. ..=0.25).contains(&(now - began))
        && (0. ..=0.6).contains(&(now - operator_at))
}
struct Job {
    id: u64,
    generation: u64,
    epoch: u64,
    observed: f64,
    history: [f32; 780],
    goal: [f32; 594],
    reference: [[f32; 3]; 3],
    context: Value,
}
struct Prediction {
    id: u64,
    generation: u64,
    epoch: u64,
    observed: f64,
    completed: f64,
    raw: [f32; 22],
    reference: [[f32; 3]; 3],
    ms: f64,
    context: Value,
}
struct Core {
    bridge: BridgeStatus,
    task: TaskState,
    enabled: bool,
    initializing: bool,
    takeover_began: f64,
    initial_sequence: Option<u64>,
    generation: u64,
    phase: String,
    fault: Option<String>,
    target: [f32; 22],
    raw: [f32; 22],
    input_count: u64,
    model_count: u64,
    inference_ms: f64,
    owner: Option<String>,
    operator_at: f64,
    operator_sequence: Option<u64>,
    requests: VecDeque<String>,
    events: VecDeque<String>,
    parity: Value,
}
impl Core {
    fn event(&mut self, s: impl Into<String>) {
        if self.events.len() >= 40 {
            self.events.pop_front();
        }
        self.events.push_back(s.into());
    }
    fn fail(&mut self, s: impl Into<String>) {
        let s = s.into();
        self.event(s.clone());
        self.fault = Some(s);
        self.enabled = false;
        self.initializing = false;
        self.phase = "FAULT".into();
        self.generation += 1;
    }
}
struct App {
    core: Arc<Mutex<Core>>,
    hardware: Hardware,
    package: Arc<Package>,
    clips: Arc<BTreeMap<String, Clip>>,
    profile_sha256: String,
    profile_path: PathBuf,
    requests: Arc<Mutex<VecDeque<Request>>>,
    flight: Recorder<Value>,
}
fn snapshot(app: &App) -> Value {
    let c = app.core.lock().unwrap();
    let now = clock();
    json!({"controller":"q1_policy","inference_location":"Q1","architecture":std::env::consts::ARCH,
        "policy_protocol":c.bridge.policy_protocol,"phase":c.phase,"enabled":c.enabled,"fault":c.fault,"guard_fault":c.bridge.fault,"bridge":c.bridge,"task":c.task,
        "input_count":c.input_count,"policy_steps":c.input_count,"model_count":c.model_count,"inference_ms":c.inference_ms,
        "input_shapes":{"obs_history":[1,10,78],"goal":[1,594]},"output_shape":[1,22],"policy_hz":50,
        "model_sha256":app.package.model_sha256,"profile_sha256":app.profile_sha256,
        "model":{"iteration":app.package.iteration,"sha256":app.package.model_sha256,"contract_sha256":app.package.contract_sha256,"path":app.package.root},
        "config":{"path":app.profile_path,"sha256":app.profile_sha256},"qualification":app.package.contract["qualification"],
        "standing_entry_limits":{"max_joint_error_rad":SUPPORTED_STAND_ENTRY_JOINT_ERROR_RAD,"max_gravity_error_rad":app.hardware.max_tilt_rad},
        "motions":app.clips.iter().filter(|(id,_)|id.as_str()!="command_000").map(|(id,c)|json!({"id":id,"frames_50hz":c.len(),"metadata":c.metadata})).collect::<Vec<_>>(),
        "joint_names":NAMES,"home":app.package.home,"target":c.target,"raw_action":c.raw,"events":c.events,"parity":c.parity,
        "sensor_age_ms":(now-c.bridge.sensors.joints_at.min(c.bridge.sensors.imu_at))*1000.,
        "feedback_age_ms":(now-c.bridge.sensors.joints_at.min(c.bridge.sensors.imu_at))*1000.,
        "effort_limits_nm":app.hardware.stand_trial.as_ref().map(|t|t.effort_limit),
        "policy_velocity_limits_rad_s":app.hardware.stand_trial.as_ref().map(|t|t.policy_velocity_limit),
        "blockers":c.bridge.blockers,"flight_log":{"directory":app.flight.directory,"dropped":app.flight.stats.dropped.load(Ordering::Relaxed)}})
}
fn reject(code: StatusCode, error: impl Into<String>) -> axum::response::Response {
    (code, Json(json!({"ok":false,"error":error.into()}))).into_response()
}
fn check_origin(headers: &HeaderMap) -> bool {
    let Some(host) = headers.get("host").and_then(|x| x.to_str().ok()) else {
        return false;
    };
    let valid = host
        .parse::<axum::http::uri::Authority>()
        .ok()
        .is_some_and(|a| matches!(a.host(), "192.168.54.41" | "localhost" | "127.0.0.1"));
    valid
        && !headers.get("origin").is_some_and(|x| {
            x.to_str().ok() != Some(&format!("http://{host}"))
                && x.to_str().ok() != Some(&format!("https://{host}"))
        })
}
async fn action(
    State(app): State<Arc<App>>,
    headers: HeaderMap,
    Json(v): Json<Value>,
) -> axum::response::Response {
    if !check_origin(&headers) {
        return reject(StatusCode::FORBIDDEN, "Host/Origin不允许");
    }
    let Some(client) = v["client_id"]
        .as_str()
        .filter(|v| !v.is_empty() && v.len() < 100)
    else {
        return reject(StatusCode::BAD_REQUEST, "client_id缺失");
    };
    let name = v["action"].as_str().unwrap_or("");
    let now = clock();
    let mut c = app.core.lock().unwrap();
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
        return reject(StatusCode::CONFLICT, "请先连接控制器");
    }
    let Some(id) = v["request_id"]
        .as_str()
        .filter(|v| !v.is_empty() && v.len() < 100)
    else {
        return reject(StatusCode::BAD_REQUEST, "request_id缺失");
    };
    if c.requests.iter().any(|old| old == id) {
        return Json(json!({"ok":true,"duplicate":true})).into_response();
    }
    let result: Result<(), String> = (|| {
        match name {
            "hold" | "release" | "reset" => {
                let request = match name {
                    "hold" => c.bridge.session.map(|session| Request::Disarm { session }),
                    "release" => Some(Request::Release {
                        session: c.bridge.session,
                    }),
                    _ => Some(Request::Reset {
                        session: c.bridge.session.unwrap_or(0),
                    }),
                };
                if let Some(request) = request {
                    let mut q = app.requests.lock().unwrap();
                    if q.len() >= 16 {
                        return Err("RT请求队列已满".into());
                    }
                    q.push_back(request);
                }
                c.enabled = false;
                c.initializing = false;
                c.generation += 1;
                c.fault = None;
                c.phase = match name {
                    "release" => "RELEASING",
                    "reset" => "RESETTING",
                    _ => "HOLDING",
                }
                .into();
            }
            "start_policy" | "stand" | "play_motion" | "sit" | "lie" | "getup"
            | "cancel_motion" => {
                if c.bridge.policy_protocol != PROTOCOL {
                    return Err("需要匹配的q1-rt --policy-v2".into());
                }
                c.bridge.sensors.validate(
                    now,
                    app.hardware.state_timeout_s,
                    app.hardware.sensor_skew_s,
                )?;
                if let Some(e) = c.bridge.fault.as_ref().or(c.fault.as_ref()) {
                    return Err(format!("请先处理故障并复位：{e}"));
                }
                if !c.bridge.blockers.is_empty() {
                    return Err(c.bridge.blockers.join(";"));
                }
                if c.bridge
                    .sensors
                    .temperature
                    .iter()
                    .any(|t| *t > app.hardware.max_temperature)
                {
                    return Err("温度超限".into());
                }
                if name == "cancel_motion" && !c.enabled {
                    return Ok(());
                }
                let mut next = c.task.clone();
                next.request_for_control(
                    c.enabled,
                    name,
                    v["clip"].as_str().or(v["clip_id"].as_str()),
                    &app.clips,
                    &c.bridge.sensors,
                    &app.hardware,
                )?;
                if !c.enabled {
                    if c.bridge.sensors.qd.iter().any(|v| v.abs() > 0.3) {
                        return Err("接管前关节须静止".into());
                    }
                    if (0..22).any(|i| {
                        c.bridge.sensors.q[i] < app.hardware.lower[i] - 0.01
                            || c.bridge.sensors.q[i] > app.hardware.upper[i] + 0.01
                    }) {
                        return Err("实测位置超出接管范围".into());
                    }
                    c.generation += 1;
                    c.enabled = true;
                    c.initializing = true;
                    c.takeover_began = now;
                    c.initial_sequence = None;
                    c.phase = "ARMING".into();
                }
                c.task = next;
            }
            _ => return Err("未知操作".into()),
        }
        Ok(())
    })();
    let error = result.as_ref().err().cloned();
    app.flight.record(
        json!({"kind":"event","at":now,"action":name,"request_id":id,"task":c.task,"error":error}),
    );
    if let Err(error) = result {
        return reject(StatusCode::BAD_REQUEST, error);
    }
    if c.requests.len() >= 128 {
        c.requests.pop_front();
    }
    c.requests.push_back(id.into());
    c.event(format!("操作 {name}"));
    Json(json!({"ok":true})).into_response()
}
#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let o = Options::parse();
    let package = Arc::new(Package::load(&o.model_dir)?);
    let hardware = Hardware::load(&o.hardware_profile)?;
    package.validate_hardware(&hardware)?;
    let clips = Arc::new(reference::load(&package)?);
    let mut model = Model::load(&package, o.ort_threads)?;
    let parity = model.golden(&package)?;
    let profile_sha256 = q1_edge::supermimic::contract::hash(&o.hardware_profile)?;
    if o.check_only {
        println!(
            "{}",
            json!({"ok":true,"controller":"q1_policy","model_sha256":package.model_sha256,"profile_sha256":profile_sha256,"iteration":package.iteration,"references":clips.len(),"parity":parity,"ros_initialized":false,"hardware_writes":false,"policy_protocol":PROTOCOL,"standing_entry_limits":{"max_joint_error_rad":SUPPORTED_STAND_ENTRY_JOINT_ERROR_RAD,"max_gravity_error_rad":hardware.max_tilt_rad}})
        );
        return Ok(());
    }
    let socket = o.socket.ok_or("--socket required unless --check-only")?;
    let flight_path = o
        .private_trace_file
        .as_ref()
        .and_then(|p| p.parent())
        .unwrap_or_else(|| socket.parent().unwrap())
        .join("flight");
    let flight = Recorder::<Value>::open(&flight_path, "policy", 64 * 1024 * 1024)?;
    flight.manifest(&json!({"schema":2,"model_sha256":package.model_sha256,"model_contract":package.contract,"profile_sha256":profile_sha256,"hardware":hardware,"parity":parity,"previous_action_semantics":"last successfully HAL-published clipped raw before actuator constraints"}))?;
    let core = Arc::new(Mutex::new(Core {
        bridge: BridgeStatus::default(),
        task: TaskState::new(),
        enabled: false,
        initializing: false,
        takeover_began: 0.,
        initial_sequence: None,
        generation: 0,
        phase: "CONNECTING".into(),
        fault: None,
        target: [0.; 22],
        raw: [0.; 22],
        input_count: 0,
        model_count: 0,
        inference_ms: 0.,
        owner: None,
        operator_at: 0.,
        operator_sequence: None,
        requests: VecDeque::new(),
        events: VecDeque::new(),
        parity,
    }));
    let jobs: Arc<Mutex<Option<Job>>> = Arc::new(Mutex::new(None));
    let predictions: Arc<Mutex<Option<Prediction>>> = Arc::new(Mutex::new(None));
    let requests = Arc::new(Mutex::new(VecDeque::<Request>::new()));
    let quit = Arc::new(AtomicBool::new(false));
    let (jc, pc, mc, qc, log) = (
        jobs.clone(),
        predictions.clone(),
        core.clone(),
        quit.clone(),
        flight.clone(),
    );
    thread::Builder::new().name("q1-policy-onnx".into()).spawn(move||{
        let mut idle_logged=0.;
        while !qc.load(Ordering::Acquire){
            let job=jc.lock().unwrap().take();
            if let Some(job)=job {
                let started=clock();let record=mc.lock().unwrap().enabled||started-idle_logged>=1.;
                if record {idle_logged=started;log.record(json!({"kind":"input","id":job.id,"generation":job.generation,"epoch":job.epoch,"observed":job.observed,"history":job.history.as_slice(),"goal":job.goal.as_slice(),"context":job.context}));}
                match model.infer(&job.history,&job.goal){
                    Ok(raw)=>{let completed=clock();let ms=(completed-started)*1000.;let mut c=mc.lock().unwrap();c.model_count+=1;c.inference_ms=ms;drop(c);
                        if record {log.record(json!({"kind":"inference","id":job.id,"generation":job.generation,"epoch":job.epoch,"completed":completed,"observed":job.observed,"raw":raw,"ms":ms,"context":job.context}));}
                        *pc.lock().unwrap()=Some(Prediction{id:job.id,generation:job.generation,epoch:job.epoch,observed:job.observed,completed,raw,reference:job.reference,ms,context:job.context});
                    }
                    Err(e)=>mc.lock().unwrap().fail(format!("ONNX推理失败：{e}")),
                }
            } else {thread::sleep(Duration::from_millis(1));}
        }
    })?;
    let (sc, sj, sp, sr, sq, hw, pkg, refs, log) = (
        core.clone(),
        jobs.clone(),
        predictions.clone(),
        requests.clone(),
        quit.clone(),
        hardware.clone(),
        package.clone(),
        clips.clone(),
        flight.clone(),
    );
    thread::Builder::new().name("q1-policy-50hz".into()).spawn(move||{
        let mut connection:Option<Socket>=None;let mut next_connect=0.;let mut history=Observation::default();let mut generation=u64::MAX;
        let mut sequence=0u64;let mut session=0u64;let mut initial_sent_at=0.;let mut last_sent_at=clock();
        let mut schedule=Instant::now();let mut state_logged=0.;
        while !sq.load(Ordering::Acquire){
            if connection.is_none()&&clock()>=next_connect {next_connect=clock()+1.;connection=Socket::connect(&socket).ok();}
            let mut disconnected=false;
            if let Some(s)=&connection {
                for _ in 0..64 {match s.receive::<BridgeStatus>() {Ok(Some(b))=>sc.lock().unwrap().bridge=b,Ok(None)=>break,Err(_)=>{disconnected=true;break;}}}
                for _ in 0..16 {let r=sr.lock().unwrap().pop_front();if let Some(r)=r {if s.send(&r).is_err(){disconnected=true;break;}}else{break;}}
            }
            if disconnected {connection=None;let mut c=sc.lock().unwrap();if c.enabled{c.fail("RT连接中断");}c.bridge=BridgeStatus::default();}
            let mut c=sc.lock().unwrap();let now=clock();
            let fresh=c.bridge.sensors.validate(now,hw.state_timeout_s,hw.sensor_skew_s);
            if c.enabled {if let Some(e)=c.bridge.fault.clone(){c.fail(e);}else if let Err(e)=&fresh{c.fail(e.clone());}}
            if fresh.is_ok()&&c.bridge.policy_protocol==PROTOCOL {
                let sensors=c.bridge.sensors.clone();
                if generation!=c.generation {generation=c.generation;history.reset();session=rand::random();initial_sent_at=0.;}
                if let Err(e)=c.task.update_readiness(&refs,&sensors,&hw){if c.enabled{c.fail(e);}}
                if c.enabled&&c.initializing {
                    if !takeover_allowed(c.takeover_began,now,c.operator_at) {c.fail("接管握手或初始操作许可超时，请重新显式启动");}
                    else if let Some(s)=&connection {
                        if c.bridge.session!=Some(session) {
                            let _=s.send(&Request::Arm{session});
                        } else if c.initial_sequence.is_some()&&c.bridge.accepted==c.initial_sequence {
                            c.initializing=false;history.reset();c.phase="POLICY".into();c.event("实测姿态接管已发布，开始统一模型任务");
                        } else {
                            if initial_sent_at==0. {initial_sent_at=now;}
                            if now-initial_sent_at>hw.target_timeout_s {c.fail("初始姿态发布确认超时");}
                            else {sequence+=1;let q=std::array::from_fn(|i|sensors.q[i].clamp(hw.lower[i],hw.upper[i]));let raw=pose_raw(q,pkg.home,pkg.offset);
                                let r=pelvis(&sensors,&hw).unwrap();let target=Target{session,epoch:c.task.epoch,sequence,generated:now,observation:sensors.joints_at.min(sensors.imu_at),expires:now+hw.target_timeout_s,operator_expires:c.operator_at+0.6,moving:false,q,raw,gain_scale:1.};
                                if c.initial_sequence.is_none(){c.initial_sequence=Some(sequence);}
                                // Keep one init target identity while waiting for its actual publication.
                                let target=Target{sequence:c.initial_sequence.unwrap(),..target};
                                if s.send(&Request::PolicyTarget{version:PROTOCOL,target:Box::new(target),reference_rotation:r}).is_err(){c.fail("初始目标发送失败");}
                                c.target=q;
                            }
                        }
                    }
                }
                if !c.initializing {
                    let mut issued_this_tick=false;
                    let previous=c.bridge.published_policy_raw.unwrap_or([0.;22]);
                    let current_ref=c.task.clip.as_ref().and_then(|id|refs.get(id)).map(|clip|c.task.rotation(clip));
                    let goal=if let Some(clip)=c.task.clip.as_ref().and_then(|id|refs.get(id)){clip.goal(c.task.frame,c.task.end)}else{command_goal(&pkg.home,[0.;3])};
                    match (history.build(&sensors,&hw,&pkg,previous,current_ref),goal) {
                        (Ok(x),Ok(goal))=>{c.input_count+=1;let reference=current_ref.unwrap_or_else(||yaw(pelvis(&sensors,&hw).unwrap()));
                            *sj.lock().unwrap()=Some(Job{id:c.input_count,generation:c.generation,epoch:c.task.epoch,observed:sensors.joints_at.min(sensors.imu_at),history:x,goal,reference,context:json!({"task":c.task,"published_raw":previous,"enabled":c.enabled})});
                        }
                        (Err(e),_)|(_,Err(e))=>c.fail(e),
                    }
                    if c.enabled {
                        if c.bridge.session!=Some(session){c.fail("RT会话已变化");}
                        else {
                            let candidate=sp.lock().unwrap();
                            if let Some(p)=candidate.as_ref().filter(|p|p.generation==c.generation&&p.epoch==c.task.epoch&&now>=p.completed&&now-p.completed<hw.target_timeout_s&&now>=p.observed&&now-p.observed<=hw.state_timeout_s) {
                                let mut q=model_target(p.raw,pkg.home,pkg.offset,&hw);let dt=(now-last_sent_at).clamp(0.,0.05) as f32;let limits=&hw.stand_trial.as_ref().unwrap().policy_velocity_limit;
                                for i in 0..22{let step=(limits[i]*dt).min(hw.max_step_rad);q[i]=(c.target[i]+(q[i]-c.target[i]).clamp(-step,step)).clamp(hw.lower[i],hw.upper[i]);}
                                sequence+=1;let raw=p.raw.map(|v|v.clamp(-20.,20.));let t=Target{session,epoch:p.epoch,sequence,generated:p.completed,observation:p.observed,expires:p.completed+hw.target_timeout_s,operator_expires:c.operator_at+0.6,moving:false,q,raw,gain_scale:1.};
                                if let Some(s)=&connection {match s.send(&Request::PolicyTarget{version:PROTOCOL,target:Box::new(t.clone()),reference_rotation:p.reference}) {
                                    Ok(())=>{c.target=q;c.raw=raw;c.inference_ms=p.ms;last_sent_at=now;issued_this_tick=true;c.phase=c.task.mode.to_uppercase();log.record(json!({"kind":"target","at":now,"prediction_id":p.id,"target":t,"reference":p.reference,"context":p.context,"published_policy_sequence":c.bridge.published_policy_sequence}));}
                                    Err(e)=>c.fail(format!("RT目标发送失败：{e}")),
                                }}
                            } else {c.phase="WAITING_MODEL".into();}
                        }
                    }
                    // Reference time advances only on an enabled policy sampling tick with a valid
                    // preceding inference. It is never advanced by browser/RT publication frequency.
                    if c.enabled&&issued_this_tick{if let Err(e)=c.task.advance(&refs,&sensors,&hw){c.fail(e);}}
                }
                if !c.enabled&&c.fault.is_none(){c.phase=if c.bridge.fault.is_some(){"FAULT"}else if c.bridge.released{"RELEASED"}else{"READY"}.into();}
            } else if !c.enabled {c.phase="CONNECTING".into();}
            if now-state_logged>=if c.enabled {0.1}else{1.} {state_logged=now;log.record(json!({"kind":"state","at":now,"policy_protocol":c.bridge.policy_protocol,"phase":c.phase,"enabled":c.enabled,"task":c.task,"bridge":c.bridge}));}
            drop(c);schedule+=Duration::from_millis(20);let n=Instant::now();if n>schedule+Duration::from_millis(20){schedule=n;}thread::sleep(schedule.saturating_duration_since(n));
        }
    })?;
    let app = Arc::new(App {
        core,
        hardware,
        package,
        clips,
        profile_sha256,
        profile_path: o.hardware_profile,
        requests,
        flight,
    });
    let routes = Router::new()
        .route(
            "/",
            get(|| async { Html(include_str!("../../web/policy.html")) }),
        )
        .route(
            "/policy.js",
            get(|| async {
                (
                    [(
                        axum::http::header::CONTENT_TYPE,
                        "text/javascript; charset=utf-8",
                    )],
                    include_str!("../../web/policy.js"),
                )
            }),
        )
        .route(
            "/supermimic.css",
            get(|| async {
                (
                    [(axum::http::header::CONTENT_TYPE, "text/css; charset=utf-8")],
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
    let listener = tokio::net::TcpListener::bind((o.host.as_str(), o.port)).await?;
    println!(
        "Q1 native policy console listening on {}:{}",
        o.host, o.port
    );
    axum::serve(listener, routes)
        .with_graceful_shutdown(async move {
            let mut term =
                tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate()).unwrap();
            tokio::select! {_=tokio::signal::ctrl_c()=>{},_=term.recv()=>{}}
            if let Some(session) = app.core.lock().unwrap().bridge.session {
                app.requests
                    .lock()
                    .unwrap()
                    .push_back(Request::Disarm { session });
            }
            tokio::time::sleep(Duration::from_millis(100)).await;
            quit.store(true, Ordering::Release);
        })
        .await?;
    Ok(())
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn initial_ownership_cannot_be_acquired_after_handshake_or_lease_expiry() {
        assert!(takeover_allowed(10., 10.2, 10.1));
        assert!(!takeover_allowed(10., 10.3, 10.29));
        assert!(!takeover_allowed(10., 10.1, 9.));
        assert!(!takeover_allowed(10., 9.9, 10.));
    }
    #[test]
    fn takeover_raw_matches_published_pose_with_model_offset() {
        let raw = pose_raw([0.7; 22], [0.1; 22], [0.1; 22]);
        assert!((raw[0] - 2.).abs() < 1e-6);
        assert_eq!(pose_raw([100.; 22], [0.; 22], [0.; 22]), [20.; 22]);
    }
    #[test]
    fn output_mapping_adds_offset_once_then_clamps_mechanics() {
        let mut h = Hardware::load(std::path::Path::new(
            "../config/supermimic-stand-trial.json",
        ))
        .unwrap();
        h.lower = [-1.; 22];
        h.upper = [1.; 22];
        let q = model_target([2.; 22], [0.1; 22], [0.1; 22], &h);
        assert!((q[0] - 0.7).abs() < 1e-6);
        assert_eq!(model_target([100.; 22], [0.1; 22], [0.1; 22], &h), [1.; 22]);
    }
}
