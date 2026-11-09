use super::{
    ipc::Socket,
    pi_mutex::PiMutex,
    types::{clock, BridgeStatus, Request, Sensors, Target},
};
use crate::supermimic::contract::{Hardware, NAMES};
use crate::supermimic::flight::Recorder;
use futures::{FutureExt, StreamExt};
use r2r::{aimdk_msgs::msg, QosProfile};
use serde::Serialize;
use std::{
    path::Path,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    thread,
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};
// Q1 device_manager release-v1.2.1.17 JointSubscriberHandle ignores names.
// Its ABI is upper SPI (8), left CAN (7), right CAN (7), in MotorModule order.
// All actor/state arrays remain in the pinned model order NAMES.
pub const HAL_ORDER: [usize; 22] = [
    14, 15, 16, 17, 18, 19, 20, 21, 13, 0, 1, 2, 3, 4, 5, 12, 6, 7, 8, 9, 10, 11,
];
#[derive(Serialize)]
struct RtStages {
    accepted_target_q: Option<[f32; 22]>,
    before_tracking_q: [f64; 22],
    after_tracking_q: [f64; 22],
    pd_active: [bool; 22],
    before_effort_tau: [f64; 22],
    after_effort_tau: [f64; 22],
    speed_scale: [f64; 22],
    after_speed_tau: [f64; 22],
    target_to_motor_delta_rad: [f64; 22],
    joint_receive_age_ms: f64,
    imu_receive_age_ms: f64,
    target_age_ms: Option<f64>,
    observation_age_ms: Option<f64>,
}
#[derive(Serialize)]
struct RtFrame {
    stages: Box<RtStages>,
    rejected_targets: u64,
    last_rejection: Option<String>,
    rejected_at: f64,
    publish_interval_ms: f64,
    requested_publish_hz: u32,
    at: f64,
    hal_sequence: u64,
    accepted: Option<u64>,
    target_sequence: Option<u64>,
    epoch: Option<u64>,
    generated: Option<f64>,
    observed: Option<f64>,
    expires: Option<f64>,
    q: [f32; 22],
    qd: [f32; 22],
    measured_effort: [f32; 22],
    temperature: [f32; 22],
    quaternion_xyzw: [f32; 4],
    omega: [f32; 3],
    joint_stamp: f64,
    imu_stamp: f64,
    commanded_q: [f32; 22],
    kp: [f32; 22],
    kd: [f32; 22],
    estimated_effort: [f32; 22],
    applied_policy_raw: Option<[f32; 22]>,
    published_policy_raw: Option<[f32; 22]>,
    reference_rotation: Option<[[f32; 3]; 3]>,
    saturation: [u8; 22],
    tilt_scale: f32,
    mode: String,
    fault: Option<String>,
    released: bool,
    interval_ms: f64,
}
struct Shared {
    rejected_targets: u64,
    last_rejection: Option<String>,
    rejected_at: f64,
    status: BridgeStatus,
    target: Option<Target>,
    last_sequence: Option<u64>,
    pending_until: f64,
    fault_pose: Option<[f32; 22]>,
    keep_fallback: bool,
    target_saturation: [u8; 22],
}
fn stamp(last: &mut i64) -> r2r::builtin_interfaces::msg::Time {
    let t = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default();
    *last = (*last + 1).max(t.as_nanos() as i64);
    r2r::builtin_interfaces::msg::Time {
        sec: (*last / 1_000_000_000) as i32,
        nanosec: (*last % 1_000_000_000) as u32,
    }
}
fn seconds(s: &r2r::builtin_interfaces::msg::Time) -> f64 {
    s.sec as f64 + s.nanosec as f64 * 1e-9
}
pub struct RunOptions {
    pub publish_hz: u32,
    pub allow: bool,
    pub commissioning: bool,
    pub stand_trial: bool,
    pub policy_v2: bool,
    pub resume_supported_hold: bool,
    pub resume_damping: bool,
    pub rt_priority: i32,
}
pub fn run(
    profile: Hardware,
    socket: &Path,
    options: RunOptions,
    quit: Arc<AtomicBool>,
) -> Result<(), String> {
    let RunOptions {
        publish_hz,
        allow,
        commissioning,
        stand_trial,
        policy_v2,
        resume_supported_hold,
        resume_damping,
        rt_priority,
    } = options;
    if !matches!(publish_hz, 50 | 500) || (publish_hz != 500 && !stand_trial) {
        return Err("Only 50/500Hz are supported; 50Hz requires explicit stand trial".into());
    }
    profile.validate()?;
    let mut blockers = if stand_trial {
        profile.stand_trial_blockers()
    } else if commissioning {
        profile.commissioning_blockers()
    } else {
        profile.blockers()
    };
    blockers.push("HAL发布者发现检查尚未完成".into());
    if !allow {
        blockers.push("本次为真实ROS只读连接，未允许执行器写入".into());
    }
    let shared = Arc::new(
        PiMutex::new(Shared {
            rejected_targets: 0,
            last_rejection: None,
            rejected_at: 0.,
            status: BridgeStatus {
                publish_hz,
                policy_protocol: if policy_v2 {
                    super::policy::PROTOCOL
                } else {
                    1
                },
                blockers,
                mode: "READ_ONLY".into(),
                ..Default::default()
            },
            target: None,
            last_sequence: None,
            pending_until: 0.,
            fault_pose: None,
            keep_fallback: false,
            target_saturation: [0; 22],
        })
        .map_err(|e| format!("Priority inheritance state mutex unavailable: {e}"))?,
    );
    let parent = socket.parent().ok_or("Invalid IPC path")?;
    std::fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    let flight = Recorder::<RtFrame>::open(&parent.join("flight"), "rt", 64 * 1024 * 1024)
        .map_err(|e| e.to_string())?;
    flight.manifest(&serde_json::json!({"source":"real_q1_ros2_hal_publish","schema":1,"stage_log_version":2,"hardware":profile,"model_order":NAMES,"hal_order_model_indices":HAL_ORDER})).map_err(|e|e.to_string())?;
    shared.lock().unwrap().status.flight_directory = flight.directory.to_string_lossy().into();
    use std::os::unix::{fs::PermissionsExt, io::AsRawFd};
    let lock = std::fs::OpenOptions::new()
        .create(true)
        .truncate(false)
        .write(true)
        .open(socket.with_extension("lock"))
        .map_err(|e| e.to_string())?;
    if unsafe { libc::flock(lock.as_raw_fd(), libc::LOCK_EX | libc::LOCK_NB) } != 0 {
        return Err("Another bridge owns the host lock".into());
    }
    if socket.exists() {
        std::fs::remove_file(socket).map_err(|e| e.to_string())?;
    }
    let listener = Socket::listen(socket).map_err(|e| e.to_string())?;
    std::fs::set_permissions(socket, std::fs::Permissions::from_mode(0o600))
        .map_err(|e| e.to_string())?;
    let mut node = r2r::Node::create(
        r2r::Context::create().map_err(|e| e.to_string())?,
        "q1_supermimic_rt",
        "",
    )
    .map_err(|e| e.to_string())?;
    let qos = QosProfile {
        depth: 1,
        ..QosProfile::sensor_data()
    };
    let mut joints = node
        .subscribe::<msg::JointStateArray>(&profile.joint_topic, qos.clone())
        .map_err(|e| e.to_string())?;
    let mut imu = node
        .subscribe::<r2r::sensor_msgs::msg::Imu>(&profile.imu_topic, qos.clone())
        .map_err(|e| e.to_string())?;
    let mut odometry = node
        .subscribe::<r2r::nav_msgs::msg::Odometry>("/odom", qos.clone())
        .map_err(|e| e.to_string())?;
    // No command publisher exists at all in the default real read-only profile.
    let publisher = if allow
        && (if stand_trial {
            profile.stand_trial_blockers()
        } else if commissioning {
            profile.commissioning_blockers()
        } else {
            profile.blockers()
        })
        .is_empty()
    {
        Some(
            node.create_publisher::<msg::JointCommandArray>(&profile.command_topic, qos)
                .map_err(|e| e.to_string())?,
        )
    } else {
        None
    };
    let io_state = shared.clone();
    let io_quit = quit.clone();
    let hw = profile.clone();
    let io_thread = thread::Builder::new()
        .name("q1-rt-ipc".into())
        .spawn(move || {
            let mut client: Option<Socket> = None;
            let mut last_status = 0.;
            while !io_quit.load(Ordering::Acquire) {
                if client.is_none() {
                    if let Ok(s) = listener.accept() {
                        client = Some(s);
                    }
                }
                let mut disconnected = false;
                if let Some(s) = &client {
                    for _ in 0..8 {
                        match s.receive::<Request>() {
                            Ok(Some(req)) => {
                                let mut a = io_state.lock().unwrap();
                                let now = clock();
                                let (req, reference) = match req {
                                    Request::PolicyTarget {version,target,reference_rotation} => {
                                        if !policy_v2 || version != super::policy::PROTOCOL || !super::policy::valid_rotation(reference_rotation) {
                                            a.status.fault.get_or_insert_with(|| "Invalid native policy protocol/reference".into());
                                            continue;
                                        }
                                        (Request::Target(target),Some(reference_rotation))
                                    }
                                    Request::Target(_) if policy_v2 => {
                                        a.status.fault.get_or_insert_with(|| "Native RT requires versioned policy targets".into());
                                        continue;
                                    }
                                    other => (other,None)
                                };
                                match req {
                                    Request::PolicyTarget {..} => unreachable!(),
                                    Request::Observe => {}
                                    Request::Arm { session } => {
                                        if a.status.session.is_none()
                                            && a.status.fault.is_none()
                                            && a.status.blockers.is_empty()
                                            && a.status
                                                .sensors
                                                .validate(now, hw.state_timeout_s, hw.sensor_skew_s)
                                                .is_ok()
                                        {
                                            a.status.session = Some(session);
                                            a.last_sequence = None;
                                            a.status.accepted = None;
                                            a.target = None;
                                            // No output until the first fresh target; allow a bounded network handshake.
                                            a.pending_until = now + 0.25;
                                            a.status.mode = "ARMING".into();
                                        }
                                    }
                                    Request::Target(mut t) => {
                                        let valid = a.status.session == Some(t.session)
                                            && a.status.fault.is_none()
                                            && a.last_sequence.is_none_or(|n| t.sequence > n)
                                            && (0. ..=hw.target_timeout_s)
                                                .contains(&(now - t.generated))
                                            && t.expires > now
                                            && t.expires - t.generated
                                                <= hw.target_timeout_s + 1e-6
                                            && (0. ..=hw.state_timeout_s)
                                                .contains(&(now - t.observation))
                                            && (!t.moving || t.operator_expires > now)
                                            && t.raw
                                                .iter()
                                                .all(|x| x.is_finite() && x.abs() <= 20.)
                                            && t.q.iter().all(|x| x.is_finite());
                                        let valid = valid
                                            && t.gain_scale.is_finite()
                                            && (0. ..=1.).contains(&t.gain_scale);
                                        if valid {
                                            let soft = stand_trial && hw.stand_trial.as_ref().unwrap().soft_limits;
                                            a.target_saturation = [0; 22];
                                            if soft {
                                                let previous=a.target.as_ref().map_or(a.status.sensors.q,|old|old.q);
                                                let max=if a.target.is_some() {hw.max_step_rad} else {hw.max_start_error_rad};
                                                for (i, old) in previous.iter().enumerate() {
                                                    let bounded=t.q[i].clamp(hw.lower[i],hw.upper[i]);
                                                    if (bounded-t.q[i]).abs()>1e-6 {a.target_saturation[i]|=1;}
                                                    let slewed=*old+(bounded-*old).clamp(-max,max);
                                                    if (slewed-bounded).abs()>1e-6 {a.target_saturation[i]|=2;}
                                                    t.q[i]=slewed.clamp(hw.lower[i],hw.upper[i]);
                                                }
                                            }
                                            let limits = (0..22).all(|i| {
                                                t.q[i] >= hw.lower[i] && t.q[i] <= hw.upper[i]
                                            });
                                            let previous = a
                                                .target
                                                .as_ref()
                                                .map_or(a.status.sensors.q, |old| old.q);
                                            let max = if a.target.is_some() {
                                                hw.max_step_rad
                                            } else {
                                                hw.max_start_error_rad
                                            };
                                            if limits
                                                && (0..22)
                                                    .all(|i| (t.q[i] - previous[i]).abs() <= max + 1e-6)
                                            {
                                                a.last_sequence = Some(t.sequence);
                                                a.status.reference_rotation = reference;
                                                a.target = Some(*t);
                                                a.keep_fallback = false;
                                                a.status.released = false;
                                                a.fault_pose = None;
                                            } else {
                                                let i=(0..22).find(|i| t.q[*i]<hw.lower[*i] || t.q[*i]>hw.upper[*i] || (t.q[*i]-previous[*i]).abs()>max+1e-6).unwrap();
                                                a.status.fault = Some(format!("{} 目标{:.6}rad，前目标{:.6}rad，允许步长{max:.6}rad",NAMES[i],t.q[i],previous[i]));
                                            }
                                        } else if a.status.session.is_some() {
                                            let bad_numeric=!t.q.iter().chain(t.raw.iter()).all(|v|v.is_finite())
                                                || t.raw.iter().any(|v|v.abs()>20.) || !t.gain_scale.is_finite() || !(0. ..=1.).contains(&t.gain_scale)
                                                || ![t.generated,t.observation,t.expires,t.operator_expires].iter().all(|v|v.is_finite());
                                            if !stand_trial || bad_numeric {
                                                a.status.fault.get_or_insert_with(||"不合法的策略目标数值或元数据".into());
                                            } else {
                                                // A late or replayed packet is rejected, not
                                                // applied or re-stamped. The current valid
                                                // target retains its original RT expiry.
                                                let reason=if a.status.session!=Some(t.session) {"session_mismatch"}
                                                    else if a.last_sequence.is_some_and(|n|t.sequence<=n) {"replayed_sequence"}
                                                    else if a.status.fault.is_some() {"session_already_faulted"}
                                                    else if now-t.observation>hw.state_timeout_s {"late_observation"}
                                                    else if now-t.generated>hw.target_timeout_s || t.expires<=now {"late_prediction"}
                                                    else if now<t.generated || now<t.observation {"future_timestamp"}
                                                    else {"expired_authorization_or_invalid_window"};
                                                a.rejected_targets+=1;
                                                a.rejected_at=now;
                                                a.last_rejection=Some(format!("{reason}: seq={}, observation_age_ms={:.3}, prediction_age_ms={:.3}",t.sequence,(now-t.observation)*1000.,(now-t.generated)*1000.));
                                            }
                                        }
                                    }
                                    Request::Disarm { session } => {
                                        if a.status.session == Some(session) {
                                            a.status.fault.get_or_insert_with(|| {
                                                "显式释放请求：执行本次配置的故障流程".into()
                                            });
                                        }
                                    }
                                    Request::Reset { session } => {
                                        if stand_trial
                                            && (a.status.session == Some(session)
                                                || (session == 0
                                                    && a.status.session.is_none()
                                                    && a.keep_fallback))
                                            && a.status.fault.is_some()
                                            && a.status
                                                .sensors
                                                .validate(now, hw.state_timeout_s, hw.sensor_skew_s)
                                                .is_ok()
                                            && a.status.sensors.qd.iter().all(|v| v.abs() < 0.25)
                                        {
                                            a.status.fault = None;
                                            a.status.session = None;
                                            a.target = None;
                                            a.last_sequence = None;
                                            a.status.accepted = None;
                                            a.status.mode = "SAFE_HOLD".into();
                                            a.keep_fallback = true;
                                            a.fault_pose = Some(a.status.sensors.q);
                                        }
                                    }
                                    Request::Release { session } => {
                                        if stand_trial && a.status.session == session {
                                            a.status.session=None;
                                            a.status.fault=None;
                                            a.status.released=true;
                                            a.status.mode="DAMPING".into();
                                            a.target=None;
                                            a.last_sequence=None;
                                            a.status.accepted=None;
                                            a.keep_fallback=true;
                                            a.fault_pose=Some(a.status.sensors.q);
                                        }
                                    }
                                }
                            }
                            Ok(None) => break,
                            Err(_) => {
                                disconnected = true;
                                break;
                            }
                        }
                    }
                    if clock() - last_status >= 0.002 {
                        let state = io_state.lock().unwrap().status.clone();
                        if let Err(e) = s.send(&state) {
                            if e.kind() != std::io::ErrorKind::WouldBlock {
                                disconnected = true;
                            }
                        }
                        last_status = clock();
                    }
                }
                if disconnected {
                    client = None;
                    let mut a = io_state.lock().unwrap();
                    if a.status.session.is_some() {
                        a.status.fault.get_or_insert_with(|| "策略IPC断开".into());
                    }
                }
                thread::sleep(Duration::from_millis(1));
            }
        })
        .map_err(|e| e.to_string())?;
    let rx_state = shared.clone();
    let rx_quit = quit.clone();
    let rx_hw = profile.clone();
    let rx_thread = thread::Builder::new()
        .name("q1-ros-sensors".into())
        .spawn(move || {
            let mut sensors = Sensors::default();
            let began = clock();
            let mut graph_at = 0.;
            while !rx_quit.load(Ordering::Acquire) {
                node.spin_once(Duration::from_micros(200));
                let now = clock();
                let graph = if now - began >= 1. && now - graph_at >= 0.1 {
                    graph_at = now;
                    Some(node.get_publishers_info_by_topic(&rx_hw.command_topic, false))
                } else {
                    None
                };
                if let Some(graph) = graph {
                    let mut a = rx_state.lock().unwrap();
                    a.status.blockers.retain(|s| !s.starts_with("HAL发布者"));
                    match graph {
                        Ok(peers)
                            if peers.len() > 1
                                || peers.iter().any(|p| p.node_name != "q1_supermimic_rt") =>
                        {
                            a.status.blockers.push("HAL发布者存在其它写入者".into());
                            if a.status.session.is_some() {
                                a.status
                                    .fault
                                    .get_or_insert_with(|| "检测到其它HAL写入者".into());
                            }
                        }
                        Err(_) => {
                            a.status.blockers.push("HAL发布者查询失败".into());
                            if a.status.session.is_some() {
                                a.status
                                    .fault
                                    .get_or_insert_with(|| "HAL发布者查询失败".into());
                            }
                        }
                        _ => {}
                    }
                }
                for _ in 0..64 {
                    let Some(Some(m)) = joints.next().now_or_never() else {
                        break;
                    };
                    let ts = seconds(&m.header.stamp);
                    if ts <= sensors.joint_stamp {
                        continue;
                    }
                    let s = &mut sensors;
                    let mut seen = [false; 22];
                    let mut good = true;
                    let mut positions = [0f32; 22];
                    let mut velocities = [0f32; 22];
                    let mut efforts = [0f32; 22];
                    let mut temperatures = [0f32; 22];
                    for j in m.joints {
                        if let Some(i) = NAMES.iter().position(|n| *n == j.name) {
                            if seen[i] || !j.position.is_finite() || !j.velocity.is_finite() {
                                good = false;
                                break;
                            }
                            seen[i] = true;
                            positions[i] = j.position as f32;
                            velocities[i] = j.velocity as f32;
                            efforts[i] = j.effort as f32;
                            temperatures[i] = j.motor_temp.max(j.coil_temp) as f32;
                            if !positions[i].is_finite()
                                || !velocities[i].is_finite()
                                || !efforts[i].is_finite()
                            {
                                good = false;
                                break;
                            }
                        }
                    }
                    s.complete = good && seen.iter().all(|x| *x);
                    if s.complete {
                        s.q = positions;
                        s.qd = velocities;
                        s.effort = efforts;
                        s.temperature = temperatures;
                    }
                    s.joint_stamp = ts;
                    s.joints_at = now;
                    s.joint_frames += 1;
                }
                for _ in 0..64 {
                    let Some(Some(m)) = imu.next().now_or_never() else {
                        break;
                    };
                    let ts = seconds(&m.header.stamp);
                    if ts <= sensors.imu_stamp {
                        continue;
                    }
                    let s = &mut sensors;
                    s.quaternion_xyzw = [
                        m.orientation.x as f32,
                        m.orientation.y as f32,
                        m.orientation.z as f32,
                        m.orientation.w as f32,
                    ];
                    s.omega = [
                        m.angular_velocity.x as f32,
                        m.angular_velocity.y as f32,
                        m.angular_velocity.z as f32,
                    ];
                    s.imu_valid = m.orientation_covariance[0] != -1.
                        && m.angular_velocity_covariance[0] != -1.
                        && m.header.frame_id == rx_hw.imu_frame
                        && crate::supermimic::observation::rotation(s.quaternion_xyzw).is_ok();
                    s.imu_frame = m.header.frame_id;
                    s.imu_at = now;
                    s.imu_stamp = ts;
                    s.imu_frames += 1;
                }
                for _ in 0..32 {
                    let Some(Some(m)) = odometry.next().now_or_never() else {
                        break;
                    };
                    let ts = seconds(&m.header.stamp);
                    if ts <= sensors.root_velocity_stamp {
                        continue;
                    }
                    let v = m.twist.twist.linear;
                    let velocity = [v.x as f32, v.y as f32, v.z as f32];
                    sensors.root_velocity = (!m.child_frame_id.is_empty()
                        && velocity.iter().all(|x| x.is_finite())
                        && m.twist.covariance.iter().all(|x| x.is_finite())
                        && [0, 7, 14].iter().all(|i| m.twist.covariance[*i] >= 0.))
                    .then_some(velocity);
                    sensors.root_velocity_at = now;
                    sensors.root_velocity_stamp = ts;
                }
                rx_state.lock().unwrap().status.sensors = sensors.clone();
                thread::sleep(Duration::from_micros(200));
            }
        })
        .map_err(|e| e.to_string())?;
    // Even a single-joint trial must fill all 22 fixed slots. Partial arrays
    // are not a supported ABI of the installed device_manager callback.
    let controlled = HAL_ORDER;
    shared.lock().unwrap().status.controlled_joints = if commissioning { 1 } else { 22 };
    let mut packet = msg::JointCommandArray {
        joints: controlled
            .iter()
            .map(|i| msg::JointCommand {
                name: NAMES[*i].into(),
                ..Default::default()
            })
            .collect(),
        ..Default::default()
    };
    if rt_priority > 0 {
        if !(1..=50).contains(&rt_priority) {
            return Err("RT priority must be 1..50".into());
        }
        let param = libc::sched_param {
            sched_priority: rt_priority,
        };
        if unsafe { libc::pthread_setschedparam(libc::pthread_self(), libc::SCHED_FIFO, &param) }
            != 0
        {
            return Err("Cannot set requested real-time scheduling priority".into());
        }
    }
    let mut deadline = Instant::now();
    let mut last = clock();
    let mut start = last;
    let mut count = 0u64;
    let mut ticks = 0u64;
    let mut last_tick = last;
    let mut last_stamp = 0;
    let mut resume_hold_pending = resume_supported_hold || resume_damping;
    let mut last_log = 0.;
    let mut logged_mode = String::new();
    let mut fast_log_until = 0.;
    let mut next_publish = clock();
    let mut published_fault: Option<String> = None;
    let mut published_released = false;
    while !quit.load(Ordering::Acquire) {
        let mut a = shared.lock().unwrap();
        let now = clock();
        let interval_ms = (now - last_tick) * 1000.;
        if now - last_tick > 0.005 {
            a.status.intervals_over_5ms += 1;
        }
        // The supported stand trial keeps bounded outputs across ordinary Linux jitter.
        // Fresh-feedback and policy expiry remain independently enforced below.
        let gap_limit = if stand_trial { 0.06 } else { 0.005 };
        if a.status.session.is_some()
            && a.target.is_some()
            && now - last_tick > gap_limit
            && a.status.fault.is_none()
        {
            a.status.fault = Some(format!("本体控制线程间隔超过{}ms", gap_limit * 1000.));
        }
        ticks += 1;
        a.status.scheduler_max_gap_ms =
            a.status.scheduler_max_gap_ms.max((now - last_tick) * 1000.);
        last_tick = now;
        if resume_hold_pending
            && a.status.blockers.is_empty()
            && a.status
                .sensors
                .validate(now, profile.state_timeout_s, profile.sensor_skew_s)
                .is_ok()
        {
            a.fault_pose = Some(a.status.sensors.q);
            a.keep_fallback = true;
            a.status.applied_gain_scale = 1.;
            a.status.released = resume_damping;
            resume_hold_pending = false;
        }
        if a.status.session.is_some() || a.keep_fallback {
            let error = a
                .status
                .sensors
                .validate(now, profile.state_timeout_s, profile.sensor_skew_s)
                .err()
                .or_else(|| {
                    if stand_trial
                        && !profile.stand_trial.as_ref().unwrap().soft_limits
                        && !a.status.released
                    {
                        let limits = &profile
                            .stand_trial
                            .as_ref()
                            .unwrap()
                            .measured_velocity_limit;
                        if let Some(i) =
                            (0..22).find(|i| a.status.sensors.qd[*i].abs() > limits[*i])
                        {
                            return Some(format!(
                                "{} 实测速度{:.2}rad/s超过{:.2}rad/s",
                                NAMES[i], a.status.sensors.qd[i], limits[i]
                            ));
                        }
                    }
                    if a.status
                        .sensors
                        .temperature
                        .iter()
                        .any(|t| *t > profile.max_temperature)
                    {
                        Some("电机温度超过已确认阈值".into())
                    } else {
                        None
                    }
                })
                .or_else(|| {
                    match crate::supermimic::observation::pelvis(&a.status.sensors, &profile) {
                        Ok(r)
                            if r[2][2] < profile.max_tilt_rad.cos()
                                && !(stand_trial
                                    && profile.stand_trial.as_ref().unwrap().soft_limits)
                                && !a.status.released =>
                        {
                            Some("机身倾角超过运行上限".into())
                        }
                        Err(e) => Some(e),
                        _ => None,
                    }
                })
                .or_else(|| {
                    a.target
                        .as_ref()
                        .filter(|_| {
                            !(stand_trial && profile.stand_trial.as_ref().unwrap().soft_limits)
                        })
                        .and_then(|t| {
                            // The trial applies a torque cap by moving the published
                            // goal. Compare feedback with that actual motor command.
                            let applied = if stand_trial && a.status.tx_count > 0 {
                                &a.status.commanded_q
                            } else {
                                &t.q
                            };
                            (0..22)
                                .find(|i| {
                                    (applied[*i] - a.status.sensors.q[*i]).abs()
                                        > profile.max_tracking_error_rad
                                })
                                .map(|i| {
                                    format!(
                                        "{} 实际下发跟踪误差{:.3}rad",
                                        NAMES[i],
                                        (applied[i] - a.status.sensors.q[i]).abs()
                                    )
                                })
                        })
                })
                .or_else(|| match &a.target {
                    Some(t) if now > t.expires => Some(format!(
                        "策略目标过期，生成距今{:.1}ms",
                        (now - t.generated) * 1000.
                    )),
                    Some(t) if t.moving && now > t.operator_expires => {
                        Some("初始化操作租期过期".into())
                    }
                    None if a.status.session.is_some() && now > a.pending_until => {
                        Some("接管首帧超时".into())
                    }
                    _ => None,
                });
            if a.status.fault.is_none() {
                a.status.fault = error;
            }
            if let Some(p) = &publisher {
                let urgent =
                    a.status.fault != published_fault || a.status.released != published_released;
                let due = publish_hz == 500 || now + 1e-6 >= next_publish || urgent;
                if (a.target.is_some() || a.status.fault.is_some() || a.keep_fallback) && due {
                    let publish_interval_ms = (now - last) * 1000.;
                    let failed = a.status.fault.is_some() || a.keep_fallback;
                    if failed && a.fault_pose.is_none() {
                        a.fault_pose = Some(a.status.sensors.q);
                    }
                    if failed {
                        a.keep_fallback = true;
                    }
                    a.status.saturation = if a.status.released {
                        [0; 22]
                    } else {
                        a.target_saturation
                    };
                    let tilt = crate::supermimic::observation::pelvis(&a.status.sensors, &profile)
                        .map(|r| {
                            if policy_v2 {
                                a.status
                                    .reference_rotation
                                    .map_or(0., |reference| super::policy::tilt(r, reference))
                            } else {
                                r[2][2].clamp(-1., 1.).acos()
                            }
                        })
                        .unwrap_or(0.);
                    a.status.tilt_scale =
                        if stand_trial && profile.stand_trial.as_ref().unwrap().soft_limits {
                            (profile.max_tilt_rad / tilt.max(profile.max_tilt_rad)).clamp(0.2, 1.)
                        } else {
                            1.
                        };
                    let mut stages = RtStages {
                        accepted_target_q: a.target.as_ref().map(|t| t.q),
                        before_tracking_q: [0.; 22],
                        after_tracking_q: [0.; 22],
                        pd_active: [false; 22],
                        before_effort_tau: [0.; 22],
                        after_effort_tau: [0.; 22],
                        speed_scale: [1.; 22],
                        after_speed_tau: [0.; 22],
                        target_to_motor_delta_rad: [0.; 22],
                        joint_receive_age_ms: (now - a.status.sensors.joints_at) * 1000.,
                        imu_receive_age_ms: (now - a.status.sensors.imu_at) * 1000.,
                        target_age_ms: a.target.as_ref().map(|t| (now - t.generated) * 1000.),
                        observation_age_ms: a
                            .target
                            .as_ref()
                            .map(|t| (now - t.observation) * 1000.),
                    };
                    for (slot, &i) in controlled.iter().enumerate() {
                        let j = &mut packet.joints[slot];
                        j.position = if failed {
                            a.fault_pose.as_ref().unwrap()[i] as f64
                        } else {
                            a.target.as_ref().unwrap().q[i] as f64
                        };
                        j.velocity = 0.;
                        j.effort = 0.;
                        j.stiffness = if failed {
                            if commissioning {
                                0.5
                            } else {
                                0.
                            }
                        } else {
                            profile.kp[i] as f64
                        };
                        j.damping = if failed {
                            profile.fault_kd[i]
                        } else {
                            profile.kd[i]
                        } as f64;
                        if commissioning && i != 17 {
                            j.position = a.status.sensors.q[i] as f64;
                            j.stiffness = 0.;
                            j.damping = 0.;
                        }
                        stages.before_tracking_q[i] = j.position;
                        stages.after_tracking_q[i] = j.position;
                        if stand_trial {
                            let t = profile.stand_trial.as_ref().unwrap();
                            let gain = a
                                .target
                                .as_ref()
                                .map_or(a.status.applied_gain_scale.max(0.1), |x| x.gain_scale);
                            // A fault captures measured pose. Keep supporting PD
                            // gains and the same torque cap; dropping stiffness
                            // on a loaded robot causes another gravity transient.
                            j.stiffness = (profile.kp[i] * gain) as f64;
                            j.damping = (profile.kd[i] * gain.max(0.2)) as f64;
                            let q = a.status.sensors.q[i] as f64;
                            let qd = a.status.sensors.qd[i] as f64;
                            if a.status.released {
                                j.stiffness = 0.;
                                j.damping = (profile.fault_kd[i].min(0.5) as f64)
                                    .min(t.effort_limit[i] as f64 / qd.abs().max(1e-6));
                                j.position = q;
                            } else if t.soft_limits {
                                let correction =
                                    (profile.max_tracking_error_rad * a.status.tilt_scale) as f64;
                                let limited = j.position.clamp(q - correction, q + correction);
                                if (limited - j.position).abs() > 1e-6 {
                                    a.status.saturation[i] |= 4;
                                }
                                j.position = limited;
                                if a.status.tilt_scale < 0.999 {
                                    a.status.saturation[i] |= 32;
                                }
                            }
                            stages.after_tracking_q[i] = j.position;
                            // Zero desired speed/effort: bound the computed PD torque by
                            // restricting the commanded position around live measured state.
                            if j.stiffness > 1e-6 {
                                stages.pd_active[i] = true;
                                let limit = t.effort_limit[i] as f64;
                                let requested_torque =
                                    j.stiffness * (j.position - q) - j.damping * qd;
                                stages.before_effort_tau[i] = requested_torque;
                                let mut torque = requested_torque.clamp(-limit, limit);
                                stages.after_effort_tau[i] = torque;
                                if t.soft_limits && torque * qd > 0. {
                                    let speed = t.measured_velocity_limit[i] as f64;
                                    let scale = ((speed - qd.abs()) / (speed * 0.2)).clamp(0., 1.);
                                    stages.speed_scale[i] = scale;
                                    if scale < 0.999 {
                                        a.status.saturation[i] |= 16;
                                        torque *= scale;
                                    }
                                }
                                stages.after_speed_tau[i] = torque;
                                let requested = j.stiffness * (j.position - q) - j.damping * qd;
                                if (torque - requested).abs() > 1e-5 {
                                    a.status.saturation[i] |= 8;
                                }
                                j.position = q + (torque + j.damping * qd) / j.stiffness;
                                let lower = q + (j.damping * qd - limit) / j.stiffness;
                                let upper = q + (j.damping * qd + limit) / j.stiffness;
                                j.position = j
                                    .position
                                    .clamp(lower, upper)
                                    .clamp(profile.lower[i] as f64, profile.upper[i] as f64);
                            }
                            a.status.estimated_effort[i] =
                                (j.stiffness * (j.position - q) - j.damping * qd) as f32;
                            a.status.applied_gain_scale = if a.status.released { 0. } else { gain };
                            a.status.applied_raw[i] = ((profile.signs[i]
                                * (j.position as f32 - profile.zeros[i])
                                - t.home[i]
                                + profile.encoder_bias[i])
                                / 0.25)
                                .clamp(-20., 20.);
                        } else if !failed {
                            a.status.applied_raw[i] = a.target.as_ref().unwrap().raw[i];
                        }
                        stages.target_to_motor_delta_rad[i] =
                            stages.before_tracking_q[i] - j.position;
                        a.status.commanded_q[i] = j.position as f32;
                        a.status.commanded_kp[i] = j.stiffness as f32;
                        a.status.commanded_kd[i] = j.damping as f32;
                    }
                    if a.status.saturation.iter().any(|v| *v != 0) {
                        a.status.saturated_frames += 1;
                    }
                    packet.header.stamp = stamp(&mut last_stamp);
                    packet.header.sequence = packet.header.sequence.wrapping_add(1);
                    if let Err(e) = p.publish(&packet) {
                        a.status.fault = Some(format!("HAL publish failed: {e}"));
                    } else {
                        if a.status.tx_count > 0 {
                            a.status.tx_max_gap_ms =
                                a.status.tx_max_gap_ms.max((now - last) * 1000.);
                        }
                        a.status.published_policy_raw = super::policy::publication(
                            a.status.published_policy_raw,
                            a.target.as_ref().map_or([0.; 22], |t| t.raw),
                            true,
                            !failed,
                        );
                        if !failed {
                            a.status.published_policy_sequence =
                                a.target.as_ref().map(|t| t.sequence);
                        }
                        a.status.tx_count += 1;
                        a.status.hal_sequence = u64::from(packet.header.sequence);
                        published_fault = a.status.fault.clone();
                        published_released = a.status.released;
                        next_publish += 1. / publish_hz as f64;
                        if next_publish <= now {
                            next_publish = now + 1. / publish_hz as f64;
                        }
                        count += 1;
                        last = now;
                        if !failed {
                            a.status.accepted = a.target.as_ref().map(|t| t.sequence);
                        }
                    }
                    a.status.mode = if a.status.released {
                        "DAMPING"
                    } else if a.status.fault.is_some() {
                        "FAULT"
                    } else if failed {
                        "SAFE_HOLD"
                    } else {
                        "ACTIVE"
                    }
                    .into();
                    if a.status.mode != logged_mode {
                        logged_mode = a.status.mode.clone();
                        fast_log_until = now + 2.;
                    }
                    if a.status.mode == "ACTIVE" || now < fast_log_until || now - last_log >= 1. {
                        let b = &a.status;
                        let s = &b.sensors;
                        let t = a.target.as_ref();
                        flight.record(RtFrame {
                            stages: Box::new(stages),
                            rejected_targets: a.rejected_targets,
                            last_rejection: a.last_rejection.clone(),
                            rejected_at: a.rejected_at,
                            publish_interval_ms,
                            requested_publish_hz: publish_hz,
                            at: now,
                            hal_sequence: b.hal_sequence,
                            accepted: b.accepted,
                            target_sequence: t.map(|v| v.sequence),
                            epoch: t.map(|v| v.epoch),
                            generated: t.map(|v| v.generated),
                            observed: t.map(|v| v.observation),
                            expires: t.map(|v| v.expires),
                            q: s.q,
                            qd: s.qd,
                            measured_effort: s.effort,
                            temperature: s.temperature,
                            quaternion_xyzw: s.quaternion_xyzw,
                            omega: s.omega,
                            joint_stamp: s.joint_stamp,
                            imu_stamp: s.imu_stamp,
                            commanded_q: b.commanded_q,
                            kp: b.commanded_kp,
                            kd: b.commanded_kd,
                            estimated_effort: b.estimated_effort,
                            applied_policy_raw: t.map(|v| v.raw),
                            published_policy_raw: b.published_policy_raw,
                            reference_rotation: b.reference_rotation,
                            saturation: b.saturation,
                            tilt_scale: b.tilt_scale,
                            mode: b.mode.clone(),
                            fault: b.fault.clone(),
                            released: b.released,
                            interval_ms,
                        });
                        last_log = now;
                    }
                    a.status.log_written = flight.stats.written.load(Ordering::Relaxed);
                    a.status.log_dropped = flight.stats.dropped.load(Ordering::Relaxed);
                    a.status.log_failed = flight.stats.failed.load(Ordering::Relaxed);
                }
            }
        }
        if now - start >= 1. {
            a.status.tx_hz = count as f64 / (now - start);
            a.status.scheduler_hz = ticks as f64 / (now - start);
            count = 0;
            ticks = 0;
            start = now;
        }
        drop(a);
        deadline += Duration::from_millis(2);
        let current = Instant::now();
        if current > deadline + Duration::from_millis(5) {
            deadline = current;
        }
        thread::sleep(deadline.saturating_duration_since(current));
    }
    // q1-rt termination relies on the independently confirmed robot watchdog.
    quit.store(true, Ordering::Release);
    let _ = io_thread.join();
    let _ = rx_thread.join();
    let _ = std::fs::remove_file(socket);
    Ok(())
}
