//! r2r transport for the high-level Q1 MC. Created only by explicit ROS mode.
use crate::sdk::{monotonic, Backend, Engine, Velocity, JOINT_NAMES};
use futures::{future::BoxFuture, FutureExt, StreamExt};
use r2r::{
    aimdk_msgs::{msg, srv},
    QosProfile,
};
use serde_json::Value;
use std::{
    collections::BTreeMap,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    thread,
    time::{Duration, SystemTime, UNIX_EPOCH},
};

pub struct RosBackend {
    state: Arc<Mutex<Engine>>,
    quit: Arc<AtomicBool>,
    worker: Option<thread::JoinHandle<()>>,
}
impl RosBackend {
    pub fn new(config: &Value) -> Result<Self, String> {
        crate::config::validate_ros(config)?;
        let state = Arc::new(Mutex::new(Engine::new(config)));
        let quit = Arc::new(AtomicBool::new(false));
        let (tx, rx) = std::sync::mpsc::sync_channel(1);
        let shared = state.clone();
        let ending = quit.clone();
        let config = config.clone();
        let worker = thread::Builder::new()
            .name("q1-ros-watchdog".into())
            .spawn(move || {
                let outcome = run(&config, shared.clone(), ending, tx);
                if let Err(error) = outcome {
                    shared.lock().unwrap().trip(&error, monotonic(), false);
                }
            })
            .map_err(|e| e.to_string())?;
        match rx.recv_timeout(Duration::from_secs(15)) {
            Ok(Ok(())) => Ok(Self {
                state,
                quit,
                worker: Some(worker),
            }),
            result => {
                quit.store(true, Ordering::Release);
                Err(format!("ROS initialization failed: {result:?}"))
            }
        }
    }
}
impl Backend for RosBackend {
    fn kind(&self) -> &'static str {
        "ros2"
    }
    fn enable(&mut self, _now: f64) -> Result<(), String> {
        self.state.lock().unwrap().enable(monotonic())
    }
    fn request_mode(&mut self, mode: &str, _: f64) -> Result<(), String> {
        if !matches!(mode, "stand" | "walk") {
            return Err("Pico 上肢功能尚未迁移到 Rust".into());
        }
        let e = self.state.lock().unwrap();
        if !e.enabled || e.fault.is_some() {
            return Err("Explicit enable required before a mode request".into());
        }
        Ok(())
    }
    fn set_velocity(&mut self, v: Velocity, _now: f64) -> Result<(), String> {
        self.state.lock().unwrap().set_velocity(v, monotonic())
    }
    fn stop(&mut self, _now: f64) -> Result<(), String> {
        self.state.lock().unwrap().stop(monotonic());
        Ok(())
    }
    fn snapshot(&self, _now: f64) -> Value {
        self.state.lock().unwrap().snapshot(monotonic())
    }
}
impl Drop for RosBackend {
    fn drop(&mut self) {
        self.state.lock().unwrap().stop(monotonic());
        let until = monotonic() + 0.5;
        while monotonic() < until {
            let e = self.state.lock().unwrap();
            if !e.registered && e.source_operation.is_none() {
                break;
            }
            drop(e);
            thread::sleep(Duration::from_millis(10));
        }
        if self.state.lock().unwrap().registered {
            eprintln!("Close could not confirm source release; use native remote");
        }
        self.quit.store(true, Ordering::Release);
        if let Some(worker) = self.worker.take() {
            let _ = worker.join();
        }
    }
}

struct Pending {
    name: &'static str,
    operation: Option<i32>,
    started: f64,
    expired: bool,
    drained: bool,
    response: BoxFuture<'static, Result<Value, String>>,
}
impl Pending {
    fn poll(&mut self, now: f64, timeout: f64) -> Option<Result<Value, String>> {
        if !self.expired && now - self.started > timeout {
            self.expired = true;
            return Some(Err(format!("{} timed out", self.name)));
        }
        let response = self.response.as_mut().now_or_never();
        self.drained = response.is_some();
        response
    }
}
fn stamp(last: &mut i64) -> r2r::builtin_interfaces::msg::Time {
    let ns = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos() as i64;
    *last = ns.max(*last + 1);
    r2r::builtin_interfaces::msg::Time {
        sec: (*last / 1_000_000_000) as i32,
        nanosec: (*last % 1_000_000_000) as u32,
    }
}
fn valid_pose(
    pairs: impl IntoIterator<Item = (String, f64)>,
    sec: i32,
    nanosec: u32,
    config: &Value,
) -> Option<(BTreeMap<String, f64>, i64)> {
    let ns = sec as i64 * 1_000_000_000 + nanosec as i64;
    let age = (SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .ok()?
        .as_nanos() as i64
        - ns) as f64
        / 1e9;
    let limit = config["ros2"]["feedback_timeout_s"]
        .as_f64()
        .unwrap_or(0.25);
    if nanosec >= 1_000_000_000 || ns <= 0 || !(-0.1..=limit).contains(&age) {
        return None;
    }
    let mut pose = BTreeMap::new();
    for (name, value) in pairs {
        if JOINT_NAMES.contains(&name.as_str())
            && (!value.is_finite() || pose.insert(name, value).is_some())
        {
            return None;
        }
    }
    if pose.len() != 10 {
        return None;
    }
    Some((pose, ns))
}

fn run(
    config: &Value,
    state: Arc<Mutex<Engine>>,
    quit: Arc<AtomicBool>,
    ready: std::sync::mpsc::SyncSender<Result<(), String>>,
) -> Result<(), String> {
    let result = initialize(config, state.clone(), quit, ready.clone());
    if let Err(error) = &result {
        let _ = ready.try_send(Err(error.clone()));
    }
    result
}
fn initialize(
    config: &Value,
    state: Arc<Mutex<Engine>>,
    quit: Arc<AtomicBool>,
    ready: std::sync::mpsc::SyncSender<Result<(), String>>,
) -> Result<(), String> {
    let err = |e: r2r::Error| e.to_string();
    let context = r2r::Context::create().map_err(err)?;
    let mut node = r2r::Node::create(
        context,
        config["ros2"]["source_name"].as_str().unwrap_or("edge_q1"),
        "",
    )
    .map_err(err)?;
    let reliable = QosProfile::default();
    let feedback = QosProfile {
        depth: 10,
        ..QosProfile::sensor_data()
    };
    let publisher = node
        .create_publisher::<msg::McLocomotionVelocity>(
            "/aima/mc/locomotion/velocity",
            reliable.clone(),
        )
        .map_err(err)?;
    let mut standard = node
        .subscribe::<r2r::sensor_msgs::msg::JointState>(
            "/upper_body_joint_states",
            feedback.clone(),
        )
        .map_err(err)?;
    let mut native = node
        .subscribe::<msg::JointStateArray>("/aima/hal/joint/state", feedback)
        .map_err(err)?;
    let action = node
        .create_client::<srv::GetMcAction::Service>(
            "/aimdk_5Fmsgs/srv/GetMcAction",
            reliable.clone(),
        )
        .map_err(err)?;
    let owner = node
        .create_client::<srv::GetCurrentInputSource::Service>(
            "/aimdk_5Fmsgs/srv/GetCurrentInputSource",
            reliable.clone(),
        )
        .map_err(err)?;
    let source = node
        .create_client::<srv::SetMcInputSource::Service>(
            "/aimdk_5Fmsgs/srv/SetMcInputSource",
            reliable,
        )
        .map_err(err)?;
    let mut discovered = [false; 3];
    let mut discovery: Vec<BoxFuture<'static, Result<(), String>>> = vec![
        r2r::Node::is_available(&action)
            .map_err(err)?
            .map(|r| r.map_err(|e| e.to_string()))
            .boxed(),
        r2r::Node::is_available(&owner)
            .map_err(err)?
            .map(|r| r.map_err(|e| e.to_string()))
            .boxed(),
        r2r::Node::is_available(&source)
            .map_err(err)?
            .map(|r| r.map_err(|e| e.to_string()))
            .boxed(),
    ];
    ready.send(Ok(())).map_err(|e| e.to_string())?;
    let mut pending: Vec<Pending> = Vec::new();
    let mut last_poll = -1.0;
    let mut last_output = -1.0;
    let mut last_stamp = 0;
    let mut sequence = 0u32;
    let mut last_motion = false;
    let mut last_velocity_log = -1.0;
    let mut sent_source: Option<(String, i32)> = None;
    let service_timeout = config["ros2"]["service_timeout_s"].as_f64().unwrap_or(3.0);
    let poll_interval = config["ros2"]["source_poll_interval_s"]
        .as_f64()
        .unwrap_or(0.1);
    while !quit.load(Ordering::Acquire) {
        let cycle = Instant::now();
        node.spin_once(Duration::from_millis(1));
        let now = monotonic();
        for (found, future) in discovered.iter_mut().zip(discovery.iter_mut()) {
            if !*found {
                if let Some(result) = future.as_mut().now_or_never() {
                    result?;
                    *found = true;
                }
            }
        }
        let mut engine = state.lock().unwrap();
        for _ in 0..64 {
            let Some(Some(message)) = standard.next().now_or_never() else {
                break;
            };
            if message.name.len() != message.position.len() {
                continue;
            }
            let mut unique = std::collections::HashSet::new();
            if message.name.iter().any(|name| !unique.insert(name)) {
                continue;
            }
            if let Some((pose, stamp)) = valid_pose(
                message.name.into_iter().zip(message.position),
                message.header.stamp.sec,
                message.header.stamp.nanosec,
                config,
            ) {
                engine.joints(pose, stamp, now);
            }
        }
        for _ in 0..64 {
            let Some(Some(message)) = native.next().now_or_never() else {
                break;
            };
            if let Some((pose, stamp)) = valid_pose(
                message.joints.into_iter().map(|j| (j.name, j.position)),
                message.header.stamp.sec,
                message.header.stamp.nanosec,
                config,
            ) {
                engine.joints(pose, stamp, now);
            }
        }
        // Poll each future once. A hung RPC never waits on this output worker.
        let mut index = 0;
        while index < pending.len() {
            let response = pending[index].poll(now, service_timeout);
            if response.is_none() {
                index += 1;
                continue;
            }
            if pending[index].expired && !pending[index].drained {
                // r2r retains its response sender internally. Keep this request
                // until its reply is drained, preventing unbounded retry storage.
                if engine.registered || pending[index].operation.is_some() {
                    engine.trip(&format!("{} timed out", pending[index].name), now, true);
                }
                index += 1;
                continue;
            }
            let request = pending.remove(index);
            if request.expired {
                continue;
            }
            let response = response.unwrap_or_else(|| Err(format!("{} timed out", request.name)));
            match response {
                Ok(v) => {
                    if request.name == "GetMcAction" {
                        if v["header"]["code"] == 0
                            && v["info"]["action_desc"]
                                .as_str()
                                .is_some_and(|s| !s.is_empty())
                        {
                            engine.action(
                                v["info"]["action_desc"].as_str().unwrap(),
                                v["info"]["status"]["value"].as_i64()
                                    == Some(msg::McActionStatus::RUNNING as i64),
                                now,
                            );
                        } else if engine.registered {
                            engine.trip("Invalid MC action response", now, true);
                        }
                    } else {
                        let task = &v["response"];
                        let status = task["state"]["value"].as_i64();
                        let accepted = task["header"]["code"] == 0
                            && (status == Some(1)
                                || (status == Some(0) && task["state"]["reason"] == 0));
                        if accepted {
                            if let Some(operation) = request.operation {
                                engine.source_response(operation, now);
                            } else if let Some(name) = v["input_source"]["name"].as_str() {
                                engine.owner(name, now);
                            }
                        } else {
                            engine.trip(&format!("{} failed: {task}", request.name), now, true);
                        }
                    }
                }
                Err(error) => {
                    if engine.registered || request.operation.is_some() {
                        engine.trip(&error, now, true);
                    }
                    // Unknown ADD outcome must retain this epoch and inhibit re-enable.
                    // Retrying ADD could create a second live source on a late server.
                }
            }
        }
        if now - last_poll >= poll_interval {
            if discovered[0] && !pending.iter().any(|p| p.name == "GetMcAction") {
                let mut request = srv::GetMcAction::Request::default();
                request.request.header.stamp = stamp(&mut last_stamp);
                if let Ok(response) = action.request(&request) {
                    pending.push(Pending {
                        name: "GetMcAction",
                        operation: None,
                        started: now,
                        expired: false,
                        drained: false,
                        response: response
                            .map(move |r| {
                                r.map_err(|e| e.to_string()).and_then(|v| {
                                    serde_json::to_value(v).map_err(|e| e.to_string())
                                })
                            })
                            .boxed(),
                    });
                }
            }
            if discovered[1] && !pending.iter().any(|p| p.name == "GetCurrentInputSource") {
                let mut request = srv::GetCurrentInputSource::Request::default();
                request.request.header.stamp = stamp(&mut last_stamp);
                if let Ok(response) = owner.request(&request) {
                    pending.push(Pending {
                        name: "GetCurrentInputSource",
                        operation: None,
                        started: now,
                        expired: false,
                        drained: false,
                        response: response
                            .map(move |r| {
                                r.map_err(|e| e.to_string()).and_then(|v| {
                                    serde_json::to_value(v).map_err(|e| e.to_string())
                                })
                            })
                            .boxed(),
                    });
                }
            }
            last_poll = now;
        }
        engine.watchdog(now);
        if let Some(operation) = engine.source_operation {
            if !discovered[2] {
                engine.source_operation = None;
                engine.trip("SetMcInputSource service unavailable", now, true);
                continue;
            }
            if !pending.iter().any(|p| p.name == "SetMcInputSource")
                && sent_source.as_ref() != Some(&(engine.source.clone(), operation))
            {
                let mut request = srv::SetMcInputSource::Request::default();
                request.request.header.stamp = stamp(&mut last_stamp);
                request.action.value = operation;
                request.input_source.name = engine.source.clone();
                if operation == 1001 {
                    request.input_source.priority =
                        config["ros2"]["priority"].as_i64().unwrap_or(30) as i32;
                    request.input_source.timeout =
                        config["ros2"]["source_timeout_ms"].as_i64().unwrap_or(1000) as i32;
                }
                eprintln!("rpc_request operation={operation} source={}", engine.source);
                sent_source = Some((engine.source.clone(), operation));
                match source.request(&request) {
                    Ok(response) => pending.push(Pending {
                        name: "SetMcInputSource",
                        operation: Some(operation),
                        started: now,
                        expired: false,
                        drained: false,
                        response: response
                            .map(|r| {
                                r.map_err(|e| e.to_string()).and_then(|v| {
                                    serde_json::to_value(v).map_err(|e| e.to_string())
                                })
                            })
                            .boxed(),
                    }),
                    Err(error) => {
                        engine.trip(
                            &format!("SetMcInputSource transport error: {error}"),
                            now,
                            true,
                        );
                        return Err(error.to_string());
                    }
                }
            }
        }
        if engine.registered && now - last_output >= 0.019 {
            let v = engine.output(now);
            let mut packet = msg::McLocomotionVelocity::default();
            packet.header.stamp = stamp(&mut last_stamp);
            packet.header.sequence = sequence;
            sequence = sequence.wrapping_add(1);
            packet.source = engine.source.clone();
            packet.forward_velocity = v.forward;
            packet.lateral_velocity = v.lateral;
            packet.angular_velocity = v.angular;
            publisher.publish(&packet).map_err(err)?;
            if v.moving() != last_motion || now - last_velocity_log >= 1.0 {
                eprintln!("velocity_output source={} sequence={} forward={:.4} lateral={:.4} angular={:.4}",engine.source,sequence,v.forward,v.lateral,v.angular);
                last_motion = v.moving();
                last_velocity_log = now;
            }
            last_output = now;
        }
        drop(engine);
        thread::sleep(Duration::from_millis(10).saturating_sub(cycle.elapsed()));
    }
    Ok(())
}
use std::time::Instant;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn native_feedback_newer_than_callers_time_is_still_fresh_after_lock() {
        let caller_time = monotonic();
        thread::sleep(Duration::from_millis(2));
        let feedback_time = monotonic();
        let mut engine = Engine::new(&serde_json::json!({}));
        engine.action(crate::sdk::WALK, true, feedback_time);
        engine.joints(
            JOINT_NAMES
                .into_iter()
                .map(|name| (name.to_owned(), 0.0))
                .collect(),
            1,
            feedback_time,
        );
        let mut backend = RosBackend {
            state: Arc::new(Mutex::new(engine)),
            quit: Arc::new(AtomicBool::new(true)),
            worker: None,
        };
        let result = backend.enable(caller_time);
        backend.state.lock().unwrap().source_operation = None;
        assert!(result.is_ok(),"fresh feedback received between caller sampling and backend lock must be accepted: {result:?}");
    }

    #[test]
    fn hung_request_is_retained_after_timeout_without_repeated_expiry() {
        let mut pending = Pending {
            name: "GetMcAction",
            operation: None,
            started: 1.0,
            expired: false,
            drained: false,
            response: futures::future::pending().boxed(),
        };
        assert!(pending.poll(4.001, 3.0).unwrap().is_err());
        assert!(pending.poll(10.0, 3.0).is_none());
    }

    #[test]
    fn already_ready_service_reply_cannot_bypass_expired_deadline() {
        let mut pending = Pending {
            name: "SetMcInputSource",
            operation: Some(1001),
            started: 1.0,
            expired: false,
            drained: false,
            response: futures::future::ready(Ok(serde_json::json!({"accepted": true}))).boxed(),
        };
        assert!(pending.poll(4.001, 3.0).unwrap().is_err());
    }

    #[test]
    fn valid_service_reply_is_accepted_within_deadline() {
        let mut pending = Pending {
            name: "GetMcAction",
            operation: None,
            started: 1.0,
            expired: false,
            drained: false,
            response: futures::future::ready(Ok(serde_json::json!({"action": "walk"}))).boxed(),
        };
        assert!(pending.poll(3.99, 3.0).unwrap().is_ok());
    }
}
