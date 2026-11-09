//! A bounded, supervised REAL hardware commissioning client, not a Mock backend.
use clap::Parser;
use q1_edge::{
    lowlevel::{
        ipc::Socket,
        types::{clock, BridgeStatus, Request, Target},
    },
    supermimic::contract::Hardware,
};
use std::{path::PathBuf, thread, time::Duration};
#[derive(Parser)]
struct Options {
    #[arg(long)]
    socket: PathBuf,
    #[arg(long)]
    hardware_profile: PathBuf,
    #[arg(long)]
    report: PathBuf,
    #[arg(long, default_value_t = 2.)]
    seconds: f64,
    #[arg(long)]
    actuate: bool,
    #[arg(long, default_value_t = 0.)]
    elbow_offset: f32,
}
fn receive(socket: &Socket, timeout: f64) -> Result<BridgeStatus, String> {
    let start = clock();
    loop {
        if let Some(s) = socket
            .receive::<BridgeStatus>()
            .map_err(|e| e.to_string())?
        {
            return Ok(s);
        }
        if clock() - start > timeout {
            return Err("Real guardian status timed out".into());
        }
        thread::sleep(Duration::from_millis(1));
    }
}
fn run(o: &Options, snapshot: &mut Option<BridgeStatus>) -> Result<(), String> {
    if !(1. ..=3.).contains(&o.seconds)
        || !o.elbow_offset.is_finite()
        || o.elbow_offset.abs() > 0.005
    {
        return Err("Air test restricted to 1..3s and elbow offset <=0.005rad".into());
    }
    let h = Hardware::load(&o.hardware_profile)?;
    let socket = Socket::connect(&o.socket).map_err(|e| e.to_string())?;
    socket.send(&Request::Observe).map_err(|e| e.to_string())?;
    let mut state = receive(&socket, 3.)?;
    let began = clock();
    while (state
        .sensors
        .validate(clock(), h.state_timeout_s, h.sensor_skew_s)
        .is_err()
        || !state.blockers.is_empty())
        && clock() - began < 5.
    {
        state = receive(&socket, 1.)?;
    }
    *snapshot = Some(state.clone());
    if !o.actuate {
        let report = serde_json::json!({"real_hardware":true,"actuated":false,"state":state});
        std::fs::write(&o.report, serde_json::to_vec_pretty(&report).unwrap())
            .map_err(|e| e.to_string())?;
        return Ok(());
    }
    if !h.commissioning_blockers().is_empty() {
        return Err(h.commissioning_blockers().join(";"));
    }
    if !state.blockers.is_empty() || state.session.is_some() || state.fault.is_some() {
        return Err(format!(
            "Guardian not ready: {:?} {:?}",
            state.blockers, state.fault
        ));
    }
    state
        .sensors
        .validate(clock(), h.state_timeout_s, h.sensor_skew_s)?;
    let origin = state.sensors.q;
    let session = rand::random();
    socket
        .send(&Request::Arm { session })
        .map_err(|e| e.to_string())?;
    let arm_at = clock();
    while state.session != Some(session) {
        state = receive(&socket, 1.)?;
        *snapshot = Some(state.clone());
        if clock() - arm_at > 1. {
            return Err("Guardian did not grant air test session".into());
        }
    }
    let start = clock();
    let mut last_send = 0.;
    let mut seq = 0;
    let mut max_delta = 0f32;
    let tx_before = state.tx_count;
    let mut last_target_at = 0.;
    while clock() - start < o.seconds {
        while let Some(s) = socket
            .receive::<BridgeStatus>()
            .map_err(|e| e.to_string())?
        {
            state = s;
        }
        *snapshot = Some(state.clone());
        if state.fault.is_some() {
            return Err(format!("Air test fault: {:?}", state.fault));
        }
        let now = clock();
        state
            .sensors
            .validate(now, h.state_timeout_s, h.sensor_skew_s)?;
        for (measured, initial) in state.sensors.q.iter().zip(origin) {
            max_delta = max_delta.max((measured - initial).abs());
        }
        if now - last_send >= 0.02 {
            seq += 1;
            let mut q = origin;
            let phase = ((now - start) / o.seconds).clamp(0., 1.) as f32;
            q[17] += o.elbow_offset * (std::f32::consts::PI * phase).sin();
            let t = Target {
                session,
                epoch: 0,
                sequence: seq,
                generated: now,
                observation: state.sensors.joints_at.min(state.sensors.imu_at),
                expires: now + h.target_timeout_s,
                operator_expires: now + 0.1,
                moving: true,
                q,
                raw: [0.; 22],
                gain_scale: 1.,
            };
            socket
                .send(&Request::Target(Box::new(t)))
                .map_err(|e| e.to_string())?;
            last_send = now;
            last_target_at = now;
        }
        thread::sleep(Duration::from_millis(1));
    }
    // Leave the socket and the 500Hz network sender alive. Repeated old targets
    // must NOT refresh the guardian's independent strategy deadline.
    let end = clock();
    while state.fault.is_none() && clock() - end < 0.5 {
        state = receive(&socket, 1.)?;
    }
    *snapshot = Some(state.clone());
    let report = serde_json::json!({"real_hardware":true,"actuated":state.accepted.is_some(),"test":"bounded_air_hold_then_policy_expiry","duration_s":o.seconds,"elbow_offset_rad":o.elbow_offset,"max_measured_joint_delta_rad":max_delta,"policy_targets":seq,"submitted_hal_frames":state.tx_count-tx_before,"tx_hz":state.tx_hz,"scheduler_hz":state.scheduler_hz,"tx_max_gap_ms":state.tx_max_gap_ms,"fault_observed_after_last_target_ms":(clock()-last_target_at)*1000.,"fault_latched":state.fault.is_some(),"fault":state.fault,"model_motion_qualified":false,"state":state});
    std::fs::write(&o.report, serde_json::to_vec_pretty(&report).unwrap())
        .map_err(|e| e.to_string())?;
    println!("{}", report);
    if !report["fault_latched"].as_bool().unwrap_or(false) {
        return Err("Guardian did not expire repeated old policy targets".into());
    }
    Ok(())
}
fn main() {
    let options = Options::parse();
    let report = options.report.clone();
    let requested = options.actuate;
    let mut snapshot = None;
    if let Err(e) = run(&options, &mut snapshot) {
        let _=std::fs::write(report,serde_json::json!({"status":"failed","real_hardware":true,"actuation_requested":requested,"error":e,"model_motion_qualified":false,"state":snapshot}).to_string());
        eprintln!("q1-air-check: {e}");
        std::process::exit(2);
    }
}
