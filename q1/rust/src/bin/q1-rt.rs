use clap::Parser;
use std::{
    path::PathBuf,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
};
#[derive(Parser)]
struct Options {
    #[arg(long)]
    hardware_profile: PathBuf,
    #[arg(long, default_value = "/run/q1-supermimic/bridge.sock")]
    socket: PathBuf,
    /// Validate the profile and exit before ROS initialization or hardware writes.
    #[arg(long)]
    validate_only: bool,
    #[arg(long)]
    enable_hardware: bool,
    #[arg(long)]
    commissioning: bool,
    #[arg(long, conflicts_with = "commissioning")]
    stand_trial: bool,
    /// Native policy IPC with reference-relative tilt and published raw feedback.
    #[arg(long, requires = "stand_trial")]
    policy_v2: bool,
    /// Explicitly resume a measured-pose support hold during an authorized RT upgrade.
    #[arg(long, requires_all = ["stand_trial", "enable_hardware"])]
    resume_supported_hold: bool,
    #[arg(long, requires_all = ["stand_trial", "enable_hardware"], conflicts_with="resume_supported_hold")]
    resume_damping: bool,
    #[arg(long, default_value_t = 0)]
    rt_priority: i32,
    #[arg(long, default_value_t = 500)]
    publish_hz: u32,
}
#[tokio::main]
async fn main() {
    let o = Options::parse();
    let h = match q1_edge::supermimic::contract::Hardware::load(&o.hardware_profile) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("{e}");
            std::process::exit(2);
        }
    };
    if o.validate_only {
        let blockers = if o.stand_trial {
            h.stand_trial_blockers()
        } else if o.commissioning {
            h.commissioning_blockers()
        } else {
            h.blockers()
        };
        if !blockers.is_empty() {
            eprintln!("{}", blockers.join("; "));
            std::process::exit(2);
        }
        println!(
            "{}",
            serde_json::json!({"valid":true,"hardware_writes":false,"ros_initialized":false,"profile":o.hardware_profile})
        );
        return;
    }
    let quit = Arc::new(AtomicBool::new(false));
    let signal = quit.clone();
    tokio::spawn(async move {
        let mut term =
            tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate()).unwrap();
        tokio::select! {_=tokio::signal::ctrl_c()=>{},_=term.recv()=>{}}
        signal.store(true, Ordering::Release);
    });
    if let Err(e) = q1_edge::lowlevel::ros::run(
        h,
        &o.socket,
        q1_edge::lowlevel::ros::RunOptions {
            allow: o.enable_hardware,
            commissioning: o.commissioning,
            stand_trial: o.stand_trial,
            policy_v2: o.policy_v2,
            resume_supported_hold: o.resume_supported_hold,
            resume_damping: o.resume_damping,
            rt_priority: o.rt_priority,
            publish_hz: o.publish_hz,
        },
        quit,
    ) {
        eprintln!("q1-rt: {e}");
        std::process::exit(2);
    }
}
