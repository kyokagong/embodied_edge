//! Numerical/timing measurement only: creates no ROS node, IPC controller or HAL writer.
use clap::Parser;
use q1_edge::supermimic::{contract, model::Model};
use serde_json::json;
use std::{
    path::PathBuf,
    thread,
    time::{Duration, Instant},
};
#[derive(Parser)]
struct Options {
    #[arg(long)]
    model_dir: PathBuf,
    #[arg(long)]
    report: PathBuf,
    #[arg(long, default_value_t = 2)]
    threads: usize,
    #[arg(long, default_value_t = 500)]
    samples: usize,
}
fn run(o: &Options) -> Result<(), String> {
    if !(100..=3000).contains(&o.samples) {
        return Err("samples must be 100..3000".into());
    }
    let root = o.model_dir.canonicalize().map_err(|e| e.to_string())?;
    contract::load(&root)?;
    let mut model = Model::load_with_threads(&root.join("policy.onnx"), o.threads)?;
    let golden = model.golden(&root.join("golden.json"))?;
    let d: serde_json::Value = serde_json::from_slice(
        &std::fs::read(root.join("golden.json")).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())?;
    let inputs: Vec<Vec<f32>> =
        serde_json::from_value(d["obs"].clone()).map_err(|e| e.to_string())?;
    let expected: Vec<Vec<f32>> =
        serde_json::from_value(d["actions"].clone()).map_err(|e| e.to_string())?;
    let mut inference = Vec::with_capacity(o.samples);
    let mut lateness = Vec::with_capacity(o.samples);
    let mut completion = Vec::with_capacity(o.samples);
    let period = Duration::from_millis(20);
    let start = Instant::now();
    let mut deadline = start;
    let mut last_start = start;
    let mut missed = 0;
    for i in 0..o.samples {
        thread::sleep(deadline.saturating_duration_since(Instant::now()));
        let began = Instant::now();
        last_start = began;
        lateness.push(began.saturating_duration_since(deadline).as_secs_f64() * 1000.);
        let actions = model.infer(&inputs[i % inputs.len()])?;
        let ended = Instant::now();
        inference.push(ended.duration_since(began).as_secs_f64() * 1000.);
        completion.push(ended.duration_since(deadline).as_secs_f64() * 1000.);
        for (a, y) in actions.iter().zip(&expected[i % expected.len()]) {
            if (a - y).abs() > 3e-5 + 3e-4 * y.abs() {
                return Err("Paced ONNX parity failure".into());
            }
        }
        deadline += period;
        if ended > deadline {
            missed += 1;
            deadline = ended;
        }
    }
    fn stats(v: &mut [f64]) -> serde_json::Value {
        v.sort_by(f64::total_cmp);
        json!({"p50":v[v.len()/2],"p99":v[v.len()*99/100],"max":v.last()})
    }
    let affinity = std::fs::read_to_string("/proc/self/status")
        .unwrap_or_default()
        .lines()
        .find(|s| s.starts_with("Cpus_allowed_list:"))
        .unwrap_or("")
        .to_owned();
    let report = json!({"hardware_command_sent":false,"ros_node_created":false,"input_source":"68 fixed export golden observations; no plant simulation", "architecture":std::env::consts::ARCH,"ort_intra_threads":o.threads,"affinity":affinity,"iteration":22100,"golden":golden,"paced":{"samples":o.samples,"target_hz":50,"observed_hz":(o.samples-1) as f64 / last_start.duration_since(start).as_secs_f64(),"inference_ms":stats(&mut inference),"start_lateness_ms":stats(&mut lateness),"completion_ms":stats(&mut completion),"missed_20ms_deadlines":missed},"motion_qualified":false});
    std::fs::write(&o.report, serde_json::to_vec_pretty(&report).unwrap())
        .map_err(|e| e.to_string())?;
    println!("{report}");
    Ok(())
}
fn main() {
    let o = Options::parse();
    if let Err(e) = run(&o) {
        eprintln!("q1-model-check: {e}");
        std::process::exit(2);
    }
}
