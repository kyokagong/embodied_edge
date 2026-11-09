use clap::Parser;
use q1_edge::{
    config,
    core::Controller,
    sdk::{Backend, MockBackend},
    server,
};
use serde_json::{json, Value};
use std::io::{self, BufRead, Write};

#[derive(Parser)]
#[command(about = "Q1 Rust hand-controller console")]
struct Options {
    #[arg(long, default_value="mock", value_parser=["mock","ros2"])]
    backend: String,
    #[arg(long, default_value="native-mc", value_parser=["native-mc","supermimic"])]
    controller: String,
    #[arg(long)]
    model_dir: Option<std::path::PathBuf>,
    #[arg(long)]
    hardware_profile: Option<std::path::PathBuf>,
    #[arg(long, default_value = "/run/q1-supermimic/bridge.sock")]
    bridge_socket: std::path::PathBuf,
    #[arg(long)]
    operator_key_file: Option<std::path::PathBuf>,
    #[arg(long)]
    shadow_only: bool,
    #[arg(long, default_value_t = 2)]
    ort_threads: usize,
    #[arg(long)]
    enable_hardware: bool,
    #[arg(long)]
    config: Option<String>,
    #[arg(long, default_value = "127.0.0.1")]
    host: String,
    #[arg(long, default_value_t = 8766)]
    port: u16,
    #[arg(long)]
    allow_host: Vec<String>,
    #[arg(long)]
    diagnose: bool,
    #[arg(long)]
    replay: bool,
    #[arg(long)]
    trace: Option<String>,
}

fn replay(config: Value) -> Result<(), String> {
    let mut core = Controller::new(Box::new(MockBackend::new()), config);
    let mut previous = 0.;
    let mut output = io::BufWriter::new(io::stdout().lock());
    for line in io::stdin().lock().lines() {
        let record: Value =
            serde_json::from_str(&line.map_err(|e| e.to_string())?).map_err(|e| e.to_string())?;
        let now = record["time"]
            .as_f64()
            .filter(|n| n.is_finite() && *n >= previous)
            .ok_or("Replay time must be finite, nonnegative and monotonic")?;
        previous = now;
        let result = if let Some(input) = record.get("input") {
            core.input(input, now)
        } else if let Some(action) = record["action"].as_str() {
            core.action(action, record["client_id"].as_str(), now)
        } else if record["tick"] == true {
            core.tick(now);
            Ok(())
        } else {
            Err("Replay event requires input, action or tick".into())
        };
        let result = match result {
            Ok(()) => json!({"ok":true,"state":core.state(now)}),
            Err(error) => json!({"ok":false,"error":error,"state":core.state(now)}),
        };
        writeln!(output, "{result}").map_err(|e| e.to_string())?;
    }
    output.flush().map_err(|e| e.to_string())
}

#[tokio::main]
async fn main() {
    if let Err(error) = run(Options::parse()).await {
        eprintln!("Q1 Rust startup failed: {error}");
        std::process::exit(2);
    }
}

async fn run(options: Options) -> Result<(), String> {
    if options.controller == "supermimic" {
        #[cfg(feature = "supermimic")]
        {
            if options.backend != "ros2" || options.replay {
                return Err("SuperMimic uses the real ROS2 bridge; select --backend ros2".into());
            }
            let profile = q1_edge::supermimic::contract::Hardware::load(
                options
                    .hardware_profile
                    .as_deref()
                    .ok_or("Missing --hardware-profile")?,
            )?;
            let root = options.model_dir.as_deref().ok_or("Missing --model-dir")?;
            if options.diagnose {
                let root = root.canonicalize().map_err(|e| e.to_string())?;
                q1_edge::supermimic::contract::load(&root)?;
                let mut model = q1_edge::supermimic::model::Model::load_with_threads(
                    &root.join("policy.onnx"),
                    options.ort_threads,
                )?;
                println!("{}", model.golden(&root.join("golden.json"))?);
                return Ok(());
            }
            let runtime = q1_edge::supermimic::runtime::Runtime::new(
                root,
                profile,
                &options.bridge_socket,
                q1_edge::supermimic::runtime::RuntimeOptions {
                    shadow_only: options.shadow_only,
                    ort_threads: options.ort_threads,
                },
            )?;
            return q1_edge::supermimic::web::run(
                runtime,
                &options.host,
                options.port,
                options.allow_host,
                options
                    .operator_key_file
                    .as_deref()
                    .ok_or("Missing --operator-key-file")?,
            )
            .await;
        }
        #[cfg(not(feature = "supermimic"))]
        return Err("Build with --features ros2,supermimic".into());
    }
    let config = config::load(options.config.as_deref())?;
    if options.diagnose {
        println!(
            "{}",
            json!({"backend":options.backend,"ros2_compiled":cfg!(feature="ros2"),"read_only":true,
            "sdk_commit":"7310314eee058323a2c15ab1f4f947df4112573e", "pico_available":false,
            "ros_distro":std::env::var("ROS_DISTRO").ok(),"rmw_implementation":std::env::var("RMW_IMPLEMENTATION").ok()})
        );
        return Ok(());
    }
    if options.replay {
        if options.backend != "mock" {
            return Err("Replay requires the mock backend".into());
        }
        return replay(config);
    }
    let backend: Box<dyn Backend> = if options.backend == "mock" {
        Box::new(MockBackend::new())
    } else {
        if !options.enable_hardware {
            return Err("ROS2 requires --enable-hardware".into());
        }
        #[cfg(feature = "ros2")]
        {
            Box::new(q1_edge::ros::RosBackend::new(&config)?)
        }
        #[cfg(not(feature = "ros2"))]
        {
            return Err("This binary was built without --features ros2".into());
        }
    };
    server::run(
        Controller::new(backend, config),
        &options.host,
        options.port,
        options.allow_host,
        options.trace,
    )
    .await
}
