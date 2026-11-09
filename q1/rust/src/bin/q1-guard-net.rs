use clap::{Parser, ValueEnum};
use std::{
    net::{IpAddr, SocketAddr},
    path::PathBuf,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
};
#[derive(Clone, ValueEnum)]
enum Mode {
    Client,
    Server,
}
#[derive(Parser)]
struct Options {
    #[arg(long, value_enum)]
    mode: Mode,
    #[arg(long)]
    bind: SocketAddr,
    #[arg(long)]
    peer: Option<SocketAddr>,
    #[arg(long)]
    allowed_ip: Option<IpAddr>,
    #[arg(long)]
    socket: PathBuf,
    #[arg(long)]
    key_file: PathBuf,
    #[arg(long)]
    hardware_profile: PathBuf,
}
#[tokio::main]
async fn main() {
    if let Err(e) = run(Options::parse()).await {
        eprintln!("q1-guard-net: {e}");
        std::process::exit(2);
    }
}
async fn run(o: Options) -> Result<(), String> {
    let key = std::fs::read(&o.key_file).map_err(|e| e.to_string())?;
    if key.len() < 32 {
        return Err("Guard key must be at least32 bytes".into());
    }
    let profile = q1_edge::lowlevel::network::profile_digest(&o.hardware_profile)
        .map_err(|e| e.to_string())?;
    let quit = Arc::new(AtomicBool::new(false));
    let stop = quit.clone();
    tokio::spawn(async move {
        let mut term =
            tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate()).unwrap();
        tokio::select! {_=tokio::signal::ctrl_c()=>{},_=term.recv()=>{}}
        stop.store(true, Ordering::Release);
    });
    match o.mode {
        Mode::Client => q1_edge::lowlevel::network::client(
            o.bind,
            o.peer.ok_or("Missing --peer")?,
            &o.socket,
            &key,
            profile,
            quit,
        ),
        Mode::Server => q1_edge::lowlevel::network::server(
            o.bind,
            o.allowed_ip.ok_or("Missing --allowed-ip")?,
            &o.socket,
            &key,
            profile,
            quit,
        ),
    }
    .map_err(|e| e.to_string())
}
