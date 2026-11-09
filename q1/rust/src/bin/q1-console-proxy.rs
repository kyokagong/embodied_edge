//! Fixed-destination console relay. All model tasks and authentication stay on Q1.
use clap::Parser;
use std::{net::SocketAddr, sync::Arc, time::Duration};
use tokio::{
    io::copy_bidirectional,
    net::{TcpListener, TcpStream},
    sync::Semaphore,
};
#[derive(Parser)]
struct Options {
    #[arg(long, default_value = "0.0.0.0:8768")]
    bind: SocketAddr,
    #[arg(long)]
    upstream: SocketAddr,
}
#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let o = Options::parse();
    let listener = TcpListener::bind(o.bind).await?;
    let limit = Arc::new(Semaphore::new(64));
    let mut term = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())?;
    println!("Q1 console relay {} -> {}", o.bind, o.upstream);
    loop {
        tokio::select! {
            _ = term.recv() => break,
            _ = tokio::signal::ctrl_c() => break,
            incoming = listener.accept() => {
                let (mut downstream, _) = incoming?;
                let Ok(permit) = limit.clone().try_acquire_owned() else { continue; };
                let address = o.upstream;
                tokio::spawn(async move {
                    let _permit = permit;
                    if let Ok(Ok(mut upstream)) = tokio::time::timeout(Duration::from_secs(2), TcpStream::connect(address)).await {
                        let _ = downstream.set_nodelay(true);
                        let _ = upstream.set_nodelay(true);
                        let _ = copy_bidirectional(&mut downstream, &mut upstream).await;
                    }
                });
            }
        }
    }
    Ok(())
}
