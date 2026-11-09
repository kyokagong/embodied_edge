//! Private, bounded JSONL flight recording. Control threads never perform disk I/O.
use serde::Serialize;
use std::{
    fs::{self, File, OpenOptions},
    io::{BufWriter, Write},
    os::unix::fs::{DirBuilderExt, OpenOptionsExt},
    path::{Path, PathBuf},
    sync::{
        atomic::{AtomicBool, AtomicU64, Ordering},
        mpsc::{sync_channel, SyncSender, TrySendError},
        Arc,
    },
    thread,
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};
#[derive(Default)]
pub struct Stats {
    pub written: AtomicU64,
    pub dropped: AtomicU64,
    pub failed: AtomicBool,
}
pub struct Recorder<T> {
    sender: SyncSender<T>,
    pub stats: Arc<Stats>,
    pub directory: PathBuf,
}
impl<T> Clone for Recorder<T> {
    fn clone(&self) -> Self {
        Self {
            sender: self.sender.clone(),
            stats: self.stats.clone(),
            directory: self.directory.clone(),
        }
    }
}
pub fn wall_ns() -> u128 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos()
}
fn open_part(directory: &Path, part: usize) -> std::io::Result<BufWriter<File>> {
    let f = OpenOptions::new()
        .write(true)
        .create(true)
        .truncate(true)
        .mode(0o600)
        .open(directory.join(format!("part-{part:02}.jsonl")))?;
    Ok(BufWriter::with_capacity(256 * 1024, f))
}
impl<T: Serialize + Send + 'static> Recorder<T> {
    pub fn open(base: &Path, stream: &str, part_bytes: u64) -> std::io::Result<Self> {
        fs::create_dir_all(base)?;
        let directory = base.join(format!("{stream}-{}-{}", wall_ns(), std::process::id()));
        fs::DirBuilder::new().mode(0o700).create(&directory)?;
        let mut writer = open_part(&directory, 0)?;
        let (sender, receiver) = sync_channel::<T>(256);
        let stats = Arc::new(Stats::default());
        let worker_stats = stats.clone();
        let output = directory.clone();
        thread::Builder::new()
            .name(format!("q1-log-{stream}"))
            .spawn(move || {
                let result = (|| -> std::io::Result<()> {
                    let mut bytes = 0u64;
                    let mut generation = 0u64;
                    let mut sequence = 0u64;
                    let mut flush_at = Instant::now();
                    let mut buffer = Vec::with_capacity(64 * 1024);
                    loop {
                        match receiver.recv_timeout(Duration::from_millis(200)) {
                            Ok(record) => {
                                sequence += 1;
                                buffer.clear();
                                #[derive(Serialize)]
                                struct Envelope<'a, T> {
                                    schema: u32,
                                    log_seq: u64,
                                    wall_ns: u128,
                                    record: &'a T,
                                }
                                serde_json::to_writer(
                                    &mut buffer,
                                    &Envelope {
                                        schema: 1,
                                        log_seq: sequence,
                                        wall_ns: wall_ns(),
                                        record: &record,
                                    },
                                )?;
                                buffer.push(b'\n');
                                if bytes + buffer.len() as u64 > part_bytes {
                                    writer.flush()?;
                                    generation += 1;
                                    writer = open_part(&output, (generation % 8) as usize)?;
                                    bytes = 0;
                                }
                                writer.write_all(&buffer)?;
                                bytes += buffer.len() as u64;
                                worker_stats.written.fetch_add(1, Ordering::Relaxed);
                            }
                            Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {}
                            Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => break,
                        }
                        if flush_at.elapsed() >= Duration::from_millis(200) {
                            writer.flush()?;
                            flush_at = Instant::now();
                        }
                    }
                    writer.flush()
                })();
                if let Err(e) = result {
                    worker_stats.failed.store(true, Ordering::Release);
                    eprintln!("Flight recording failed: {e}");
                }
            })?;
        Ok(Self {
            sender,
            stats,
            directory,
        })
    }
    pub fn record(&self, value: T) {
        match self.sender.try_send(value) {
            Ok(()) => {}
            Err(TrySendError::Full(_)) => {
                self.stats.dropped.fetch_add(1, Ordering::Relaxed);
            }
            Err(TrySendError::Disconnected(_)) => {
                self.stats.failed.store(true, Ordering::Release);
            }
        }
    }
    pub fn manifest(&self, value: &impl Serialize) -> std::io::Result<()> {
        let mut file = OpenOptions::new()
            .write(true)
            .create_new(true)
            .mode(0o600)
            .open(self.directory.join("manifest.json"))?;
        serde_json::to_writer(&mut file, value)?;
        file.write_all(b"\n")
    }
}
