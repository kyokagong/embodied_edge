//! Authenticated, challenge-bounded network adapter for the local guard IPC.
//! Neither peer compares CLOCK_MONOTONIC values from different computers.
use super::{
    ipc::Socket,
    types::{clock, BridgeStatus, Request},
};
use bincode::Options;
use hmac::{Hmac, Mac};
use serde::{Deserialize, Serialize};
use sha2::Sha256;
use std::{
    collections::VecDeque,
    io,
    net::{SocketAddr, UdpSocket},
    path::Path,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    thread,
    time::{Duration, Instant},
};
const VERSION: u32 = 6;
const LIMIT: usize = 4096;
const FRESH: f64 = 0.06;
#[derive(Serialize, Deserialize)]
enum Payload {
    Request(Box<Request>),
    Status(Box<BridgeStatus>),
}
#[derive(Serialize, Deserialize)]
struct Frame {
    version: u32,
    id: u64,
    echo: u64,
    echo_received: f64,
    profile: [u8; 32],
    sent: f64,
    payload: Payload,
}
fn codec() -> impl Options {
    bincode::DefaultOptions::new()
        .with_fixint_encoding()
        .with_limit(LIMIT as u64)
        .reject_trailing_bytes()
}
fn seal(f: &Frame, key: &[u8]) -> io::Result<Vec<u8>> {
    let mut b = codec().serialize(f).map_err(io::Error::other)?;
    if b.len() + 32 > LIMIT {
        return Err(io::Error::other("Guard frame too large"));
    }
    let mut mac = Hmac::<Sha256>::new_from_slice(key).map_err(io::Error::other)?;
    mac.update(&b);
    b.extend_from_slice(&mac.finalize().into_bytes());
    Ok(b)
}
fn open(b: &[u8], key: &[u8]) -> io::Result<Frame> {
    if b.len() < 32 || b.len() > LIMIT {
        return Err(io::Error::other("Invalid guard frame size"));
    }
    let (data, tag) = b.split_at(b.len() - 32);
    let mut mac = Hmac::<Sha256>::new_from_slice(key).map_err(io::Error::other)?;
    mac.update(data);
    mac.verify_slice(tag).map_err(io::Error::other)?;
    let f: Frame = codec().deserialize(data).map_err(io::Error::other)?;
    if f.version != VERSION || !f.sent.is_finite() || !f.echo_received.is_finite() {
        return Err(io::Error::other("Guard version/time invalid"));
    }
    Ok(f)
}
fn challenge_age(issued: &VecDeque<(u64, f64)>, id: u64, now: f64) -> Option<f64> {
    issued
        .iter()
        .find(|x| x.0 == id)
        .map(|x| now - x.1)
        .filter(|age| (0. ..=FRESH).contains(age))
}
fn remember(issued: &mut VecDeque<(u64, f64)>, id: u64, now: f64) {
    issued.push_back((id, now));
    while issued.front().is_some_and(|x| now - x.1 > FRESH) || issued.len() > 64 {
        issued.pop_front();
    }
}
#[derive(Default)]
struct ClockBounds {
    samples: VecDeque<(f64, f64, f64)>,
    poisoned: bool,
}
impl ClockBounds {
    fn observe(
        &mut self,
        local_send: f64,
        remote_receive: f64,
        remote_send: f64,
        local_receive: f64,
    ) -> bool {
        if self.poisoned
            || ![local_send, remote_receive, remote_send, local_receive]
                .iter()
                .all(|v| v.is_finite())
            || remote_receive > remote_send
            || local_send > local_receive
        {
            return false;
        }
        let lower = local_send - remote_receive;
        let upper = local_receive - remote_send;
        if lower > upper {
            return false;
        }
        self.samples.push_back((local_receive, lower, upper));
        while self
            .samples
            .front()
            .is_some_and(|v| local_receive - v.0 > 0.25)
            || self.samples.len() > 64
        {
            self.samples.pop_front();
        }
        if self.interval(local_receive).is_none() {
            self.poisoned = true;
            return false;
        }
        true
    }
    fn interval(&self, now: f64) -> Option<(f64, f64)> {
        if self.poisoned || self.samples.back().is_none_or(|v| now - v.0 > FRESH) {
            return None;
        }
        let mut lo = f64::NEG_INFINITY;
        let mut hi = f64::INFINITY;
        for &(at, l, u) in &self.samples {
            // 0.5%/s is a conservative drift allowance for local monotonic clocks.
            let drift = 0.005 * (now - at).max(0.);
            lo = lo.max(l - drift);
            hi = hi.min(u + drift);
        }
        if lo <= hi {
            Some((lo, hi))
        } else {
            None
        }
    }
    fn translate(&self, remote_time: f64, now: f64) -> Option<f64> {
        let (lower, _) = self.interval(now)?;
        let value = remote_time + lower;
        value.is_finite().then_some(value)
    }
}
fn send(
    udp: &UdpSocket,
    peer: SocketAddr,
    profile: [u8; 32],
    echo: (u64, f64),
    payload: Payload,
    key: &[u8],
    issued: &mut VecDeque<(u64, f64)>,
) -> io::Result<()> {
    let now = clock();
    let id = rand::random();
    let b = seal(
        &Frame {
            version: VERSION,
            id,
            echo: echo.0,
            echo_received: echo.1,
            profile,
            sent: now,
            payload,
        },
        key,
    )?;
    udp.send_to(&b, peer)?;
    remember(issued, id, now);
    Ok(())
}
pub fn profile_digest(p: &Path) -> io::Result<[u8; 32]> {
    use sha2::Digest;
    Ok(Sha256::digest(std::fs::read(p)?).into())
}

/// Mini PC endpoint: 500 Hz authenticated output, Unix IPC toward q1-edge.
pub fn client(
    bind: SocketAddr,
    peer: SocketAddr,
    path: &Path,
    key: &[u8],
    profile: [u8; 32],
    quit: Arc<AtomicBool>,
) -> io::Result<()> {
    use std::os::unix::{fs::PermissionsExt, io::AsRawFd};
    std::fs::create_dir_all(
        path.parent()
            .ok_or_else(|| io::Error::other("No IPC parent"))?,
    )?;
    let lock = std::fs::OpenOptions::new()
        .write(true)
        .create(true)
        .truncate(false)
        .open(path.with_extension("lock"))?;
    if unsafe { libc::flock(lock.as_raw_fd(), libc::LOCK_EX | libc::LOCK_NB) } != 0 {
        return Err(io::Error::other("Guard proxy already running"));
    }
    if path.exists() {
        std::fs::remove_file(path)?;
    }
    let listener = Socket::listen(path)?;
    std::fs::set_permissions(path, std::fs::Permissions::from_mode(0o600))?;
    let udp = UdpSocket::bind(bind)?;
    udp.set_nonblocking(true)?;
    let mut client: Option<Socket> = None;
    let mut latest = Request::Observe;
    let mut status = BridgeStatus::default();
    let mut received = 0.;
    let mut echo = (0, 0.);
    let mut clocks = ClockBounds::default();
    let mut issued = VecDeque::new();
    let mut deadline = Instant::now();
    let mut last_status = 0.;
    let mut buffer = [0; LIMIT + 1];
    while !quit.load(Ordering::Acquire) {
        if client.is_none() {
            if let Ok(c) = listener.accept() {
                client = Some(c);
                latest = Request::Observe;
            }
        }
        let mut disconnected = false;
        if let Some(c) = &client {
            for _ in 0..8 {
                match c.receive::<Request>() {
                    Ok(Some(r)) => latest = r,
                    Ok(None) => break,
                    Err(_) => {
                        disconnected = true;
                        break;
                    }
                }
            }
        }
        if disconnected {
            client = None;
            latest = Request::Observe;
        }
        for _ in 0..32 {
            match udp.recv_from(&mut buffer) {
                Ok((n, source)) => {
                    if source != peer {
                        continue;
                    }
                    let Ok(f) = open(&buffer[..n], key) else {
                        continue;
                    };
                    if f.profile != profile {
                        continue;
                    }
                    let now = clock();
                    let Some(bound) = challenge_age(&issued, f.echo, now) else {
                        continue;
                    };
                    if !clocks.observe(now - bound, f.echo_received, f.sent, now) {
                        continue;
                    }
                    if let Payload::Status(mut s) = f.payload {
                        let Some(joints_at) = clocks.translate(s.sensors.joints_at, now) else {
                            continue;
                        };
                        let Some(imu_at) = clocks.translate(s.sensors.imu_at, now) else {
                            continue;
                        };
                        s.sensors.joints_at = joints_at;
                        s.sensors.imu_at = imu_at;
                        if s.sensors.root_velocity.is_some() {
                            s.sensors.root_velocity_at = clocks
                                .translate(s.sensors.root_velocity_at, now)
                                .unwrap_or(0.);
                        }
                        status = *s;
                        received = now;
                        echo = (f.id, now);
                    }
                }
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => break,
                Err(e) => return Err(e),
            }
        }
        let now = clock();
        if received > 0. && now - received > FRESH {
            status.fault = Some("本体保护回包超时；运动会话不可自动恢复".into());
            status.sensors.complete = false;
            latest = Request::Observe;
        }
        let _ = send(
            &udp,
            peer,
            profile,
            echo,
            Payload::Request(Box::new(latest.clone())),
            key,
            &mut issued,
        );
        if now - last_status >= 0.002 {
            if let Some(c) = &client {
                if let Err(e) = c.send(&status) {
                    if e.kind() != io::ErrorKind::WouldBlock {
                        client = None;
                        latest = Request::Observe;
                    }
                }
            }
            last_status = now;
        }
        deadline += Duration::from_millis(2);
        let t = Instant::now();
        if t > deadline + Duration::from_millis(5) {
            deadline = t;
        }
        thread::sleep(deadline.saturating_duration_since(t));
    }
    let _ = std::fs::remove_file(path);
    Ok(())
}

/// Q1 endpoint: forwards only fresh, authenticated, non-replayed requests to q1-rt.
/// q1-rt independently enforces expiry even if this adapter or Wi-Fi disappears.
pub fn server(
    bind: SocketAddr,
    allowed_ip: std::net::IpAddr,
    path: &Path,
    key: &[u8],
    profile: [u8; 32],
    quit: Arc<AtomicBool>,
) -> io::Result<()> {
    let udp = UdpSocket::bind(bind)?;
    udp.set_nonblocking(true)?;
    let mut local: Option<Socket> = None;
    let mut peer = None;
    let mut status = BridgeStatus::default();
    let mut issued = VecDeque::new();
    let mut echo = (0, 0.);
    let mut clocks = ClockBounds::default();
    let mut last_status = 0.;
    let mut next_connect = 0.;
    let mut last_target: Option<(u64, u64)> = None;
    let mut buffer = [0; LIMIT + 1];
    while !quit.load(Ordering::Acquire) {
        let now = clock();
        if local.is_none() && now >= next_connect {
            next_connect = now + 0.1;
            if let Ok(s) = Socket::connect(path) {
                local = Some(s);
            }
        }
        if let Some(s) = &local {
            for _ in 0..8 {
                match s.receive::<BridgeStatus>() {
                    Ok(Some(v)) => status = v,
                    Ok(None) => break,
                    Err(_) => {
                        local = None;
                        break;
                    }
                }
            }
        }
        for _ in 0..32 {
            match udp.recv_from(&mut buffer) {
                Ok((n, source)) => {
                    if source.ip() != allowed_ip {
                        continue;
                    }
                    let Ok(f) = open(&buffer[..n], key) else {
                        continue;
                    };
                    if f.profile != profile {
                        continue;
                    }
                    let Payload::Request(mut req) = f.payload else {
                        continue;
                    };
                    // Observe bootstraps a challenge but grants no actuator permission.
                    let now = clock();
                    peer = Some(source);
                    echo = (f.id, now);
                    if matches!(*req, Request::Observe) {
                        continue;
                    }
                    let now = clock();
                    let Some(bound) = challenge_age(&issued, f.echo, now) else {
                        continue;
                    };
                    if !clocks.observe(now - bound, f.echo_received, f.sent, now) {
                        continue;
                    }
                    if let Request::Target(t) = req.as_mut() {
                        let identity = (t.session, t.sequence);
                        if last_target.is_some_and(|old| old.0 == identity.0 && old.1 >= identity.1)
                        {
                            continue;
                        }
                        let age = f.sent - t.generated;
                        let obs_age = f.sent - t.observation;
                        let ttl = t.expires - t.generated;
                        let operator_remaining = t.operator_expires - f.sent;
                        if ![age, obs_age, ttl, operator_remaining]
                            .iter()
                            .all(|x| x.is_finite())
                            || age < 0.
                            || obs_age < 0.
                            || ttl <= 0.
                            || ttl > FRESH + 1e-6
                            || age >= ttl
                        {
                            continue;
                        }
                        let Some(generated) = clocks.translate(t.generated, now) else {
                            continue;
                        };
                        let Some(observation) = clocks.translate(t.observation, now) else {
                            continue;
                        };
                        let Some(operator_expires) = clocks.translate(t.operator_expires, now)
                        else {
                            continue;
                        };
                        t.generated = generated;
                        t.observation = observation;
                        t.expires = t.generated + ttl;
                        t.operator_expires = operator_expires;
                        if now >= t.expires {
                            continue;
                        }
                        if let Some(s) = &local {
                            if s.send(&*req).is_ok() {
                                last_target = Some(identity);
                            }
                        }
                    } else if let Some(s) = &local {
                        let _ = s.send(&*req);
                    }
                }
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => break,
                Err(e) => return Err(e),
            }
        }
        if now - last_status >= 0.002 {
            if let Some(p) = peer {
                let _ = send(
                    &udp,
                    p,
                    profile,
                    echo,
                    Payload::Status(Box::new(status.clone())),
                    key,
                    &mut issued,
                );
            }
            last_status = now;
        }
        thread::sleep(Duration::from_millis(1));
    }
    Ok(())
}

#[cfg(test)]
mod protocol_checks {
    use super::*;
    #[test]
    fn asymmetric_delay_keeps_translated_observation_older_than_reality() {
        let mut clocks = ClockBounds::default();
        assert!(clocks.observe(1000., 100.005, 100.010, 1000.030));
        assert!(clocks.observe(1000.040, 100.041, 100.042, 1000.044));
        let (lo, hi) = clocks.interval(1000.045).unwrap();
        assert!(lo <= 900. && hi >= 900.);
        assert!(clocks.translate(100.043, 1000.045).unwrap() <= 1000.043);
        assert!(clocks.translate(100.043, 1000.2).is_none());
    }
    #[test]
    fn inconsistent_clock_mapping_requires_restart() {
        let mut clocks = ClockBounds::default();
        assert!(clocks.observe(1000., 100.001, 100.002, 1000.003));
        assert!(!clocks.observe(1000.004, 99.005, 99.006, 1000.007));
        assert!(clocks.translate(100., 1000.008).is_none());
    }
    #[test]
    fn authenticated_frame_rejects_tampering_wrong_key_and_truncation() {
        let key = [7u8; 32];
        let f = Frame {
            version: VERSION,
            id: 23,
            echo: 41,
            echo_received: 122.,
            profile: [9; 32],
            sent: 123.,
            payload: Payload::Request(Box::new(Request::Observe)),
        };
        let original = seal(&f, &key).unwrap();
        assert_eq!(open(&original, &key).unwrap().id, 23);
        let mut changed = original.clone();
        changed[12] ^= 1;
        assert!(open(&changed, &key).is_err());
        assert!(open(&original, &[8; 32]).is_err());
        assert!(open(&original[..original.len() - 1], &key).is_err());
        assert!(open(&[0; LIMIT + 1], &key).is_err());
    }
    #[test]
    fn unknown_expired_or_future_challenge_never_renews_permission() {
        let mut issued = VecDeque::new();
        remember(&mut issued, 42, 10.);
        assert!(challenge_age(&issued, 42, 10.02).is_some());
        assert!(challenge_age(&issued, 42, 10.1).is_none());
        assert!(challenge_age(&issued, 42, 9.99).is_none());
        assert!(challenge_age(&issued, 43, 10.02).is_none());
        remember(&mut issued, 44, 10.2);
        assert!(challenge_age(&issued, 42, 10.21).is_none());
    }
}
