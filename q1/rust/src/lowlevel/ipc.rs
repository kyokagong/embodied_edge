//! Bounded SOCK_SEQPACKET IPC; no partial frames or unbounded request queues.
use serde::{de::DeserializeOwned, Serialize};
use std::{
    io,
    os::fd::{AsRawFd, FromRawFd, OwnedFd},
    path::Path,
};
const MAX: usize = 16384;
pub struct Socket(OwnedFd);
fn address(path: &Path) -> io::Result<libc::sockaddr_un> {
    use std::os::unix::ffi::OsStrExt;
    let bytes = path.as_os_str().as_bytes();
    let mut a: libc::sockaddr_un = unsafe { std::mem::zeroed() };
    if bytes.is_empty() || bytes.len() >= a.sun_path.len() {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "IPC path too long",
        ));
    }
    a.sun_family = libc::AF_UNIX as _;
    for (d, s) in a.sun_path.iter_mut().zip(bytes) {
        *d = *s as _;
    }
    Ok(a)
}
impl Socket {
    fn new() -> io::Result<Self> {
        let fd = unsafe {
            libc::socket(
                libc::AF_UNIX,
                libc::SOCK_SEQPACKET | libc::SOCK_CLOEXEC | libc::SOCK_NONBLOCK,
                0,
            )
        };
        if fd < 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(Self(unsafe { OwnedFd::from_raw_fd(fd) }))
    }
    pub fn listen(path: &Path) -> io::Result<Self> {
        let s = Self::new()?;
        let a = address(path)?;
        if unsafe {
            libc::bind(
                s.0.as_raw_fd(),
                (&a as *const libc::sockaddr_un).cast(),
                std::mem::size_of_val(&a) as _,
            )
        } < 0
            || unsafe { libc::listen(s.0.as_raw_fd(), 1) } < 0
        {
            return Err(io::Error::last_os_error());
        }
        Ok(s)
    }
    pub fn connect(path: &Path) -> io::Result<Self> {
        let s = Self::new()?;
        let a = address(path)?;
        if unsafe {
            libc::connect(
                s.0.as_raw_fd(),
                (&a as *const libc::sockaddr_un).cast(),
                std::mem::size_of_val(&a) as _,
            )
        } < 0
        {
            return Err(io::Error::last_os_error());
        }
        Ok(s)
    }
    pub fn accept(&self) -> io::Result<Self> {
        let fd = unsafe {
            libc::accept4(
                self.0.as_raw_fd(),
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                libc::SOCK_CLOEXEC | libc::SOCK_NONBLOCK,
            )
        };
        if fd < 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(Self(unsafe { OwnedFd::from_raw_fd(fd) }))
    }
    pub fn send<T: Serialize>(&self, value: &T) -> io::Result<()> {
        let b = bincode::serialize(value).map_err(io::Error::other)?;
        if b.len() > MAX {
            return Err(io::Error::other("IPC payload too large"));
        }
        let n = unsafe {
            libc::send(
                self.0.as_raw_fd(),
                b.as_ptr().cast(),
                b.len(),
                libc::MSG_NOSIGNAL,
            )
        };
        if n != b.len() as isize {
            return Err(io::Error::last_os_error());
        }
        Ok(())
    }
    pub fn receive<T: DeserializeOwned>(&self) -> io::Result<Option<T>> {
        let mut b = [0u8; MAX];
        let n = unsafe {
            libc::recv(
                self.0.as_raw_fd(),
                b.as_mut_ptr().cast(),
                b.len(),
                libc::MSG_TRUNC,
            )
        };
        if n < 0 {
            let e = io::Error::last_os_error();
            if e.kind() == io::ErrorKind::WouldBlock {
                return Ok(None);
            }
            return Err(e);
        }
        if n == 0 || n as usize > MAX {
            return Err(io::Error::other("IPC closed or oversized frame"));
        }
        use bincode::Options;
        bincode::DefaultOptions::new()
            .with_fixint_encoding()
            .with_limit(MAX as u64)
            .reject_trailing_bytes()
            .deserialize(&b[..n as usize])
            .map(Some)
            .map_err(io::Error::other)
    }
}
