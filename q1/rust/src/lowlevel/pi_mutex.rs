//! Linux priority inheritance for the short sensor/IPC state handoff.
use std::{
    cell::UnsafeCell,
    io,
    marker::PhantomData,
    ops::{Deref, DerefMut},
};

struct Inner<T> {
    mutex: UnsafeCell<libc::pthread_mutex_t>,
    value: UnsafeCell<T>,
}
pub struct PiMutex<T> {
    inner: Box<Inner<T>>,
}
// The pthread mutex serializes every access, and the heap address never moves.
unsafe impl<T: Send> Send for PiMutex<T> {}
unsafe impl<T: Send> Sync for PiMutex<T> {}
impl<T> PiMutex<T> {
    pub fn new(value: T) -> io::Result<Self> {
        let inner = Box::new(Inner {
            mutex: UnsafeCell::new(unsafe { std::mem::zeroed() }),
            value: UnsafeCell::new(value),
        });
        let mut attr = unsafe { std::mem::zeroed() };
        let rc = unsafe { libc::pthread_mutexattr_init(&mut attr) };
        if rc != 0 {
            return Err(io::Error::from_raw_os_error(rc));
        }
        let mut rc =
            unsafe { libc::pthread_mutexattr_setprotocol(&mut attr, libc::PTHREAD_PRIO_INHERIT) };
        if rc == 0 {
            rc = unsafe { libc::pthread_mutex_init(inner.mutex.get(), &attr) };
        }
        unsafe {
            libc::pthread_mutexattr_destroy(&mut attr);
        }
        if rc != 0 {
            return Err(io::Error::from_raw_os_error(rc));
        }
        Ok(Self { inner })
    }
    pub fn lock(&self) -> io::Result<Guard<'_, T>> {
        let rc = unsafe { libc::pthread_mutex_lock(self.inner.mutex.get()) };
        if rc != 0 {
            return Err(io::Error::from_raw_os_error(rc));
        }
        Ok(Guard {
            owner: self,
            not_send: PhantomData,
        })
    }
}
impl<T> Drop for PiMutex<T> {
    fn drop(&mut self) {
        unsafe {
            libc::pthread_mutex_destroy(self.inner.mutex.get());
        }
    }
}
pub struct Guard<'a, T> {
    owner: &'a PiMutex<T>,
    not_send: PhantomData<*mut ()>,
}
impl<T> Deref for Guard<'_, T> {
    type Target = T;
    fn deref(&self) -> &T {
        unsafe { &*self.owner.inner.value.get() }
    }
}
impl<T> DerefMut for Guard<'_, T> {
    fn deref_mut(&mut self) -> &mut T {
        unsafe { &mut *self.owner.inner.value.get() }
    }
}
impl<T> Drop for Guard<'_, T> {
    fn drop(&mut self) {
        unsafe {
            libc::pthread_mutex_unlock(self.owner.inner.mutex.get());
        }
    }
}
