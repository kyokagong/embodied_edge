use serde::{Deserialize, Serialize};

pub fn clock() -> f64 {
    let mut t = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // Linux monotonic clock is shared by both local processes.
    unsafe {
        libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut t);
    }
    t.tv_sec as f64 + t.tv_nsec as f64 * 1e-9
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Sensors {
    pub q: [f32; 22],
    pub qd: [f32; 22],
    pub effort: [f32; 22],
    pub quaternion_xyzw: [f32; 4],
    pub omega: [f32; 3],
    pub temperature: [f32; 22],
    pub joints_at: f64,
    pub imu_at: f64,
    pub joint_stamp: f64,
    pub imu_stamp: f64,
    pub complete: bool,
    pub imu_valid: bool,
    pub joint_frames: u64,
    pub imu_frames: u64,
    pub imu_frame: String,
    pub root_velocity: Option<[f32; 3]>,
    pub root_velocity_at: f64,
    pub root_velocity_stamp: f64,
}
impl Default for Sensors {
    fn default() -> Self {
        Self {
            q: [0.; 22],
            qd: [0.; 22],
            effort: [0.; 22],
            quaternion_xyzw: [0.; 4],
            omega: [0.; 3],
            temperature: [0.; 22],
            joints_at: 0.,
            imu_at: 0.,
            joint_stamp: 0.,
            imu_stamp: 0.,
            complete: false,
            imu_valid: false,
            joint_frames: 0,
            imu_frames: 0,
            imu_frame: String::new(),
            root_velocity: None,
            root_velocity_at: 0.,
            root_velocity_stamp: 0.,
        }
    }
}
impl Sensors {
    pub fn validate(&self, now: f64, timeout: f64, skew: f64) -> Result<(), String> {
        if !self.complete || !self.imu_valid {
            return Err("完整22关节或有效IMU尚未就绪".into());
        }
        if ![self.joints_at, self.imu_at]
            .iter()
            .all(|t| (0. ..=timeout).contains(&(now - t)))
        {
            return Err(format!(
                "关节或IMU反馈过期：joint_age={:.2}ms, imu_age={:.2}ms, limit={:.0}ms",
                (now - self.joints_at) * 1000.,
                (now - self.imu_at) * 1000.,
                timeout * 1000.
            ));
        }
        if (self.joint_stamp - self.imu_stamp).abs() > skew {
            return Err(format!(
                "关节/IMU采样时间差过大：skew={:.2}ms, limit={:.2}ms",
                (self.joint_stamp - self.imu_stamp).abs() * 1000.,
                skew * 1000.
            ));
        }
        let wall = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_err(|e| e.to_string())?
            .as_secs_f64();
        if ![self.joint_stamp, self.imu_stamp]
            .iter()
            .all(|t| (wall - t).abs() < 0.15)
        {
            return Err(format!(
                "源采样时间陈旧或本体时钟异常：joint_stamp_age={:.2}ms, imu_stamp_age={:.2}ms",
                (wall - self.joint_stamp) * 1000.,
                (wall - self.imu_stamp) * 1000.
            ));
        }
        if !self
            .q
            .iter()
            .chain(self.qd.iter())
            .chain(self.omega.iter())
            .all(|v| v.is_finite())
        {
            return Err("反馈存在非有限值".into());
        }
        Ok(())
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Target {
    pub session: u64,
    pub epoch: u64,
    pub sequence: u64,
    pub generated: f64,
    pub observation: f64,
    pub expires: f64,
    pub operator_expires: f64,
    pub moving: bool,
    pub q: [f32; 22],
    pub raw: [f32; 22],
    pub gain_scale: f32,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum Request {
    Observe,
    PolicyTarget {
        version: u32,
        target: Box<Target>,
        reference_rotation: [[f32; 3]; 3],
    },
    Arm {
        session: u64,
    },
    Target(Box<Target>),
    Disarm {
        session: u64,
    },
    Reset {
        session: u64,
    },
    Release {
        session: Option<u64>,
    },
}
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct BridgeStatus {
    pub policy_protocol: u32,
    pub published_policy_raw: Option<[f32; 22]>,
    pub published_policy_sequence: Option<u64>,
    pub reference_rotation: Option<[[f32; 3]; 3]>,
    pub publish_hz: u32,
    pub flight_directory: String,
    pub log_written: u64,
    pub log_dropped: u64,
    pub log_failed: bool,
    pub released: bool,
    pub saturation: [u8; 22],
    pub saturated_frames: u64,
    pub tilt_scale: f32,
    pub applied_raw: [f32; 22],
    pub commanded_q: [f32; 22],
    pub estimated_effort: [f32; 22],
    pub commanded_kp: [f32; 22],
    pub commanded_kd: [f32; 22],
    pub hal_sequence: u64,
    pub applied_gain_scale: f32,
    pub controlled_joints: usize,
    pub intervals_over_5ms: u64,
    pub sensors: Sensors,
    pub session: Option<u64>,
    pub accepted: Option<u64>,
    pub tx_count: u64,
    pub tx_hz: f64,
    pub tx_max_gap_ms: f64,
    pub scheduler_hz: f64,
    pub scheduler_max_gap_ms: f64,
    pub fault: Option<String>,
    pub blockers: Vec<String>,
    pub mode: String,
}
