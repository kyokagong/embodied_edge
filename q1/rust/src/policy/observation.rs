use super::package::Package;
pub use crate::supermimic::observation::{multiply, pelvis, rotation, transpose, yaw, Rotation};
use crate::{lowlevel::types::Sensors, supermimic::contract::Hardware};
use std::collections::VecDeque;

#[derive(Default)]
pub struct Observation {
    history: VecDeque<[f32; 78]>,
}
impl Observation {
    pub fn reset(&mut self) {
        self.history.clear();
    }
    /// The caller validates freshness against its policy clock before advancing history.
    pub fn build(
        &mut self,
        s: &Sensors,
        h: &Hardware,
        p: &Package,
        previous: [f32; 22],
        reference: Option<Rotation>,
    ) -> Result<[f32; 780], String> {
        if !s.complete || !s.imu_valid || s.imu_frame != h.imu_frame {
            return Err("Incomplete sensors or mismatched IMU frame".into());
        }
        let r = pelvis(s, h)?;
        let mut frame = [0.; 78];
        for i in 0..22 {
            frame[i] = h.signs[i] * (s.q[i] - h.zeros[i]) - p.home[i] - p.offset[i];
            frame[22 + i] = h.signs[i] * s.qd[i];
            frame[50 + i] = previous[i];
        }
        for i in 0..3 {
            // sensor_msgs/Imu reports angular velocity in the IMU sensor frame.
            frame[44 + i] = (0..3).map(|j| h.imu_to_pelvis[i][j] * s.omega[j]).sum();
            frame[47 + i] = -r[2][i];
        }
        let rel = multiply(transpose(r), reference.unwrap_or_else(|| yaw(r)));
        for i in 0..3 {
            for j in 0..2 {
                frame[72 + 2 * i + j] = rel[i][j];
            }
        }
        if !frame.iter().all(|v| v.is_finite()) || previous.iter().any(|v| v.abs() > 20.) {
            return Err("Nonfinite observation or invalid published clipped raw".into());
        }
        if self.history.is_empty() {
            self.history.extend(std::iter::repeat_n(frame, 10));
        } else {
            self.history.pop_front();
            self.history.push_back(frame);
        }
        let mut out = [0.; 780];
        for (i, f) in self.history.iter().enumerate() {
            out[i * 78..(i + 1) * 78].copy_from_slice(f);
        }
        Ok(out)
    }
}
pub fn command_goal(home: &[f32; 22], command: [f32; 3]) -> Result<[f32; 594], String> {
    if !home.iter().chain(command.iter()).all(|v| v.is_finite())
        || command
            .iter()
            .zip([0.6, 0.35, 1.])
            .any(|(v, b)| v.abs() > b)
    {
        return Err("Nonfinite home or command exceeds recorded training bounds".into());
    }
    let mut g = [0.; 594];
    for k in 0..10 {
        g[k * 59 + 3] = 1.;
        g[k * 59 + 6] = 1.;
        g[k * 59 + 15..k * 59 + 37].copy_from_slice(home);
    }
    g[590] = 1.;
    g[591..].copy_from_slice(&command);
    Ok(g)
}
