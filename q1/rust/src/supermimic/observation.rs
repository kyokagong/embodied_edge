use super::contract::{Contract, Hardware};
use crate::lowlevel::types::Sensors;
use std::collections::VecDeque;
pub type Rotation = [[f32; 3]; 3];
pub fn transpose(r: Rotation) -> Rotation {
    std::array::from_fn(|i| std::array::from_fn(|j| r[j][i]))
}
pub fn multiply(a: Rotation, b: Rotation) -> Rotation {
    std::array::from_fn(|i| std::array::from_fn(|j| (0..3).map(|k| a[i][k] * b[k][j]).sum()))
}
pub fn rotation(q: [f32; 4]) -> Result<Rotation, String> {
    let n = q.iter().map(|x| x * x).sum::<f32>();
    if !n.is_finite() || (n - 1.).abs() > 0.01 {
        return Err("Invalid IMU quaternion norm".into());
    }
    let [x, y, z, w] = q.map(|v| v / n.sqrt());
    Ok([
        [
            1. - 2. * (y * y + z * z),
            2. * (x * y - z * w),
            2. * (x * z + y * w),
        ],
        [
            2. * (x * y + z * w),
            1. - 2. * (x * x + z * z),
            2. * (y * z - x * w),
        ],
        [
            2. * (x * z - y * w),
            2. * (y * z + x * w),
            1. - 2. * (x * x + y * y),
        ],
    ])
}
pub fn yaw(r: Rotation) -> Rotation {
    let (x, y) = if r[0][0] * r[0][0] + r[1][0] * r[1][0] > 1e-8 {
        (r[0][0], r[1][0])
    } else {
        (r[1][1], -r[0][1])
    };
    let n = (x * x + y * y).sqrt().max(1e-8);
    [[x / n, -y / n, 0.], [y / n, x / n, 0.], [0., 0., 1.]]
}
pub fn pelvis(s: &Sensors, h: &Hardware) -> Result<Rotation, String> {
    Ok(multiply(
        rotation(s.quaternion_xyzw)?,
        transpose(h.imu_to_pelvis),
    ))
}
pub struct Condition<'a> {
    pub command: [f32; 3],
    pub step: u64,
    pub reference: Option<(Rotation, &'a [f32])>,
}
#[derive(Default)]
pub struct Observation {
    history: VecDeque<[f32; 78]>,
}
impl Observation {
    pub fn reset(&mut self) {
        self.history.clear();
    }
    pub fn build(
        &mut self,
        s: &Sensors,
        h: &Hardware,
        c: &Contract,
        previous: [f32; 22],
        condition: Condition<'_>,
    ) -> Result<Vec<f32>, String> {
        let Condition {
            command,
            step,
            reference,
        } = condition;
        let r = pelvis(s, h)?;
        let mut frame = [0.; 78];
        for i in 0..22 {
            frame[i] = h.signs[i] * (s.q[i] - h.zeros[i]) - c.action.home[i] + h.encoder_bias[i];
            frame[22 + i] = h.signs[i] * s.qd[i];
            frame[50 + i] = previous[i];
        }
        // sensor_msgs/Imu angular velocity is in its sensor frame.
        for i in 0..3 {
            frame[44 + i] = (0..3).map(|k| h.imu_to_pelvis[i][k] * s.omega[k]).sum();
            frame[47 + i] = -r[2][i];
        }
        let rel = multiply(transpose(r), reference.map_or_else(|| yaw(r), |x| x.0));
        for i in 0..3 {
            for j in 0..2 {
                frame[72 + 2 * i + j] = rel[i][j];
            }
        }
        if !frame.iter().all(|v| v.is_finite()) {
            return Err("Nonfinite observation".into());
        }
        if self.history.is_empty() {
            self.history.extend(std::iter::repeat_n(frame, 10));
        } else {
            self.history.pop_front();
            self.history.push_back(frame);
        }
        let mut x = Vec::with_capacity(1376);
        for frame in &self.history {
            x.extend_from_slice(frame);
        }
        x.push(if reference.is_some() { 1. } else { 0. });
        x.extend_from_slice(&command);
        let phase = step as f32 * (2. * std::f32::consts::PI * c.gait_frequency_hz * 0.02);
        x.extend_from_slice(
            &if reference.is_none() && command.iter().any(|v| v.abs() > 1e-6) {
                [phase.sin(), phase.cos()]
            } else {
                [0., 0.]
            },
        );
        if let Some((_, goal)) = reference {
            if goal.len() != 590 {
                return Err("Invalid goal shape".into());
            }
            x.extend_from_slice(goal);
        } else {
            x.resize(1376, 0.);
        }
        if x.len() != 1376 || !x.iter().all(|v| v.is_finite()) {
            return Err("Invalid actor input".into());
        }
        Ok(x)
    }
}
