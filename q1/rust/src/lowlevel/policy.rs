//! Policy protocol math, independent of ROS and ONNX.
pub const PROTOCOL: u32 = 2;
pub type Rotation = [[f32; 3]; 3];
pub fn tilt(actual: Rotation, reference: Rotation) -> f32 {
    let a = actual[2];
    let b = reference[2];
    let cross = [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ];
    let sine = cross.iter().map(|v| v * v).sum::<f32>().sqrt();
    let cosine = (0..3).map(|i| a[i] * b[i]).sum::<f32>();
    sine.atan2(cosine)
}
pub fn valid_rotation(r: Rotation) -> bool {
    if !r.iter().flatten().all(|v| v.is_finite()) {
        return false;
    }
    let orthogonal = (0..3).all(|i| {
        (0..3).all(|j| {
            ((0..3).map(|k| r[i][k] * r[j][k]).sum::<f32>() - if i == j { 1. } else { 0. }).abs()
                < 0.01
        })
    });
    let det = r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
        - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
        + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]);
    orthogonal && (det - 1.).abs() < 0.01
}
pub fn publication(
    previous: Option<[f32; 22]>,
    candidate: [f32; 22],
    success: bool,
    active: bool,
) -> Option<[f32; 22]> {
    if success && active {
        Some(candidate)
    } else {
        previous
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    const I: Rotation = [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]];
    #[test]
    fn tilted_reference_is_not_upright_error() {
        let prone = [[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]];
        assert!(tilt(prone, prone).abs() < 1e-6);
        assert!((tilt(prone, I) - std::f32::consts::FRAC_PI_2).abs() < 1e-6);
    }
    #[test]
    fn rotation_rejects_finite_non_rotation() {
        assert!(valid_rotation(I));
        assert!(!valid_rotation([[0.; 3]; 3]));
        assert!(!valid_rotation([[-1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]));
    }
    #[test]
    fn only_successful_active_publication_updates_history() {
        let old = Some([2.; 22]);
        let new = [4.; 22];
        assert_eq!(publication(old, new, false, true), old);
        assert_eq!(publication(old, new, true, false), old);
        assert_eq!(publication(old, new, true, true), Some(new));
    }
}
