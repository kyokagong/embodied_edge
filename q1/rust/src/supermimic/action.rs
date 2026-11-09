//! One action mapping for every condition of the unified actor.
use super::contract::{Contract, Hardware, N};
use serde::Serialize;
#[derive(Clone, Serialize)]
pub struct NativeAction {
    pub policy_raw: [f32; N],
    pub model_q: [f32; N],
    pub motor_safe_q: [f32; N],
    pub raw_limited: [bool; N],
    pub mechanical_limited: [bool; N],
}
pub fn map(raw: [f32; N], c: &Contract, h: &Hardware) -> NativeAction {
    let clipped = raw.map(|v| v.clamp(-c.action.clip, c.action.clip));
    let model_q = std::array::from_fn(|i| c.action.home[i] + c.action.scale * clipped[i]);
    let q = std::array::from_fn(|i| model_q[i].clamp(h.lower[i], h.upper[i]));
    NativeAction {
        // Training observes action_manager.action, before all actuator safety transforms.
        policy_raw: clipped,
        model_q,
        motor_safe_q: q,
        raw_limited: std::array::from_fn(|i| raw[i] != clipped[i]),
        mechanical_limited: std::array::from_fn(|i| (model_q[i] - q[i]).abs() > 1e-6),
    }
}
