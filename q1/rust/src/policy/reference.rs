//! Preloaded native 50 Hz references. Goals are rebuilt at the active phase boundary.
use super::{
    observation::{multiply, Rotation},
    package::{read_json, Package},
};
use crate::supermimic::contract::NAMES;
use ndarray::{Array2, Array3};
use ndarray_npy::NpzReader;
use serde_json::Value;
use std::collections::BTreeMap;

#[derive(Clone)]
pub struct Clip {
    pub metadata: Value,
    pub rotations: Array3<f32>,
    pub q: Array2<f32>,
    pub qd: Array2<f32>,
    pub root_p: Array2<f32>,
    pub root_v_local: Array2<f32>,
    pub root_w_local: Array2<f32>,
}
impl Clip {
    pub fn len(&self) -> usize {
        self.q.nrows()
    }
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
    pub fn rotation(&self, frame: usize) -> Rotation {
        let f = frame.min(self.len() - 1);
        std::array::from_fn(|i| std::array::from_fn(|j| self.rotations[[f, i, j]]))
    }
    pub fn goal(&self, frame: usize, end_inclusive: usize) -> Result<[f32; 594], String> {
        if self.is_empty() || frame > end_inclusive || end_inclusive >= self.len() {
            return Err("Frame/end is outside the active reference".into());
        }
        let r = self.rotation(frame);
        // Exact tools.inputs.motion_goal heading, including its vertical-heading convention.
        let norm = (r[0][0] * r[0][0] + r[1][0] * r[1][0] + 1e-9f32).sqrt();
        let (c, s) = (r[0][0] / norm, r[1][0] / norm);
        let ht = [[c, s, 0.], [-s, c, 0.], [0., 0., 1.]];
        let mut goal = [0.; 594];
        for k in 0..10 {
            let u = frame.saturating_add(k + 1).min(end_inclusive);
            let offset = k * 59;
            for i in 0..3 {
                goal[offset + i] = (0..3)
                    .map(|j| ht[i][j] * (self.root_p[[u, j]] - self.root_p[[frame, j]]))
                    .sum();
                goal[offset + 9 + i] = self.root_v_local[[u, i]];
                goal[offset + 12 + i] = self.root_w_local[[u, i]];
            }
            let rot = multiply(ht, self.rotation(u));
            for i in 0..3 {
                for j in 0..2 {
                    goal[offset + 3 + i * 2 + j] = rot[i][j];
                }
            }
            for j in 0..22 {
                goal[offset + 15 + j] = self.q[[u, j]];
                goal[offset + 37 + j] = self.qd[[u, j]] * 0.1;
            }
        }
        if !goal.iter().all(|v| v.is_finite()) {
            return Err("Nonfinite reference goal".into());
        }
        Ok(goal)
    }
}

pub fn load(package: &Package) -> Result<BTreeMap<String, Clip>, String> {
    let manifest = read_json(&package.verified_file("references/manifest.json")?)?;
    let schema = manifest["schema"]
        .as_str()
        .ok_or("Missing reference schema")?;
    if !schema.starts_with("q1-")
        || !schema.ends_with("-reference-subset-v1")
        || manifest["fps"] != 50
        || manifest["control_dt"] != 0.02
        || manifest["joint_names"] != serde_json::json!(NAMES)
        || manifest["trained_manifest_sha256"] != package.contract["trained_manifest_sha256"]
    {
        return Err("Unsupported reference manifest protocol/joint order".into());
    }
    let entries = manifest["clips"]
        .as_object()
        .ok_or("Missing reference clips")?;
    if Some(entries.len() as u64) != package.contract["reference_subset"]["count"].as_u64()
        || entries.is_empty()
    {
        return Err("Reference count differs from contract".into());
    }
    let mut clips = BTreeMap::new();
    for (id, entry) in entries {
        if id.is_empty() || !id.bytes().all(|v| v.is_ascii_alphanumeric() || v == b'_') {
            return Err("Invalid reference id".into());
        }
        let name = entry["file"].as_str().ok_or("Missing NPZ file")?;
        let p = package.verified_file(&format!("references/{name}"))?;
        if crate::supermimic::contract::hash(&p)?
            != entry["npz_sha256"].as_str().ok_or("Missing NPZ digest")?
        {
            return Err(format!("Reference digest differs: {id}"));
        }
        let n = entry["frames_50hz"]
            .as_u64()
            .and_then(|n| usize::try_from(n).ok())
            .filter(|n| *n > 0)
            .ok_or("Invalid reference frame count")?;
        if entry["npz_fps"] != 50
            || entry["duration_seconds"]
                .as_f64()
                .is_none_or(|d| !d.is_finite() || (d - (n - 1) as f64 * 0.02).abs() > 1e-5)
        {
            return Err(format!("Invalid NPZ timebase: {id}"));
        }
        let phases = entry["phases"]
            .as_object()
            .ok_or("Missing phase metadata")?;
        for (name, time) in phases {
            if !time
                .as_f64()
                .is_some_and(|t| t.is_finite() && t >= 0. && t <= n as f64 * 0.02 + 1e-6)
            {
                return Err(format!("Invalid reference phase {id}/{name}"));
            }
        }
        let mut npz = NpzReader::new(std::fs::File::open(&p).map_err(|e| e.to_string())?)
            .map_err(|e| e.to_string())?;
        let rotations: Array3<f32> = npz.by_name("root_R").map_err(|e| e.to_string())?;
        let q: Array2<f32> = npz.by_name("q").map_err(|e| e.to_string())?;
        let qd: Array2<f32> = npz.by_name("qd").map_err(|e| e.to_string())?;
        let root_p: Array2<f32> = npz.by_name("root_p").map_err(|e| e.to_string())?;
        let root_v_local: Array2<f32> = npz.by_name("root_v_local").map_err(|e| e.to_string())?;
        let root_w_local: Array2<f32> = npz.by_name("root_w_local").map_err(|e| e.to_string())?;
        if rotations.shape() != [n, 3, 3]
            || q.shape() != [n, 22]
            || qd.shape() != [n, 22]
            || [root_p.shape(), root_v_local.shape(), root_w_local.shape()]
                .iter()
                .any(|s| *s != [n, 3])
            || !rotations
                .iter()
                .chain(q.iter())
                .chain(qd.iter())
                .chain(root_p.iter())
                .chain(root_v_local.iter())
                .chain(root_w_local.iter())
                .all(|v| v.is_finite())
        {
            return Err(format!("Invalid native reference shapes/values: {id}"));
        }
        let clip = Clip {
            metadata: entry.clone(),
            rotations,
            q,
            qd,
            root_p,
            root_v_local,
            root_w_local,
        };
        for f in 0..n {
            let r = clip.rotation(f);
            for i in 0..3 {
                for j in 0..3 {
                    let dot = (0..3).map(|k| r[i][k] * r[j][k]).sum::<f32>();
                    if (dot - if i == j { 1. } else { 0. }).abs() > 2e-3 {
                        return Err(format!("Invalid reference rotation: {id}/{f}"));
                    }
                }
            }
            let det = r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
                - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
                + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]);
            if (det - 1.).abs() > 2e-3 {
                return Err(format!("Improper reference rotation: {id}/{f}"));
            }
        }
        clips.insert(id.clone(), clip);
    }
    for id in ["standing", "floor_fold", "official_lie", "official_getUp"] {
        if !clips.contains_key(id) {
            return Err(format!("Missing task reference {id}"));
        }
    }
    for (id, keys) in [
        (
            "floor_fold",
            &[
                "hold_start",
                "hold_end",
                "return_start",
                "return_squat_start",
                "return_end",
                "stand",
            ][..],
        ),
        ("official_lie", &["hold_start", "hold_end"][..]),
        ("official_getUp", &["rise_start", "stand_start"][..]),
    ] {
        let c = &clips[id];
        let phases = &c.metadata["phases"];
        let mut previous = 0.;
        for &key in keys {
            let t = phases[key]
                .as_f64()
                .ok_or_else(|| format!("Missing task phase {id}/{key}"))?;
            if t < previous {
                return Err(format!("Nonmonotonic task phase {id}/{key}"));
            }
            previous = t;
        }
    }
    for (id, c) in &clips {
        if c.metadata["group"] == "lafan1" {
            let mut previous = 0.;
            for key in ["entry", "dance", "return", "stand"] {
                let time = c.metadata["phases"][key]
                    .as_f64()
                    .ok_or_else(|| format!("Missing dance phase {id}/{key}"))?;
                if time < previous {
                    return Err(format!("Nonmonotonic dance phase {id}/{key}"));
                }
                previous = time;
            }
        }
    }
    Ok(clips)
}
