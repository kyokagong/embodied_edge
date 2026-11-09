use super::{contract::contained, observation::Rotation};
use ndarray::{Array1, Array2, Array3};
use ndarray_npy::NpzReader;
use serde_json::Value;
use std::{collections::BTreeMap, path::Path};
pub struct Clip {
    pub metadata: Value,
    pub goals: Array2<f32>,
    pub rotations: Array3<f32>,
}
impl Clip {
    pub fn frame(&self, f: usize) -> (Rotation, &[f32]) {
        let f = f.min(self.goals.nrows() - 1);
        (
            std::array::from_fn(|i| std::array::from_fn(|j| self.rotations[[f, i, j]])),
            &self.goals.as_slice().unwrap()[f * 590..(f + 1) * 590],
        )
    }
}
pub fn load(root: &Path) -> Result<BTreeMap<String, Clip>, String> {
    let catalog: Value = serde_json::from_slice(
        &std::fs::read(root.join("dance_catalog.json")).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())?;
    let mut clips = BTreeMap::new();
    for entry in catalog["clips"].as_array().ok_or("Missing clip list")? {
        let id = entry["id"].as_str().ok_or("Missing clip id")?;
        if !id.bytes().all(|x| x.is_ascii_alphanumeric() || x == b'_') {
            return Err("Invalid clip id".into());
        }
        let p = contained(root, entry["file"].as_str().ok_or("Missing clip file")?)?;
        if super::contract::hash(&p)? != entry["sha256"].as_str().ok_or("Missing clip hash")? {
            return Err("Clip digest differs".into());
        }
        let mut npz = NpzReader::new(std::fs::File::open(p).map_err(|e| e.to_string())?)
            .map_err(|e| e.to_string())?;
        let goals: Array2<f32> = npz.by_name("goal_590").map_err(|e| e.to_string())?;
        let rotations: Array3<f32> = npz.by_name("root_R").map_err(|e| e.to_string())?;
        let times: Array1<f64> = npz.by_name("time_s").map_err(|e| e.to_string())?;
        let n = entry["frames_50hz"].as_u64().ok_or("Missing frame count")? as usize;
        if goals.shape() != [n, 590]
            || rotations.shape() != [n, 3, 3]
            || times.len() != n
            || n == 0
            || !goals.iter().chain(rotations.iter()).all(|x| x.is_finite())
            || !times
                .iter()
                .enumerate()
                .all(|(i, t)| (*t - i as f64 * 0.02).abs() < 1e-7)
        {
            return Err("Invalid dance data shape/time/values".into());
        }
        if clips
            .insert(
                id.to_owned(),
                Clip {
                    metadata: entry.clone(),
                    goals,
                    rotations,
                },
            )
            .is_some()
        {
            return Err("Duplicate clip".into());
        }
    }
    if clips.len() != 9 {
        return Err("Expected 8 dances and standing".into());
    }
    Ok(clips)
}
