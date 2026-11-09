//! Native dual-input inference. The graph and package independently describe the same ABI.
use super::{observation::command_goal, package::Package, reference};
use ndarray::{Array2, Array3};
use ndarray_npy::NpzReader;
use ort::{
    session::Session,
    tensor::TensorElementType,
    value::{Tensor, ValueType},
};
use serde_json::{json, Value};

pub struct Model {
    session: Session,
}
fn tensor_is(t: &ValueType, want: &[i64]) -> bool {
    matches!(t,ValueType::Tensor {ty:TensorElementType::Float32,shape,..} if shape.as_ref()==want)
}
impl Model {
    pub fn load(package: &Package, threads: usize) -> Result<Self, String> {
        if !(1..=4).contains(&threads) {
            return Err("ORT intra-op threads must be 1..4".into());
        }
        if crate::supermimic::contract::hash(&package.model_path)? != package.model_sha256 {
            return Err("Native model changed after package validation".into());
        }
        let session = Session::builder()
            .map_err(|e| e.to_string())?
            .with_intra_threads(threads)
            .map_err(|e| e.to_string())?
            .commit_from_file(&package.model_path)
            .map_err(|e| e.to_string())?;
        if session.inputs.len() != 2
            || session.outputs.len() != 1
            || session.inputs[0].name != "obs_history"
            || !tensor_is(&session.inputs[0].input_type, &[1, 10, 78])
            || session.inputs[1].name != "goal"
            || !tensor_is(&session.inputs[1].input_type, &[1, 594])
            || session.outputs[0].name != "actions"
            || !tensor_is(&session.outputs[0].output_type, &[1, 22])
        {
            return Err("Actual ONNX ABI differs from native float32 obs_history[1,10,78], goal[1,594], actions[1,22]".into());
        }
        Ok(Self { session })
    }
    pub fn infer(&mut self, history: &[f32], goal: &[f32]) -> Result<[f32; 22], String> {
        if history.len() != 780
            || goal.len() != 594
            || !history.iter().chain(goal.iter()).all(|v| v.is_finite())
        {
            return Err("Invalid native ONNX inputs".into());
        }
        let h =
            Tensor::from_array(([1usize, 10, 78], history.to_vec())).map_err(|e| e.to_string())?;
        let g = Tensor::from_array(([1usize, 594], goal.to_vec())).map_err(|e| e.to_string())?;
        let result = self
            .session
            .run(ort::inputs!["obs_history"=>h,"goal"=>g])
            .map_err(|e| e.to_string())?;
        let (shape, data) = result["actions"]
            .try_extract_tensor::<f32>()
            .map_err(|e| e.to_string())?;
        if shape.as_ref() != [1, 22] || data.len() != 22 || !data.iter().all(|v| v.is_finite()) {
            return Err("Invalid native ONNX output".into());
        }
        Ok(std::array::from_fn(|i| data[i]))
    }
    pub fn golden(&mut self, package: &Package) -> Result<Value, String> {
        let cases: Vec<Value> = serde_json::from_slice(
            &std::fs::read(package.verified_file("validation/cases.json")?)
                .map_err(|e| e.to_string())?,
        )
        .map_err(|e| e.to_string())?;
        let mut gold = NpzReader::new(
            std::fs::File::open(package.verified_file("validation/golden.npz")?)
                .map_err(|e| e.to_string())?,
        )
        .map_err(|e| e.to_string())?;
        let history: Array3<f32> = gold.by_name("obs_history").map_err(|e| e.to_string())?;
        let goals: Array2<f32> = gold.by_name("goal").map_err(|e| e.to_string())?;
        let expected: Array2<f32> = gold
            .by_name("expected_actions")
            .map_err(|e| e.to_string())?;
        let n = cases.len();
        if n == 0
            || history.shape() != [n, 10, 78]
            || goals.shape() != [n, 594]
            || expected.shape() != [n, 22]
            || !history
                .iter()
                .chain(goals.iter())
                .chain(expected.iter())
                .all(|v| v.is_finite())
        {
            return Err("Invalid golden native shapes/values/case count".into());
        }
        let refs = reference::load(package)?;
        let mut max_error = 0f32;
        let mut times = Vec::with_capacity(n);
        for (i, case) in cases.iter().enumerate() {
            let rebuilt = match case["mode"].as_str() {
                Some("velocity") => command_goal(
                    &package.home,
                    serde_json::from_value(case["command"].clone()).map_err(|e| e.to_string())?,
                )?,
                Some("motion") => {
                    let clip = refs
                        .get(
                            case["name"]
                                .as_str()
                                .ok_or("Missing golden reference name")?,
                        )
                        .ok_or("Unknown golden reference")?;
                    let frame = case["frame"]
                        .as_u64()
                        .and_then(|f| usize::try_from(f).ok())
                        .ok_or("Invalid golden frame")?;
                    clip.goal(frame, clip.len() - 1)?
                }
                _ => return Err("Invalid golden task mode".into()),
            };
            let goal = goals.row(i);
            for (j, (&a, &b)) in rebuilt.iter().zip(goal.iter()).enumerate() {
                if (a - b).abs() > 2e-5 + 2e-5 * b.abs() {
                    return Err(format!(
                        "Rebuilt golden goal differs at case {i}, feature {j}"
                    ));
                }
            }
            let h = history.index_axis(ndarray::Axis(0), i);
            let at = std::time::Instant::now();
            let actual = self.infer(
                h.as_slice().ok_or("Noncontiguous golden history")?,
                goal.as_slice().ok_or("Noncontiguous golden goal")?,
            )?;
            times.push(at.elapsed().as_secs_f64() * 1000.);
            for j in 0..22 {
                let e = (actual[j] - expected[[i, j]]).abs();
                max_error = max_error.max(e);
                if e > 2e-4 + 2e-5 * expected[[i, j]].abs() {
                    return Err(format!(
                        "Native ONNX golden parity failed at case {i}, action {j}: {e}"
                    ));
                }
            }
        }
        times.sort_by(f64::total_cmp);
        Ok(
            json!({"samples":n,"max_action_abs_error":max_error,"reference_goal_checks":true,
            "model_sha256":package.model_sha256,"contract_sha256":package.contract_sha256,
            "native_abi":{"obs_history":[1,10,78],"goal":[1,594],"actions":[1,22],"dtype":"float32"},
            "latency_ms":{"p50":times[n/2],"p99":times[(n-1)*99/100],"max":times[n-1]},
            "hardware_command_sent":false,"closed_loop_qualified":false}),
        )
    }
}
