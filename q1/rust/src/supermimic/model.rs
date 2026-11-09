use ort::{session::Session, value::Tensor};
use std::path::Path;
pub struct Model {
    session: Session,
}
impl Model {
    pub fn load(p: &Path) -> Result<Self, String> {
        Self::load_with_threads(p, 2)
    }
    pub fn load_with_threads(p: &Path, threads: usize) -> Result<Self, String> {
        if !(1..=4).contains(&threads) {
            return Err("ORT intra-op threads must be 1..4".into());
        }
        let session = Session::builder()
            .map_err(|e| e.to_string())?
            .with_intra_threads(threads)
            .map_err(|e| e.to_string())?
            .commit_from_file(p)
            .map_err(|e| e.to_string())?;
        Ok(Self { session })
    }
    pub fn infer(&mut self, x: &[f32]) -> Result<[f32; 22], String> {
        if x.len() != 1376 || !x.iter().all(|v| v.is_finite()) {
            return Err("Invalid ONNX observation".into());
        }
        let input = Tensor::from_array(([1usize, 1376], x.to_vec())).map_err(|e| e.to_string())?;
        let output = self
            .session
            .run(ort::inputs!["obs"=>input])
            .map_err(|e| e.to_string())?;
        let (shape, data) = output["actions"]
            .try_extract_tensor::<f32>()
            .map_err(|e| e.to_string())?;
        if shape.as_ref() != [1, 22] || data.len() != 22 || !data.iter().all(|v| v.is_finite()) {
            return Err("Invalid ONNX output".into());
        }
        Ok(std::array::from_fn(|i| data[i]))
    }
    pub fn golden(&mut self, p: &Path) -> Result<serde_json::Value, String> {
        let d: serde_json::Value =
            serde_json::from_slice(&std::fs::read(p).map_err(|e| e.to_string())?)
                .map_err(|e| e.to_string())?;
        let inputs: Vec<Vec<f32>> =
            serde_json::from_value(d["obs"].clone()).map_err(|e| e.to_string())?;
        let expected: Vec<Vec<f32>> =
            serde_json::from_value(d["actions"].clone()).map_err(|e| e.to_string())?;
        if inputs.len() != 68 || expected.len() != 68 {
            return Err("Golden sample count differs".into());
        }
        let mut max = 0f32;
        let mut times = vec![];
        for (x, y) in inputs.iter().zip(&expected) {
            let t = std::time::Instant::now();
            let a = self.infer(x)?;
            times.push(t.elapsed().as_secs_f64() * 1000.);
            if y.len() != 22 {
                return Err("Bad golden output shape".into());
            }
            for i in 0..22 {
                let e = (a[i] - y[i]).abs();
                max = max.max(e);
                if e > 3e-5 + 3e-4 * y[i].abs() {
                    return Err(format!("Rust ONNX parity failed: {e}"));
                }
            }
        }
        for i in 0..1200 {
            let t = std::time::Instant::now();
            self.infer(&inputs[i % 68])?;
            if i >= 100 {
                times.push(t.elapsed().as_secs_f64() * 1000.);
            }
        }
        times.sort_by(f64::total_cmp);
        Ok(
            serde_json::json!({"samples":68,"max_action_abs_error":max,"latency_ms":{"p50":times[times.len()/2],"p99":times[times.len()*99/100],"max":times.last()},"hardware_command_sent":false}),
        )
    }
}
