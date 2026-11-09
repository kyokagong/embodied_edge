use serde_json::Value;

pub fn load(path: Option<&str>) -> Result<Value, String> {
    let mut value: Value = serde_json::from_str(include_str!("../../config/default.json"))
        .map_err(|e| e.to_string())?;
    if let Some(path) = path {
        let text = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
        let supplied: Value = serde_json::from_str(&text).map_err(|e| e.to_string())?;
        merge(&mut value, &supplied)?;
    }
    validate(&value)?;
    Ok(value)
}

fn merge(target: &mut Value, supplied: &Value) -> Result<(), String> {
    let fields = supplied
        .as_object()
        .ok_or("Configuration must be a JSON object")?;
    let object = target
        .as_object_mut()
        .ok_or("Configuration must be a JSON object")?;
    for (key, item) in fields {
        let current = object
            .get_mut(key)
            .ok_or_else(|| format!("Unknown configuration key: {key}"))?;
        if current.is_object() {
            merge(current, item)?;
        } else {
            *current = item.clone();
        }
    }
    Ok(())
}

fn number(value: &Value, name: &str) -> Result<f64, String> {
    value
        .as_f64()
        .filter(|x| x.is_finite())
        .ok_or_else(|| format!("Invalid {name}"))
}

fn validate(value: &Value) -> Result<(), String> {
    for key in [
        "input_timeout_s",
        "pico_timeout_s",
        "neutral_threshold",
        "deadzone",
    ] {
        if number(&value[key], key)? <= 0. {
            return Err(format!("Invalid {key}"));
        }
    }
    if value["neutral_threshold"].as_f64() > value["deadzone"].as_f64()
        || number(&value["deadzone"], "deadzone")? >= 1.
    {
        return Err("Require neutral_threshold <= deadzone < 1".into());
    }
    for (name, limits) in value["upper"]["limits"].as_object().unwrap() {
        let limits = limits
            .as_array()
            .ok_or_else(|| format!("Invalid joint limits: {name}"))?;
        if limits.len() != 2 || number(&limits[0], name)? >= number(&limits[1], name)? {
            return Err(format!("Invalid joint limits: {name}"));
        }
    }
    for (key, setting) in value["velocity"].as_object().unwrap() {
        if number(setting, key)? < 0. {
            return Err(format!("Invalid velocity setting: {key}"));
        }
    }
    let minimum = number(&value["velocity"]["min_nonzero"], "min_nonzero")?;
    for key in ["max_forward", "max_lateral", "max_angular"] {
        if number(&value["velocity"][key], key)? < minimum {
            return Err("Velocity maxima must cover the configured nonzero minimum".into());
        }
    }
    for (key, maximum) in [
        ("max_forward", 0.3),
        ("max_lateral", 0.2),
        ("max_angular", 0.5),
    ] {
        if number(&value["velocity"][key], key)? > maximum {
            return Err(format!(
                "Velocity setting {key} exceeds native SDK limit {maximum}"
            ));
        }
    }
    if !value["upper"]["hardware_limits_confirmed"].is_boolean()
        || !value["gamepad"]["rb_toggle_enabled"].is_boolean()
    {
        return Err("Hardware confirmation and RB toggle must be boolean".into());
    }
    if !(1. ..=3.).contains(&number(&value["gamepad"]["rb_hold_s"], "rb_hold_s")?) {
        return Err("rb_hold_s must be between 1 and 3 seconds".into());
    }
    let mut buttons = std::collections::HashSet::new();
    for index in value["buttons"].as_object().unwrap().values() {
        let index = index
            .as_u64()
            .filter(|x| *x < 17)
            .ok_or("Button indices must be integers in [0,16]")?;
        if !buttons.insert(index) {
            return Err("Each control requires a distinct button".into());
        }
    }
    validate_ros(value)
}

/// Validate the native SDK's configuration even when used without the HTTP CLI.
pub fn validate_ros(config: &Value) -> Result<(), String> {
    if !config.is_object() {
        return Err("Configuration must be a JSON object".into());
    }
    let empty = serde_json::json!({});
    let ros = config.get("ros2").unwrap_or(&empty);
    if !ros.is_object() {
        return Err("ros2 configuration must be an object".into());
    }
    let source = ros
        .get("source_name")
        .map_or(Some("edge_q1"), Value::as_str)
        .ok_or("source_name must be a string")?;
    let valid_source = source == "edge_q1"
        || source.strip_prefix("edge_q1_").is_some_and(|suffix| {
            suffix.split('_').all(|segment| {
                !segment.is_empty() && segment.bytes().all(|c| c.is_ascii_alphanumeric())
            })
        });
    if !valid_source {
        return Err("Use a unique edge_q1 source prefix".into());
    }
    let priority = ros
        .get("priority")
        .map_or(Some(30), Value::as_u64)
        .ok_or("priority must be an integer")?;
    if !(1..80).contains(&priority) {
        return Err("priority must be below native rc80".into());
    }
    let timeout = ros
        .get("source_timeout_ms")
        .map_or(Some(1000), Value::as_u64)
        .ok_or("source_timeout_ms must be an integer")?;
    if !(250..=1000).contains(&timeout) {
        return Err("source_timeout_ms must be 250–1000 ms".into());
    }
    for (key, default, maximum) in [
        ("feedback_timeout_s", 0.25, 0.5),
        ("command_timeout_s", 0.25, 0.5),
        ("service_timeout_s", 3., f64::INFINITY),
        ("transition_timeout_s", 20., f64::INFINITY),
        ("source_poll_interval_s", 0.1, f64::INFINITY),
    ] {
        let value = ros.get(key).map_or(Ok(default), |v| number(v, key))?;
        if value <= 0. || value > maximum {
            return Err(format!("Invalid {key}"));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    #[test]
    fn defaults_match_current_pad_limits() {
        let config = super::load(None).unwrap();
        assert_eq!(config["deadzone"], 0.18);
        assert_eq!(config["input_timeout_s"], 0.25);
        assert_eq!(config["gamepad"]["rb_hold_s"], 1.0);
    }

    #[test]
    fn direct_sdk_config_rejects_invalid_native_arbitration_settings() {
        use serde_json::json;
        for invalid in [
            json!({"priority":80}),
            json!({"source_name":"rc"}),
            json!({"source_name":"edge_q1__bad"}),
            json!({"source_timeout_ms":249}),
            json!({"command_timeout_s":0.6}),
            json!({"service_timeout_s":0.}),
        ] {
            assert!(super::validate_ros(&json!({"ros2":invalid})).is_err());
        }
        super::validate_ros(&json!({"ros2":{"source_name":"edge_q1_test_A1","priority":30}}))
            .unwrap();
    }
}
