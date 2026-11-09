use super::runtime::Runtime;
use axum::{
    body::to_bytes,
    extract::{Request, State},
    http::{header, Method, StatusCode},
    response::{IntoResponse, Response},
    Router,
};
use serde_json::{json, Value};
use std::{
    collections::HashSet,
    path::Path,
    sync::{Arc, Mutex},
};
struct App {
    runtime: Runtime,
    key: String,
    hosts: HashSet<String>,
    actions: Mutex<()>,
}
fn response(status: StatusCode, value: String, mime: &str) -> Response {
    let mut r = (status, value).into_response();
    r.headers_mut()
        .insert(header::CONTENT_TYPE, mime.parse().unwrap());
    r.headers_mut()
        .insert(header::CACHE_CONTROL, "no-store".parse().unwrap());
    r.headers_mut()
        .insert("X-Content-Type-Options", "nosniff".parse().unwrap());
    r.headers_mut().insert("Content-Security-Policy","default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; frame-ancestors 'none'".parse().unwrap());
    r
}
fn json_response(code: StatusCode, value: Value) -> Response {
    response(code, value.to_string(), "application/json")
}
fn equal(a: &str, b: &str) -> bool {
    a.len() == b.len() && a.bytes().zip(b.bytes()).fold(0, |x, (a, b)| x | (a ^ b)) == 0
}
async fn handle(State(app): State<Arc<App>>, request: Request) -> Response {
    let host = request
        .headers()
        .get(header::HOST)
        .and_then(|v| v.to_str().ok())
        .unwrap_or("");
    let allowed = host
        .parse::<axum::http::uri::Authority>()
        .ok()
        .is_some_and(|a| app.hosts.contains(a.host()));
    if !allowed {
        return json_response(StatusCode::FORBIDDEN, json!({"error":"Host not allowed"}));
    }
    let path = request.uri().path().to_owned();
    if request.method() == Method::GET {
        return match path.as_str() {
            "/" => response(
                StatusCode::OK,
                include_str!("../../web/supermimic.html").into(),
                "text/html; charset=utf-8",
            ),
            "/supermimic.js" => response(
                StatusCode::OK,
                include_str!("../../web/supermimic.js").into(),
                "text/javascript; charset=utf-8",
            ),
            "/supermimic.css" => response(
                StatusCode::OK,
                include_str!("../../web/supermimic.css").into(),
                "text/css; charset=utf-8",
            ),
            "/api/state" => json_response(StatusCode::OK, app.runtime.snapshot()),
            "/api/capabilities" => json_response(
                StatusCode::OK,
                app.runtime.snapshot()["capabilities"].clone(),
            ),
            "/api/dances" => json_response(
                StatusCode::OK,
                json!(app
                    .runtime
                    .clips
                    .values()
                    .filter(|c| c.metadata["role"] == "dance")
                    .map(|c| {
                        let mut m = c.metadata.clone();
                        m["hardware_allowed"] = json!(app
                            .runtime
                            .hardware
                            .supported_motion
                            .iter()
                            .any(|s| Some(s.as_str()) == m["id"].as_str()));
                        m["observation_inference_allowed"] = json!(app.runtime.shadow_only);
                        m
                    })
                    .collect::<Vec<_>>()),
            ),
            _ => json_response(StatusCode::NOT_FOUND, json!({"error":"Not found"})),
        };
    }
    if request.method() != Method::POST {
        return json_response(
            StatusCode::METHOD_NOT_ALLOWED,
            json!({"error":"POST required"}),
        );
    }
    let key = request
        .headers()
        .get("X-Q1-Operator")
        .and_then(|v| v.to_str().ok())
        .unwrap_or("");
    if !equal(key, &app.key) {
        return json_response(StatusCode::UNAUTHORIZED, json!({"error":"请输入操作口令"}));
    }
    if request.headers().get(header::ORIGIN).is_some_and(|v| {
        v != format!("http://{host}").as_str() && v != format!("https://{host}").as_str()
    }) {
        return json_response(StatusCode::FORBIDDEN, json!({"error":"Origin not allowed"}));
    }
    if !request
        .headers()
        .get(header::CONTENT_TYPE)
        .is_some_and(|v| v.to_str().unwrap_or("").starts_with("application/json"))
    {
        return json_response(
            StatusCode::UNSUPPORTED_MEDIA_TYPE,
            json!({"error":"JSON required"}),
        );
    }
    let body = match tokio::time::timeout(
        std::time::Duration::from_millis(250),
        to_bytes(request.into_body(), 8192),
    )
    .await
    {
        Ok(Ok(b)) => b,
        _ => {
            return json_response(
                StatusCode::BAD_REQUEST,
                json!({"error":"Body timeout or oversized"}),
            )
        }
    };
    let payload: Value = match serde_json::from_slice::<Value>(&body) {
        Ok(v) if v.is_object() => v,
        _ => return json_response(StatusCode::BAD_REQUEST, json!({"error":"Invalid JSON"})),
    };
    let result = if path == "/api/input" {
        app.runtime.input(&payload)
    } else {
        let name = match path.as_str() {
            "/api/action" => payload["action"].as_str().unwrap_or(""),
            "/api/dance/start" => "dance",
            "/api/dance/cancel" => "cancel",
            "/api/dance/queue" => {
                if payload["operation"] == "clear" {
                    "clear_queue"
                } else if payload["operation"] == "remove" {
                    "remove_queue"
                } else {
                    "queue"
                }
            }
            _ => return json_response(StatusCode::NOT_FOUND, json!({"error":"Unknown endpoint"})),
        };
        let _serial = app.actions.lock().unwrap();
        let client = payload["client_id"].as_str();
        let id = payload["request_id"].as_str();
        let stop = matches!(name, "stop" | "cancel" | "disable");
        let authorized = {
            let s = app.runtime.state.lock().unwrap();
            if !stop
                && ((!app.runtime.shadow_only && client != s.owner.as_deref())
                    || client.is_none_or(|s| s.is_empty() || s.len() > 80)
                    || id.is_none_or(|s| s.len() > 100 || s.is_empty()))
            {
                false
            } else if let Some(id) = id {
                if s.requests.iter().any(|old| old == id) {
                    return json_response(StatusCode::OK, json!({"ok":true,"duplicate":true}));
                }
                true
            } else {
                true
            }
        };
        if !authorized {
            Err("Action requires the current operator and request_id".into())
        } else {
            let result = app.runtime.action(name, Some(&payload));
            if result.is_ok() {
                if let Some(id) = id {
                    let mut s = app.runtime.state.lock().unwrap();
                    if s.requests.len() >= 64 {
                        s.requests.pop_front();
                    }
                    s.requests.push_back(id.into());
                }
            }
            result
        }
    };
    match result {
        Ok(()) => json_response(StatusCode::OK, json!({"ok":true})),
        Err(e) => json_response(StatusCode::BAD_REQUEST, json!({"ok":false,"error":e})),
    }
}
pub async fn run(
    runtime: Runtime,
    host: &str,
    port: u16,
    allowed: Vec<String>,
    key_file: &Path,
) -> Result<(), String> {
    let key = std::fs::read_to_string(key_file)
        .map_err(|e| format!("Operator key file: {e}"))?
        .trim()
        .to_owned();
    if key.len() < 32 {
        return Err("Operator key must contain at least32 characters".into());
    }
    let mut hosts: HashSet<String> = ["localhost", "127.0.0.1", "::1"]
        .into_iter()
        .map(String::from)
        .collect();
    hosts.extend(allowed);
    if !matches!(host, "0.0.0.0" | "::") {
        hosts.insert(host.into());
    }
    let app = Arc::new(App {
        runtime,
        key,
        hosts,
        actions: Mutex::new(()),
    });
    let listener = tokio::net::TcpListener::bind((host, port))
        .await
        .map_err(|e| e.to_string())?;
    println!("SuperMimic real ROS console: http://{host}:{port}");
    let stopping = app.clone();
    axum::serve(listener, Router::new().fallback(handle).with_state(app))
        .with_graceful_shutdown(async move {
            let mut term =
                tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate()).unwrap();
            tokio::select! {_=tokio::signal::ctrl_c()=>{},_=term.recv()=>{}}
            let _ = stopping.runtime.action("stop", None);
        })
        .await
        .map_err(|e| e.to_string())
}
