use crate::{core::Controller, sdk::monotonic};
use axum::{
    body::to_bytes,
    extract::{Request, State},
    http::{header, HeaderMap, Method, StatusCode},
    response::{IntoResponse, Response},
    Router,
};
use rand::RngCore;
use serde_json::{json, Value};
use std::{
    collections::HashSet,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    time::{Duration, Instant},
};

#[derive(Clone)]
pub struct App {
    controller: Arc<Mutex<Controller>>,
    token: String,
    hosts: HashSet<String>,
}

impl App {
    pub fn new(controller: Controller, host: &str, allowed: Vec<String>) -> Self {
        let mut bytes = [0; 32];
        rand::thread_rng().fill_bytes(&mut bytes);
        let token = bytes.iter().map(|byte| format!("{byte:02x}")).collect();
        let mut hosts: HashSet<String> = ["localhost", "127.0.0.1", "::1"]
            .into_iter()
            .map(String::from)
            .collect();
        hosts.extend(allowed);
        if host != "0.0.0.0" && host != "::" {
            hosts.insert(host.into());
        }
        Self {
            controller: Arc::new(Mutex::new(controller)),
            token,
            hosts,
        }
    }

    pub fn router(&self) -> Router {
        Router::new().fallback(handle).with_state(self.clone())
    }

    fn state(&self) -> Value {
        self.controller
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .state(monotonic())
    }

    fn stop(&self) {
        let _ = self
            .controller
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .action("stop", None, monotonic());
    }

    fn host_ok(&self, headers: &HeaderMap) -> bool {
        headers
            .get(header::HOST)
            .and_then(|h| h.to_str().ok())
            .and_then(|h| h.parse::<axum::http::uri::Authority>().ok())
            .is_some_and(|a| self.hosts.contains(a.host().trim_matches(['[', ']'])))
    }
}

fn token_equal(supplied: &str, expected: &str) -> bool {
    if supplied.len() != expected.len() {
        return false;
    }
    supplied
        .bytes()
        .zip(expected.bytes())
        .fold(0u8, |difference, (a, b)| difference | (a ^ b))
        == 0
}

fn response(status: StatusCode, payload: impl Into<String>, mime: &str) -> Response {
    let mut result = (status, payload.into()).into_response();
    let headers = result.headers_mut();
    headers.insert(header::CONTENT_TYPE, mime.parse().unwrap());
    headers.insert(header::CACHE_CONTROL, "no-store".parse().unwrap());
    headers.insert("X-Content-Type-Options", "nosniff".parse().unwrap());
    headers.insert("Referrer-Policy", "same-origin".parse().unwrap());
    headers.insert("Content-Security-Policy", "default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; img-src 'self' data:; frame-ancestors 'none'; base-uri 'none'".parse().unwrap());
    if status.is_client_error() || status.is_server_error() {
        headers.insert(header::CONNECTION, "close".parse().unwrap());
    }
    result
}

fn json_response(status: StatusCode, value: Value) -> Response {
    response(status, value.to_string(), "application/json; charset=utf-8")
}

fn reject(status: StatusCode, message: &str) -> Response {
    json_response(status, json!({"ok":false,"error":message}))
}

async fn handle(State(app): State<App>, request: Request) -> Response {
    if !app.host_ok(request.headers()) {
        return reject(StatusCode::FORBIDDEN, "Host not allowed");
    }
    let path = request.uri().path().to_owned();
    if request.method() == Method::GET {
        return match path.as_str() {
            "/api/state" => json_response(StatusCode::OK, app.state()),
            "/" => response(
                StatusCode::OK,
                include_str!("../../q1_edge/web/index.html")
                    .replace("__Q1_API_TOKEN__", &app.token),
                "text/html; charset=utf-8",
            ),
            "/app.js" => response(
                StatusCode::OK,
                include_str!("../../q1_edge/web/app.js"),
                "text/javascript; charset=utf-8",
            ),
            "/style.css" => response(
                StatusCode::OK,
                include_str!("../../q1_edge/web/style.css"),
                "text/css; charset=utf-8",
            ),
            _ => reject(StatusCode::NOT_FOUND, "Not found"),
        };
    }
    if request.method() != Method::POST {
        return reject(StatusCode::METHOD_NOT_ALLOWED, "Method not allowed");
    }
    let headers = request.headers();
    let host = headers
        .get(header::HOST)
        .and_then(|v| v.to_str().ok())
        .unwrap_or("");
    if let Some(origin) = headers.get(header::ORIGIN) {
        let origin = origin.to_str().unwrap_or("");
        if origin != format!("http://{host}") && origin != format!("https://{host}") {
            return reject(StatusCode::FORBIDDEN, "Origin or Host not allowed");
        }
    }
    let token = headers
        .get("X-Q1-Token")
        .and_then(|v| v.to_str().ok())
        .unwrap_or("");
    if !token_equal(token, &app.token) {
        return reject(StatusCode::FORBIDDEN, "Missing or invalid operator token");
    }
    let mime = headers
        .get(header::CONTENT_TYPE)
        .and_then(|v| v.to_str().ok())
        .unwrap_or("");
    if !mime
        .split(';')
        .next()
        .unwrap_or("")
        .trim()
        .eq_ignore_ascii_case("application/json")
    {
        return reject(StatusCode::UNSUPPORTED_MEDIA_TYPE, "JSON required");
    }
    let size = headers
        .get(header::CONTENT_LENGTH)
        .and_then(|v| v.to_str().ok())
        .and_then(|v| v.parse::<usize>().ok())
        .unwrap_or(0);
    if size == 0 || size > 32768 {
        return reject(
            if size > 32768 {
                StatusCode::PAYLOAD_TOO_LARGE
            } else {
                StatusCode::BAD_REQUEST
            },
            "Invalid request length",
        );
    }
    let compact = path == "/api/input" && headers.get("X-Q1-Compact").is_some_and(|v| v == "1");
    let body =
        match tokio::time::timeout(Duration::from_secs(3), to_bytes(request.into_body(), 32768))
            .await
        {
            Ok(Ok(body)) => body,
            _ => {
                return reject(
                    StatusCode::BAD_REQUEST,
                    "Request body timed out or exceeded limit",
                )
            }
        };
    let payload: Value = match serde_json::from_slice::<Value>(&body) {
        Ok(value) if value.is_object() => value,
        _ => {
            return reject(
                StatusCode::BAD_REQUEST,
                "Request must be a finite JSON object",
            )
        }
    };
    if !matches!(path.as_str(), "/api/input" | "/api/action" | "/api/pico") {
        return reject(StatusCode::NOT_FOUND, "Not found");
    }
    let mut core = app.controller.lock().unwrap_or_else(|e| e.into_inner());
    let now = monotonic();
    let result = match path.as_str() {
        "/api/input" => core.input(&payload, now),
        "/api/action" => {
            let name = payload["action"].as_str().unwrap_or("");
            let client = payload["client_id"].as_str();
            if !matches!(name, "stop" | "disable")
                && !client.is_some_and(|v| !v.is_empty() && v.chars().count() <= 80)
            {
                Err("Action requires a valid client_id".into())
            } else {
                core.action(name, client, now)
            }
        }
        _ => Err("Pico 上肢功能尚未迁移到 Rust；请使用 Python 控制服务".into()),
    };
    let state = core.state(now);
    match result {
        Ok(()) if compact => json_response(
            StatusCode::OK,
            json!({"ok":true,"ack":{"sequence":state["input"]["sequence"].as_u64().map_or(json!(-1),|s|json!(s))}}),
        ),
        Ok(()) => json_response(StatusCode::OK, json!({"ok":true,"state":state})),
        Err(error) => json_response(
            StatusCode::BAD_REQUEST,
            json!({"ok":false,"error":error,"state":state}),
        ),
    }
}

pub async fn run(
    controller: Controller,
    host: &str,
    port: u16,
    allowed: Vec<String>,
    trace_path: Option<String>,
) -> Result<(), String> {
    let app = App::new(controller, host, allowed);
    let listener = tokio::net::TcpListener::bind((host, port))
        .await
        .map_err(|e| e.to_string())?;
    let address = listener.local_addr().map_err(|e| e.to_string())?;
    let quit = Arc::new(AtomicBool::new(false));
    let trace = if let Some(path) = trace_path {
        let file = std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(path)
            .map_err(|e| e.to_string())?;
        let (sender, receiver) = std::sync::mpsc::sync_channel::<Value>(32);
        std::thread::spawn(move || {
            use std::io::Write;
            let mut file = file;
            while let Ok(value) = receiver.recv() {
                if writeln!(file, "{value}").is_err() {
                    break;
                }
            }
        });
        Some(sender)
    } else {
        None
    };
    let control = app.controller.clone();
    let worker_quit = quit.clone();
    let worker = std::thread::Builder::new()
        .name("q1-controller".into())
        .spawn(move || {
            let mut deadline = Instant::now();
            let mut next_trace = 0.;
            while !worker_quit.load(Ordering::Relaxed) {
                let mut core = control.lock().unwrap_or_else(|e| e.into_inner());
                let now = monotonic();
                core.tick(now);
                if now >= next_trace {
                    if let Some(sender) = &trace {
                        let _ = sender.try_send(core.state(now));
                    }
                    next_trace = now + 0.1;
                }
                drop(core);
                deadline += Duration::from_millis(10);
                let instant = Instant::now();
                if instant > deadline + Duration::from_millis(50) {
                    deadline = instant;
                }
                std::thread::sleep(deadline.saturating_duration_since(instant));
            }
        })
        .map_err(|e| e.to_string())?;
    println!("Q1 Edge Rust console: http://{address}/");
    let signal_quit = quit.clone();
    let signal_app = app.clone();
    let shutdown = async move {
        #[cfg(unix)]
        {
            let mut term =
                tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())
                    .expect("SIGTERM handler");
            tokio::select! { _ = tokio::signal::ctrl_c() => {}, _ = term.recv() => {} }
        }
        #[cfg(not(unix))]
        let _ = tokio::signal::ctrl_c().await;
        signal_quit.store(true, Ordering::Relaxed);
        signal_app.stop();
    };
    let result = axum::serve(listener, app.router())
        .with_graceful_shutdown(shutdown)
        .await;
    quit.store(true, Ordering::Relaxed);
    let _ = worker.join();
    app.stop();
    result.map_err(|e| e.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdk::MockBackend;
    use axum::body::Body;
    use tower::ServiceExt;
    #[test]
    fn token_comparison_rejects_different_lengths() {
        assert!(!token_equal("abc", "abcd"));
        assert!(token_equal("abc", "abc"));
        assert!(!token_equal("abc", "abx"));
    }
    #[tokio::test]
    async fn http_rejects_foreign_host_missing_token_and_cross_origin() {
        let app = App::new(
            Controller::new(
                Box::new(MockBackend::new()),
                crate::config::load(None).unwrap(),
            ),
            "127.0.0.1",
            vec![],
        );
        let request = Request::builder()
            .uri("/api/state")
            .header("Host", "attacker.example")
            .body(Body::empty())
            .unwrap();
        assert_eq!(
            app.router().oneshot(request).await.unwrap().status(),
            StatusCode::FORBIDDEN
        );
        for token in ["invalid", app.token.as_str()] {
            let request = Request::builder()
                .method("POST")
                .uri("/api/action")
                .header("Host", "localhost")
                .header("Origin", "http://attacker.example")
                .header("X-Q1-Token", token)
                .body(Body::from("{}"))
                .unwrap();
            assert_eq!(
                app.router().oneshot(request).await.unwrap().status(),
                StatusCode::FORBIDDEN
            );
        }
        let request = Request::builder()
            .method("POST")
            .uri("/api/action")
            .header("Host", "localhost")
            .header("Origin", "http://localhost")
            .body(Body::from("{}"))
            .unwrap();
        assert_eq!(
            app.router().oneshot(request).await.unwrap().status(),
            StatusCode::FORBIDDEN
        );
        let request = Request::builder()
            .method("POST")
            .uri("/api/input")
            .header("Host", "localhost")
            .header("X-Q1-Token", &app.token)
            .header("Content-Type", "application/json")
            .header("Content-Length", "32769")
            .body(Body::from("{}"))
            .unwrap();
        assert_eq!(
            app.router().oneshot(request).await.unwrap().status(),
            StatusCode::PAYLOAD_TOO_LARGE
        );
    }
    #[tokio::test]
    async fn http_embeds_token_and_ack_has_only_sequence() {
        use http_body_util::BodyExt;
        let app = App::new(
            Controller::new(
                Box::new(MockBackend::new()),
                crate::config::load(None).unwrap(),
            ),
            "127.0.0.1",
            vec![],
        );
        let request = Request::builder()
            .uri("/")
            .header("Host", "localhost")
            .body(Body::empty())
            .unwrap();
        let result = app
            .router()
            .oneshot(request)
            .await
            .unwrap()
            .into_body()
            .collect()
            .await
            .unwrap()
            .to_bytes();
        assert!(std::str::from_utf8(&result).unwrap().contains(&app.token));
        let payload = json!({"client_id":"http-test","source":"browser_gamepad","sequence":1,"axes":[0.,0.,0.,0.],"buttons":[],"connected":true}).to_string();
        let request = Request::builder()
            .method("POST")
            .uri("/api/input")
            .header("Host", "localhost")
            .header("X-Q1-Token", &app.token)
            .header("Content-Type", "application/json")
            .header("Content-Length", payload.len())
            .header("X-Q1-Compact", "1")
            .body(Body::from(payload))
            .unwrap();
        let result = app.router().oneshot(request).await.unwrap();
        assert_eq!(result.status(), StatusCode::OK);
        let bytes = result.into_body().collect().await.unwrap().to_bytes();
        assert_eq!(
            serde_json::from_slice::<Value>(&bytes).unwrap(),
            json!({"ok":true,"ack":{"sequence":1}})
        );
    }
}
