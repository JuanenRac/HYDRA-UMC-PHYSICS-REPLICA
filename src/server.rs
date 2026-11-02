// =============================================================================
// HYDRA-UMC-PHYSICS-REPLICA - src/server.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0-or-later - see LICENSE
// =============================================================================
//! The only network surface of this program, and it is off unless asked for: `serve` starts a small HTTP listener (`tiny_http`, blocking, no async
//! runtime). `GET /health` (liveness, open) and `GET /metrics` (the numbers of the service in the text format of Prometheus, behind the token when
//! one is set) are always there. With `--urdf PATH` the computing routes of `service_api.rs` are there too (`POST /fk`, `/ik`, `/check-trajectory`,
//! `/inverse-dynamics`, `GET|POST /capsules`), behind the token when one is set; without a URDF they answer 404. The body of a request is bounded in
//! size, a path is never read from a request, and a URDF in a request is refused unless the configuration (`--config`) allows it.
//!
//! By default it listens on the loopback address only. A listener on another address is refused unless a token is set (`--token` or
//! `PHYSICS_REPLICA_TOKEN`) or the operator says on purpose that it may be open (`--allow-anonymous`).

use std::io::Write as _;

use serde_json::json;
use tiny_http::{Header, Method, Request, Response, Server};

use crate::messages::{Lang, Msg};
use crate::service_api::{self, Context, ServiceConfig};
use crate::service_metrics::{is_metrics_request, ServiceMetrics};

const NAME: &str = "HYDRA-UMC-PHYSICS-REPLICA";
const VERSION: &str = env!("CARGO_PKG_VERSION");
pub const DEFAULT_ADDR: &str = "127.0.0.1:8118";
pub const TOKEN_ENV: &str = "PHYSICS_REPLICA_TOKEN";

/// The numbers of this service for `GET /metrics` (see `service_metrics.rs`).
static METRICS: ServiceMetrics = ServiceMetrics::new("hydra-umc-physics-replica");

/// What `serve` was asked for.
#[derive(Debug, PartialEq)]
pub struct Options {
    pub addr: String,
    pub token: Option<String>,
    pub allow_anonymous: bool,
    /// The robot of the computing routes (None: they are off).
    pub urdf: Option<String>,
    /// The JSON file with the limits and tolerances of the service.
    pub config: Option<String>,
}

/// Reads the options after `serve`; `env_token` is the value of `PHYSICS_REPLICA_TOKEN`, used when `--token` is not given.
pub fn parse_options(args: &[String], env_token: Option<String>) -> Result<Options, String> {
    let mut options = Options {
        addr: DEFAULT_ADDR.to_string(),
        token: env_token.filter(|t| !t.is_empty()),
        allow_anonymous: false,
        urdf: None,
        config: None,
    };
    let mut iter = args.iter();
    while let Some(arg) = iter.next() {
        match arg.as_str() {
            "--addr" => {
                options.addr = iter.next().ok_or("--addr needs HOST:PORT")?.clone();
            }
            "--token" => {
                let token = iter.next().ok_or("--token needs a value")?;
                options.token = Some(token.clone()).filter(|t| !t.is_empty());
            }
            "--allow-anonymous" => options.allow_anonymous = true,
            "--urdf" => options.urdf = Some(iter.next().ok_or("--urdf needs a PATH")?.clone()),
            "--config" => {
                options.config = Some(iter.next().ok_or("--config needs a PATH")?.clone())
            }
            other => return Err(format!("unknown option '{other}'")),
        }
    }
    Ok(options)
}

/// True for the addresses that cannot be reached from another machine (`127.0.0.1:8118`, `[::1]:8118`, `localhost:8118`).
pub fn is_loopback_addr(addr: &str) -> bool {
    if let Ok(socket) = addr.parse::<std::net::SocketAddr>() {
        return socket.ip().is_loopback();
    }
    let host = match addr.rsplit_once(':') {
        Some((host, port)) if !host.contains(':') && port.chars().all(|c| c.is_ascii_digit()) => {
            host
        }
        _ => addr,
    };
    if host.eq_ignore_ascii_case("localhost") {
        return true;
    }
    host.trim_matches(|c| c == '[' || c == ']')
        .parse::<std::net::IpAddr>()
        .map(|ip| ip.is_loopback())
        .unwrap_or(false)
}

/// Why the listener must not start: the numbers of the service must not be open on the network by accident.
pub fn startup_problem(options: &Options) -> Option<String> {
    if is_loopback_addr(&options.addr) || options.token.is_some() || options.allow_anonymous {
        return None;
    }
    Some(format!(
        "refusing to listen on {} without a token: set --token (or {TOKEN_ENV}), bind a loopback address, or pass --allow-anonymous to run an open listener on purpose",
        options.addr
    ))
}

fn constant_time_eq(a: &[u8], b: &[u8]) -> bool {
    if a.len() != b.len() {
        return false;
    }
    a.iter().zip(b).fold(0u8, |acc, (x, y)| acc | (x ^ y)) == 0
}

fn authorized(request: &Request, token: &Option<String>) -> bool {
    let Some(expected) = token else {
        return true;
    };
    request
        .headers()
        .iter()
        .find(|h| h.field.equiv("Authorization"))
        .and_then(|h| h.value.as_str().strip_prefix("Bearer "))
        .is_some_and(|given| constant_time_eq(given.as_bytes(), expected.as_bytes()))
}

fn write_json(request: Request, status: u16, body: &serde_json::Value) {
    METRICS.count(status);
    let header = Header::from_bytes(&b"Content-Type"[..], &b"application/json"[..]).unwrap();
    let response = Response::from_string(body.to_string())
        .with_status_code(status)
        .with_header(header);
    let _ = request.respond(response);
}

pub fn bind(addr: &str) -> std::io::Result<Server> {
    Server::http(addr).map_err(std::io::Error::other)
}

fn header_value(request: &Request, name: &str) -> Option<String> {
    request
        .headers()
        .iter()
        .find(|h| h.field.as_str().as_str().eq_ignore_ascii_case(name))
        .map(|h| h.value.as_str().to_string())
}

/// Reads at most `limit` bytes of the body; None when the body is larger.
fn read_body(request: &mut Request, limit: usize) -> Option<Vec<u8>> {
    use std::io::Read as _;
    let mut body = Vec::new();
    request
        .as_reader()
        .take(limit as u64 + 1)
        .read_to_end(&mut body)
        .ok()?;
    (body.len() <= limit).then_some(body)
}

fn error_body(status: u16, msg: Msg, detail: Option<String>, lang: Lang) -> serde_json::Value {
    service_api::ApiError {
        status,
        msg,
        detail,
    }
    .body(lang)
}

/// Serves requests until the listener closes.
pub fn run(server: &Server, token: &Option<String>, ctx: &Context) {
    METRICS.start();
    for mut request in server.incoming_requests() {
        if is_metrics_request(&request) {
            let allowed = authorized(&request, token);
            METRICS.respond(request, VERSION, allowed, &[]);
            continue;
        }
        let url = request.url().to_string();
        let (path, query) = url.split_once('?').unwrap_or((url.as_str(), ""));
        let lang = service_api::request_language(
            &ctx.config,
            query,
            header_value(&request, "Accept-Language").as_deref(),
        );
        if request.method() == &Method::Get && path == "/health" {
            write_json(
                request,
                200,
                &json!({"status": "ok", "name": NAME, "version": VERSION}),
            );
        } else if service_api::is_compute_path(path) {
            if !authorized(&request, token) {
                write_json(
                    request,
                    401,
                    &error_body(401, Msg::Unauthorized, None, lang),
                );
                continue;
            }
            // a body larger than the limit is refused without being read to the end
            let declared = request.body_length().unwrap_or(0);
            let body = if declared > ctx.config.max_body_bytes {
                None
            } else {
                read_body(&mut request, ctx.config.max_body_bytes)
            };
            let method = request.method().as_str().to_string();
            let (status, answer) = match body {
                Some(body) => service_api::handle(&method, path, &body, ctx, lang),
                None => (
                    413,
                    error_body(
                        413,
                        Msg::PayloadTooLarge,
                        Some(format!("the limit is {} bytes", ctx.config.max_body_bytes)),
                        lang,
                    ),
                ),
            };
            write_json(request, status, &answer);
        } else {
            write_json(request, 404, &error_body(404, Msg::NotFound, None, lang));
        }
    }
}

/// Reads a small file of the operator (never a path from a request).
fn read_small_file(path: &str, limit: u64) -> Result<String, String> {
    let meta = std::fs::metadata(path).map_err(|e| format!("could not read '{path}': {e}"))?;
    if meta.len() > limit {
        return Err(format!("'{path}' is larger than {limit} bytes"));
    }
    std::fs::read_to_string(path).map_err(|e| format!("could not read '{path}': {e}"))
}

/// The configuration and the robot of the service, from the options.
pub fn build_context(options: &Options) -> Result<Context, String> {
    let config = match &options.config {
        Some(path) => {
            let text = read_small_file(path, 1024 * 1024)?;
            let raw: serde_json::Value = serde_json::from_str(&text)
                .map_err(|e| format!("'{path}' is not valid JSON: {e}"))?;
            service_api::parse_config(&raw).map_err(|e| format!("'{path}': {e}"))?
        }
        None => ServiceConfig::default(),
    };
    let model = match &options.urdf {
        Some(path) => {
            let xml = read_small_file(path, 16 * 1024 * 1024)?;
            Some(crate::urdf::parse_urdf_model(&xml).map_err(|e| format!("'{path}': {e}"))?)
        }
        None => None,
    };
    Ok(Context { config, model })
}

/// The `serve` command: returns the process exit code.
pub fn serve_command(args: &[String]) -> std::process::ExitCode {
    let options = match parse_options(args, std::env::var(TOKEN_ENV).ok()) {
        Ok(options) => options,
        Err(e) => {
            eprintln!("serve: {e}");
            eprintln!("Usage: serve [--addr HOST:PORT] [--token TOKEN] [--allow-anonymous] [--urdf PATH] [--config PATH]");
            return std::process::ExitCode::FAILURE;
        }
    };
    if let Some(problem) = startup_problem(&options) {
        eprintln!("serve: {problem}");
        return std::process::ExitCode::FAILURE;
    }
    let ctx = match build_context(&options) {
        Ok(ctx) => ctx,
        Err(e) => {
            eprintln!("serve: {e}");
            return std::process::ExitCode::FAILURE;
        }
    };
    let server = match bind(&options.addr) {
        Ok(server) => server,
        Err(e) => {
            eprintln!("serve: could not listen on {}: {e}", options.addr);
            return std::process::ExitCode::FAILURE;
        }
    };
    let routes = if ctx.model.is_some() || ctx.config.allow_inline_urdf {
        "GET /health, GET /metrics, POST /fk, /ik, /check-trajectory, /inverse-dynamics, GET|POST /capsules"
    } else {
        "GET /health, GET /metrics"
    };
    let _ = writeln!(
        std::io::stderr(),
        "{NAME} listening on {} ({routes})",
        options.addr
    );
    run(&server, &options.token, &ctx);
    std::process::ExitCode::SUCCESS
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::{Read, Write};
    use std::net::TcpStream;

    fn start(token: Option<&str>) -> u16 {
        start_with(
            token,
            Context {
                config: ServiceConfig::default(),
                model: None,
            },
        )
    }

    fn start_with(token: Option<&str>, ctx: Context) -> u16 {
        let server = bind("127.0.0.1:0").unwrap();
        let port = server.server_addr().to_ip().unwrap().port();
        let token = token.map(String::from);
        std::thread::spawn(move || run(&server, &token, &ctx));
        port
    }

    fn request(port: u16, method: &str, path: &str, headers: &str, body: &str) -> (u16, String) {
        let mut stream = TcpStream::connect(("127.0.0.1", port)).unwrap();
        write!(
            stream,
            "{method} {path} HTTP/1.1\r\nHost: localhost\r\n{headers}Content-Length: {}\r\nConnection: close\r\n\r\n{body}",
            body.len()
        )
        .unwrap();
        let mut raw = String::new();
        stream.read_to_string(&mut raw).unwrap();
        let status = raw.split_whitespace().nth(1).unwrap().parse().unwrap();
        (
            status,
            raw.split_once("\r\n\r\n")
                .map_or("", |(_, b)| b)
                .to_string(),
        )
    }

    fn get(port: u16, path: &str, bearer: Option<&str>) -> (u16, String) {
        let mut stream = TcpStream::connect(("127.0.0.1", port)).unwrap();
        let auth = bearer.map_or(String::new(), |t| format!("Authorization: Bearer {t}\r\n"));
        write!(
            stream,
            "GET {path} HTTP/1.1\r\nHost: localhost\r\n{auth}Connection: close\r\n\r\n"
        )
        .unwrap();
        let mut raw = String::new();
        stream.read_to_string(&mut raw).unwrap();
        let status = raw.split_whitespace().nth(1).unwrap().parse().unwrap();
        let body = raw
            .split_once("\r\n\r\n")
            .map_or("", |(_, b)| b)
            .to_string();
        (status, body)
    }

    fn args(list: &[&str]) -> Vec<String> {
        list.iter().map(|s| s.to_string()).collect()
    }

    #[test]
    fn the_listener_is_loopback_by_default_and_refuses_the_network_without_a_token() {
        let defaults = parse_options(&[], None).unwrap();
        assert_eq!(defaults.addr, DEFAULT_ADDR);
        assert!(startup_problem(&defaults).is_none());
        let open = parse_options(&args(&["--addr", "0.0.0.0:8118"]), None).unwrap();
        assert!(startup_problem(&open).is_some());
        let with_token =
            parse_options(&args(&["--addr", "0.0.0.0:8118", "--token", "t"]), None).unwrap();
        assert!(startup_problem(&with_token).is_none());
        let from_env = parse_options(&args(&["--addr", "0.0.0.0:8118"]), Some("t".into())).unwrap();
        assert!(startup_problem(&from_env).is_none());
        let anonymous = parse_options(
            &args(&["--addr", "0.0.0.0:8118", "--allow-anonymous"]),
            None,
        )
        .unwrap();
        assert!(startup_problem(&anonymous).is_none());
        assert!(parse_options(&args(&["--nope"]), None).is_err());
        assert!(parse_options(&args(&["--addr"]), None).is_err());
    }

    #[test]
    fn health_and_metrics_answer_and_everything_else_is_404() {
        let port = start(None);
        let (status, body) = get(port, "/health", None);
        assert_eq!(status, 200, "{body}");
        assert!(body.contains(NAME));
        assert_eq!(get(port, "/fk", None).0, 404);
        let (status, text) = get(port, "/metrics", None);
        assert_eq!(status, 200, "{text}");
        assert!(text
            .contains("hydra_umc_service_info{service=\"hydra-umc-physics-replica\",version=\""));
        assert!(text.contains("hydra_umc_service_health{service=\"hydra-umc-physics-replica\"} 1"));
        assert!(text.contains("class=\"4xx\"}"));
        assert!(!text.contains("/fk"));
    }

    #[test]
    fn the_metrics_ask_for_the_token_when_one_is_set_and_health_stays_open() {
        let port = start(Some("s3cret-token"));
        assert_eq!(get(port, "/health", None).0, 200);
        assert_eq!(get(port, "/metrics", None).0, 401);
        assert_eq!(get(port, "/metrics", Some("wrong-token")).0, 401);
        let (status, text) = get(port, "/metrics", Some("s3cret-token"));
        assert_eq!(status, 200);
        assert!(!text.contains("s3cret-token"));
    }

    const ARM: &str = r#"<robot name="a"><link name="base"/><link name="l1"/>
      <joint name="j" type="revolute"><parent link="base"/><child link="l1"/><origin xyz="0 0 0.5"/><axis xyz="0 0 1"/>
        <limit lower="-3" upper="3" effort="1" velocity="2"/></joint></robot>"#;

    fn arm_context() -> Context {
        Context {
            config: ServiceConfig::default(),
            model: Some(crate::urdf::parse_urdf_model(ARM).unwrap()),
        }
    }

    #[test]
    fn the_computing_routes_work_over_http_behind_the_token_and_in_the_language_asked() {
        let port = start_with(Some("tok"), arm_context());
        let body = r#"{"joints": {"j": 0.5}}"#;
        // no token: 401, in the language of the header
        let (status, text) = request(port, "POST", "/fk", "Accept-Language: de\r\n", body);
        assert_eq!(status, 401, "{text}");
        assert!(text.contains("Ein gültiges Token ist erforderlich"));
        let auth = "Authorization: Bearer tok\r\n";
        let (status, text) = request(port, "POST", "/fk", auth, body);
        assert_eq!(status, 200, "{text}");
        assert!(text.contains("\"joint\":\"j\""));
        // wrong method, unknown route, query language
        assert_eq!(request(port, "GET", "/fk", auth, "").0, 405);
        let (status, text) = request(port, "GET", "/nope?lang=ja", auth, "");
        assert_eq!(status, 404);
        assert!(text.contains("見つかりません"));
        // the trajectory route
        let traj = r#"{"samples": [{"t":0,"joints":{"j":0}},{"t":1,"joints":{"j":0.5}}]}"#;
        let (status, text) = request(port, "POST", "/check-trajectory?lang=es", auth, traj);
        assert_eq!(status, 200, "{text}");
        assert!(text.contains("\"ok\":true") && text.contains("la trayectoria cumple los límites"));
        // health stays open
        assert_eq!(request(port, "GET", "/health", "", "").0, 200);
    }

    #[test]
    fn a_body_over_the_limit_is_refused_over_http() {
        let mut ctx = arm_context();
        ctx.config.max_body_bytes = 1024;
        let port = start_with(None, ctx);
        let big = format!("{{\"joints\": {{}}, \"pad\": \"{}\"}}", "x".repeat(3000));
        assert_eq!(request(port, "POST", "/fk", "", &big).0, 413);
        assert_eq!(
            request(port, "POST", "/fk", "", r#"{"joints": {"j": 0.1}}"#).0,
            200
        );
    }

    #[test]
    fn serve_options_take_the_urdf_and_the_configuration() {
        let o = parse_options(&args(&["--urdf", "a.urdf", "--config", "c.json"]), None).unwrap();
        assert_eq!(o.urdf.as_deref(), Some("a.urdf"));
        assert_eq!(o.config.as_deref(), Some("c.json"));
        assert!(parse_options(&args(&["--urdf"]), None).is_err());
        let missing = parse_options(&args(&["--urdf", "does-not-exist.urdf"]), None).unwrap();
        assert!(build_context(&missing).is_err());
        assert!(build_context(&parse_options(&[], None).unwrap())
            .unwrap()
            .model
            .is_none());
    }
}
