// =============================================================================
// HYDRA-UMC-PHYSICS-REPLICA - src/server_tests.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0-or-later - see LICENSE
// =============================================================================
//! The options of `serve`, the files it reads, every refusal of the configuration, and the command itself run in this process: the clean exits of a
//! bad command line, of an open listener without a token, of a file that is not there and of a port in use, and one real listener on a free port.

use super::*;

use std::io::Read;
use std::net::{TcpListener, TcpStream};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::Duration;

static COUNTER: AtomicUsize = AtomicUsize::new(0);

fn scratch(tag: &str) -> std::path::PathBuf {
    let n = COUNTER.fetch_add(1, Ordering::SeqCst);
    let dir = std::env::temp_dir().join(format!("replica-server-{}-{tag}-{n}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

fn args(list: &[&str]) -> Vec<String> {
    list.iter().map(|s| s.to_string()).collect()
}

fn same(left: std::process::ExitCode, right: std::process::ExitCode) -> bool {
    format!("{left:?}") == format!("{right:?}")
}

const ARM: &str = r#"<robot name="a"><link name="base"/><link name="l1"/>
  <joint name="j" type="revolute"><parent link="base"/><child link="l1"/><origin xyz="0 0 0.5"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="1" velocity="2"/></joint></robot>"#;

#[test]
fn each_option_that_needs_a_value_says_so_and_an_unknown_one_is_refused() {
    for (flag, fragment) in [
        ("--addr", "--addr needs HOST:PORT"),
        ("--token", "--token needs a value"),
        ("--urdf", "--urdf needs a PATH"),
        ("--config", "--config needs a PATH"),
    ] {
        assert_eq!(parse_options(&args(&[flag]), None).unwrap_err(), fragment);
    }
    assert_eq!(
        parse_options(&args(&["--bogus"]), None).unwrap_err(),
        "unknown option '--bogus'"
    );
    assert_eq!(
        parse_options(&args(&["--addr", "x:1", "extra"]), None).unwrap_err(),
        "unknown option 'extra'"
    );
}

#[test]
fn an_empty_token_is_no_token_and_the_flag_beats_the_environment() {
    assert_eq!(parse_options(&[], Some(String::new())).unwrap().token, None);
    assert_eq!(
        parse_options(&args(&["--token", ""]), Some("env".into()))
            .unwrap()
            .token,
        None
    );
    assert_eq!(
        parse_options(&args(&["--token", "flag"]), Some("env".into()))
            .unwrap()
            .token
            .as_deref(),
        Some("flag")
    );
    assert_eq!(
        parse_options(&[], Some("env".into()))
            .unwrap()
            .token
            .as_deref(),
        Some("env")
    );
    assert!(
        parse_options(&args(&["--allow-anonymous"]), None)
            .unwrap()
            .allow_anonymous
    );
    assert!(!parse_options(&[], None).unwrap().allow_anonymous);
}

#[test]
fn only_addresses_that_stay_on_this_machine_count_as_loopback() {
    for yes in [
        "127.0.0.1:8118",
        "127.0.0.1",
        "127.9.9.9:1",
        "[::1]:8118",
        "[::1]",
        "::1",
        "localhost",
        "localhost:80",
        "LOCALHOST:80",
        "LocalHost",
    ] {
        assert!(is_loopback_addr(yes), "{yes}");
    }
    for no in [
        "0.0.0.0:8118",
        "10.0.0.5:80",
        "192.168.0.1",
        "example.com:80",
        "[::]:8118",
        "",
        ":80",
        "garbage",
        "localhost.example.com:80",
        "127.0.0.1.evil.com:80",
    ] {
        assert!(!is_loopback_addr(no), "{no}");
    }
}

#[test]
fn an_open_listener_needs_a_token_or_the_operator_saying_so() {
    let mut options = parse_options(&args(&["--addr", "10.1.2.3:9"]), None).unwrap();
    let problem = startup_problem(&options).unwrap();
    assert!(
        problem.contains("10.1.2.3:9") && problem.contains("--allow-anonymous"),
        "{problem}"
    );
    options.token = Some("t".into());
    assert!(startup_problem(&options).is_none());
    options.token = None;
    options.allow_anonymous = true;
    assert!(startup_problem(&options).is_none());
}

#[test]
fn the_comparison_of_tokens_looks_at_every_byte() {
    assert!(constant_time_eq(b"", b""));
    assert!(constant_time_eq(b"secret", b"secret"));
    assert!(!constant_time_eq(b"secret", b"secreT"));
    assert!(!constant_time_eq(b"secret", b"secre"));
    assert!(!constant_time_eq(b"secret", b"secrets"));
    assert!(!constant_time_eq(b"", b"x"));
    // a sweep with a fixed seed: one byte changed anywhere is always found
    let mut state = 0x1234_5678u32;
    for _ in 0..64 {
        state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        let base: Vec<u8> = (0..16)
            .map(|i| (state >> (i % 24)) as u8 ^ i as u8)
            .collect();
        let at = (state >> 24) as usize % base.len();
        let mut other = base.clone();
        other[at] ^= 1 << (state % 8);
        assert!(constant_time_eq(&base, &base));
        assert!(!constant_time_eq(&base, &other));
    }
}

#[test]
fn a_file_of_the_operator_is_read_only_when_it_is_there_and_small_enough() {
    let dir = scratch("small");
    let file = dir.join("a.txt");
    std::fs::write(&file, "0123456789").unwrap();
    let path = file.to_string_lossy().to_string();
    assert_eq!(read_small_file(&path, 10).unwrap(), "0123456789");
    assert!(read_small_file(&path, 9)
        .unwrap_err()
        .contains("is larger than 9 bytes"));
    let missing = dir.join("none.txt").to_string_lossy().to_string();
    assert!(read_small_file(&missing, 10)
        .unwrap_err()
        .contains("could not read"));
    // a folder has a size too, and reading it as text fails with the same sentence
    assert!(read_small_file(&dir.to_string_lossy(), u64::MAX)
        .unwrap_err()
        .contains("could not read"));
    std::fs::write(&file, [0xff, 0xfe, 0x00]).unwrap();
    assert!(read_small_file(&path, 10)
        .unwrap_err()
        .contains("could not read"));
}

#[test]
fn the_context_names_the_file_that_is_wrong() {
    let dir = scratch("context");
    let write = |name: &str, text: &str| {
        let path = dir.join(name);
        std::fs::write(&path, text).unwrap();
        path.to_string_lossy().to_string()
    };
    let options = |urdf: Option<&str>, config: Option<&str>| Options {
        addr: DEFAULT_ADDR.into(),
        token: None,
        allow_anonymous: false,
        urdf: urdf.map(String::from),
        config: config.map(String::from),
    };
    let arm = write("arm.urdf", ARM);
    let good = write("good.json", r#"{"language":"es","max_samples":10}"#);
    let ctx = build_context(&options(Some(&arm), Some(&good))).unwrap();
    assert!(ctx.model.is_some());
    assert_eq!(ctx.config.language, Lang::Es);
    assert_eq!(ctx.config.max_samples, 10);

    let broken = write("broken.json", "{not json");
    let error = build_context(&options(None, Some(&broken))).err().unwrap();
    assert!(error.contains("is not valid JSON"), "{error}");
    let unknown = write("unknown.json", r#"{"no_such_field":1}"#);
    let error = build_context(&options(None, Some(&unknown))).err().unwrap();
    assert!(error.contains("unknown field"), "{error}");
    let bad_urdf = write("bad.urdf", "<robot><nothing/>");
    let error = build_context(&options(Some(&bad_urdf), None))
        .err()
        .unwrap();
    assert!(error.contains("bad.urdf"), "{error}");
    let missing = dir.join("none.json").to_string_lossy().to_string();
    assert!(build_context(&options(None, Some(&missing)))
        .err()
        .unwrap()
        .contains("could not read"));
    assert!(build_context(&options(Some(&missing), None))
        .err()
        .unwrap()
        .contains("could not read"));
}

#[test]
fn every_wrong_field_of_the_configuration_is_named() {
    for (text, fragment) in [
        ("[]", "must be a JSON object"),
        ("7", "must be a JSON object"),
        (r#"{"nope":1,"also":2}"#, "unknown field(s): also, nope"),
        (r#"{"language":"xx"}"#, "language must be one of"),
        (r#"{"language":7}"#, "language must be one of"),
        (r#"{"max_body_bytes":1023}"#, "max_body_bytes must be"),
        (r#"{"max_body_bytes":67108865}"#, "max_body_bytes must be"),
        (r#"{"max_body_bytes":"big"}"#, "max_body_bytes must be"),
        (r#"{"max_samples":0}"#, "max_samples must be"),
        (r#"{"max_samples":-1}"#, "max_samples must be"),
        (
            r#"{"allow_inline_urdf":"yes"}"#,
            "allow_inline_urdf must be true or false",
        ),
        (r#"{"metrics":true}"#, "metrics must be an object"),
        (
            r#"{"metrics":{"enabled":true,"extra":1}}"#,
            "unknown field: extra",
        ),
        (
            r#"{"metrics":{"enabled":"on"}}"#,
            "metrics.enabled must be true or false",
        ),
        (r#"{"ik":[]}"#, "ik must be an object"),
        (r#"{"ik":{"nope":1}}"#, "unknown field: nope"),
        (
            r#"{"ik":{"enabled":1}}"#,
            "ik.enabled must be true or false",
        ),
        (r#"{"ik":{"max_millis":0}}"#, "ik.max_millis must be"),
        (r#"{"ik":{"max_millis":60001}}"#, "ik.max_millis must be"),
        (
            r#"{"ik":{"max_iterations_cap":0}}"#,
            "ik.max_iterations_cap must be",
        ),
        (
            r#"{"ik":{"max_iterations_cap":100001}}"#,
            "ik.max_iterations_cap must be",
        ),
        (
            r#"{"ik":{"tolerance":0}}"#,
            "ik.tolerance must be a number above zero",
        ),
        (
            r#"{"ik":{"damping":-1}}"#,
            "ik.damping must be a number above zero",
        ),
        (
            r#"{"ik":{"max_step":"far"}}"#,
            "ik.max_step must be a number above zero",
        ),
    ] {
        let value: serde_json::Value = serde_json::from_str(text).unwrap();
        let error = service_api::parse_config(&value)
            .err()
            .unwrap_or_else(|| format!("accepted: {text}"));
        assert!(error.contains(fragment), "{text}: {error}");
    }
}

#[test]
fn the_configuration_that_is_right_is_taken_as_written() {
    let value: serde_json::Value = serde_json::from_str(
        r#"{"language":"pt","max_body_bytes":2048,"max_samples":5,"allow_inline_urdf":true,"metrics":{"enabled":true},
            "ik":{"enabled":false,"max_millis":10,"max_iterations_cap":7,"tolerance":0.5,"damping":0.25,"max_step":0.125}}"#,
    )
    .unwrap();
    let config = service_api::parse_config(&value).unwrap();
    assert_eq!(config.language, Lang::Pt);
    assert_eq!(config.max_body_bytes, 2048);
    assert_eq!(config.max_samples, 5);
    assert!(config.allow_inline_urdf && config.metrics && !config.ik.enabled);
    assert_eq!(config.ik.max_millis, 10);
    assert_eq!(config.ik.max_iterations_cap, 7);
    assert!(
        config.ik.defaults.max_iterations <= 7,
        "the default never goes above the cap"
    );
    assert_eq!(
        (
            config.ik.defaults.tolerance,
            config.ik.defaults.damping,
            config.ik.defaults.max_step
        ),
        (0.5, 0.25, 0.125)
    );
    assert!(service_api::parse_config(&serde_json::json!({})).is_ok());
}

#[test]
fn serve_exits_cleanly_when_it_cannot_start() {
    let dir = scratch("serve-fail");
    let failure = std::process::ExitCode::FAILURE;
    assert!(same(serve_command(&args(&["--bogus"])), failure));
    assert!(same(
        serve_command(&args(&["--addr", "10.255.255.254:9"])),
        std::process::ExitCode::FAILURE
    ));
    let missing = dir.join("none.urdf").to_string_lossy().to_string();
    assert!(same(
        serve_command(&args(&["--addr", "127.0.0.1:0", "--urdf", &missing])),
        std::process::ExitCode::FAILURE
    ));
    let taken = TcpListener::bind("127.0.0.1:0").unwrap();
    let busy = taken.local_addr().unwrap().to_string();
    assert!(same(
        serve_command(&args(&["--addr", &busy])),
        std::process::ExitCode::FAILURE
    ));
}

fn get(port: u16, path: &str, bearer: Option<&str>) -> (u16, String) {
    use std::io::Write;
    let mut stream = TcpStream::connect(("127.0.0.1", port)).unwrap();
    stream
        .set_read_timeout(Some(Duration::from_secs(5)))
        .unwrap();
    let auth = bearer.map_or(String::new(), |t| format!("Authorization: Bearer {t}\r\n"));
    write!(
        stream,
        "GET {path} HTTP/1.1\r\nHost: localhost\r\n{auth}Connection: close\r\n\r\n"
    )
    .unwrap();
    let mut raw = String::new();
    let _ = stream.read_to_string(&mut raw);
    let status = raw
        .split_whitespace()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);
    (
        status,
        raw.split_once("\r\n\r\n")
            .map_or("", |(_, b)| b)
            .to_string(),
    )
}

fn wait_for(port: u16) -> bool {
    for _ in 0..100 {
        if TcpStream::connect(("127.0.0.1", port)).is_ok() {
            return true;
        }
        std::thread::sleep(Duration::from_millis(50));
    }
    false
}

fn free_port() -> u16 {
    TcpListener::bind("127.0.0.1:0")
        .unwrap()
        .local_addr()
        .unwrap()
        .port()
}

#[test]
fn a_real_serve_with_a_robot_and_a_token_answers_health_and_guards_the_rest() {
    let dir = scratch("serve-live");
    let urdf = dir.join("arm.urdf");
    std::fs::write(&urdf, ARM).unwrap();
    let port = free_port();
    let list = args(&[
        "--addr",
        &format!("127.0.0.1:{port}"),
        "--token",
        "s3cret-token",
        "--urdf",
        &urdf.to_string_lossy(),
    ]);
    std::thread::spawn(move || {
        let _ = serve_command(&list);
    });
    assert!(wait_for(port), "the listener never opened");
    let (status, body) = get(port, "/health", None);
    assert_eq!(status, 200, "{body}");
    assert!(body.contains("HYDRA-UMC-PHYSICS-REPLICA"), "{body}");
    assert_eq!(
        get(port, "/fk", None).0,
        401,
        "a computing route asks for the token"
    );
    assert_eq!(get(port, "/fk", Some("wrong")).0, 401);
    assert_eq!(get(port, "/nowhere", None).0, 404);
    assert_eq!(get(port, "/nowhere?lang=es", None).0, 404);
}

#[test]
fn a_real_serve_without_a_robot_has_only_the_open_routes() {
    let port = free_port();
    let list = args(&["--addr", &format!("127.0.0.1:{port}")]);
    std::thread::spawn(move || {
        let _ = serve_command(&list);
    });
    assert!(wait_for(port), "the listener never opened");
    assert_eq!(get(port, "/health", None).0, 200);
    // the numbers of the service are off unless the configuration opens them
    assert_eq!(get(port, "/metrics", None).0, 404);
}
