// HYDRA-UMC-PHYSICS-REPLICA - tests/cli_refusals.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0-or-later - see LICENSE
//
// What a caller of the binary sees when it asks for something it cannot have: every command that needs a model says so and fails, a model that
// cannot be read or parsed is reported by name, and `serve` stops before it listens when the options are wrong. None of these starts a server.

use std::path::PathBuf;
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicUsize, Ordering};

static COUNTER: AtomicUsize = AtomicUsize::new(0);

fn run(args: &[&str]) -> (Option<i32>, String, String) {
    let output = Command::new(env!("CARGO_BIN_EXE_hydra-umc-physics-replica"))
        .args(args)
        .env_remove("PHYSICS_REPLICA_TOKEN")
        .stdin(Stdio::null())
        .output()
        .unwrap();
    (
        output.status.code(),
        String::from_utf8_lossy(&output.stdout).to_string(),
        String::from_utf8_lossy(&output.stderr).to_string(),
    )
}

fn file(name: &str, content: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!(
        "physics-replica-refusals-{}-{}",
        std::process::id(),
        COUNTER.fetch_add(1, Ordering::SeqCst)
    ));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join(name);
    std::fs::write(&path, content).unwrap();
    path
}

const COMMANDS: [&str; 14] = [
    "fk",
    "fk-checked",
    "validate-limits",
    "jacobian",
    "ik",
    "predict-collision",
    "self-collision",
    "capsules",
    "inverse-dynamics",
    "simulate",
    "mjcf",
    "mujoco-check",
    "mujoco-step",
    "check-trajectory",
];

#[test]
fn every_command_that_needs_a_model_says_so_and_fails_without_one() {
    for command in COMMANDS {
        let (code, _, stderr) = run(&[command]);
        assert!(
            matches!(code, Some(c) if c != 0),
            "{command}: a missing model is a failure, got {code:?}"
        );
        let without_mujoco = command.starts_with("mujoco") && !cfg!(feature = "mujoco");
        let expected = if without_mujoco {
            "no MuJoCo"
        } else {
            "missing required --urdf PATH"
        };
        assert!(stderr.contains(expected), "{command}: {stderr}");
    }
}

#[test]
fn a_model_that_does_not_exist_is_reported_and_fails() {
    for command in COMMANDS {
        let (code, _, stderr) = run(&[command, "--urdf", "no-such-robot-model.urdf"]);
        assert!(matches!(code, Some(c) if c != 0), "{command}: got {code:?}");
        assert!(!stderr.trim().is_empty(), "{command}: it says why");
    }
}

#[test]
fn a_model_that_is_not_a_robot_is_reported_and_fails() {
    let junk = file("junk.urdf", "this is not a robot description");
    let path = junk.to_str().unwrap();
    for command in COMMANDS {
        let (code, _, stderr) = run(&[command, "--urdf", path]);
        assert!(matches!(code, Some(c) if c != 0), "{command}: got {code:?}");
        assert!(!stderr.trim().is_empty(), "{command}: it says why");
    }
}

#[test]
fn a_command_that_needs_a_target_or_a_goal_says_which() {
    let model = file(
        "arm.urdf",
        r#"<robot name="arm"><link name="base"/><link name="tip"/>
  <joint name="j" type="revolute"><parent link="base"/><child link="tip"/>
    <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint></robot>"#,
    );
    let path = model.to_str().unwrap();
    let (code, _, stderr) = run(&["jacobian", "--urdf", path]);
    assert!(matches!(code, Some(c) if c != 0));
    assert!(
        stderr.contains("missing required --target JOINT"),
        "{stderr}"
    );
    let (code, _, stderr) = run(&["ik", "--urdf", path]);
    assert!(matches!(code, Some(c) if c != 0));
    assert!(
        stderr.contains("missing required --target JOINT"),
        "{stderr}"
    );
    let (code, _, stderr) = run(&["ik", "--urdf", path, "--target", "j"]);
    assert!(matches!(code, Some(c) if c != 0));
    assert!(stderr.contains("missing required --goal"), "{stderr}");
}

#[test]
fn serve_stops_before_it_listens_when_its_options_are_wrong() {
    let junk = file("junk.urdf", "not a robot");
    let not_json = file("config.json", "not json at all");
    let mut cases: Vec<(Vec<&str>, &str)> = vec![
        (vec!["serve", "--nope"], "Usage: serve"),
        (vec!["serve", "--addr"], "--addr needs HOST:PORT"),
        (vec!["serve", "--urdf"], "--urdf needs a PATH"),
        (vec!["serve", "--addr", "0.0.0.0:0"], "refusing to listen"),
        (
            vec!["serve", "--urdf", "no-such-robot-model.urdf"],
            "serve:",
        ),
    ];
    cases.push((vec!["serve", "--urdf", junk.to_str().unwrap()], "serve:"));
    cases.push((
        vec!["serve", "--config", not_json.to_str().unwrap()],
        "is not valid JSON",
    ));
    for (args, fragment) in cases {
        let (code, _, stderr) = run(&args);
        assert_eq!(code, Some(1), "{args:?}: {stderr}");
        assert!(stderr.contains(fragment), "{args:?}: {stderr}");
        assert!(!stderr.contains("listening on"), "{args:?}: {stderr}");
    }
}
