// HYDRA-UMC-PHYSICS-REPLICA - tests/cli_jacobian_ik.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0-or-later - see LICENSE
//
// What a caller of the binary sees on the commands the other CLI tests do
// not reach: the identity banner, the required flags of every command, the
// Jacobian of a planar two-link arm, the numeric options of `ik`, and the
// checked forward kinematics.

use std::path::PathBuf;
use std::process::Command;

// Planar arm: two revolute joints about z, one metre apart, and a fixed tip one metre further.
const URDF: &str = r#"<robot name="planar">
  <link name="base"/><link name="l1"/><link name="l2"/><link name="tool"/>
  <joint name="shoulder" type="revolute"><parent link="base"/><child link="l1"/>
    <axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="10" velocity="2"/></joint>
  <joint name="elbow" type="revolute"><parent link="l1"/><child link="l2"/>
    <origin xyz="1 0 0"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="10" velocity="2"/></joint>
  <joint name="tip" type="fixed"><parent link="l2"/><child link="tool"/>
    <origin xyz="1 0 0"/></joint>
</robot>"#;

const NO_LIMITS: &str = r#"<robot name="free">
  <link name="base"/><link name="l1"/>
  <joint name="spin" type="revolute"><parent link="base"/><child link="l1"/>
    <axis xyz="0 0 1"/></joint>
</robot>"#;

fn write_once(name: &str, content: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("physics-replica-jik-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join(name);
    if !path.exists() {
        let tmp = dir.join(format!("{name}.{:?}.tmp", std::thread::current().id()));
        std::fs::write(&tmp, content).unwrap();
        let _ = std::fs::rename(&tmp, &path);
        let _ = std::fs::remove_file(&tmp);
    }
    path
}

fn run_raw(args: &[String]) -> (Option<i32>, String, String) {
    let output = Command::new(env!("CARGO_BIN_EXE_hydra-umc-physics-replica"))
        .args(args)
        .output()
        .unwrap();
    (
        output.status.code(),
        String::from_utf8_lossy(&output.stdout).to_string(),
        String::from_utf8_lossy(&output.stderr).to_string(),
    )
}

fn run(args: &[&str]) -> (Option<i32>, String, String) {
    let urdf = write_once("planar.urdf", URDF);
    let mut all: Vec<String> = vec![
        args[0].to_string(),
        "--urdf".into(),
        urdf.to_string_lossy().to_string(),
    ];
    all.extend(args[1..].iter().map(|a| a.to_string()));
    run_raw(&all)
}

fn number_after(text: &str, key: &str) -> f64 {
    let start = text
        .find(key)
        .unwrap_or_else(|| panic!("{key} not in {text}"))
        + key.len();
    let rest = &text[start..];
    let end = rest
        .find(|c: char| !(c.is_ascii_digit() || c == '.' || c == '-'))
        .unwrap_or(rest.len());
    rest[..end].parse().unwrap()
}

#[test]
fn no_arguments_or_an_unknown_command_print_the_identity_and_succeed() {
    for args in [vec![], vec!["nonsense".to_string()]] {
        let (code, stdout, _) = run_raw(&args);
        assert_eq!(code, Some(0));
        assert!(
            stdout.starts_with("HYDRA-UMC-PHYSICS-REPLICA v"),
            "{stdout}"
        );
        assert!(
            stdout.contains("no dynamics, no physics engine"),
            "{stdout}"
        );
    }
}

#[test]
fn every_command_requires_its_urdf_and_a_missing_file_is_an_error() {
    for command in ["fk", "fk-checked", "validate-limits", "jacobian", "ik"] {
        let (code, _, stderr) = run_raw(&[command.to_string()]);
        assert_eq!(code, Some(2), "{command}: {stderr}");
        assert!(
            stderr.contains("missing required --urdf"),
            "{command}: {stderr}"
        );

        let (code, _, stderr) = run_raw(&[
            command.to_string(),
            "--urdf".into(),
            "no-such-robot.urdf".into(),
            "--target".into(),
            "tip".into(),
            "--goal".into(),
            "1,0,0".into(),
        ]);
        assert_eq!(code, Some(2), "{command}: {stderr}");
        assert!(stderr.contains("could not read"), "{command}: {stderr}");
    }
}

#[test]
fn fk_at_zero_and_at_a_right_angle_gives_the_planar_geometry() {
    let (code, stdout, stderr) = run(&["fk", "--joints", "shoulder=0,elbow=0"]);
    assert_eq!(code, Some(0), "{stderr}");
    assert!(stderr.contains("kinematic estimate"), "{stderr}");
    let tip_line = stdout.lines().find(|l| l.starts_with("tip:")).unwrap();
    assert!(
        (number_after(tip_line, "x=") - 2.0).abs() < 1e-6,
        "{tip_line}"
    );
    assert!(number_after(tip_line, "y=").abs() < 1e-6, "{tip_line}");

    let (code, stdout, _) = run(&["fk", "--joints", "shoulder=1.5707963267948966,elbow=0"]);
    assert_eq!(code, Some(0));
    let tip_line = stdout.lines().find(|l| l.starts_with("tip:")).unwrap();
    assert!(number_after(tip_line, "x=").abs() < 1e-6, "{tip_line}");
    assert!(
        (number_after(tip_line, "y=") - 2.0).abs() < 1e-6,
        "{tip_line}"
    );
}

#[test]
fn fk_rejects_a_malformed_joint_list() {
    for (spec, fragment) in [
        ("shoulder", "expected 'name=value'"),
        ("shoulder=abc", "is not a valid number"),
        ("shoulder=1,shoulder=2", "more than once"),
    ] {
        let (code, _, stderr) = run(&["fk", "--joints", spec]);
        assert_eq!(code, Some(2), "{spec}: {stderr}");
        assert!(
            stderr.contains("invalid --joints value"),
            "{spec}: {stderr}"
        );
        assert!(stderr.contains(fragment), "{spec}: {stderr}");
    }
}

#[test]
fn fk_checked_accepts_in_range_poses_and_names_the_violation_outside() {
    let (code, stdout, _) = run(&["fk-checked", "--joints", "shoulder=0.5,elbow=0.5"]);
    assert_eq!(code, Some(0), "{stdout}");
    assert!(stdout.contains("tip:"), "{stdout}");

    let (code, stdout, _) = run(&["fk-checked", "--joints", "shoulder=4,elbow=0"]);
    assert_eq!(code, Some(1), "{stdout}");
    assert!(
        stdout.contains("LIMIT VIOLATION: joint 'shoulder'"),
        "{stdout}"
    );
    assert!(
        stdout.contains("refusing to compute an unreachable pose"),
        "{stdout}"
    );
}

#[test]
fn fk_checked_and_validate_limits_refuse_a_joint_that_has_no_limit() {
    let urdf = write_once("free.urdf", NO_LIMITS);
    let path = urdf.to_string_lossy().to_string();
    let (code, _, stderr) = run_raw(&[
        "fk-checked".into(),
        "--urdf".into(),
        path.clone(),
        "--joints".into(),
        "spin=1".into(),
    ]);
    assert_eq!(code, Some(2), "{stderr}");
    assert!(stderr.contains("have no <limit>"), "{stderr}");

    let (code, _, stderr) = run_raw(&[
        "validate-limits".into(),
        "--urdf".into(),
        path,
        "--joints".into(),
        "spin=1".into(),
    ]);
    assert_eq!(code, Some(2), "{stderr}");
    assert!(stderr.contains("have no <limit>"), "{stderr}");
}

#[test]
fn the_jacobian_of_the_planar_arm_has_the_expected_columns() {
    let (code, stdout, stderr) = run(&[
        "jacobian",
        "--target",
        "tip",
        "--joints",
        "shoulder=0,elbow=0",
    ]);
    assert_eq!(code, Some(0), "{stdout} {stderr}");
    assert!(stdout.starts_with("columns: shoulder elbow"), "{stdout}");
    let vy = stdout.lines().find(|l| l.starts_with("vy:")).unwrap();
    let cells: Vec<f64> = vy[3..]
        .split_whitespace()
        .map(|v| v.parse().unwrap())
        .collect();
    // a turn of the shoulder swings a tip 2 m away; a turn of the elbow swings it 1 m
    assert!((cells[0] - 2.0).abs() < 1e-6, "{vy}");
    assert!((cells[1] - 1.0).abs() < 1e-6, "{vy}");
    let wz = stdout.lines().find(|l| l.starts_with("wz:")).unwrap();
    assert!(wz.contains("1.000000 1.000000"), "{wz}");
}

#[test]
fn the_jacobian_as_json_has_target_joints_and_six_rows() {
    let (code, stdout, _) = run(&[
        "jacobian",
        "--target",
        "tip",
        "--json",
        "--joints",
        "shoulder=0,elbow=0",
    ]);
    assert_eq!(code, Some(0), "{stdout}");
    let parsed: serde_like::Json = serde_like::parse(&stdout);
    assert!(parsed.target_is("tip"));
    assert_eq!(parsed.row_count, 6);
}

#[test]
fn the_jacobian_needs_a_target_that_exists() {
    let (code, _, stderr) = run(&["jacobian"]);
    assert_eq!(code, Some(2));
    assert!(stderr.contains("missing required --target"), "{stderr}");

    let (code, _, stderr) = run(&["jacobian", "--target", "no_such_joint"]);
    assert_eq!(code, Some(2));
    assert!(stderr.starts_with("jacobian:"), "{stderr}");
}

#[test]
fn ik_reaches_a_goal_inside_the_workspace_and_prints_sorted_positions() {
    let (code, stdout, stderr) = run(&[
        "ik",
        "--target",
        "tip",
        "--goal",
        "1,1,0",
        "--joints",
        "shoulder=0.3,elbow=0.3",
    ]);
    assert_eq!(code, Some(0), "{stdout} {stderr}");
    let names: Vec<&str> = stdout
        .lines()
        .map(|l| l.split('=').next().unwrap())
        .collect();
    assert_eq!(
        names,
        vec!["elbow", "shoulder"],
        "positions are printed in name order"
    );
    assert!(stderr.contains("ik: reached within"), "{stderr}");
}

#[test]
fn ik_checks_every_numeric_option_and_the_goal_shape() {
    for (extra, fragment) in [
        (vec!["--goal", "1,1"], "three finite numbers"),
        (vec!["--goal", "1,1,x"], "three finite numbers"),
        (
            vec!["--goal", "1,1,0", "--tolerance", "0"],
            "--tolerance wants a positive number",
        ),
        (
            vec!["--goal", "1,1,0", "--damping", "-1"],
            "--damping wants a positive number",
        ),
        (
            vec!["--goal", "1,1,0", "--max-step", "abc"],
            "--max-step wants a positive number",
        ),
        (
            vec!["--goal", "1,1,0", "--max-iterations", "0"],
            "--max-iterations wants a positive whole number",
        ),
    ] {
        let mut args = vec!["ik", "--target", "tip"];
        args.extend(extra.iter());
        let (code, _, stderr) = run(&args);
        assert_eq!(code, Some(2), "{extra:?}: {stderr}");
        assert!(stderr.contains(fragment), "{extra:?}: {stderr}");
    }
    let (code, _, stderr) = run(&["ik", "--goal", "1,1,0"]);
    assert_eq!(code, Some(2));
    assert!(stderr.contains("missing required --target"), "{stderr}");
    let (code, _, stderr) = run(&["ik", "--target", "tip"]);
    assert_eq!(code, Some(2));
    assert!(stderr.contains("missing required --goal"), "{stderr}");
}

#[test]
fn ik_fails_with_exit_one_for_a_goal_out_of_reach() {
    let (code, stdout, stderr) = run(&["ik", "--target", "tip", "--goal", "10,0,0"]);
    assert_eq!(code, Some(1), "{stdout} {stderr}");
    assert!(stderr.starts_with("ik:"), "{stderr}");
}

// A tiny, dependency-free reader for the one JSON shape `jacobian --json` prints.
mod serde_like {
    pub struct Json {
        text: String,
        pub row_count: usize,
    }

    impl Json {
        pub fn target_is(&self, name: &str) -> bool {
            self.text.contains(&format!("\"target\":\"{name}\""))
        }
    }

    pub fn parse(text: &str) -> Json {
        let rows = text.split("\"rows\":[").nth(1).expect("rows key");
        let row_count = rows.matches('[').count();
        Json {
            text: text.trim().to_string(),
            row_count,
        }
    }
}
