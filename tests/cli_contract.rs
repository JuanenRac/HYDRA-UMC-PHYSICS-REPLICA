// HYDRA-UMC-PHYSICS-REPLICA - tests/cli_contract.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0 - see LICENSE
//
// What a caller of the binary sees: a check that was not made is an error
// (exit 2), never "No limit violations.".

use std::path::PathBuf;
use std::process::Command;

const URDF: &str = r#"<robot name="t">
  <link name="base"/><link name="l1"/><link name="l2"/>
  <joint name="elbow" type="revolute"><parent link="base"/><child link="l1"/>
    <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
  <joint name="wrist" type="revolute"><parent link="l1"/><child link="l2"/>
    <origin xyz="1 0 0"/><axis xyz="0 1 0"/><limit lower="-2" upper="2" effort="1" velocity="1"/></joint>
</robot>"#;

fn urdf_file() -> PathBuf {
    let dir = std::env::temp_dir().join(format!("physics-replica-cli-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("robot.urdf");
    std::fs::write(&path, URDF).unwrap();
    path
}

fn run(args: &[&str]) -> (Option<i32>, String, String) {
    let urdf = urdf_file();
    let mut all: Vec<String> = vec![
        args[0].to_string(),
        "--urdf".into(),
        urdf.to_string_lossy().to_string(),
    ];
    all.extend(args[1..].iter().map(|a| a.to_string()));
    let output = Command::new(env!("CARGO_BIN_EXE_hydra-umc-physics-replica"))
        .args(&all)
        .output()
        .unwrap();
    (
        output.status.code(),
        String::from_utf8_lossy(&output.stdout).to_string(),
        String::from_utf8_lossy(&output.stderr).to_string(),
    )
}

#[test]
fn a_misspelt_joint_name_is_an_error_not_a_pass() {
    let (code, stdout, stderr) = run(&["validate-limits", "--joints", "elbw=5,wrist=0"]);
    assert_eq!(code, Some(2), "{stdout} {stderr}");
    assert!(stderr.contains("unknown joint name"), "{stderr}");
    assert!(!stdout.contains("No limit violations"));
}

#[test]
fn a_limited_joint_with_no_position_is_an_error_unless_partial_is_asked_for() {
    let (code, _, stderr) = run(&["validate-limits", "--joints", "elbow=0.5"]);
    assert_eq!(code, Some(2), "{stderr}");
    assert!(stderr.contains("wrist"), "{stderr}");
    let (code, stdout, _) = run(&[
        "validate-limits",
        "--joints",
        "elbow=0.5",
        "--allow-partial",
    ]);
    assert_eq!(code, Some(0));
    assert!(stdout.contains("No limit violations"));
    let (code, stdout, _) = run(&["validate-limits", "--joints", "elbow=5", "--allow-partial"]);
    assert_eq!(code, Some(1), "{stdout}");
}

#[test]
fn a_joint_given_twice_is_refused() {
    let (code, _, stderr) = run(&["fk", "--joints", "elbow=0.1,elbow=0.2,wrist=0"]);
    assert_eq!(code, Some(2));
    assert!(stderr.contains("more than once"), "{stderr}");
}

#[test]
fn fk_can_print_the_full_pose_as_json() {
    let (code, stdout, stderr) = run(&[
        "fk",
        "--joints",
        "elbow=1.5707963267948966,wrist=0",
        "--json",
    ]);
    assert_eq!(code, Some(0), "{stderr}");
    assert!(
        stdout.starts_with('[') && stdout.trim_end().ends_with(']'),
        "{stdout}"
    );
    // the wrist is 1 m along the elbow's x after a 90 degree turn about z: (0, 1, 0)
    assert!(stdout.contains("\"joint\":\"wrist\""), "{stdout}");
    assert!(
        stdout.contains("\"x\":0.000000000,\"y\":1.000000000"),
        "{stdout}"
    );
    assert!(stdout.contains("\"quaternion\""), "{stdout}");
    // the plain output is unchanged
    let (_, plain, _) = run(&["fk", "--joints", "elbow=0,wrist=0"]);
    assert!(
        plain.contains("wrist: x=1.000000 y=0.000000 z=0.000000"),
        "{plain}"
    );
}
