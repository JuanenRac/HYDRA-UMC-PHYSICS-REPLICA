// HYDRA-UMC-PHYSICS-REPLICA - tests/tree_contract.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0 - see LICENSE
//
// What a caller of the binary sees of a branching URDF: both fingers of a gripper placed from the palm, the follower of a
// <mimic> worked out and never chosen, and the soft, velocity and effort limits checked.

use std::path::PathBuf;
use std::process::Command;

// A palm that hangs from an arm, with two fingers; the right finger follows the left one mirrored (multiplier -1).
const GRIPPER: &str = r#"<robot name="gripper">
  <link name="base"/><link name="palm"/><link name="left"/><link name="right"/>
  <joint name="wrist" type="revolute"><parent link="base"/><child link="palm"/>
    <origin xyz="0 0 1"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="20" velocity="2"/>
    <safety_controller soft_lower_limit="-2.5" soft_upper_limit="2.5" k_position="10" k_velocity="1"/></joint>
  <joint name="finger_left" type="prismatic"><parent link="palm"/><child link="left"/>
    <origin xyz="0.1 0 0"/><axis xyz="1 0 0"/><limit lower="0" upper="0.04" effort="5" velocity="0.1"/></joint>
  <joint name="finger_right" type="prismatic"><parent link="palm"/><child link="right"/>
    <origin xyz="-0.1 0 0"/><axis xyz="1 0 0"/><limit lower="-0.04" upper="0" effort="5" velocity="0.1"/>
    <mimic joint="finger_left" multiplier="-1" offset="0"/></joint>
</robot>"#;

fn urdf_file() -> PathBuf {
    static FILE: std::sync::OnceLock<PathBuf> = std::sync::OnceLock::new();
    FILE.get_or_init(|| {
        let dir = std::env::temp_dir().join(format!("physics-replica-tree-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("gripper.urdf");
        std::fs::write(&path, GRIPPER).unwrap();
        path
    })
    .clone()
}

fn run(args: &[&str]) -> (Option<i32>, String, String) {
    let mut all: Vec<String> = vec![
        args[0].to_string(),
        "--urdf".into(),
        urdf_file().to_string_lossy().to_string(),
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
fn both_fingers_are_placed_from_the_palm_not_one_after_the_other() {
    let (code, stdout, stderr) = run(&["fk", "--joints", "wrist=0,finger_left=0.02"]);
    assert_eq!(code, Some(0), "{stderr}");
    // the palm is 1 m up; each finger hangs from it with its own offset, and the right one has been moved -0.02 by its mimic
    assert!(
        stdout.contains("wrist: x=0.000000 y=0.000000 z=1.000000"),
        "{stdout}"
    );
    assert!(
        stdout.contains("finger_left: x=0.120000 y=0.000000 z=1.000000"),
        "{stdout}"
    );
    assert!(
        stdout.contains("finger_right: x=-0.120000 y=0.000000 z=1.000000"),
        "{stdout}"
    );
}

#[test]
fn the_wrist_turns_both_fingers_with_the_palm() {
    let (code, stdout, stderr) = run(&["fk", "--joints", "wrist=1.5707963267948966,finger_left=0"]);
    assert_eq!(code, Some(0), "{stderr}");
    assert!(
        stdout.contains("finger_left: x=0.000000 y=0.100000 z=1.000000"),
        "{stdout}"
    );
    assert!(stdout.contains("y=-0.100000 z=1.000000"), "{stdout}");
}

#[test]
fn a_position_for_a_follower_is_refused_not_ignored() {
    let (code, _, stderr) = run(&[
        "fk",
        "--joints",
        "wrist=0,finger_left=0.01,finger_right=0.01",
    ]);
    assert_eq!(code, Some(2));
    assert!(stderr.contains("follows another joint"), "{stderr}");
}

#[test]
fn the_limits_of_a_follower_are_checked_with_the_position_its_master_gives_it() {
    // finger_left 0.03 is inside [0, 0.04] and finger_right is then -0.03, inside [-0.04, 0]
    let (code, stdout, stderr) = run(&["validate-limits", "--joints", "wrist=0,finger_left=0.03"]);
    assert_eq!(code, Some(0), "{stdout} {stderr}");
    assert!(stdout.contains("No limit violations"));
    // the master out of range is a violation of the master; here the follower is out of its range because of the multiplier
    let (code, stdout, _) = run(&["validate-limits", "--joints", "wrist=0,finger_left=0.05"]);
    assert_eq!(code, Some(1));
    assert!(
        stdout.contains("finger_left") && stdout.contains("finger_right"),
        "{stdout}"
    );
}

#[test]
fn fk_checked_refuses_a_pose_whose_follower_is_out_of_range() {
    let (code, stdout, _) = run(&["fk-checked", "--joints", "wrist=0,finger_left=0.05"]);
    assert_eq!(code, Some(1));
    assert!(stdout.contains("LIMIT VIOLATION"), "{stdout}");
}

#[test]
fn soft_limits_are_checked_only_when_asked_for() {
    let (code, stdout, _) = run(&["validate-limits", "--joints", "wrist=2.8,finger_left=0.01"]);
    assert_eq!(code, Some(0), "{stdout}");
    let (code, stdout, _) = run(&[
        "validate-limits",
        "--joints",
        "wrist=2.8,finger_left=0.01",
        "--soft",
    ]);
    assert_eq!(code, Some(1));
    assert!(
        stdout.contains("SOFT LIMIT VIOLATION: joint 'wrist'"),
        "{stdout}"
    );
    let (code, stdout, _) = run(&[
        "validate-limits",
        "--joints",
        "wrist=2.0,finger_left=0.01",
        "--soft",
    ]);
    assert_eq!(code, Some(0), "{stdout}");
}

#[test]
fn speeds_and_efforts_above_their_limits_are_violations_and_the_sign_does_not_matter() {
    let base = ["validate-limits", "--joints", "wrist=0,finger_left=0.01"];
    let with = |extra: &[&str]| {
        let mut args = base.to_vec();
        args.extend_from_slice(extra);
        run(&args)
    };
    let (code, stdout, _) = with(&["--velocities", "wrist=1.5,finger_left=0.05"]);
    assert_eq!(code, Some(0), "{stdout}");
    let (code, stdout, _) = with(&["--velocities", "wrist=-2.5"]);
    assert_eq!(code, Some(1));
    assert!(
        stdout.contains("VELOCITY LIMIT VIOLATION: joint 'wrist'"),
        "{stdout}"
    );
    let (code, stdout, _) = with(&["--efforts", "finger_left=6"]);
    assert_eq!(code, Some(1));
    assert!(
        stdout.contains("EFFORT LIMIT VIOLATION: joint 'finger_left'"),
        "{stdout}"
    );
}

#[test]
fn a_speed_for_a_joint_that_has_no_such_limit_or_does_not_exist_is_an_error() {
    let (code, _, stderr) = run(&[
        "validate-limits",
        "--joints",
        "wrist=0,finger_left=0",
        "--velocities",
        "elbow=1",
    ]);
    assert_eq!(code, Some(2));
    assert!(stderr.contains("unknown joint name"), "{stderr}");
}
