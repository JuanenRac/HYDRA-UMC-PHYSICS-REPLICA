// HYDRA-UMC-PHYSICS-REPLICA - tests/cli_predict_collision.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0-or-later - see LICENSE
//
// What a caller of the binary sees from `predict-collision`: the exit status (0 clear, 1 contact predicted, 2 bad input), the JSON
// answer, and the refusals. The planar arm is the one of the Jacobian tests; its tool is at (2, 0, 0) at zero.
//
// First, the commands the new one is built on are fixed as they are today: the pose of the frame of every joint of that arm.

use std::path::PathBuf;
use std::process::Command;

const URDF: &str = r#"<robot name="planar">
  <link name="base"/><link name="l1"/><link name="l2"/><link name="tool"/>
  <joint name="shoulder" type="revolute"><parent link="base"/><child link="l1"/>
    <axis xyz="0 0 1"/><limit lower="-3.1" upper="3.1" effort="10" velocity="2"/></joint>
  <joint name="elbow" type="revolute"><parent link="l1"/><child link="l2"/>
    <origin xyz="1 0 0"/><axis xyz="0 0 1"/><limit lower="-3.1" upper="3.1" effort="10" velocity="2"/></joint>
  <joint name="tip" type="fixed"><parent link="l2"/><child link="tool"/>
    <origin xyz="1 0 0"/></joint>
</robot>"#;

fn dir() -> PathBuf {
    let dir =
        std::env::temp_dir().join(format!("physics-replica-collision-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

fn file(name: &str, content: &str) -> String {
    let path = dir().join(format!("{:?}-{name}", std::thread::current().id()));
    std::fs::write(&path, content).unwrap();
    path.to_string_lossy().to_string()
}

fn run(args: &[&str]) -> (Option<i32>, String, String) {
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

const CONFIG: &str = r#"{"link_radius_m": 0.05, "tool_extent_m": 0.05}"#;
const WALL_AT_TOOL: &str = r#"{"zones":[{"id":"press","level":"danger","min":{"x":1.9,"y":-0.2,"z":-0.2},"max":{"x":2.2,"y":0.2,"z":0.2}}]}"#;
const WALL_FAR: &str = r#"{"zones":[{"id":"far","level":"warning","min":{"x":8,"y":8,"z":8},"max":{"x":9,"y":9,"z":9}}]}"#;

fn predict(zones: &str, trajectory: &str, config: &str) -> (Option<i32>, String, String) {
    let urdf = file("arm.urdf", URDF);
    let zones = file("zones.json", zones);
    let trajectory = file("trajectory.json", trajectory);
    let config = file("config.json", config);
    run(&[
        "predict-collision",
        "--urdf",
        &urdf,
        "--zones",
        &zones,
        "--trajectory",
        &trajectory,
        "--config",
        &config,
        "--json",
    ])
}

#[test]
fn characterization_the_frames_of_the_planar_arm_that_the_prediction_is_built_on() {
    let urdf = file("fk.urdf", URDF);
    let (code, out, _) = run(&[
        "fk",
        "--urdf",
        &urdf,
        "--joints",
        "shoulder=0,elbow=0",
        "--json",
    ]);
    assert_eq!(code, Some(0));
    let poses: serde_json::Value = serde_json::from_str(&out).unwrap();
    let x = |i: usize| poses[i]["position"]["x"].as_f64().unwrap();
    assert!((x(0) - 0.0).abs() < 1e-9 && (x(1) - 1.0).abs() < 1e-9 && (x(2) - 2.0).abs() < 1e-9);
    let (_, out, _) = run(&[
        "fk",
        "--urdf",
        &urdf,
        "--joints",
        "shoulder=1.5707963267948966,elbow=0",
        "--json",
    ]);
    let poses: serde_json::Value = serde_json::from_str(&out).unwrap();
    assert!((poses[2]["position"]["y"].as_f64().unwrap() - 2.0).abs() < 1e-9);
}

#[test]
fn a_tool_inside_a_danger_zone_exits_1_and_names_the_zone() {
    let (code, out, _) = predict(
        WALL_AT_TOOL,
        r#"{"states":[{"shoulder":0,"elbow":0}]}"#,
        CONFIG,
    );
    assert_eq!(code, Some(1), "{out}");
    let answer: serde_json::Value = serde_json::from_str(&out).unwrap();
    assert_eq!(answer["collision"], true);
    assert_eq!(answer["level"], "danger");
    assert_eq!(answer["hits"][0]["zone"], "press");
}

#[test]
fn a_far_zone_exits_0_and_reports_a_positive_clearance() {
    let (code, out, _) = predict(
        WALL_FAR,
        r#"{"states":[{"shoulder":0,"elbow":0},{"shoulder":0.5,"elbow":0.5}]}"#,
        CONFIG,
    );
    assert_eq!(code, Some(0), "{out}");
    let answer: serde_json::Value = serde_json::from_str(&out).unwrap();
    assert_eq!(answer["collision"], false);
    assert_eq!(answer["level"], "clear");
    assert!(answer["min_clearance_m"].as_f64().unwrap() > 1.0);
    assert!(answer["samples"].as_u64().unwrap() >= 2);
}

#[test]
fn a_contact_between_two_states_is_found_by_the_trajectory_and_not_by_its_ends() {
    let zones = r#"{"zones":[{"id":"between","level":"warning","min":{"x":1.2,"y":1.2,"z":-0.1},"max":{"x":1.6,"y":1.6,"z":0.1}}]}"#;
    let (code, _, _) = predict(
        zones,
        r#"{"states":[{"shoulder":0,"elbow":0},{"shoulder":1.5707963,"elbow":0}]}"#,
        CONFIG,
    );
    assert_eq!(code, Some(1));
    let (code, _, _) = predict(zones, r#"{"states":[{"shoulder":0,"elbow":0}]}"#, CONFIG);
    assert_eq!(code, Some(0));
}

#[test]
fn the_text_answer_says_what_it_is() {
    let urdf = file("t.urdf", URDF);
    let zones = file("t-zones.json", WALL_AT_TOOL);
    let trajectory = file("t-traj.json", r#"{"states":[{"shoulder":0,"elbow":0}]}"#);
    let config = file("t-config.json", CONFIG);
    let (code, out, err) = run(&[
        "predict-collision",
        "--urdf",
        &urdf,
        "--zones",
        &zones,
        "--trajectory",
        &trajectory,
        "--config",
        &config,
    ]);
    assert_eq!(code, Some(1));
    assert!(out.starts_with("COLLISION PREDICTED"), "{out}");
    assert!(out.contains("press"), "{out}");
    assert!(err.contains("kinematic estimate"), "{err}");
}

#[test]
fn bad_input_is_exit_2_with_a_reason_and_never_an_answer() {
    for (zones, trajectory, config, reason) in [
        (WALL_FAR, r#"{"states":[{"shoulder":0}]}"#, CONFIG, "elbow"),
        (
            WALL_FAR,
            r#"{"states":[{"shoulder":0,"elbow":0,"wrist":1}]}"#,
            CONFIG,
            "unknown joint",
        ),
        (WALL_FAR, r#"{"states":[]}"#, CONFIG, "no states"),
        (
            WALL_FAR,
            r#"{"states":[{"shoulder":0,"elbow":0}]}"#,
            r#"{"link_radius_m":0.05}"#,
            "tool_extent_m",
        ),
        (
            WALL_FAR,
            r#"{"states":[{"shoulder":0,"elbow":0}]}"#,
            r#"{"tool_extent_m":0.05}"#,
            "link_radius_m",
        ),
        (
            "{\"zones\":[{\"id\":\"a\",\"level\":\"danger\",\"shape\":\"capsule\"}]}",
            r#"{"states":[{"shoulder":0,"elbow":0}]}"#,
            CONFIG,
            "shapes",
        ),
        (
            "not json",
            r#"{"states":[{"shoulder":0,"elbow":0}]}"#,
            CONFIG,
            "not valid JSON",
        ),
    ] {
        let (code, out, err) = predict(zones, trajectory, config);
        assert_eq!(code, Some(2), "{reason}: {out} {err}");
        assert!(out.is_empty(), "{reason}: an answer was printed: {out}");
        assert!(err.contains(reason), "{reason}: {err}");
    }
}

#[test]
fn every_required_flag_is_named_when_it_is_missing() {
    let urdf = file("m.urdf", URDF);
    for (args, flag) in [
        (vec!["predict-collision"], "--urdf"),
        (
            vec!["predict-collision", "--urdf", urdf.as_str()],
            "--zones",
        ),
    ] {
        let (code, _, err) = run(&args);
        assert_eq!(code, Some(2));
        assert!(err.contains(flag), "{err}");
    }
}

#[test]
fn the_zones_file_may_use_the_shapes_of_the_zones_service_when_it_switches_them_on() {
    let zones = r#"{"geometry":{"shapes":true},"zones":[{"id":"post","level":"danger","shape":"cylinder","base_center":{"x":2,"y":0,"z":-1},"radius":0.3,"height":2}]}"#;
    let (code, out, _) = predict(zones, r#"{"states":[{"shoulder":0,"elbow":0}]}"#, CONFIG);
    assert_eq!(code, Some(1), "{out}");
}

#[test]
fn a_base_state_gives_the_joints_a_state_does_not_mention() {
    let urdf = file("b.urdf", URDF);
    let zones = file("b-zones.json", WALL_AT_TOOL);
    let config = file("b-config.json", CONFIG);
    // the trajectory names no joint at all: both come from the base, which also holds a joint this robot does not have
    let trajectory = file("b-traj.json", r#"{"states":[{}]}"#);
    let base = file(
        "b-base.json",
        r#"{"shoulder":0,"elbow":0,"other_robot_axis":3}"#,
    );
    let (code, out, err) = run(&[
        "predict-collision",
        "--urdf",
        &urdf,
        "--zones",
        &zones,
        "--trajectory",
        &trajectory,
        "--config",
        &config,
        "--base",
        &base,
        "--json",
    ]);
    assert_eq!(code, Some(1), "{out} {err}");
    // without the base the same trajectory is an error, never zeros
    let (code, out, err) = run(&[
        "predict-collision",
        "--urdf",
        &urdf,
        "--zones",
        &zones,
        "--trajectory",
        &trajectory,
        "--config",
        &config,
        "--json",
    ]);
    assert_eq!(code, Some(2), "{out}");
    assert!(err.contains("shoulder"), "{err}");
}
