// HYDRA-UMC-PHYSICS-REPLICA - tests/cli_simulate.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0-or-later - see LICENSE
//
// What a caller of the binary sees on `simulate`: the forward dynamics of the rigid-body module, run for a time under given torques.

use std::path::PathBuf;
use std::process::Command;

// A rod whose centre of mass is 0.25 m from a vertical axis: gravity is along the axis, so only the torque moves it.
// Moment of inertia about the axis: 0.01 + 1 * 0.25^2 = 0.0725 kg m^2.
const URDF: &str = r#"<robot name="turn">
  <link name="base"/>
  <link name="arm"><inertial><origin xyz="0.25 0 0"/><mass value="1"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial></link>
  <joint name="shoulder" type="revolute"><parent link="base"/><child link="arm"/>
    <origin xyz="0 0 0.3"/><axis xyz="0 0 1"/><limit lower="-30" upper="30" effort="20" velocity="20"/></joint>
</robot>"#;

fn write_file(name: &str, content: &str) -> PathBuf {
    static COUNTER: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
    let n = COUNTER.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
    let dir =
        std::env::temp_dir().join(format!("physics-replica-sim-{}-{}", std::process::id(), n));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join(name);
    std::fs::write(&path, content).unwrap();
    path
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

#[test]
fn a_constant_torque_gives_the_closed_form_motion() {
    let urdf = write_file("turn.urdf", URDF);
    let (code, out, err) = run(&[
        "simulate",
        "--urdf",
        urdf.to_str().unwrap(),
        "--torques",
        "shoulder=1",
        "--duration",
        "0.5",
        "--dt",
        "0.001",
        "--json",
    ]);
    assert_eq!(code, Some(0), "{err}");
    let v: serde_json::Value = serde_json::from_str(&out).unwrap();
    let samples = v["samples"].as_array().unwrap();
    let last = samples.last().unwrap();
    let alpha = 1.0 / 0.0725;
    let q = last["positions"][0].as_f64().unwrap();
    let qd = last["velocities"][0].as_f64().unwrap();
    assert!((q - 0.5 * alpha * 0.25).abs() < 1e-3, "position {q}");
    assert!((qd - alpha * 0.5).abs() < 1e-3, "speed {qd}");
    assert!((last["t_s"].as_f64().unwrap() - 0.5).abs() < 1e-9);
    assert_eq!(v["steps"], 500);
    assert!(err.contains("not tried with a real robot"));
}

#[test]
fn without_torque_the_robot_stays_where_it_is() {
    let urdf = write_file("turn.urdf", URDF);
    let (code, out, err) = run(&[
        "simulate",
        "--urdf",
        urdf.to_str().unwrap(),
        "--joints",
        "shoulder=0.3",
        "--duration",
        "0.2",
        "--json",
    ]);
    assert_eq!(code, Some(0), "{err}");
    let v: serde_json::Value = serde_json::from_str(&out).unwrap();
    let last = v["samples"].as_array().unwrap().last().unwrap().clone();
    assert!((last["positions"][0].as_f64().unwrap() - 0.3).abs() < 1e-9);
    assert!(last["velocities"][0].as_f64().unwrap().abs() < 1e-9);
}

#[test]
fn the_text_output_has_a_header_and_one_row_per_sample() {
    let urdf = write_file("turn.urdf", URDF);
    let (code, out, _) = run(&[
        "simulate",
        "--urdf",
        urdf.to_str().unwrap(),
        "--duration",
        "0.1",
        "--dt",
        "0.01",
        "--every",
        "5",
    ]);
    assert_eq!(code, Some(0));
    let lines: Vec<&str> = out.lines().collect();
    assert_eq!(lines[0], "t_s shoulder");
    assert_eq!(lines.len(), 1 + 1 + 2); // the start, the step 5 and the step 10
}

#[test]
fn bad_input_is_refused_with_exit_code_two() {
    let urdf = write_file("turn.urdf", URDF);
    let path = urdf.to_str().unwrap();
    for args in [
        vec!["simulate"],
        vec!["simulate", "--urdf", path, "--torques", "elbow=1"],
        vec!["simulate", "--urdf", path, "--dt", "0"],
        vec!["simulate", "--urdf", path, "--duration", "1", "--dt", "2"],
        vec!["simulate", "--urdf", path, "--duration", "nan"],
        vec!["simulate", "--urdf", path, "--every", "0"],
        vec![
            "simulate",
            "--urdf",
            path,
            "--duration",
            "100000",
            "--dt",
            "0.0000001",
        ],
    ] {
        let (code, _, err) = run(&args);
        assert_eq!(code, Some(2), "{args:?}: {err}");
    }
}
