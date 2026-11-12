// HYDRA-UMC-PHYSICS-REPLICA - tests/cli_simulate_pd.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0-or-later - see LICENSE
//
// What a caller of the binary sees on `simulate` with a motor law: `--targets` with `--kp`, `--kd` and `--max-torque` close the loop (a saturated
// proportional-derivative law on each joint that has a target); without `--targets` the command is the open-loop one it always was.

use std::path::PathBuf;
use std::process::Command;

// A rod on a vertical axis (gravity is along the axis, so only the torque moves it); the moment of inertia about the axis is 0.01 + 0.25^2 = 0.0725.
const URDF: &str = r#"<robot name="turn">
  <link name="base"/>
  <link name="arm"><inertial><origin xyz="0.25 0 0"/><mass value="1"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial></link>
  <joint name="shoulder" type="revolute"><parent link="base"/><child link="arm"/>
    <origin xyz="0 0 0.3"/><axis xyz="0 0 1"/><limit lower="-30" upper="30" effort="20" velocity="20"/></joint>
</robot>"#;
const INERTIA: f64 = 0.0725;

fn write_file(name: &str, content: &str) -> PathBuf {
    static COUNTER: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
    let n = COUNTER.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
    let dir = std::env::temp_dir().join(format!("physics-replica-pd-{}-{}", std::process::id(), n));
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

fn last_sample(extra: &[&str], duration: &str) -> (f64, f64, String) {
    let urdf = write_file("turn.urdf", URDF);
    let mut args = vec![
        "simulate",
        "--urdf",
        urdf.to_str().unwrap(),
        "--duration",
        duration,
        "--dt",
        "0.001",
        "--json",
    ];
    args.extend_from_slice(extra);
    let (code, out, err) = run(&args);
    assert_eq!(code, Some(0), "{err}");
    let v: serde_json::Value = serde_json::from_str(&out).unwrap();
    let last = v["samples"].as_array().unwrap().last().unwrap().clone();
    (
        last["positions"][0].as_f64().unwrap(),
        last["velocities"][0].as_f64().unwrap(),
        err,
    )
}

#[test]
fn a_motor_with_damping_settles_on_its_target() {
    // critical damping: kd = 2 sqrt(kp * inertia)
    let kd = 2.0 * (20.0_f64 * INERTIA).sqrt();
    let kd_text = format!("{kd}");
    let (q, qd, err) = last_sample(
        &[
            "--targets",
            "shoulder=0.5",
            "--kp",
            "20",
            "--kd",
            &kd_text,
            "--max-torque",
            "100",
        ],
        "3",
    );
    assert!((q - 0.5).abs() < 1e-3, "position {q}");
    assert!(qd.abs() < 1e-3, "speed {qd}");
    assert!(err.contains("proportional-derivative"));
    assert!(err.contains("not tried with a real robot"));
}

#[test]
fn the_motor_cannot_give_more_than_its_top_torque() {
    // far from the target the torque is the top one all along: constant acceleration, as in the open-loop closed form
    let (q, qd, _) = last_sample(
        &[
            "--targets",
            "shoulder=20",
            "--kp",
            "1000",
            "--max-torque",
            "1",
        ],
        "0.5",
    );
    let alpha = 1.0 / INERTIA;
    assert!((qd - alpha * 0.5).abs() < 1e-3, "speed {qd}");
    assert!((q - 0.5 * alpha * 0.25).abs() < 1e-3, "position {q}");
}

#[test]
fn a_target_below_the_joint_pushes_it_the_other_way() {
    let (q, qd, _) = last_sample(
        &[
            "--targets",
            "shoulder=-20",
            "--kp",
            "1000",
            "--max-torque",
            "1",
        ],
        "0.5",
    );
    assert!(q < 0.0 && qd < 0.0, "{q} {qd}");
}

#[test]
fn a_named_list_and_a_single_number_mean_the_same_for_one_joint() {
    let a = last_sample(
        &[
            "--targets",
            "shoulder=0.4",
            "--kp",
            "30",
            "--kd",
            "1",
            "--max-torque",
            "5",
        ],
        "0.5",
    );
    let b = last_sample(
        &[
            "--targets",
            "shoulder=0.4",
            "--kp",
            "shoulder=30",
            "--kd",
            "shoulder=1",
            "--max-torque",
            "shoulder=5",
        ],
        "0.5",
    );
    assert_eq!((a.0, a.1), (b.0, b.1));
}

#[test]
fn the_open_loop_command_is_unchanged_without_targets() {
    let (q, qd, err) = last_sample(&["--torques", "shoulder=1"], "0.5");
    let alpha = 1.0 / INERTIA;
    assert!((q - 0.5 * alpha * 0.25).abs() < 1e-3 && (qd - alpha * 0.5).abs() < 1e-3);
    assert!(!err.contains("proportional-derivative"));
}

#[test]
fn the_motor_flags_are_checked() {
    let urdf = write_file("turn.urdf", URDF);
    let path = urdf.to_str().unwrap();
    for extra in [
        vec!["--kp", "10"],                                     // gains without a target
        vec!["--targets", "shoulder=0.1", "--max-torque", "5"], // no stiffness
        vec!["--targets", "shoulder=0.1", "--kp", "10"],        // no top torque
        vec![
            "--targets",
            "shoulder=0.1",
            "--kp",
            "-1",
            "--max-torque",
            "5",
        ], // negative stiffness
        vec![
            "--targets",
            "shoulder=0.1",
            "--kp",
            "1",
            "--kd",
            "-1",
            "--max-torque",
            "5",
        ],
        vec![
            "--targets",
            "shoulder=0.1",
            "--kp",
            "1",
            "--max-torque",
            "0",
        ], // no torque at all
        vec![
            "--targets",
            "shoulder=0.1",
            "--kp",
            "nan",
            "--max-torque",
            "5",
        ],
        vec!["--targets", "elbow=0.1", "--kp", "1", "--max-torque", "5"], // unknown joint
        vec![
            "--targets",
            "shoulder=abc",
            "--kp",
            "1",
            "--max-torque",
            "5",
        ],
        vec![
            "--targets",
            "shoulder=0.1",
            "--kp",
            "elbow=1",
            "--max-torque",
            "5",
        ],
    ] {
        let mut args = vec!["simulate", "--urdf", path];
        args.extend_from_slice(&extra);
        let (code, _, err) = run(&args);
        assert_eq!(code, Some(2), "{extra:?}: {err}");
    }
}
