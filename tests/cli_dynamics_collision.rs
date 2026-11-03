// HYDRA-UMC-PHYSICS-REPLICA - tests/cli_dynamics_collision.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0-or-later - see LICENSE
//
// What a caller of the binary sees on the commands of the capsule self-collision, the inverse dynamics, the MJCF export and the
// trajectory check, and that `serve` answers the computing routes of a robot it was given.

use std::io::{Read, Write};
use std::net::TcpStream;
use std::path::PathBuf;
use std::process::{Command, Stdio};

// A shoulder that swings a rod over a post that stands beside it.
const URDF: &str = r#"<robot name="swing">
  <link name="base"><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link>
  <link name="arm">
    <collision><origin xyz="0.25 0 0" rpy="0 1.5707963267948966 0"/><geometry><cylinder radius="0.02" length="0.5"/></geometry></collision>
    <inertial><origin xyz="0.25 0 0"/><mass value="1"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial></link>
  <link name="post"><collision><geometry><sphere radius="0.03"/></geometry></collision></link>
  <joint name="shoulder" type="revolute"><parent link="base"/><child link="arm"/>
    <origin xyz="0 0 0.3"/><axis xyz="0 0 1"/><limit lower="-3.1" upper="3.1" effort="20" velocity="2"/></joint>
  <joint name="mount" type="fixed"><parent link="base"/><child link="post"/><origin xyz="0 0.4 0.3"/></joint>
</robot>"#;

fn write_file(name: &str, content: &str) -> PathBuf {
    // one directory per call: tests run in parallel and a shared file would be read while another test truncates it
    static COUNTER: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
    let n = COUNTER.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
    let dir = std::env::temp_dir().join(format!("physics-replica-dc-{}-{}", std::process::id(), n));
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
fn capsules_come_out_as_json_in_the_frame_of_each_link() {
    let urdf = write_file("swing.urdf", URDF);
    let (code, out, err) = run(&["capsules", "--urdf", urdf.to_str().unwrap()]);
    assert_eq!(code, Some(0), "{err}");
    let v: serde_json::Value = serde_json::from_str(&out).unwrap();
    assert_eq!(v.as_array().unwrap().len(), 3);
    assert!(v
        .as_array()
        .unwrap()
        .iter()
        .any(|c| c["link"] == "arm" && c["radius_m"] == 0.02));
}

#[test]
fn self_collision_exit_status_follows_the_contact() {
    let urdf = write_file("swing.urdf", URDF);
    let clear = write_file("clear.json", r#"{"states": [{"shoulder": 0.0}]}"#);
    let hit = write_file(
        "hit.json",
        r#"{"states": [{"shoulder": 0.3}, {"shoulder": 1.9}]}"#,
    );
    let u = urdf.to_str().unwrap();
    let (code, out, _) = run(&[
        "self-collision",
        "--urdf",
        u,
        "--trajectory",
        clear.to_str().unwrap(),
    ]);
    assert_eq!(code, Some(0), "{out}");
    assert!(out.starts_with("clear"));
    let (code, out, _) = run(&[
        "self-collision",
        "--urdf",
        u,
        "--trajectory",
        hit.to_str().unwrap(),
        "--json",
    ]);
    assert_eq!(code, Some(1), "{out}");
    let v: serde_json::Value = serde_json::from_str(&out).unwrap();
    assert_eq!(v["collision"], true);
    // a pair allowed to touch is not compared
    let config = write_file("ignore.json", r#"{"ignore_pairs": [["arm", "post"]]}"#);
    let (code, _, _) = run(&[
        "self-collision",
        "--urdf",
        u,
        "--trajectory",
        hit.to_str().unwrap(),
        "--config",
        config.to_str().unwrap(),
    ]);
    assert_eq!(code, Some(0));
    // a wrong input is 2
    let (code, _, err) = run(&["self-collision", "--urdf", u]);
    assert_eq!(code, Some(2));
    assert!(err.contains("--trajectory"));
}

#[test]
fn inverse_dynamics_holds_the_rod_against_nothing_and_flags_an_overload() {
    let urdf = write_file("swing.urdf", URDF);
    let u = urdf.to_str().unwrap();
    // gravity is along z and the axis is z: no holding torque; the inertia about z is 0.01 + 1 * 0.25^2 = 0.0725
    let (code, out, err) = run(&[
        "inverse-dynamics",
        "--urdf",
        u,
        "--joints",
        "shoulder=0.4",
        "--velocities",
        "shoulder=0",
        "--accelerations",
        "shoulder=2",
        "--allow-missing-inertial",
    ]);
    assert_eq!(code, Some(0), "{err}");
    let torque: f64 = out
        .trim()
        .strip_prefix("shoulder=")
        .unwrap()
        .parse()
        .unwrap();
    assert!((torque - 0.0725 * 2.0).abs() < 1e-9, "{out}");
    // a link without <inertial> is refused unless allowed
    let (code, _, err) = run(&[
        "inverse-dynamics",
        "--urdf",
        u,
        "--joints",
        "shoulder=0.4",
        "--velocities",
        "shoulder=0",
        "--accelerations",
        "shoulder=2",
    ]);
    assert_eq!(code, Some(2));
    assert!(err.contains("no <inertial>"));
    // 400 rad/s^2 asks for 29 N m against a limit of 20
    let (code, out, _) = run(&[
        "inverse-dynamics",
        "--urdf",
        u,
        "--joints",
        "shoulder=0.4",
        "--velocities",
        "shoulder=0",
        "--accelerations",
        "shoulder=400",
        "--allow-missing-inertial",
    ]);
    assert_eq!(code, Some(1));
    assert!(out.contains("EFFORT VIOLATION"));
}

#[test]
fn mjcf_is_written_and_a_mujoco_check_needs_the_feature() {
    let urdf = write_file("swing.urdf", URDF);
    let u = urdf.to_str().unwrap();
    let (code, out, _) = run(&["mjcf", "--urdf", u]);
    assert_eq!(code, Some(0));
    assert!(out.starts_with("<mujoco model=\"swing\">"));
    assert!(out.contains("type=\"hinge\""));
    let (code, _, err) = run(&["mujoco-check", "--urdf", u]);
    if cfg!(feature = "mujoco") {
        assert_eq!(code, Some(2));
    } else {
        assert_eq!(code, Some(2));
        assert!(err.contains("--features mujoco"));
    }
}

#[test]
fn check_trajectory_on_a_file_uses_the_configuration() {
    let urdf = write_file("swing.urdf", URDF);
    let u = urdf.to_str().unwrap();
    let slow = write_file(
        "slow.json",
        r#"{"samples": [{"t": 0, "joints": {"shoulder": 0.0}}, {"t": 1, "joints": {"shoulder": 0.5}}]}"#,
    );
    let fast = write_file(
        "fast.json",
        r#"{"samples": [{"t": 0, "joints": {"shoulder": 0.0}}, {"t": 0.1, "joints": {"shoulder": 0.5}}]}"#,
    );
    let (code, out, _) = run(&[
        "check-trajectory",
        "--urdf",
        u,
        "--samples",
        slow.to_str().unwrap(),
    ]);
    assert_eq!(code, Some(0), "{out}");
    let v: serde_json::Value = serde_json::from_str(&out).unwrap();
    assert_eq!(v["ok"], true);
    let (code, out, _) = run(&[
        "check-trajectory",
        "--urdf",
        u,
        "--samples",
        fast.to_str().unwrap(),
    ]);
    assert_eq!(code, Some(1));
    assert!(out.contains("\"kind\":\"velocity\""));
    // the tolerance lives in the configuration file, not in the code
    let config = write_file(
        "service.json",
        r#"{"trajectory": {"velocity_tolerance": 10}}"#,
    );
    let (code, _, _) = run(&[
        "check-trajectory",
        "--urdf",
        u,
        "--samples",
        fast.to_str().unwrap(),
        "--config",
        config.to_str().unwrap(),
    ]);
    assert_eq!(code, Some(0));
}

#[test]
fn serve_answers_the_computing_routes_for_the_robot_it_was_given() {
    let urdf = write_file("swing.urdf", URDF);
    // find a free port, then let the program take it
    let port = std::net::TcpListener::bind("127.0.0.1:0")
        .unwrap()
        .local_addr()
        .unwrap()
        .port();
    let mut child = Command::new(env!("CARGO_BIN_EXE_hydra-umc-physics-replica"))
        .args([
            "serve",
            "--addr",
            &format!("127.0.0.1:{port}"),
            "--urdf",
            urdf.to_str().unwrap(),
        ])
        .stderr(Stdio::null())
        .spawn()
        .unwrap();
    let mut answer = None;
    for _ in 0..50 {
        if let Ok(mut stream) = TcpStream::connect(("127.0.0.1", port)) {
            let body = r#"{"joints": {"shoulder": 0.0}}"#;
            write!(stream, "POST /fk HTTP/1.1\r\nHost: x\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{body}", body.len()).unwrap();
            let mut raw = String::new();
            stream.read_to_string(&mut raw).unwrap();
            answer = Some(raw);
            break;
        }
        std::thread::sleep(std::time::Duration::from_millis(100));
    }
    let _ = child.kill();
    let _ = child.wait();
    let raw = answer.expect("the service did not start");
    assert!(raw.starts_with("HTTP/1.1 200"), "{raw}");
    assert!(raw.contains("\"joint\":\"shoulder\""));
}
