// HYDRA-UMC-PHYSICS-REPLICA - tests/robot_arms.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0 - see LICENSE
//
// Two real six- and seven-axis arms, a UR5e and a Franka Emika Panda, as seen by a caller of the binary. No file of the manufacturers is
// copied: each URDF is generated here from the Denavit-Hartenberg parameters that the manufacturers publish (the UR5e in the standard
// convention, the Panda in Craig's modified one), and the position of the tool that the binary computes is compared with the one that
// the DH chain gives by multiplying the 4x4 matrices directly - a different route to the same number - and with the tool positions at
// the zero pose that the manufacturers document.

use std::path::PathBuf;
use std::process::Command;

const PI: f64 = std::f64::consts::PI;

#[derive(Clone, Copy)]
struct Dh {
    a: f64,
    d: f64,
    alpha: f64,
}

#[derive(Clone, Copy, PartialEq)]
enum Convention {
    Standard,
    Modified,
}

struct Arm {
    name: &'static str,
    convention: Convention,
    dh: Vec<Dh>,
    /// The fixed link from the last joint to the tool (a flange), as one more DH row with no joint.
    tool: Dh,
    limits: Vec<(f64, f64)>,
}

fn ur5e() -> Arm {
    // Universal Robots, "DH parameters for calculations of kinematics and dynamics", UR5e.
    let row = |a: f64, d: f64, alpha: f64| Dh { a, d, alpha };
    Arm {
        name: "ur5e",
        convention: Convention::Standard,
        dh: vec![
            row(0.0, 0.1625, PI / 2.0),
            row(-0.425, 0.0, 0.0),
            row(-0.3922, 0.0, 0.0),
            row(0.0, 0.1333, PI / 2.0),
            row(0.0, 0.0997, -PI / 2.0),
            row(0.0, 0.0996, 0.0),
        ],
        tool: row(0.0, 0.0, 0.0),
        limits: vec![(-2.0 * PI, 2.0 * PI); 6],
    }
}

fn panda() -> Arm {
    // Franka Emika, "Robot and interface specifications": Craig's modified DH, rows (a, d, alpha) of the link before each joint.
    let row = |a: f64, d: f64, alpha: f64| Dh { a, d, alpha };
    Arm {
        name: "panda",
        convention: Convention::Modified,
        dh: vec![
            row(0.0, 0.333, 0.0),
            row(0.0, 0.0, -PI / 2.0),
            row(0.0, 0.316, PI / 2.0),
            row(0.0825, 0.0, PI / 2.0),
            row(-0.0825, 0.384, -PI / 2.0),
            row(0.0, 0.0, PI / 2.0),
            row(0.088, 0.0, PI / 2.0),
        ],
        tool: row(0.0, 0.107, 0.0),
        limits: vec![
            (-2.8973, 2.8973),
            (-1.7628, 1.7628),
            (-2.8973, 2.8973),
            (-3.0718, -0.0698),
            (-2.8973, 2.8973),
            (-0.0175, 3.7525),
            (-2.8973, 2.8973),
        ],
    }
}

// --- the URDF, generated from the DH rows ------------------------------------------------------------------------------------

fn origin(xyz: (f64, f64, f64), roll: f64) -> String {
    format!(
        r#"<origin xyz="{:.10} {:.10} {:.10}" rpy="{:.10} 0 0"/>"#,
        xyz.0, xyz.1, xyz.2, roll
    )
}

/// The fixed transform that a DH row contributes, as a URDF origin.
fn fixed_part(convention: Convention, row: Dh) -> String {
    match convention {
        // standard: Rz(theta) Tz(d) Tx(a) Rx(alpha) - the part after the rotation is translate(a, 0, d) then Rx(alpha)
        Convention::Standard => origin((row.a, 0.0, row.d), row.alpha),
        // modified: Rx(alpha) Tx(a) Rz(theta) Tz(d) - translate(a, -d sin(alpha), d cos(alpha)) then Rx(alpha), before the rotation
        Convention::Modified => origin(
            (row.a, -row.d * row.alpha.sin(), row.d * row.alpha.cos()),
            row.alpha,
        ),
    }
}

fn urdf(arm: &Arm) -> String {
    let mut xml = format!(r#"<robot name="{}">"#, arm.name);
    let n = arm.dh.len();
    for link in 0..=n {
        xml += &format!(r#"<link name="link{link}"/>"#);
    }
    xml += r#"<link name="tool"/>"#;
    for (i, ((row, (lower, upper)), joint)) in arm.dh.iter().zip(&arm.limits).zip(1..).enumerate() {
        // in the standard convention the rotation comes first, so the origin of a joint is the fixed part of the row before it
        let at = match arm.convention {
            Convention::Modified => fixed_part(arm.convention, *row),
            Convention::Standard if i == 0 => origin((0.0, 0.0, 0.0), 0.0),
            Convention::Standard => fixed_part(arm.convention, arm.dh[i - 1]),
        };
        xml += &format!(
            r#"<joint name="j{joint}" type="revolute"><parent link="link{i}"/><child link="link{joint}"/>{at}<axis xyz="0 0 1"/><limit lower="{lower}" upper="{upper}" effort="100" velocity="3"/></joint>"#
        );
    }
    let at = match arm.convention {
        Convention::Standard => fixed_part(arm.convention, arm.dh[n - 1]),
        Convention::Modified => fixed_part(arm.convention, arm.tool),
    };
    xml += &format!(
        r#"<joint name="tool0" type="fixed"><parent link="link{n}"/><child link="tool"/>{at}</joint>"#
    );
    xml + "</robot>"
}

// --- the same chain by multiplying 4x4 matrices -----------------------------------------------------------------------------

type M = [[f64; 4]; 4];

fn mul(a: &M, b: &M) -> M {
    let mut out = [[0.0; 4]; 4];
    for i in 0..4 {
        for j in 0..4 {
            out[i][j] = (0..4).map(|k| a[i][k] * b[k][j]).sum();
        }
    }
    out
}

fn rot_z(t: f64) -> M {
    [
        [t.cos(), -t.sin(), 0.0, 0.0],
        [t.sin(), t.cos(), 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
}
fn rot_x(t: f64) -> M {
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, t.cos(), -t.sin(), 0.0],
        [0.0, t.sin(), t.cos(), 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
}
fn trans(x: f64, y: f64, z: f64) -> M {
    [
        [1.0, 0.0, 0.0, x],
        [0.0, 1.0, 0.0, y],
        [0.0, 0.0, 1.0, z],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

fn dh_tool(arm: &Arm, q: &[f64]) -> (f64, f64, f64) {
    let mut t = trans(0.0, 0.0, 0.0);
    t[0][0] = 1.0;
    for (row, angle) in arm.dh.iter().zip(q) {
        let step = match arm.convention {
            Convention::Standard => mul(
                &mul(
                    &mul(&rot_z(*angle), &trans(0.0, 0.0, row.d)),
                    &trans(row.a, 0.0, 0.0),
                ),
                &rot_x(row.alpha),
            ),
            Convention::Modified => mul(
                &mul(
                    &mul(&rot_x(row.alpha), &trans(row.a, 0.0, 0.0)),
                    &rot_z(*angle),
                ),
                &trans(0.0, 0.0, row.d),
            ),
        };
        t = mul(&t, &step);
    }
    if arm.convention == Convention::Modified {
        t = mul(
            &mul(&t, &rot_x(arm.tool.alpha)),
            &trans(arm.tool.a, 0.0, arm.tool.d),
        );
    }
    (t[0][3], t[1][3], t[2][3])
}

// --- the binary ---------------------------------------------------------------------------------------------------------------

fn urdf_file(arm: &Arm) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("physics-replica-arms-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join(format!("{}.urdf", arm.name));
    std::fs::write(&path, urdf(arm)).unwrap();
    path
}

fn run(arm: &Arm, command: &str, q: &[f64]) -> (Option<i32>, String, String) {
    let joints: Vec<String> = q
        .iter()
        .enumerate()
        .map(|(i, v)| format!("j{}={:.12}", i + 1, v))
        .collect();
    let output = Command::new(env!("CARGO_BIN_EXE_hydra-umc-physics-replica"))
        .args([
            command,
            "--urdf",
            &urdf_file(arm).to_string_lossy(),
            "--joints",
            &joints.join(","),
        ])
        .output()
        .unwrap();
    (
        output.status.code(),
        String::from_utf8_lossy(&output.stdout).to_string(),
        String::from_utf8_lossy(&output.stderr).to_string(),
    )
}

fn tool_position(stdout: &str) -> (f64, f64, f64) {
    let line = stdout
        .lines()
        .find(|l| l.starts_with("tool0:"))
        .unwrap_or_else(|| panic!("no tool0 line in {stdout}"));
    let number = |key: &str| -> f64 {
        let start = line.find(key).unwrap() + key.len();
        line[start..]
            .split_whitespace()
            .next()
            .unwrap()
            .parse()
            .unwrap()
    };
    (number("x="), number("y="), number("z="))
}

fn close(a: (f64, f64, f64), b: (f64, f64, f64), tolerance: f64) -> bool {
    (a.0 - b.0).abs() <= tolerance
        && (a.1 - b.1).abs() <= tolerance
        && (a.2 - b.2).abs() <= tolerance
}

/// A few poses per arm that are not special: each joint at a different fraction of its range.
fn poses(arm: &Arm) -> Vec<Vec<f64>> {
    [0.15, 0.5, 0.85]
        .iter()
        .enumerate()
        .map(|(k, fraction)| {
            arm.limits
                .iter()
                .enumerate()
                .map(|(j, (lower, upper))| {
                    let f = (fraction + 0.11 * ((j + k) % 3) as f64).min(0.95);
                    // keep UR5e's full-turn range to a quarter of itself, which is plenty and keeps the numbers readable
                    let (lower, upper) = if *upper - *lower > 7.0 {
                        (-PI, PI)
                    } else {
                        (*lower, *upper)
                    };
                    lower + f * (upper - lower)
                })
                .collect()
        })
        .collect()
}

#[test]
fn ur5e_at_the_zero_pose_is_where_the_manufacturer_documents_it() {
    let arm = ur5e();
    let (code, stdout, stderr) = run(&arm, "fk", &[0.0; 6]);
    assert_eq!(code, Some(0), "{stderr}");
    // x = a2 + a3, y = -(d4 + d6), z = d1 - d5
    let tcp = tool_position(&stdout);
    assert!(close(tcp, (-0.8172, -0.2329, 0.0628), 1e-6), "{tcp:?}");
}

#[test]
fn panda_at_the_zero_pose_is_where_the_manufacturer_documents_it() {
    let arm = panda();
    let (code, stdout, stderr) = run(&arm, "fk", &[0.0; 7]);
    assert_eq!(code, Some(0), "{stderr}");
    // the flange at q = 0: x = a7 = 0.088, z = d1 + d3 + d5 - d_flange = 0.926
    let tcp = tool_position(&stdout);
    assert!(close(tcp, (0.088, 0.0, 0.926), 1e-6), "{tcp:?}");
}

#[test]
fn the_urdf_made_from_dh_gives_the_tool_that_the_dh_chain_gives() {
    for arm in [ur5e(), panda()] {
        for q in poses(&arm) {
            let (code, stdout, stderr) = run(&arm, "fk", &q);
            assert_eq!(code, Some(0), "{}: {stderr}", arm.name);
            let from_urdf = tool_position(&stdout);
            let from_dh = dh_tool(&arm, &q);
            assert!(
                close(from_urdf, from_dh, 1e-5),
                "{} at {q:?}: urdf {from_urdf:?}, dh {from_dh:?}",
                arm.name
            );
        }
    }
}

#[test]
fn the_reach_of_the_arm_never_exceeds_the_sum_of_its_links() {
    // a property that holds for any pose of any serial arm: the tool cannot be farther from the base than the links are long
    for (arm, reach) in [
        (ur5e(), 0.1625 + 0.425 + 0.3922 + 0.1333 + 0.0997 + 0.0996),
        (
            panda(),
            0.333 + 0.316 + 0.384 + 0.107 + 0.0825 * 2.0 + 0.088,
        ),
    ] {
        for q in poses(&arm) {
            let (_, stdout, _) = run(&arm, "fk", &q);
            let (x, y, z) = tool_position(&stdout);
            assert!(
                (x * x + y * y + z * z).sqrt() <= reach + 1e-9,
                "{} at {q:?}",
                arm.name
            );
        }
    }
}

#[test]
fn the_published_joint_limits_are_enforced() {
    let panda = panda();
    // inside every limit
    let inside = [0.0, 0.0, 0.0, -1.5, 0.0, 1.5, 0.0];
    let (code, stdout, stderr) = run(&panda, "validate-limits", &inside);
    assert_eq!(code, Some(0), "{stdout} {stderr}");
    assert!(stdout.contains("No limit violations"), "{stdout}");
    // joint 4 of the Panda cannot reach zero (its range is -3.0718 to -0.0698), and joint 2 stops at 1.7628
    let outside = [0.0, 1.8, 0.0, 0.0, 0.0, 1.5, 0.0];
    let (code, stdout, _) = run(&panda, "validate-limits", &outside);
    assert_eq!(code, Some(1), "{stdout}");
    assert!(stdout.contains("j2") && stdout.contains("j4"), "{stdout}");
    assert!(!stdout.contains("j1") && !stdout.contains("j3"), "{stdout}");
    // the checked forward kinematics refuses the same pose
    let (code, _, _) = run(&panda, "fk-checked", &outside);
    assert_ne!(code, Some(0));
    let (code, _, stderr) = run(&panda, "fk-checked", &inside);
    assert_eq!(code, Some(0), "{stderr}");
    // the UR5e turns a full circle either way: 6 rad is inside, 6.5 rad is not
    let ur = ur5e();
    assert_eq!(
        run(&ur, "validate-limits", &[6.0, 0.0, 0.0, 0.0, 0.0, 0.0]).0,
        Some(0)
    );
    assert_eq!(
        run(&ur, "validate-limits", &[6.5, 0.0, 0.0, 0.0, 0.0, 0.0]).0,
        Some(1)
    );
}

#[test]
fn a_pose_with_a_missing_joint_is_refused() {
    let (code, _, stderr) = run(&ur5e(), "fk", &[0.0; 5]);
    assert_eq!(code, Some(2));
    assert!(stderr.contains("j6"), "{stderr}");
}
