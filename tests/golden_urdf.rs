// HYDRA-UMC-PHYSICS-REPLICA - tests/golden_urdf.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0-or-later - see LICENSE
//
// The shared golden URDF set (tests/golden/): the same files are checked in the URDF editor and in the STL editor, so all
// of them read origin, rpy and axis the same way. Each case names a URDF, some joint positions and where every joint's
// child frame has to end up; the binary's `fk` answers with those positions.

use serde_json::Value;
use std::collections::BTreeMap;
use std::path::PathBuf;
use std::process::Command;

fn golden() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("golden")
}

fn read_cases() -> Value {
    let text =
        std::fs::read_to_string(golden().join("cases.json")).expect("tests/golden/cases.json");
    serde_json::from_str(&text).expect("cases.json is JSON")
}

fn fk(urdf: &str, joints: &serde_json::Map<String, Value>) -> BTreeMap<String, [f64; 3]> {
    let spec: Vec<String> = joints
        .iter()
        .map(|(name, value)| format!("{name}={:.17}", value.as_f64().unwrap()))
        .collect();
    let mut args = vec![
        "fk".to_string(),
        "--urdf".into(),
        golden().join(urdf).to_string_lossy().to_string(),
    ];
    if !spec.is_empty() {
        args.push("--joints".into());
        args.push(spec.join(","));
    }
    let output = Command::new(env!("CARGO_BIN_EXE_hydra-umc-physics-replica"))
        .args(&args)
        .output()
        .unwrap();
    assert_eq!(
        output.status.code(),
        Some(0),
        "{urdf}: {}",
        String::from_utf8_lossy(&output.stderr)
    );
    let mut found = BTreeMap::new();
    for line in String::from_utf8_lossy(&output.stdout).lines() {
        let Some((name, rest)) = line.split_once(": ") else {
            continue;
        };
        let mut xyz = [0.0; 3];
        for part in rest.split_whitespace() {
            let (axis, number) = part.split_once('=').unwrap();
            let value: f64 = number.parse().unwrap();
            match axis {
                "x" => xyz[0] = value,
                "y" => xyz[1] = value,
                "z" => xyz[2] = value,
                _ => {}
            }
        }
        found.insert(name.to_string(), xyz);
    }
    found
}

#[test]
fn every_golden_case_puts_every_joint_frame_where_the_reference_says() {
    let cases = read_cases();
    // the binary prints six decimals, so the tolerance is the printing resolution, not the file's own
    let tolerance = cases["tolerance"].as_f64().unwrap().max(2e-6);
    let list = cases["cases"].as_array().unwrap();
    assert!(list.len() >= 10, "the golden set has shrunk");
    for case in list {
        let urdf = case["urdf"].as_str().unwrap();
        let found = fk(urdf, case["joints"].as_object().unwrap());
        for (joint, expected) in case["expected"].as_object().unwrap() {
            let want: Vec<f64> = expected
                .as_array()
                .unwrap()
                .iter()
                .map(|v| v.as_f64().unwrap())
                .collect();
            let got = found
                .get(joint)
                .unwrap_or_else(|| panic!("{urdf}: no pose for joint {joint}"));
            for (axis, (g, w)) in got.iter().zip(&want).enumerate() {
                assert!(
                    (g - w).abs() <= tolerance,
                    "{urdf} joint {joint} axis {axis}: got {g} want {w}"
                );
            }
        }
    }
}

#[test]
fn every_golden_robot_is_used_by_a_case() {
    let cases = read_cases();
    let used: std::collections::BTreeSet<String> = cases["cases"]
        .as_array()
        .unwrap()
        .iter()
        .map(|c| c["urdf"].as_str().unwrap().to_string())
        .collect();
    let mut on_disk = std::collections::BTreeSet::new();
    for entry in std::fs::read_dir(golden().join("urdf")).unwrap() {
        on_disk.insert(format!(
            "urdf/{}",
            entry.unwrap().file_name().to_string_lossy()
        ));
    }
    assert_eq!(used, on_disk);
}
