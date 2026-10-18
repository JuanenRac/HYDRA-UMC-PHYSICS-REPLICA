// =============================================================================
// HYDRA-UMC-PHYSICS-REPLICA - src/main.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0 - see LICENSE
// =============================================================================
//! Entry point for HYDRA-UMC-PHYSICS-REPLICA.
//!
//! Bare invocation (no arguments) is unchanged from the skeleton stage:
//! prints identity and exits 0.
//!
//! Real v0 subcommands - `fk` (forward kinematics) and `validate-limits` -
//! run this project's actual kinematic-validation logic over a (documented,
//! partial) URDF subset. See `urdf.rs`/`kinematics.rs`/`limits.rs` for what
//! "real" means here, and what is still out of scope (branching URDF trees,
//! any actual MuJoCo/PhysX rigid-body or contact simulation).

#[cfg(test)]
mod corpus;
mod kinematics;
mod limits;
mod transform;
mod urdf;

use std::collections::HashMap;
use std::env;
use std::fs;
use std::process::ExitCode;

const PROJECT_NAME: &str = "HYDRA-UMC-PHYSICS-REPLICA";
const VERSION: &str = env!("CARGO_PKG_VERSION");
/// Printed on stderr by every command that reports a pose: what these numbers
/// are and are not, so they are never mistaken for a measurement.
const ESTIMATE_NOTICE: &str = "note: kinematic estimate from the URDF geometry only - no dynamics, and not calibrated against a physical robot";

const ROLE: &str =
    "URDF forward kinematics and joint-limit validation for serial chains (no dynamics, no physics engine).";

/// Parses `"name=value,name=value"` into a position map. Returns `Err` with
/// a human-readable reason for the first malformed entry found.
fn parse_joint_positions(spec: &str) -> Result<HashMap<String, f64>, String> {
    let mut positions = HashMap::new();
    if spec.trim().is_empty() {
        return Ok(positions);
    }
    for entry in spec.split(',') {
        let entry = entry.trim();
        let (name, value_str) = entry
            .split_once('=')
            .ok_or_else(|| format!("expected 'name=value', got '{entry}'"))?;
        let value: f64 = value_str
            .trim()
            .parse()
            .map_err(|_| format!("'{value_str}' is not a valid number (joint '{name}')"))?;
        let name = name.trim();
        if name.is_empty() {
            return Err("joint name must not be empty".to_string());
        }
        if !value.is_finite() {
            return Err(format!(
                "'{value_str}' is not a finite number (joint '{name}')"
            ));
        }
        if positions.insert(name.to_string(), value).is_some() {
            return Err(format!("joint '{name}' is given more than once"));
        }
    }
    Ok(positions)
}

/// Refuses joint names in `--joints` that the URDF does not have: a typo
/// would otherwise be ignored and the check would answer for a configuration
/// that is not the one asked about.
fn check_joint_names(chain: &urdf::Chain, positions: &HashMap<String, f64>) -> Result<(), String> {
    let mut unknown: Vec<&str> = positions
        .keys()
        .filter(|name| !chain.joints.iter().any(|j| &j.name == *name))
        .map(String::as_str)
        .collect();
    if unknown.is_empty() {
        return Ok(());
    }
    unknown.sort_unstable();
    Err(format!(
        "unknown joint name(s) in --joints: {} (the URDF has: {})",
        unknown.join(", "),
        chain
            .joints
            .iter()
            .map(|j| j.name.as_str())
            .collect::<Vec<_>>()
            .join(", ")
    ))
}

fn json_escape(text: &str) -> String {
    let mut out = String::with_capacity(text.len() + 2);
    for c in text.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out
}

/// One joint's pose as a JSON object: position in metres and the rotation as
/// a unit quaternion (w >= 0).
fn pose_json(name: &str, transform: &transform::Mat4) -> String {
    let p = transform.translation_part();
    let (w, x, y, z) = transform.quaternion();
    format!(
        "{{\"joint\":\"{}\",\"position\":{{\"x\":{:.9},\"y\":{:.9},\"z\":{:.9}}},\"quaternion\":{{\"w\":{:.9},\"x\":{:.9},\"y\":{:.9},\"z\":{:.9}}}}}",
        json_escape(name), p.x, p.y, p.z, w, x, y, z
    )
}

fn print_poses(result: &[(String, transform::Mat4)], json: bool) {
    if json {
        let items: Vec<String> = result.iter().map(|(n, t)| pose_json(n, t)).collect();
        println!("[{}]", items.join(","));
    } else {
        for (name, transform) in result {
            let p = transform.translation_part();
            println!("{name}: x={:.6} y={:.6} z={:.6}", p.x, p.y, p.z);
        }
    }
}

fn has_flag(args: &[String], flag: &str) -> bool {
    args.iter().any(|a| a == flag)
}

fn find_flag(args: &[String], flag: &str) -> Option<String> {
    args.iter()
        .position(|a| a == flag)
        .and_then(|i| args.get(i + 1))
        .cloned()
}

fn load_chain(urdf_path: &str) -> Result<urdf::Chain, String> {
    let xml =
        fs::read_to_string(urdf_path).map_err(|e| format!("could not read '{urdf_path}': {e}"))?;
    urdf::parse_urdf(&xml).map_err(|e| e.to_string())
}

fn run_fk(args: &[String]) -> ExitCode {
    let Some(urdf_path) = find_flag(args, "--urdf") else {
        eprintln!("fk: missing required --urdf PATH");
        return ExitCode::from(2);
    };
    let joints_spec = find_flag(args, "--joints").unwrap_or_default();

    let chain = match load_chain(&urdf_path) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("fk: {e}");
            return ExitCode::from(2);
        }
    };
    let positions = match parse_joint_positions(&joints_spec) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("fk: invalid --joints value: {e}");
            return ExitCode::from(2);
        }
    };

    if let Err(e) = check_joint_names(&chain, &positions) {
        eprintln!("fk: {e}");
        return ExitCode::from(2);
    }

    match kinematics::forward_kinematics(&chain, &positions) {
        Ok(result) => {
            eprintln!("{ESTIMATE_NOTICE}");
            print_poses(&result, has_flag(args, "--json"));
            ExitCode::SUCCESS
        }
        Err(e) => {
            eprintln!("fk: {e}");
            ExitCode::from(2)
        }
    }
}

fn run_fk_checked(args: &[String]) -> ExitCode {
    let Some(urdf_path) = find_flag(args, "--urdf") else {
        eprintln!("fk-checked: missing required --urdf PATH");
        return ExitCode::from(2);
    };
    let joints_spec = find_flag(args, "--joints").unwrap_or_default();

    let chain = match load_chain(&urdf_path) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("fk-checked: {e}");
            return ExitCode::from(2);
        }
    };
    let positions = match parse_joint_positions(&joints_spec) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("fk-checked: invalid --joints value: {e}");
            return ExitCode::from(2);
        }
    };

    if let Err(e) = check_joint_names(&chain, &positions) {
        eprintln!("fk-checked: {e}");
        return ExitCode::from(2);
    }

    match kinematics::forward_kinematics_checked(&chain, &positions) {
        Ok(result) => {
            eprintln!("{ESTIMATE_NOTICE}");
            print_poses(&result, has_flag(args, "--json"));
            ExitCode::SUCCESS
        }
        Err(kinematics::CheckedKinematicsError::MissingLimits(joints)) => {
            eprintln!(
                "fk-checked: joint(s) {} have no <limit>: a position cannot be checked against a limit that is not there",
                joints.join(", ")
            );
            ExitCode::from(2)
        }
        Err(kinematics::CheckedKinematicsError::LimitViolation(violations)) => {
            for v in &violations {
                println!(
                    "LIMIT VIOLATION: joint '{}' = {:.6} (allowed [{:.6}, {:.6}]) - refusing to compute an unreachable pose",
                    v.joint, v.value, v.lower, v.upper
                );
            }
            ExitCode::from(1)
        }
        Err(kinematics::CheckedKinematicsError::Kinematics(e)) => {
            eprintln!("fk-checked: {e}");
            ExitCode::from(2)
        }
    }
}

fn run_validate_limits(args: &[String]) -> ExitCode {
    let Some(urdf_path) = find_flag(args, "--urdf") else {
        eprintln!("validate-limits: missing required --urdf PATH");
        return ExitCode::from(2);
    };
    let joints_spec = find_flag(args, "--joints").unwrap_or_default();

    let chain = match load_chain(&urdf_path) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("validate-limits: {e}");
            return ExitCode::from(2);
        }
    };
    let positions = match parse_joint_positions(&joints_spec) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("validate-limits: invalid --joints value: {e}");
            return ExitCode::from(2);
        }
    };

    if let Err(e) = check_joint_names(&chain, &positions) {
        eprintln!("validate-limits: {e}");
        return ExitCode::from(2);
    }
    // A joint with a limit and no position given is a check that was not
    // made: "No limit violations." must not be said about it.
    if !has_flag(args, "--allow-partial") {
        let unchecked: Vec<&str> = chain
            .joints
            .iter()
            .filter(|j| j.limit.is_some() && !positions.contains_key(&j.name))
            .map(|j| j.name.as_str())
            .collect();
        if !unchecked.is_empty() {
            eprintln!(
                "validate-limits: no position given for limited joint(s) {} (give them in --joints, or pass --allow-partial to check only the joints given)",
                unchecked.join(", ")
            );
            return ExitCode::from(2);
        }
    }
    let missing = limits::joints_without_limits(&chain);
    if !missing.is_empty() {
        eprintln!(
            "validate-limits: joint(s) {} have no <limit>: they cannot be checked",
            missing.join(", ")
        );
        return ExitCode::from(2);
    }

    let violations = limits::validate_limits(&chain, &positions);
    if violations.is_empty() {
        println!("No limit violations.");
        ExitCode::SUCCESS
    } else {
        for v in &violations {
            println!(
                "LIMIT VIOLATION: joint '{}' = {:.6} (allowed [{:.6}, {:.6}])",
                v.joint, v.value, v.lower, v.upper
            );
        }
        ExitCode::from(1)
    }
}

fn main() -> ExitCode {
    let args: Vec<String> = env::args().skip(1).collect();

    match args.first().map(|s| s.as_str()) {
        Some("fk") => run_fk(&args[1..]),
        Some("fk-checked") => run_fk_checked(&args[1..]),
        Some("validate-limits") => run_validate_limits(&args[1..]),
        _ => {
            println!("{PROJECT_NAME} v{VERSION}");
            println!("{ROLE}");
            ExitCode::SUCCESS
        }
    }
}

#[cfg(test)]
mod position_parse_tests {
    use super::{parse_joint_positions, ESTIMATE_NOTICE};

    #[test]
    fn the_estimate_notice_says_what_the_numbers_are_and_are_not() {
        assert!(ESTIMATE_NOTICE.contains("kinematic estimate"));
        assert!(ESTIMATE_NOTICE.contains("no dynamics"));
        assert!(ESTIMATE_NOTICE.contains("not calibrated"));
    }

    #[test]
    fn joint_positions_reject_empty_names_and_non_finite_values() {
        assert!(parse_joint_positions("=0.5").is_err());
        assert!(parse_joint_positions("shoulder=nan").is_err());
        assert!(parse_joint_positions("shoulder=inf").is_err());
    }
}
