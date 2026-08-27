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
const ROLE: &str =
    "High-fidelity MuJoCo/PhysX simulation of URDF kinematic chains for the Digital Twin.";

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
        positions.insert(name.trim().to_string(), value);
    }
    Ok(positions)
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

    match kinematics::forward_kinematics(&chain, &positions) {
        Ok(result) => {
            for (name, transform) in &result {
                let p = transform.translation_part();
                println!("{name}: x={:.6} y={:.6} z={:.6}", p.x, p.y, p.z);
            }
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

    match kinematics::forward_kinematics_checked(&chain, &positions) {
        Ok(result) => {
            for (name, transform) in &result {
                let p = transform.translation_part();
                println!("{name}: x={:.6} y={:.6} z={:.6}", p.x, p.y, p.z);
            }
            ExitCode::SUCCESS
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
