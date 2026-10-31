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
//! partial) URDF subset: serial chains and branching trees, `<mimic>` joints,
//! and the position, soft, velocity and effort limits. See `urdf.rs`/
//! `kinematics.rs`/`limits.rs` for what "real" means here, and what is still
//! out of scope (any actual MuJoCo/PhysX rigid-body or contact simulation).

mod collision;
#[cfg(test)]
mod corpus;
mod jacobian;
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
    "URDF forward kinematics and joint-limit validation for kinematic trees (no dynamics, no physics engine).";

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

fn load_model(urdf_path: &str) -> Result<urdf::Model, String> {
    let xml =
        fs::read_to_string(urdf_path).map_err(|e| format!("could not read '{urdf_path}': {e}"))?;
    urdf::parse_urdf_model(&xml).map_err(|e| e.to_string())
}

/// The given positions plus, for every joint that follows another (`<mimic>`), the position its master gives it, when the master has one.
/// A position given for a follower is an error: it is derived, never chosen.
fn positions_with_followers(
    model: &urdf::Model,
    given: &HashMap<String, f64>,
) -> Result<HashMap<String, f64>, String> {
    let mut all = given.clone();
    for (index, joint) in model.chain.joints.iter().enumerate() {
        if model.extras[index].mimic.is_some() && given.contains_key(&joint.name) {
            return Err(format!(
                "a position was given for joint '{}', which follows another joint (<mimic>): its position is derived",
                joint.name
            ));
        }
    }
    loop {
        let mut changed = false;
        for (index, joint) in model.chain.joints.iter().enumerate() {
            let Some(mimic) = &model.extras[index].mimic else {
                continue;
            };
            if all.contains_key(&joint.name) {
                continue;
            }
            if let Some(master) = all.get(&mimic.joint) {
                let value = mimic.multiplier * master + mimic.offset;
                all.insert(joint.name.clone(), value);
                changed = true;
            }
        }
        if !changed {
            return Ok(all);
        }
    }
}

fn run_fk(args: &[String]) -> ExitCode {
    let Some(urdf_path) = find_flag(args, "--urdf") else {
        eprintln!("fk: missing required --urdf PATH");
        return ExitCode::from(2);
    };
    let joints_spec = find_flag(args, "--joints").unwrap_or_default();

    let model = match load_model(&urdf_path) {
        Ok(m) => m,
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

    if let Err(e) = check_joint_names(&model.chain, &positions) {
        eprintln!("fk: {e}");
        return ExitCode::from(2);
    }

    match kinematics::forward_kinematics_model(&model, &positions) {
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

    let model = match load_model(&urdf_path) {
        Ok(m) => m,
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

    if let Err(e) = check_joint_names(&model.chain, &positions) {
        eprintln!("fk-checked: {e}");
        return ExitCode::from(2);
    }

    match kinematics::forward_kinematics_checked_model(&model, &positions) {
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

    let model = match load_model(&urdf_path) {
        Ok(m) => m,
        Err(e) => {
            eprintln!("validate-limits: {e}");
            return ExitCode::from(2);
        }
    };
    let chain = &model.chain;
    let given = match parse_joint_positions(&joints_spec) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("validate-limits: invalid --joints value: {e}");
            return ExitCode::from(2);
        }
    };

    if let Err(e) = check_joint_names(chain, &given) {
        eprintln!("validate-limits: {e}");
        return ExitCode::from(2);
    }
    // The followers (<mimic>) have the position their master gives them.
    let positions = match positions_with_followers(&model, &given) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("validate-limits: {e}");
            return ExitCode::from(2);
        }
    };
    // The speeds and efforts to check, by joint name (--velocities and --efforts take the same name=value list as --joints).
    let mut magnitudes: Vec<(&str, &str, HashMap<String, f64>)> = Vec::new();
    for (flag, kind) in [("--velocities", "velocity"), ("--efforts", "effort")] {
        let Some(spec) = find_flag(args, flag) else {
            continue;
        };
        let values = match parse_joint_positions(&spec) {
            Ok(v) => v,
            Err(e) => {
                eprintln!("validate-limits: invalid {flag} value: {e}");
                return ExitCode::from(2);
            }
        };
        if let Err(e) = check_joint_names(chain, &values) {
            eprintln!("validate-limits: {flag}: {e}");
            return ExitCode::from(2);
        }
        let unchecked: Vec<&str> = values
            .keys()
            .filter(|name| {
                let index = chain
                    .joints
                    .iter()
                    .position(|j| &j.name == *name)
                    .unwrap_or(0);
                let limit = if kind == "velocity" {
                    model.extras[index].velocity_limit
                } else {
                    model.extras[index].effort_limit
                };
                limit.is_none()
            })
            .map(String::as_str)
            .collect();
        if !unchecked.is_empty() {
            eprintln!(
                "validate-limits: joint(s) {} have no {kind} limit in the URDF: they cannot be checked",
                unchecked.join(", ")
            );
            return ExitCode::from(2);
        }
        magnitudes.push((flag, kind, values));
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
    let missing = limits::joints_without_limits(chain);
    if !missing.is_empty() {
        eprintln!(
            "validate-limits: joint(s) {} have no <limit>: they cannot be checked",
            missing.join(", ")
        );
        return ExitCode::from(2);
    }

    let violations = limits::validate_limits(chain, &positions);
    let soft_violations = if has_flag(args, "--soft") {
        limits::soft_limit_violations(&model, &positions)
    } else {
        Vec::new()
    };
    let mut speed_violations: Vec<(&str, limits::MagnitudeViolation)> = Vec::new();
    for (_, kind, values) in &magnitudes {
        let found = if *kind == "velocity" {
            limits::velocity_violations(&model, values)
        } else {
            limits::effort_violations(&model, values)
        };
        speed_violations.extend(found.into_iter().map(|v| (*kind, v)));
    }
    if violations.is_empty() && soft_violations.is_empty() && speed_violations.is_empty() {
        println!("No limit violations.");
        ExitCode::SUCCESS
    } else {
        for v in &violations {
            println!(
                "LIMIT VIOLATION: joint '{}' = {:.6} (allowed [{:.6}, {:.6}])",
                v.joint, v.value, v.lower, v.upper
            );
        }
        for v in &soft_violations {
            println!(
                "SOFT LIMIT VIOLATION: joint '{}' = {:.6} (soft range [{:.6}, {:.6}])",
                v.joint, v.value, v.lower, v.upper
            );
        }
        for (kind, v) in &speed_violations {
            println!(
                "{} LIMIT VIOLATION: joint '{}' = {:.6} (at most {:.6})",
                kind.to_uppercase(),
                v.joint,
                v.value,
                v.limit
            );
        }
        ExitCode::from(1)
    }
}

fn run_jacobian(args: &[String]) -> ExitCode {
    let Some(urdf_path) = find_flag(args, "--urdf") else {
        eprintln!("jacobian: missing required --urdf PATH");
        return ExitCode::from(2);
    };
    let Some(target) = find_flag(args, "--target") else {
        eprintln!("jacobian: missing required --target JOINT");
        return ExitCode::from(2);
    };
    let model = match load_model(&urdf_path) {
        Ok(m) => m,
        Err(e) => {
            eprintln!("jacobian: {e}");
            return ExitCode::from(2);
        }
    };
    let positions = match parse_joint_positions(&find_flag(args, "--joints").unwrap_or_default()) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("jacobian: invalid --joints value: {e}");
            return ExitCode::from(2);
        }
    };
    if let Err(e) = check_joint_names(&model.chain, &positions) {
        eprintln!("jacobian: {e}");
        return ExitCode::from(2);
    }
    match jacobian::jacobian(&model, &positions, &target) {
        Ok(j) => {
            eprintln!("{ESTIMATE_NOTICE}");
            if has_flag(args, "--json") {
                let rows: Vec<String> = j
                    .rows
                    .iter()
                    .map(|row| {
                        let cells: Vec<String> = row.iter().map(|v| format!("{v:.9}")).collect();
                        format!("[{}]", cells.join(","))
                    })
                    .collect();
                let names: Vec<String> = j
                    .joints
                    .iter()
                    .map(|n| format!("\"{}\"", json_escape(n)))
                    .collect();
                println!(
                    "{{\"target\":\"{}\",\"joints\":[{}],\"rows\":[{}]}}",
                    json_escape(&target),
                    names.join(","),
                    rows.join(",")
                );
            } else {
                println!("columns: {}", j.joints.join(" "));
                for (label, row) in ["vx", "vy", "vz", "wx", "wy", "wz"].iter().zip(&j.rows) {
                    let cells: Vec<String> = row.iter().map(|v| format!("{v:.6}")).collect();
                    println!("{label}: {}", cells.join(" "));
                }
            }
            ExitCode::SUCCESS
        }
        Err(e) => {
            eprintln!("jacobian: {e}");
            ExitCode::from(2)
        }
    }
}

fn run_ik(args: &[String]) -> ExitCode {
    let Some(urdf_path) = find_flag(args, "--urdf") else {
        eprintln!("ik: missing required --urdf PATH");
        return ExitCode::from(2);
    };
    let Some(target) = find_flag(args, "--target") else {
        eprintln!("ik: missing required --target JOINT");
        return ExitCode::from(2);
    };
    let Some(goal_text) = find_flag(args, "--goal") else {
        eprintln!("ik: missing required --goal X,Y,Z (metres, world frame)");
        return ExitCode::from(2);
    };
    let goal: Vec<f64> = goal_text
        .split(',')
        .filter_map(|v| v.trim().parse::<f64>().ok())
        .filter(|v| v.is_finite())
        .collect();
    if goal.len() != 3 || goal_text.split(',').count() != 3 {
        eprintln!("ik: --goal wants three finite numbers: X,Y,Z");
        return ExitCode::from(2);
    }
    let mut options = jacobian::IkOptions::default();
    for (flag, slot) in [
        ("--tolerance", &mut options.tolerance),
        ("--damping", &mut options.damping),
        ("--max-step", &mut options.max_step),
    ] {
        if let Some(text) = find_flag(args, flag) {
            match text.parse::<f64>() {
                Ok(v) if v.is_finite() && v > 0.0 => *slot = v,
                _ => {
                    eprintln!("ik: {flag} wants a positive number");
                    return ExitCode::from(2);
                }
            }
        }
    }
    if let Some(text) = find_flag(args, "--max-iterations") {
        match text.parse::<usize>() {
            Ok(v) if v > 0 => options.max_iterations = v,
            _ => {
                eprintln!("ik: --max-iterations wants a positive whole number");
                return ExitCode::from(2);
            }
        }
    }
    let model = match load_model(&urdf_path) {
        Ok(m) => m,
        Err(e) => {
            eprintln!("ik: {e}");
            return ExitCode::from(2);
        }
    };
    let initial = match parse_joint_positions(&find_flag(args, "--joints").unwrap_or_default()) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("ik: invalid --joints value: {e}");
            return ExitCode::from(2);
        }
    };
    if let Err(e) = check_joint_names(&model.chain, &initial) {
        eprintln!("ik: {e}");
        return ExitCode::from(2);
    }
    match jacobian::ik_position(
        &model,
        &initial,
        &target,
        [goal[0], goal[1], goal[2]],
        options,
    ) {
        Ok(solution) => {
            eprintln!("{ESTIMATE_NOTICE}");
            let mut names: Vec<&String> = solution.positions.keys().collect();
            names.sort();
            for name in names {
                println!("{name}={:.9}", solution.positions[name]);
            }
            eprintln!(
                "ik: reached within {:.6} m in {} iteration(s)",
                solution.residual, solution.iterations
            );
            ExitCode::SUCCESS
        }
        Err(e) => {
            eprintln!("ik: {e}");
            ExitCode::from(1)
        }
    }
}

/// Reads a JSON file for `predict-collision`, with the name of the flag in the error.
fn read_json_flag(args: &[String], flag: &str) -> Result<serde_json::Value, String> {
    let path = find_flag(args, flag).ok_or_else(|| format!("missing required {flag} PATH"))?;
    // a file the size of a map of the cell, not of a movie: refuse anything that is not a small text file
    let meta = fs::metadata(&path).map_err(|e| format!("could not read '{path}': {e}"))?;
    if meta.len() > 4 * 1024 * 1024 {
        return Err(format!("'{path}' is larger than 4 MiB"));
    }
    let text = fs::read_to_string(&path).map_err(|e| format!("could not read '{path}': {e}"))?;
    serde_json::from_str(&text).map_err(|e| format!("'{path}' is not valid JSON: {e}"))
}

/// Exit status: 0 no contact predicted, 1 a contact with a zone or the robot itself is predicted, 2 the input is wrong.
fn run_predict_collision(args: &[String]) -> ExitCode {
    let Some(urdf_path) = find_flag(args, "--urdf") else {
        eprintln!("predict-collision: missing required --urdf PATH");
        return ExitCode::from(2);
    };
    let model = match load_model(&urdf_path) {
        Ok(m) => m,
        Err(e) => {
            eprintln!("predict-collision: {e}");
            return ExitCode::from(2);
        }
    };
    let loaded = (|| -> Result<_, String> {
        let zones = collision::parse_zones(&read_json_flag(args, "--zones")?)?;
        let config = collision::parse_config(&read_json_flag(args, "--config")?)?;
        let mut states = collision::parse_trajectory(&read_json_flag(args, "--trajectory")?)?;
        if find_flag(args, "--base").is_some() {
            let base = collision::parse_base(&read_json_flag(args, "--base")?)?;
            states = collision::fill_from_base(&model, &states, &base);
        }
        Ok((zones, config, states))
    })();
    let (zones, config, states) = match loaded {
        Ok(v) => v,
        Err(e) => {
            eprintln!("predict-collision: {e}");
            return ExitCode::from(2);
        }
    };
    match collision::predict(&model, &zones, &config, &states) {
        Ok(prediction) => {
            eprintln!("{ESTIMATE_NOTICE}");
            if has_flag(args, "--json") {
                println!("{}", prediction.to_json());
            } else {
                println!(
                    "{}: {} state(s), {} sample(s), smallest clearance to a zone {} m, to itself {} m, {} contact(s)",
                    if prediction.collision() {
                        "COLLISION PREDICTED"
                    } else {
                        "clear"
                    },
                    prediction.states,
                    prediction.samples,
                    prediction
                        .min_zone_clearance_m
                        .map_or("-".to_string(), |v| format!("{v:.6}")),
                    prediction
                        .min_self_clearance_m
                        .map_or("-".to_string(), |v| format!("{v:.6}")),
                    prediction.hits_total
                );
                for hit in &prediction.hits {
                    println!(
                        "  at state {:.3}: {} {} with {} (clearance {:.6} m)",
                        hit.at,
                        hit.kind,
                        hit.link,
                        hit.zone
                            .as_deref()
                            .or(hit.other_link.as_deref())
                            .unwrap_or("?"),
                        hit.clearance_m
                    );
                }
            }
            if prediction.collision() {
                ExitCode::from(1)
            } else {
                ExitCode::SUCCESS
            }
        }
        Err(e) => {
            eprintln!("predict-collision: {e}");
            ExitCode::from(2)
        }
    }
}

fn main() -> ExitCode {
    let args: Vec<String> = env::args().skip(1).collect();

    match args.first().map(|s| s.as_str()) {
        Some("fk") => run_fk(&args[1..]),
        Some("fk-checked") => run_fk_checked(&args[1..]),
        Some("validate-limits") => run_validate_limits(&args[1..]),
        Some("jacobian") => run_jacobian(&args[1..]),
        Some("ik") => run_ik(&args[1..]),
        Some("predict-collision") => run_predict_collision(&args[1..]),
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
