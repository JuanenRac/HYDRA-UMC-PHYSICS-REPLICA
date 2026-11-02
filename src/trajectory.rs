// =============================================================================
// HYDRA-UMC-PHYSICS-REPLICA - src/trajectory.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0-or-later - see LICENSE
// =============================================================================
//! Checks a sampled trajectory against what the URDF and the configuration declare, before it is sent to the robot or to the
//! hardware-in-the-loop bridge: position limits (and soft limits), speed against `<limit velocity>`, acceleration against the limits
//! of the configuration (the URDF has none), optionally the joint torques against `<limit effort>` (inverse dynamics) and the contact
//! between the links of the robot (capsules from `<collision>`).
//!
//! Speed and acceleration are finite differences of the samples, so they are only as good as the sampling: a spike between two
//! samples is not seen. The check says what it did and did not do (`complete`); a caller that gates motion on it must require
//! `ok` AND `complete`. A joint without a declared limit is not checked against it and is never assumed to be within one.

use std::collections::HashMap;

use serde_json::{json, Value};

use crate::dynamics;
use crate::kinematics::effective_positions;
use crate::selfcollision;
use crate::urdf::{JointType, Model};

/// When to look for contact between the links.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SelfCollisionMode {
    Off,
    /// Only when the model has collision shapes; if they cannot be turned into capsules the answer says so and is `incomplete`.
    Auto,
    /// Always; a model whose shapes cannot be used is an error.
    On,
}

#[derive(Debug, Clone)]
pub struct Config {
    pub position_tolerance: f64,
    pub velocity_tolerance: f64,
    pub acceleration_tolerance: f64,
    /// Radians per second squared (metres for prismatic joints) by joint name.
    pub acceleration_limits: HashMap<String, f64>,
    pub default_acceleration_limit: Option<f64>,
    pub check_soft_limits: bool,
    pub check_effort: bool,
    pub gravity: [f64; 3],
    pub allow_missing_inertial: bool,
    pub self_collision: SelfCollisionMode,
    pub collision: selfcollision::Config,
    /// How many violations are listed (the total is always given).
    pub max_listed: usize,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            position_tolerance: crate::limits::LIMIT_TOLERANCE,
            velocity_tolerance: crate::limits::LIMIT_TOLERANCE,
            acceleration_tolerance: crate::limits::LIMIT_TOLERANCE,
            acceleration_limits: HashMap::new(),
            default_acceleration_limit: None,
            check_soft_limits: true,
            check_effort: false,
            gravity: dynamics::DEFAULT_GRAVITY,
            allow_missing_inertial: false,
            self_collision: SelfCollisionMode::Auto,
            collision: selfcollision::Config::default(),
            max_listed: 64,
        }
    }
}

const CONFIG_FIELDS: [&str; 12] = [
    "position_tolerance",
    "velocity_tolerance",
    "acceleration_tolerance",
    "acceleration_limits",
    "default_acceleration_limit",
    "check_soft_limits",
    "check_effort",
    "gravity",
    "allow_missing_inertial",
    "self_collision",
    "collision",
    "max_listed",
];

fn non_negative(value: &Value, what: &str) -> Result<f64, String> {
    value
        .as_f64()
        .filter(|n| n.is_finite() && *n >= 0.0)
        .ok_or_else(|| format!("{what} must be a finite number, not negative"))
}

fn boolean(value: &Value, what: &str) -> Result<bool, String> {
    value
        .as_bool()
        .ok_or_else(|| format!("{what} must be true or false"))
}

/// Reads the `trajectory` section of the service configuration; every field is optional and an unknown field is an error.
pub fn parse_config(raw: &Value) -> Result<Config, String> {
    let object = raw
        .as_object()
        .ok_or("the trajectory configuration must be a JSON object")?;
    let mut unknown: Vec<&str> = object
        .keys()
        .map(String::as_str)
        .filter(|k| !CONFIG_FIELDS.contains(k))
        .collect();
    if !unknown.is_empty() {
        unknown.sort_unstable();
        return Err(format!(
            "the trajectory configuration has unknown field(s): {}",
            unknown.join(", ")
        ));
    }
    let mut config = Config::default();
    if let Some(v) = object.get("position_tolerance") {
        config.position_tolerance = non_negative(v, "position_tolerance")?;
    }
    if let Some(v) = object.get("velocity_tolerance") {
        config.velocity_tolerance = non_negative(v, "velocity_tolerance")?;
    }
    if let Some(v) = object.get("acceleration_tolerance") {
        config.acceleration_tolerance = non_negative(v, "acceleration_tolerance")?;
    }
    if let Some(v) = object.get("acceleration_limits") {
        let map = v
            .as_object()
            .ok_or("acceleration_limits must be an object of joint name to limit")?;
        for (name, limit) in map {
            let limit = non_negative(limit, &format!("acceleration_limits.{name}"))?;
            config.acceleration_limits.insert(name.clone(), limit);
        }
    }
    if let Some(v) = object.get("default_acceleration_limit") {
        config.default_acceleration_limit = if v.is_null() {
            None
        } else {
            Some(non_negative(v, "default_acceleration_limit")?)
        };
    }
    if let Some(v) = object.get("check_soft_limits") {
        config.check_soft_limits = boolean(v, "check_soft_limits")?;
    }
    if let Some(v) = object.get("check_effort") {
        config.check_effort = boolean(v, "check_effort")?;
    }
    if let Some(v) = object.get("allow_missing_inertial") {
        config.allow_missing_inertial = boolean(v, "allow_missing_inertial")?;
    }
    if let Some(v) = object.get("gravity") {
        let list = v
            .as_array()
            .filter(|l| l.len() == 3)
            .ok_or("gravity must be [x, y, z] in metres per second squared")?;
        for (slot, item) in config.gravity.iter_mut().zip(list) {
            *slot = item
                .as_f64()
                .filter(|n| n.is_finite())
                .ok_or("gravity must be three finite numbers")?;
        }
    }
    if let Some(v) = object.get("self_collision") {
        config.self_collision = match v.as_str() {
            Some("off") => SelfCollisionMode::Off,
            Some("auto") => SelfCollisionMode::Auto,
            Some("on") => SelfCollisionMode::On,
            _ => return Err("self_collision must be \"off\", \"auto\" or \"on\"".into()),
        };
    }
    if let Some(v) = object.get("collision") {
        config.collision = selfcollision::parse_config(v)?;
    }
    if let Some(v) = object.get("max_listed") {
        config.max_listed =
            v.as_u64()
                .filter(|n| (1..=10_000).contains(n))
                .ok_or("max_listed must be a whole number from 1 to 10000")? as usize;
    }
    Ok(config)
}

#[derive(Debug, Clone, PartialEq)]
pub struct Sample {
    pub t: f64,
    pub joints: HashMap<String, f64>,
}

/// `{"samples": [{"t": 0.0, "joints": {"name": position}}, ...]}`, time in seconds and strictly increasing.
pub fn parse_samples(raw: &Value, max_samples: usize) -> Result<Vec<Sample>, String> {
    let list = raw
        .get("samples")
        .and_then(Value::as_array)
        .ok_or("the request needs a \"samples\" list")?;
    if list.is_empty() {
        return Err("the trajectory has no samples".into());
    }
    if list.len() > max_samples {
        return Err(format!(
            "the trajectory has more than {max_samples} samples"
        ));
    }
    let mut out: Vec<Sample> = Vec::with_capacity(list.len());
    for (i, item) in list.iter().enumerate() {
        let t = item
            .get("t")
            .and_then(Value::as_f64)
            .filter(|t| t.is_finite())
            .ok_or_else(|| format!("sample {i} needs a finite \"t\" in seconds"))?;
        if let Some(previous) = out.last() {
            if t <= previous.t {
                return Err(format!(
                    "the time of sample {i} must be above that of sample {}",
                    i - 1
                ));
            }
        }
        let object = item
            .get("joints")
            .and_then(Value::as_object)
            .ok_or_else(|| {
                format!("sample {i} needs a \"joints\" object of joint name to position")
            })?;
        let mut joints = HashMap::new();
        for (name, value) in object {
            let v = value
                .as_f64()
                .filter(|v| v.is_finite())
                .ok_or_else(|| format!("sample {i} joint {name} must be a finite number"))?;
            joints.insert(name.clone(), v);
        }
        out.push(Sample { t, joints });
    }
    Ok(out)
}

#[derive(Debug, Clone, PartialEq)]
pub struct Violation {
    /// "position", "soft_position", "velocity", "acceleration" or "effort"
    pub kind: &'static str,
    pub joint: String,
    pub sample: usize,
    pub t: f64,
    pub value: f64,
    /// The bound that was broken (the nearer one, for a position).
    pub limit: f64,
}

#[derive(Debug, Clone)]
pub struct Outcome {
    pub ok: bool,
    pub complete: bool,
    pub samples: usize,
    pub violations_total: usize,
    pub violations: Vec<Violation>,
    pub max_velocity: HashMap<String, f64>,
    pub max_acceleration: HashMap<String, f64>,
    /// The report of the contact check, or why it was not made.
    pub self_collision: Value,
    pub notes: Vec<String>,
}

impl Outcome {
    pub fn to_json(&self) -> Value {
        let list: Vec<Value> = self
            .violations
            .iter()
            .map(|v| {
                json!({"kind": v.kind, "joint": v.joint, "sample": v.sample, "t": v.t, "value": v.value, "limit": v.limit})
            })
            .collect();
        let sorted = |m: &HashMap<String, f64>| {
            let mut pairs: Vec<(&String, &f64)> = m.iter().collect();
            pairs.sort_by(|a, b| a.0.cmp(b.0));
            Value::Object(
                pairs
                    .into_iter()
                    .map(|(k, v)| (k.clone(), json!(v)))
                    .collect(),
            )
        };
        json!({
            "ok": self.ok,
            "complete": self.complete,
            "samples": self.samples,
            "violations_total": self.violations_total,
            "violations": list,
            "max_velocity": sorted(&self.max_velocity),
            "max_acceleration": sorted(&self.max_acceleration),
            "self_collision": self.self_collision,
            "notes": self.notes,
            "model": "finite differences of the samples against the limits of the URDF and the configuration; an estimate, not tried with a real robot",
        })
    }
}

/// Checks the samples. An error means the request itself is wrong (an unknown joint, a missing position); violations are an answer.
pub fn check(model: &Model, config: &Config, samples: &[Sample]) -> Result<Outcome, String> {
    let joints = &model.chain.joints;
    let n = samples.len();
    let mut eff: Vec<HashMap<String, f64>> = Vec::with_capacity(n);
    for (i, s) in samples.iter().enumerate() {
        for name in s.joints.keys() {
            if model.joint_index(name).is_none() {
                return Err(format!("sample {i}: unknown joint {name:?}"));
            }
        }
        eff.push(effective_positions(model, &s.joints).map_err(|e| format!("sample {i}: {e}"))?);
    }
    let mut total = 0usize;
    let mut listed: Vec<Violation> = Vec::new();
    let mut record = |v: Violation| {
        total += 1;
        if listed.len() < config.max_listed {
            listed.push(v);
        }
    };
    let moving: Vec<usize> = (0..joints.len())
        .filter(|&i| joints[i].joint_type != JointType::Fixed)
        .collect();

    // positions
    for (k, s) in samples.iter().enumerate() {
        for &i in &moving {
            let name = &joints[i].name;
            let value = eff[k][name];
            let mut check_range = |kind: &'static str, lo: f64, hi: f64| {
                let tol = config.position_tolerance;
                if value < lo - tol {
                    record(Violation {
                        kind,
                        joint: name.clone(),
                        sample: k,
                        t: s.t,
                        value,
                        limit: lo,
                    });
                } else if value > hi + tol {
                    record(Violation {
                        kind,
                        joint: name.clone(),
                        sample: k,
                        t: s.t,
                        value,
                        limit: hi,
                    });
                }
            };
            if let Some((lo, hi)) = joints[i].limit {
                check_range("position", lo, hi);
            }
            if config.check_soft_limits {
                if let Some(soft) = model.extras[i].soft {
                    check_range("soft_position", soft.lower, soft.upper);
                }
            }
        }
    }

    // speeds on the intervals, accelerations at the interior samples
    let mut max_velocity: HashMap<String, f64> = HashMap::new();
    let mut max_acceleration: HashMap<String, f64> = HashMap::new();
    let mut velocity: Vec<HashMap<String, f64>> = Vec::new();
    for k in 0..n.saturating_sub(1) {
        let dt = samples[k + 1].t - samples[k].t;
        let mut v = HashMap::new();
        for &i in &moving {
            let name = &joints[i].name;
            let rate = (eff[k + 1][name] - eff[k][name]) / dt;
            let slot = max_velocity.entry(name.clone()).or_insert(0.0);
            *slot = slot.max(rate.abs());
            if let Some(limit) = model.extras[i].velocity_limit {
                if rate.abs() > limit + config.velocity_tolerance {
                    record(Violation {
                        kind: "velocity",
                        joint: name.clone(),
                        sample: k + 1,
                        t: samples[k + 1].t,
                        value: rate,
                        limit,
                    });
                }
            }
            v.insert(name.clone(), rate);
        }
        velocity.push(v);
    }
    // acceleration at sample k (1 <= k <= n-2) from the two intervals around it
    let mut accel_at: Vec<Option<HashMap<String, f64>>> = vec![None; n];
    for k in 1..n.saturating_sub(1) {
        let span = (samples[k + 1].t - samples[k - 1].t) / 2.0;
        let mut a = HashMap::new();
        for &i in &moving {
            let name = &joints[i].name;
            let rate = (velocity[k][name] - velocity[k - 1][name]) / span;
            let slot = max_acceleration.entry(name.clone()).or_insert(0.0);
            *slot = slot.max(rate.abs());
            let limit = config
                .acceleration_limits
                .get(name)
                .copied()
                .or(config.default_acceleration_limit);
            if let Some(limit) = limit {
                if rate.abs() > limit + config.acceleration_tolerance {
                    record(Violation {
                        kind: "acceleration",
                        joint: name.clone(),
                        sample: k,
                        t: samples[k].t,
                        value: rate,
                        limit,
                    });
                }
            }
            a.insert(name.clone(), rate);
        }
        accel_at[k] = Some(a);
    }

    let mut complete = true;
    let mut notes = Vec::new();

    // torques at the interior samples
    if config.check_effort {
        if n < 3 {
            complete = false;
            notes.push("the effort was not checked: it needs at least three samples".to_string());
        } else {
            let options = dynamics::Options {
                gravity: config.gravity,
                allow_missing_inertial: config.allow_missing_inertial,
            };
            let free: Vec<&str> = moving
                .iter()
                .filter(|&&i| model.extras[i].mimic.is_none())
                .map(|&i| joints[i].name.as_str())
                .collect();
            for k in 1..n - 1 {
                let accel = accel_at[k].as_ref().expect("interior sample");
                // the speed at a sample: the mean of the two intervals around it, weighted by their length
                let (d0, d1) = (
                    samples[k].t - samples[k - 1].t,
                    samples[k + 1].t - samples[k].t,
                );
                let pick = |m: &HashMap<String, f64>| -> HashMap<String, f64> {
                    free.iter().map(|n| (n.to_string(), m[*n])).collect()
                };
                let v: HashMap<String, f64> = free
                    .iter()
                    .map(|n| {
                        (
                            n.to_string(),
                            (velocity[k - 1][*n] * d1 + velocity[k][*n] * d0) / (d0 + d1),
                        )
                    })
                    .collect();
                let torques = dynamics::inverse_dynamics(
                    model,
                    &samples[k].joints,
                    &v,
                    &pick(accel),
                    &options,
                )
                .map_err(|e| format!("effort at sample {k}: {e}"))?;
                if !torques.without_inertial.is_empty() {
                    let note = format!(
                        "the effort counts link(s) {} as massless: it is under-estimated",
                        torques.without_inertial.join(", ")
                    );
                    if !notes.contains(&note) {
                        notes.push(note);
                    }
                    complete = false;
                }
                for violation in crate::limits::effort_violations(model, &torques.torques) {
                    record(Violation {
                        kind: "effort",
                        joint: violation.joint,
                        sample: k,
                        t: samples[k].t,
                        value: violation.value,
                        limit: violation.limit,
                    });
                }
            }
        }
    }

    // contact between the links
    let states: Vec<HashMap<String, f64>> = samples.iter().map(|s| s.joints.clone()).collect();
    let has_shapes = model.links.values().any(|l| !l.collisions.is_empty());
    let mut self_collision = json!({"checked": false, "reason": "off"});
    let mut contact = false;
    match config.self_collision {
        SelfCollisionMode::Off => {
            complete = false;
            notes.push("the contact between the links was not checked: it is switched off".into());
        }
        SelfCollisionMode::Auto if !has_shapes => {
            complete = false;
            self_collision =
                json!({"checked": false, "reason": "the URDF has no <collision> shapes"});
            notes.push(
                "the contact between the links was not checked: the URDF has no <collision> shapes"
                    .into(),
            );
        }
        mode => match selfcollision::check_states(model, &config.collision, &states) {
            Ok(report) => {
                contact = report.collision();
                let mut j = report.to_json();
                j["checked"] = json!(true);
                self_collision = j;
            }
            Err(e) if mode == SelfCollisionMode::On => return Err(format!("self collision: {e}")),
            Err(e) => {
                complete = false;
                self_collision = json!({"checked": false, "reason": e});
                notes.push(format!(
                    "the contact between the links was not checked: {e}"
                ));
            }
        },
    }

    Ok(Outcome {
        ok: total == 0 && !contact,
        complete,
        samples: n,
        violations_total: total,
        violations: listed,
        max_velocity,
        max_acceleration,
        self_collision,
        notes,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::urdf::parse_urdf_model;

    const ARM: &str = r#"<robot name="t">
      <link name="base"/>
      <link name="rod"><inertial><origin xyz="0.4 0 0"/><mass value="2"/>
        <inertia ixx="0" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0"/></inertial></link>
      <joint name="j" type="revolute"><parent link="base"/><child link="rod"/>
        <axis xyz="0 -1 0"/><limit lower="-1" upper="1" effort="10" velocity="2"/>
        <safety_controller soft_lower_limit="-0.9" soft_upper_limit="0.9" k_position="1" k_velocity="1"/></joint>
    </robot>"#;

    fn samples(points: &[(f64, f64)]) -> Vec<Sample> {
        points
            .iter()
            .map(|(t, q)| Sample {
                t: *t,
                joints: HashMap::from([("j".to_string(), *q)]),
            })
            .collect()
    }

    fn quiet() -> Config {
        Config {
            self_collision: SelfCollisionMode::Off,
            ..Config::default()
        }
    }

    #[test]
    fn a_gentle_trajectory_inside_the_limits_passes() {
        let model = parse_urdf_model(ARM).unwrap();
        let s = samples(&[(0.0, 0.0), (0.1, 0.05), (0.2, 0.15), (0.3, 0.3)]);
        let out = check(&model, &quiet(), &s).unwrap();
        assert!(out.ok, "{:?}", out.violations);
        assert!(out.max_velocity["j"] > 2.0 * 0.5 - 1e-9 && out.max_velocity["j"] < 2.0);
        // switched off means incomplete, never a silent pass
        assert!(!out.complete);
    }

    #[test]
    fn position_soft_velocity_and_acceleration_violations_are_found() {
        let model = parse_urdf_model(ARM).unwrap();
        // 0.95 is past the soft limit; 1.2 past the hard one
        let out = check(
            &model,
            &quiet(),
            &samples(&[(0.0, 0.0), (1.0, 0.95), (2.0, 1.2)]),
        )
        .unwrap();
        assert!(out
            .violations
            .iter()
            .any(|v| v.kind == "soft_position" && v.sample == 1));
        assert!(out
            .violations
            .iter()
            .any(|v| v.kind == "position" && v.sample == 2 && v.limit == 1.0));
        assert!(!out.ok);
        // 0.5 rad in 0.1 s is 5 rad/s against 2
        let fast = check(&model, &quiet(), &samples(&[(0.0, 0.0), (0.1, 0.5)])).unwrap();
        let v = fast
            .violations
            .iter()
            .find(|v| v.kind == "velocity")
            .unwrap();
        assert!((v.value - 5.0).abs() < 1e-9 && v.limit == 2.0);
        // acceleration: needs a limit from the configuration
        let mut config = quiet();
        config.default_acceleration_limit = Some(10.0);
        let jerk = samples(&[(0.0, 0.0), (0.1, 0.0), (0.2, 0.2)]);
        let out = check(&model, &config, &jerk).unwrap();
        let a = out
            .violations
            .iter()
            .find(|v| v.kind == "acceleration")
            .unwrap();
        // the speed goes from 0 to 2 rad/s over 0.1 s: 20 rad/s^2 at the middle sample, against 10
        assert_eq!(a.sample, 1);
        assert!(a.value > 0.0);
        config.acceleration_limits.insert("j".into(), 1000.0);
        assert!(check(&model, &config, &jerk).unwrap().ok);
    }

    #[test]
    fn the_tolerance_of_the_configuration_decides_the_edge() {
        let model = parse_urdf_model(ARM).unwrap();
        let s = samples(&[(0.0, 0.0), (1.0, 1.0 + 1e-6)]);
        let mut strict = quiet();
        strict.check_soft_limits = false;
        assert!(!check(&model, &strict, &s).unwrap().ok);
        let mut config = strict.clone();
        config.position_tolerance = 1e-5;
        assert!(check(&model, &config, &s).unwrap().ok);
    }

    #[test]
    fn effort_is_compared_with_the_limit_when_asked() {
        let model = parse_urdf_model(ARM).unwrap();
        // holding a 2 kg rod 0.4 m out against gravity needs 7.8 N m at the horizontal: under the 10 N m limit
        let still = samples(&[(0.0, 0.0), (0.1, 0.0), (0.2, 0.0)]);
        let mut config = quiet();
        config.check_effort = true;
        let out = check(&model, &config, &still).unwrap();
        assert!(out.ok, "{:?}", out.violations);
        // adding acceleration pushes it over
        let accel = samples(&[(0.0, 0.0), (0.1, 0.0), (0.2, 0.2), (0.3, 0.6)]);
        let out = check(&model, &config, &accel).unwrap();
        assert!(
            out.violations.iter().any(|v| v.kind == "effort"),
            "{:?}",
            out.violations
        );
        // two samples cannot be checked for effort: incomplete
        let two = check(&model, &config, &samples(&[(0.0, 0.0), (0.1, 0.0)])).unwrap();
        assert!(!two.complete);
    }

    #[test]
    fn contact_is_found_through_the_capsules_and_reported_incomplete_without_shapes() {
        let model = parse_urdf_model(ARM).unwrap();
        let auto = Config::default();
        let out = check(&model, &auto, &samples(&[(0.0, 0.0), (1.0, 0.1)])).unwrap();
        assert!(!out.complete);
        assert_eq!(out.self_collision["checked"], json!(false));
        // base (node 0) and arm (node 2) are two joints apart; the arm swings on a circle of 0.2 m around the base
        let robot = |base_shape: &str, arm_radius: f64| {
            format!(
                r#"<robot name="c"><link name="base"><collision><geometry>{base_shape}</geometry></collision></link>
                <link name="mid"/>
                <link name="arm"><collision><origin xyz="0.2 0 0"/><geometry><sphere radius="{arm_radius}"/></geometry></collision></link>
                <joint name="j" type="revolute"><parent link="base"/><child link="mid"/><axis xyz="0 0 1"/>
                  <limit lower="-4" upper="4" effort="1" velocity="9"/></joint>
                <joint name="k" type="fixed"><parent link="mid"/><child link="arm"/></joint></robot>"#
            )
        };
        let sphere = "<sphere radius=\"0.05\"/>";
        let at = |q: f64, t: f64| Sample {
            t,
            joints: HashMap::from([("j".to_string(), q)]),
        };
        let apart = parse_urdf_model(&robot(sphere, 0.05)).unwrap();
        let out = check(&apart, &auto, &[at(0.0, 0.0), at(1.0, 2.0)]).unwrap();
        assert!(out.ok && out.self_collision["checked"] == json!(true));
        let touching = parse_urdf_model(&robot(sphere, 0.16)).unwrap();
        let out = check(&touching, &auto, &[at(0.0, 0.0), at(1.0, 2.0)]).unwrap();
        assert!(!out.ok);
        assert_eq!(out.self_collision["collision"], json!(true));
        // a mesh without a declared radius: auto reports incomplete, on is an error
        let mesh = parse_urdf_model(&robot("<mesh filename=\"a.stl\"/>", 0.05)).unwrap();
        let out = check(&mesh, &auto, &[at(0.0, 0.0)]).unwrap();
        assert!(!out.complete);
        assert!(out.notes.iter().any(|n| n.contains("mesh")));
        let on = Config {
            self_collision: SelfCollisionMode::On,
            ..Config::default()
        };
        assert!(check(&mesh, &on, &[at(0.0, 0.0)]).is_err());
    }

    #[test]
    fn bad_samples_are_errors() {
        assert!(parse_samples(&json!({}), 10).is_err());
        assert!(parse_samples(&json!({"samples": []}), 10).is_err());
        let two = json!({"samples": [{"t": 0, "joints": {"j": 0}}, {"t": 0, "joints": {"j": 0}}]});
        assert!(parse_samples(&two, 10).unwrap_err().contains("above"));
        assert!(parse_samples(&two, 1).unwrap_err().contains("more than"));
        assert!(parse_samples(&json!({"samples": [{"t": 0, "joints": {"j": "x"}}]}), 10).is_err());
        let model = parse_urdf_model(ARM).unwrap();
        let unknown = vec![Sample {
            t: 0.0,
            joints: HashMap::from([("nope".to_string(), 0.0)]),
        }];
        assert!(check(&model, &quiet(), &unknown).is_err());
        let missing = vec![Sample {
            t: 0.0,
            joints: HashMap::new(),
        }];
        assert!(check(&model, &quiet(), &missing).is_err());
    }

    #[test]
    fn the_configuration_is_strict() {
        let c = parse_config(&json!({"self_collision": "on", "gravity": [0, 0, -1.6], "acceleration_limits": {"j": 3}, "max_listed": 5})).unwrap();
        assert_eq!(c.self_collision, SelfCollisionMode::On);
        assert_eq!(c.gravity[2], -1.6);
        assert_eq!(c.acceleration_limits["j"], 3.0);
        assert!(parse_config(&json!({"self_collision": "yes"})).is_err());
        assert!(parse_config(&json!({"velocity_tolerance": -1})).is_err());
        assert!(parse_config(&json!({"gravity": [1, 2]})).is_err());
        assert!(parse_config(&json!({"surprise": 1})).is_err());
    }
}
