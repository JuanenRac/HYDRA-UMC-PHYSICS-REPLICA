// =============================================================================
// HYDRA-UMC-PHYSICS-REPLICA - src/limits.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0 - see LICENSE
// =============================================================================
//! Real joint-limit validation - the "Kinematic Validation" feature this
//! project's README already advertised before any of it existed in code.
//! A joint with no `<limit>` in the URDF (e.g. `continuous`, or a
//! `revolute`/`prismatic` whose author omitted it) is never flagged: v0
//! only checks limits it was actually given, honestly.

use std::collections::HashMap;

use crate::urdf::Chain;

#[derive(Debug, Clone, PartialEq)]
pub struct LimitViolation {
    pub joint: String,
    pub value: f64,
    pub lower: f64,
    pub upper: f64,
}

pub fn validate_limits(chain: &Chain, positions: &HashMap<String, f64>) -> Vec<LimitViolation> {
    let mut violations = Vec::new();
    for joint in &chain.joints {
        let Some((lower, upper)) = joint.limit else {
            continue;
        };
        let Some(&value) = positions.get(&joint.name) else {
            continue;
        };
        // PHYS-01 (P0): urdf.rs's own parser now rejects a non-finite or
        // inverted limit at parse time, but Joint's fields are public -
        // any other caller building a Chain directly (a test, a future
        // second source) could still hand this function a NaN/infinite
        // or inverted limit. Every IEEE-754 comparison against NaN is
        // false, so `value < lower || value > upper` alone would
        // silently report NO violation no matter what value is checked
        // against it - a corrupted limit must never look like "no
        // violation"; it fails safe as a violation instead.
        if !lower.is_finite() || !upper.is_finite() || lower > upper {
            violations.push(LimitViolation {
                joint: joint.name.clone(),
                value,
                lower,
                upper,
            });
            continue;
        }
        if value < lower || value > upper {
            violations.push(LimitViolation {
                joint: joint.name.clone(),
                value,
                lower,
                upper,
            });
        }
    }
    violations
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transform::{Mat4, Vec3};
    use crate::urdf::{Joint, JointType};

    fn joint_with_limit(name: &str, lower: f64, upper: f64) -> Joint {
        Joint {
            name: name.to_string(),
            joint_type: JointType::Revolute,
            origin: Mat4::identity(),
            axis: Vec3::new(0.0, 0.0, 1.0),
            limit: Some((lower, upper)),
        }
    }

    #[test]
    fn within_limits_is_not_a_violation() {
        let chain = Chain {
            joints: vec![joint_with_limit("j1", -1.0, 1.0)],
        };
        let mut positions = HashMap::new();
        positions.insert("j1".to_string(), 0.5);
        assert!(validate_limits(&chain, &positions).is_empty());
    }

    #[test]
    fn above_upper_is_a_violation() {
        let chain = Chain {
            joints: vec![joint_with_limit("j1", -1.0, 1.0)],
        };
        let mut positions = HashMap::new();
        positions.insert("j1".to_string(), 1.5);
        let violations = validate_limits(&chain, &positions);
        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].joint, "j1");
    }

    #[test]
    fn below_lower_is_a_violation() {
        let chain = Chain {
            joints: vec![joint_with_limit("j1", -1.0, 1.0)],
        };
        let mut positions = HashMap::new();
        positions.insert("j1".to_string(), -1.5);
        assert_eq!(validate_limits(&chain, &positions).len(), 1);
    }

    #[test]
    fn joint_without_limit_is_never_flagged() {
        let chain = Chain {
            joints: vec![Joint {
                name: "j1".to_string(),
                joint_type: JointType::Continuous,
                origin: Mat4::identity(),
                axis: Vec3::new(0.0, 0.0, 1.0),
                limit: None,
            }],
        };
        let mut positions = HashMap::new();
        positions.insert("j1".to_string(), 1000.0);
        assert!(validate_limits(&chain, &positions).is_empty());
    }

    #[test]
    fn boundary_values_are_not_violations() {
        let chain = Chain {
            joints: vec![joint_with_limit("j1", -1.0, 1.0)],
        };
        let mut positions = HashMap::new();
        positions.insert("j1".to_string(), 1.0);
        assert!(validate_limits(&chain, &positions).is_empty());
    }

    // PHYS-01 (P0): urdf.rs's own parser now rejects a non-finite limit at parse
    // time, but Joint's fields are public - this defense-in-depth check
    // covers a Chain built directly (bypassing the parser entirely, as
    // this test itself does), the same real gap the review's probe
    // exercised. Every IEEE-754 comparison against NaN is false, so
    // `value < lower || value > upper` alone would otherwise report NO
    // violation no matter what value is checked - it must fail safe.
    #[test]
    fn a_nan_limit_is_always_a_violation_regardless_of_value() {
        let chain = Chain {
            joints: vec![joint_with_limit("j1", f64::NAN, 1.0)],
        };
        let mut positions = HashMap::new();
        positions.insert("j1".to_string(), 0.0); // well within [−1, 1] if the limit were real
        let violations = validate_limits(&chain, &positions);
        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].joint, "j1");
    }

    #[test]
    fn an_inverted_limit_is_always_a_violation_regardless_of_value() {
        let chain = Chain {
            joints: vec![joint_with_limit("j1", 1.0, -1.0)],
        };
        let mut positions = HashMap::new();
        positions.insert("j1".to_string(), 0.0);
        let violations = validate_limits(&chain, &positions);
        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].joint, "j1");
    }
}
