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
}
