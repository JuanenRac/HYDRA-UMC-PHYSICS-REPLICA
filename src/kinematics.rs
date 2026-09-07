// =============================================================================
// HYDRA-UMC-PHYSICS-REPLICA - src/kinematics.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0 - see LICENSE
// =============================================================================
//! Real forward kinematics over a `Chain` (see `urdf.rs`): given a joint
//! position for every non-fixed joint, computes each joint's world-frame
//! transform by chaining `origin * joint_motion` down the serial chain.

use std::collections::HashMap;

use crate::limits::{validate_limits, LimitViolation};
use crate::transform::Mat4;
use crate::urdf::{Chain, Joint, JointType};

#[derive(Debug, PartialEq)]
pub enum KinematicsError {
    MissingJointPosition(String),
    NonFiniteJointPosition(String),
}

impl std::fmt::Display for KinematicsError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            KinematicsError::MissingJointPosition(name) => {
                write!(f, "no position given for non-fixed joint '{name}'")
            }
            KinematicsError::NonFiniteJointPosition(name) => {
                write!(f, "non-finite position given for joint '{name}'")
            }
        }
    }
}

#[derive(Debug, PartialEq)]
pub enum CheckedKinematicsError {
    /// Same failures `forward_kinematics` itself can produce.
    Kinematics(KinematicsError),
    /// At least one given joint position is outside its declared URDF
    /// limit - the requested configuration is not physically reachable,
    /// so no world-frame transform is computed for it at all.
    LimitViolation(Vec<LimitViolation>),
}

impl std::fmt::Display for CheckedKinematicsError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            CheckedKinematicsError::Kinematics(e) => write!(f, "{e}"),
            CheckedKinematicsError::LimitViolation(violations) => {
                write!(f, "{} joint limit violation(s)", violations.len())
            }
        }
    }
}

fn joint_motion(joint: &Joint, position: f64) -> Mat4 {
    match joint.joint_type {
        JointType::Fixed => Mat4::identity(),
        JointType::Revolute | JointType::Continuous => {
            Mat4::rotation_axis_angle(joint.axis, position)
        }
        JointType::Prismatic => {
            let a = joint.axis.normalized();
            Mat4::translation(crate::transform::Vec3::new(
                a.x * position,
                a.y * position,
                a.z * position,
            ))
        }
    }
}

/// Returns each joint's name paired with its resulting world-frame
/// transform, in chain order. `positions` must contain an entry for every
/// non-`fixed` joint by name; a `fixed` joint's position, if given, is
/// ignored (it has none in the physical robot either).
pub fn forward_kinematics(
    chain: &Chain,
    positions: &HashMap<String, f64>,
) -> Result<Vec<(String, Mat4)>, KinematicsError> {
    let mut world = Mat4::identity();
    let mut out = Vec::with_capacity(chain.joints.len());

    for joint in &chain.joints {
        let position = if joint.joint_type == JointType::Fixed {
            0.0
        } else {
            *positions
                .get(&joint.name)
                .ok_or_else(|| KinematicsError::MissingJointPosition(joint.name.clone()))?
        };
        if !position.is_finite() {
            return Err(KinematicsError::NonFiniteJointPosition(joint.name.clone()));
        }
        world = world.mul(&joint.origin).mul(&joint_motion(joint, position));
        out.push((joint.name.clone(), world));
    }

    Ok(out)
}

/// The fail-safe counterpart to `forward_kinematics`: checks every given
/// joint position against its declared URDF limit FIRST, and refuses to
/// compute a world-frame transform at all if any position is out of
/// range - a limit violation "wins" over the math, the same
/// detect-before-act precedence used across this ecosystem's other v0
/// safety layers (e.g. HYDRA-UMC-SAFETY-ZONES's calibration check,
/// HYDRA-UMC-VISUAL-SERVOING-API's `authorize_correction`). Plain
/// `forward_kinematics` stays available unchanged for callers that
/// genuinely want the unchecked math (e.g. exploring what pose an
/// out-of-range value WOULD produce, for tuning limits themselves).
pub fn forward_kinematics_checked(
    chain: &Chain,
    positions: &HashMap<String, f64>,
) -> Result<Vec<(String, Mat4)>, CheckedKinematicsError> {
    let violations = validate_limits(chain, positions);
    if !violations.is_empty() {
        return Err(CheckedKinematicsError::LimitViolation(violations));
    }
    forward_kinematics(chain, positions).map_err(CheckedKinematicsError::Kinematics)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transform::Vec3;
    use crate::urdf::Joint;

    fn revolute(name: &str, origin_z: f64, axis: Vec3, limit: Option<(f64, f64)>) -> Joint {
        Joint {
            name: name.to_string(),
            joint_type: JointType::Revolute,
            origin: Mat4::translation(Vec3::new(0.0, 0.0, origin_z)),
            axis,
            limit,
        }
    }

    #[test]
    fn single_joint_at_zero_matches_its_origin() {
        let chain = Chain {
            joints: vec![revolute("j1", 1.0, Vec3::new(0.0, 0.0, 1.0), None)],
        };
        let mut positions = HashMap::new();
        positions.insert("j1".to_string(), 0.0);
        let result = forward_kinematics(&chain, &positions).unwrap();
        assert_eq!(result.len(), 1);
        assert_eq!(result[0].1.translation_part(), Vec3::new(0.0, 0.0, 1.0));
    }

    #[test]
    fn two_link_chain_composes_transforms() {
        let chain = Chain {
            joints: vec![
                revolute("j1", 1.0, Vec3::new(0.0, 0.0, 1.0), None),
                Joint {
                    name: "j2".to_string(),
                    joint_type: JointType::Fixed,
                    origin: Mat4::translation(Vec3::new(1.0, 0.0, 0.0)),
                    axis: Vec3::new(1.0, 0.0, 0.0),
                    limit: None,
                },
            ],
        };
        let mut positions = HashMap::new();
        positions.insert("j1".to_string(), 0.0);
        let result = forward_kinematics(&chain, &positions).unwrap();
        // j1 origin puts us at z=1; j2's fixed origin then offsets x by 1.
        assert_eq!(result[1].1.translation_part(), Vec3::new(1.0, 0.0, 1.0));
    }

    #[test]
    fn missing_position_for_non_fixed_joint_is_an_error() {
        let chain = Chain {
            joints: vec![revolute("j1", 0.0, Vec3::new(0.0, 0.0, 1.0), None)],
        };
        let result = forward_kinematics(&chain, &HashMap::new());
        assert_eq!(
            result,
            Err(KinematicsError::MissingJointPosition("j1".to_string()))
        );
    }

    #[test]
    fn non_finite_position_is_refused_before_transform_math() {
        let chain = Chain {
            joints: vec![revolute("j1", 0.0, Vec3::new(0.0, 0.0, 1.0), None)],
        };
        let mut positions = HashMap::new();
        positions.insert("j1".to_string(), f64::NAN);
        assert_eq!(
            forward_kinematics(&chain, &positions),
            Err(KinematicsError::NonFiniteJointPosition("j1".to_string()))
        );
    }

    #[test]
    fn fixed_joint_needs_no_position() {
        let chain = Chain {
            joints: vec![Joint {
                name: "fx".to_string(),
                joint_type: JointType::Fixed,
                origin: Mat4::translation(Vec3::new(0.0, 0.0, 2.0)),
                axis: Vec3::new(1.0, 0.0, 0.0),
                limit: None,
            }],
        };
        let result = forward_kinematics(&chain, &HashMap::new()).unwrap();
        assert_eq!(result[0].1.translation_part(), Vec3::new(0.0, 0.0, 2.0));
    }

    #[test]
    fn prismatic_joint_translates_along_axis() {
        let chain = Chain {
            joints: vec![Joint {
                name: "p1".to_string(),
                joint_type: JointType::Prismatic,
                origin: Mat4::identity(),
                axis: Vec3::new(0.0, 0.0, 1.0),
                limit: None,
            }],
        };
        let mut positions = HashMap::new();
        positions.insert("p1".to_string(), 0.5);
        let result = forward_kinematics(&chain, &positions).unwrap();
        assert_eq!(result[0].1.translation_part(), Vec3::new(0.0, 0.0, 0.5));
    }

    // -- forward_kinematics_checked: regression corpus, out-of-range inputs --

    mod checked_regressions {
        use super::*;
        use crate::corpus;

        #[test]
        fn within_range_position_computes_a_real_pose() {
            let chain = corpus::single_joint_chain(corpus::revolute_with_limit("j1", -1.0, 1.0));
            let mut positions = HashMap::new();
            positions.insert("j1".to_string(), 0.5);
            assert!(forward_kinematics_checked(&chain, &positions).is_ok());
        }

        #[test]
        fn exactly_at_the_upper_boundary_computes_a_real_pose() {
            // Boundary: the limit's own edge counts as reachable, not violating.
            let chain = corpus::single_joint_chain(corpus::revolute_with_limit("j1", -1.0, 1.0));
            let mut positions = HashMap::new();
            positions.insert("j1".to_string(), 1.0);
            assert!(forward_kinematics_checked(&chain, &positions).is_ok());
        }

        #[test]
        fn exactly_at_the_lower_boundary_computes_a_real_pose() {
            let chain = corpus::single_joint_chain(corpus::revolute_with_limit("j1", -1.0, 1.0));
            let mut positions = HashMap::new();
            positions.insert("j1".to_string(), -1.0);
            assert!(forward_kinematics_checked(&chain, &positions).is_ok());
        }

        #[test]
        fn one_unit_past_the_upper_boundary_is_refused() {
            let chain = corpus::single_joint_chain(corpus::revolute_with_limit("j1", -1.0, 1.0));
            let mut positions = HashMap::new();
            positions.insert("j1".to_string(), 1.000_001);
            let err = forward_kinematics_checked(&chain, &positions).unwrap_err();
            match err {
                CheckedKinematicsError::LimitViolation(v) => assert_eq!(v.len(), 1),
                other => panic!("expected LimitViolation, got {other:?}"),
            }
        }

        #[test]
        fn wildly_out_of_range_prismatic_input_is_refused_not_silently_computed() {
            // The real gap this closes: forward_kinematics() alone would
            // happily compute (and a caller might trust) a pose for a
            // prismatic joint extended 1000 units past its declared
            // travel - physically meaningless for a real robot.
            let chain = corpus::single_joint_chain(corpus::prismatic_with_limit("rail", 0.0, 0.5));
            let mut positions = HashMap::new();
            positions.insert("rail".to_string(), 1000.0);
            assert!(matches!(
                forward_kinematics_checked(&chain, &positions),
                Err(CheckedKinematicsError::LimitViolation(_))
            ));
            // The unchecked function, by contrast, computes it anyway -
            // documenting exactly the gap `_checked` closes.
            assert!(forward_kinematics(&chain, &positions).is_ok());
        }

        #[test]
        fn fixed_joint_in_a_mixed_chain_is_never_flagged() {
            let chain = Chain {
                joints: vec![
                    corpus::revolute_with_limit("j1", -1.0, 1.0),
                    corpus::fixed("mount"),
                ],
            };
            let mut positions = HashMap::new();
            positions.insert("j1".to_string(), 0.5);
            // "mount" needs no entry - it's fixed.
            assert!(forward_kinematics_checked(&chain, &positions).is_ok());
        }

        #[test]
        fn continuous_joint_has_no_limit_to_violate() {
            let chain = corpus::single_joint_chain(corpus::continuous_unlimited("spin"));
            let mut positions = HashMap::new();
            positions.insert("spin".to_string(), 1_000_000.0);
            assert!(forward_kinematics_checked(&chain, &positions).is_ok());
        }

        #[test]
        fn multi_joint_chain_reports_every_violating_joint() {
            let chain = corpus::shoulder_elbow_chain();
            let mut positions = HashMap::new();
            positions.insert("shoulder".to_string(), 0.0); // within [-pi, pi]
            positions.insert("elbow".to_string(), 5.0); // outside [-2.0, 2.0]
            let err = forward_kinematics_checked(&chain, &positions).unwrap_err();
            match err {
                CheckedKinematicsError::LimitViolation(v) => {
                    assert_eq!(v.len(), 1);
                    assert_eq!(v[0].joint, "elbow");
                }
                other => panic!("expected LimitViolation, got {other:?}"),
            }
        }

        #[test]
        fn limit_violation_is_reported_before_a_missing_position_error() {
            // A chain with an out-of-range joint AND a genuinely missing
            // position for another joint - the limit violation (a real
            // safety concern) must be what's reported, not silently
            // superseded by the unrelated missing-input error.
            let chain = corpus::shoulder_elbow_chain();
            let mut positions = HashMap::new();
            positions.insert("shoulder".to_string(), 100.0); // out of [-pi, pi]
                                                             // "elbow" deliberately omitted.
            let err = forward_kinematics_checked(&chain, &positions).unwrap_err();
            assert!(matches!(err, CheckedKinematicsError::LimitViolation(_)));
        }
    }
}
