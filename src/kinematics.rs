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
use crate::urdf::{Chain, Joint, JointType, Model};

#[derive(Debug, PartialEq)]
pub enum KinematicsError {
    MissingJointPosition(String),
    NonFiniteJointPosition(String),
    /// A position was given for a joint that follows another (`<mimic>`): its position is derived, never chosen.
    MimicPositionGiven(String),
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
            KinematicsError::MimicPositionGiven(name) => write!(
                f,
                "a position was given for joint '{name}', which follows another joint (<mimic>): its position is derived from that joint"
            ),
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
    /// A revolute or prismatic joint has no `<limit>`: its position cannot be
    /// checked, and a checked pose for it would be a claim nothing supports.
    MissingLimits(Vec<String>),
}

impl std::fmt::Display for CheckedKinematicsError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            CheckedKinematicsError::Kinematics(e) => write!(f, "{e}"),
            CheckedKinematicsError::LimitViolation(violations) => {
                write!(f, "{} joint limit violation(s)", violations.len())
            }
            CheckedKinematicsError::MissingLimits(joints) => write!(
                f,
                "joint(s) {} have no <limit>: their positions cannot be checked",
                joints.join(", ")
            ),
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
// Part of the serial-chain API, kept for the tests and for anybody who links this crate; the command line reads trees.
#[allow(dead_code)]
pub fn forward_kinematics(
    chain: &Chain,
    positions: &HashMap<String, f64>,
) -> Result<Vec<(String, Mat4)>, KinematicsError> {
    let serial: Vec<Option<usize>> = (0..chain.joints.len()).map(|i| i.checked_sub(1)).collect();
    forward_kinematics_core(chain, &serial, positions)
}

/// The poses of every joint of a tree: each joint is placed relative to the joint that carries its parent link, so the fingers of a
/// gripper are all placed relative to the palm, not one after the other.
fn forward_kinematics_core(
    chain: &Chain,
    parents: &[Option<usize>],
    positions: &HashMap<String, f64>,
) -> Result<Vec<(String, Mat4)>, KinematicsError> {
    let mut worlds: Vec<Mat4> = Vec::with_capacity(chain.joints.len());
    let mut out = Vec::with_capacity(chain.joints.len());

    for (index, joint) in chain.joints.iter().enumerate() {
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
        let base = match parents[index] {
            Some(parent) => worlds[parent],
            None => Mat4::identity(),
        };
        let world = base.mul(&joint.origin).mul(&joint_motion(joint, position));
        worlds.push(world);
        out.push((joint.name.clone(), world));
    }

    Ok(out)
}

/// The position of every moving joint of a model: the ones given, and for each joint that follows another (`<mimic>`)
/// `multiplier * master + offset`. A position given for a follower is refused.
pub fn effective_positions(
    model: &Model,
    given: &HashMap<String, f64>,
) -> Result<HashMap<String, f64>, KinematicsError> {
    let mut effective: HashMap<String, f64> = HashMap::new();
    let mut followers: Vec<usize> = Vec::new();
    for (index, joint) in model.chain.joints.iter().enumerate() {
        if joint.joint_type == JointType::Fixed {
            continue;
        }
        if model.extras[index].mimic.is_some() {
            if given.contains_key(&joint.name) {
                return Err(KinematicsError::MimicPositionGiven(joint.name.clone()));
            }
            followers.push(index);
            continue;
        }
        let value = *given
            .get(&joint.name)
            .ok_or_else(|| KinematicsError::MissingJointPosition(joint.name.clone()))?;
        if !value.is_finite() {
            return Err(KinematicsError::NonFiniteJointPosition(joint.name.clone()));
        }
        effective.insert(joint.name.clone(), value);
    }
    // a follower may follow a follower: settle them in passes (the parser refuses circles, so each pass settles at least one)
    while !followers.is_empty() {
        let before = followers.len();
        let mut waiting = Vec::new();
        for index in followers {
            let mimic = model.extras[index]
                .mimic
                .as_ref()
                .expect("a follower has a mimic");
            match effective.get(&mimic.joint) {
                Some(master) => {
                    let value = mimic.multiplier * master + mimic.offset;
                    if !value.is_finite() {
                        return Err(KinematicsError::NonFiniteJointPosition(
                            model.chain.joints[index].name.clone(),
                        ));
                    }
                    effective.insert(model.chain.joints[index].name.clone(), value);
                }
                None => waiting.push(index),
            }
        }
        if waiting.len() == before {
            return Err(KinematicsError::MissingJointPosition(
                model.chain.joints[waiting[0]].name.clone(),
            ));
        }
        followers = waiting;
    }
    Ok(effective)
}

/// `forward_kinematics` for a whole tree, with the followers (`<mimic>`) worked out.
pub fn forward_kinematics_model(
    model: &Model,
    given: &HashMap<String, f64>,
) -> Result<Vec<(String, Mat4)>, KinematicsError> {
    let effective = effective_positions(model, given)?;
    forward_kinematics_core(&model.chain, &model.parents, &effective)
}

/// `forward_kinematics_checked` for a whole tree: the limits of every joint, the followers included, are checked first.
pub fn forward_kinematics_checked_model(
    model: &Model,
    given: &HashMap<String, f64>,
) -> Result<Vec<(String, Mat4)>, CheckedKinematicsError> {
    let missing = crate::limits::joints_without_limits(&model.chain);
    if !missing.is_empty() {
        return Err(CheckedKinematicsError::MissingLimits(missing));
    }
    let effective =
        effective_positions(model, given).map_err(CheckedKinematicsError::Kinematics)?;
    let violations = validate_limits(&model.chain, &effective);
    if !violations.is_empty() {
        return Err(CheckedKinematicsError::LimitViolation(violations));
    }
    forward_kinematics_core(&model.chain, &model.parents, &effective)
        .map_err(CheckedKinematicsError::Kinematics)
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
// Part of the serial-chain API, kept for the tests and for anybody who links this crate; the command line reads trees.
#[allow(dead_code)]
pub fn forward_kinematics_checked(
    chain: &Chain,
    positions: &HashMap<String, f64>,
) -> Result<Vec<(String, Mat4)>, CheckedKinematicsError> {
    let missing = crate::limits::joints_without_limits(chain);
    if !missing.is_empty() {
        return Err(CheckedKinematicsError::MissingLimits(missing));
    }
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

    #[test]
    fn a_checked_pose_is_refused_for_a_moving_joint_with_no_limit() {
        let chain = Chain {
            joints: vec![revolute("j1", 1.0, Vec3::new(0.0, 0.0, 1.0), None)],
        };
        let mut positions = HashMap::new();
        positions.insert("j1".to_string(), 1000.0);
        assert_eq!(
            forward_kinematics_checked(&chain, &positions).unwrap_err(),
            CheckedKinematicsError::MissingLimits(vec!["j1".to_string()])
        );
    }

    // ---- trees and followers

    const FOLLOWERS: &str = r#"<robot name="t">
        <joint name="a" type="revolute"><parent link="x"/><child link="y"/><axis xyz="0 0 1"/><limit lower="-3" upper="3"/></joint>
        <joint name="c" type="revolute"><parent link="z"/><child link="w"/><axis xyz="0 0 1"/><limit lower="-9" upper="9"/><mimic joint="b" offset="1"/></joint>
        <joint name="b" type="revolute"><parent link="y"/><child link="z"/><axis xyz="0 0 1"/><limit lower="-9" upper="9"/><mimic joint="a" multiplier="2"/></joint>
      </robot>"#;

    #[test]
    fn a_follower_of_a_follower_gets_its_position_through_both() {
        let model = crate::urdf::parse_urdf_model(FOLLOWERS).unwrap();
        let mut given = HashMap::new();
        given.insert("a".to_string(), 0.5);
        let effective = effective_positions(&model, &given).unwrap();
        assert_eq!(effective["a"], 0.5);
        assert_eq!(effective["b"], 1.0);
        assert_eq!(effective["c"], 2.0);
    }

    #[test]
    fn a_follower_given_a_position_of_its_own_is_an_error() {
        let model = crate::urdf::parse_urdf_model(FOLLOWERS).unwrap();
        let mut given = HashMap::new();
        given.insert("a".to_string(), 0.5);
        given.insert("b".to_string(), 3.0);
        assert_eq!(
            effective_positions(&model, &given),
            Err(KinematicsError::MimicPositionGiven("b".to_string()))
        );
    }

    #[test]
    fn the_master_of_a_follower_is_still_required() {
        let model = crate::urdf::parse_urdf_model(FOLLOWERS).unwrap();
        assert_eq!(
            effective_positions(&model, &HashMap::new()),
            Err(KinematicsError::MissingJointPosition("a".to_string()))
        );
    }

    #[test]
    fn a_follower_pushed_past_its_own_limit_is_refused_by_the_checked_pose() {
        let model = crate::urdf::parse_urdf_model(FOLLOWERS).unwrap();
        let mut given = HashMap::new();
        given.insert("a".to_string(), 3.0); // b = 6, c = 7: inside [-9, 9]; a itself is inside [-3, 3]
        assert!(forward_kinematics_checked_model(&model, &given).is_ok());
        given.insert("a".to_string(), 3.1);
        assert!(matches!(
            forward_kinematics_checked_model(&model, &given),
            Err(CheckedKinematicsError::LimitViolation(_))
        ));
    }
}
