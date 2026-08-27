// =============================================================================
// HYDRA-UMC-PHYSICS-REPLICA - src/kinematics.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0 - see LICENSE
// =============================================================================
//! Real forward kinematics over a `Chain` (see `urdf.rs`): given a joint
//! position for every non-fixed joint, computes each joint's world-frame
//! transform by chaining `origin * joint_motion` down the serial chain.

use std::collections::HashMap;

use crate::transform::Mat4;
use crate::urdf::{Chain, Joint, JointType};

#[derive(Debug, PartialEq)]
pub enum KinematicsError {
    MissingJointPosition(String),
}

impl std::fmt::Display for KinematicsError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            KinematicsError::MissingJointPosition(name) => {
                write!(f, "no position given for non-fixed joint '{name}'")
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
        world = world.mul(&joint.origin).mul(&joint_motion(joint, position));
        out.push((joint.name.clone(), world));
    }

    Ok(out)
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
}
