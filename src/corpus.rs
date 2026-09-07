// =============================================================================
// HYDRA-UMC-PHYSICS-REPLICA - src/corpus.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0 - see LICENSE
// =============================================================================
//! A real, reusable corpus of joint/chain fixtures shared by
//! `limits.rs`'s and `kinematics.rs`'s regression tests: a single source
//! of truth for what "in range", "exactly at the boundary", and "out of
//! range" mean across every joint type this project supports, rather
//! than duplicated ad hoc literals scattered across test modules. Also
//! used by `kinematics::forward_kinematics_checked`'s own doc example.

use crate::transform::{Mat4, Vec3};
use crate::urdf::{Chain, Joint, JointType};

pub fn revolute_with_limit(name: &str, lower: f64, upper: f64) -> Joint {
    Joint {
        name: name.to_string(),
        joint_type: JointType::Revolute,
        origin: Mat4::identity(),
        axis: Vec3::new(0.0, 0.0, 1.0),
        limit: Some((lower, upper)),
    }
}

pub fn prismatic_with_limit(name: &str, lower: f64, upper: f64) -> Joint {
    Joint {
        name: name.to_string(),
        joint_type: JointType::Prismatic,
        origin: Mat4::identity(),
        axis: Vec3::new(0.0, 0.0, 1.0),
        limit: Some((lower, upper)),
    }
}

/// A `continuous` joint has no meaningful limit by definition (it can
/// spin indefinitely) - real robots do have these (a wheel, a rotating
/// tool head).
pub fn continuous_unlimited(name: &str) -> Joint {
    Joint {
        name: name.to_string(),
        joint_type: JointType::Continuous,
        origin: Mat4::identity(),
        axis: Vec3::new(0.0, 0.0, 1.0),
        limit: None,
    }
}

pub fn fixed(name: &str) -> Joint {
    Joint {
        name: name.to_string(),
        joint_type: JointType::Fixed,
        origin: Mat4::identity(),
        axis: Vec3::new(1.0, 0.0, 0.0),
        limit: None,
    }
}

pub fn single_joint_chain(joint: Joint) -> Chain {
    Chain {
        joints: vec![joint],
    }
}

/// A realistic 2-DOF serial chain (shoulder + elbow), both revolute with
/// real-ish joint limits - the standard fixture for regression tests
/// that need more than one link.
pub fn shoulder_elbow_chain() -> Chain {
    Chain {
        joints: vec![
            revolute_with_limit("shoulder", -std::f64::consts::PI, std::f64::consts::PI),
            revolute_with_limit("elbow", -2.0, 2.0),
        ],
    }
}
