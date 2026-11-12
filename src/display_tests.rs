// =============================================================================
// HYDRA-UMC-PHYSICS-REPLICA - src/display_tests.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0-or-later - see LICENSE
// =============================================================================
//! The sentences of every error the library can give (a person reads them in the answer of the service), and the quaternion of a rotation in each of
//! the four ways the conversion takes it: a sweep over rotations that start from a different dominant axis, checked against the matrix itself.

use crate::jacobian::JacobianError;
use crate::kinematics::{CheckedKinematicsError, KinematicsError};
use crate::transform::{Mat4, Vec3};
use crate::urdf::UrdfError;

fn text(joint: &str) -> String {
    joint.to_string()
}

#[test]
fn every_error_of_the_urdf_says_what_is_wrong_and_where() {
    let cases: Vec<(UrdfError, &str)> = vec![
        (
            UrdfError::Xml(text("unclosed tag")),
            "XML parse error: unclosed tag",
        ),
        (
            UrdfError::MissingAttribute {
                joint: text("j1"),
                attribute: "type",
            },
            "joint 'j1' is missing required attribute 'type'",
        ),
        (
            UrdfError::UnsupportedJointType {
                joint: text("j2"),
                joint_type: text("floating"),
            },
            "unsupported type 'floating'",
        ),
        (
            UrdfError::InvalidNumber {
                joint: text("j3"),
                text: text("abc"),
            },
            "non-numeric value: 'abc'",
        ),
        (
            UrdfError::NonFiniteNumber {
                joint: text("j4"),
                text: text("nan"),
            },
            "non-finite value",
        ),
        (
            UrdfError::InvertedLimit {
                joint: text("j5"),
                text: text("2 < 1"),
            },
            "inverted limit (2 < 1)",
        ),
        (UrdfError::NoRootLink, "could not find a real root link"),
        (
            UrdfError::MultipleRootLinks {
                roots: vec![text("a"), text("b")],
            },
            "found 2 real root links (a, b)",
        ),
        (
            UrdfError::Branching {
                link: text("hub"),
                joint_count: 3,
            },
            "link 'hub' has 3 child joints",
        ),
        (
            UrdfError::UnreachableJoints { count: 4 },
            "4 joint(s) declared in this document are never reached",
        ),
        (
            UrdfError::DuplicateJointName { joint: text("j6") },
            "joint name 'j6' is used by more than one joint",
        ),
        (
            UrdfError::LinkWithTwoParents { link: text("l1") },
            "link 'l1' is the child of more than one joint",
        ),
        (
            UrdfError::UnknownLink {
                joint: text("j7"),
                link: text("ghost"),
            },
            "names link 'ghost', which no <link> declares",
        ),
        (
            UrdfError::ZeroAxis { joint: text("j8") },
            "moves along an axis of length 0",
        ),
        (
            UrdfError::InvalidMimic {
                joint: text("j9"),
                reason: text("it follows itself"),
            },
            "invalid <mimic>: it follows itself",
        ),
        (
            UrdfError::InvalidSoftLimits {
                joint: text("j10"),
                reason: text("inverted"),
            },
            "invalid soft limits: inverted",
        ),
    ];
    for (error, fragment) in cases {
        let message = error.to_string();
        assert!(message.contains(fragment), "{error:?}: {message}");
    }
}

#[test]
fn every_error_of_the_kinematics_and_the_inverse_says_what_is_wrong() {
    let positions = [
        (
            KinematicsError::MissingJointPosition(text("a")),
            "no position given for non-fixed joint 'a'",
        ),
        (
            KinematicsError::NonFiniteJointPosition(text("b")),
            "non-finite position given for joint 'b'",
        ),
        (
            KinematicsError::MimicPositionGiven(text("c")),
            "which follows another joint (<mimic>)",
        ),
    ];
    for (error, fragment) in positions {
        assert!(error.to_string().contains(fragment), "{error:?}");
    }
    let checked =
        CheckedKinematicsError::Kinematics(KinematicsError::MissingJointPosition(text("a")))
            .to_string();
    assert!(checked.contains("no position given"), "{checked}");
    assert_eq!(
        CheckedKinematicsError::LimitViolation(Vec::new()).to_string(),
        "0 joint limit violation(s)"
    );
    let missing = CheckedKinematicsError::MissingLimits(vec![text("j1"), text("j2")]).to_string();
    assert!(
        missing.contains("joint(s) j1, j2 have no <limit>"),
        "{missing}"
    );

    let jacobian = [
        (
            JacobianError::Kinematics(KinematicsError::NonFiniteJointPosition(text("b"))),
            "non-finite position",
        ),
        (
            JacobianError::UnknownJoint(text("elbow")),
            "the URDF has no joint 'elbow'",
        ),
        (JacobianError::Singular, "is singular"),
        (
            JacobianError::NotReached {
                residual: 0.25,
                iterations: 12,
            },
            "not reached after 12 iteration(s): 0.250000 m away",
        ),
    ];
    for (error, fragment) in jacobian {
        let message = error.to_string();
        assert!(message.contains(fragment), "{error:?}: {message}");
    }
    let wrapped: JacobianError = KinematicsError::MissingJointPosition(text("a")).into();
    assert_eq!(
        wrapped,
        JacobianError::Kinematics(KinematicsError::MissingJointPosition(text("a")))
    );
}

/// Rotates a vector by the quaternion `(w, x, y, z)`.
fn rotate(q: (f64, f64, f64, f64), v: Vec3) -> Vec3 {
    let (w, x, y, z) = q;
    let t = Vec3::new(
        2.0 * (y * v.z - z * v.y),
        2.0 * (z * v.x - x * v.z),
        2.0 * (x * v.y - y * v.x),
    );
    Vec3::new(
        v.x + w * t.x + (y * t.z - z * t.y),
        v.y + w * t.y + (z * t.x - x * t.z),
        v.z + w * t.z + (x * t.y - y * t.x),
    )
}

fn close(a: Vec3, b: Vec3) -> bool {
    (a.x - b.x).abs() < 1e-9 && (a.y - b.y).abs() < 1e-9 && (a.z - b.z).abs() < 1e-9
}

#[test]
fn the_quaternion_of_a_rotation_turns_a_vector_as_the_matrix_does_in_every_branch() {
    let probe = Vec3::new(0.3, -0.7, 1.1);
    // half turns about each axis leave the trace at -1, so each of the three "largest diagonal" branches is taken once; the other angles take the first
    let mut axes = vec![
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 1.0, 0.0),
        Vec3::new(0.0, 1.0, 1.0),
        Vec3::new(1.0, 0.0, 1.0),
        Vec3::new(1.0, 2.0, 3.0),
        Vec3::new(-3.0, 2.0, -1.0),
    ];
    // a sweep with a fixed seed adds axes of any direction
    let mut state = 0x2026_1112u32;
    for _ in 0..24 {
        let mut next = || {
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (state >> 8) as f64 / (1u32 << 24) as f64 * 2.0 - 1.0
        };
        axes.push(Vec3::new(next(), next(), next()));
    }
    let angles = [
        0.0,
        0.5,
        1.5,
        std::f64::consts::FRAC_PI_2,
        2.9,
        std::f64::consts::PI,
        3.5,
        -2.0,
        -std::f64::consts::PI,
    ];
    for axis in &axes {
        if axis.norm() < 1e-6 {
            continue;
        }
        for angle in angles {
            let matrix = Mat4::rotation_axis_angle(*axis, angle);
            let q = matrix.quaternion();
            let norm = (q.0 * q.0 + q.1 * q.1 + q.2 * q.2 + q.3 * q.3).sqrt();
            assert!(
                (norm - 1.0).abs() < 1e-9,
                "unit quaternion for {axis:?} {angle}"
            );
            assert!(q.0 >= 0.0, "the sign is canonical for {axis:?} {angle}");
            assert!(
                close(rotate(q, probe), matrix.transform_point(probe)),
                "{axis:?} {angle}"
            );
        }
    }
}
