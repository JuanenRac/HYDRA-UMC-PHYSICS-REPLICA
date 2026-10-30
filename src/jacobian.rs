// =============================================================================
// HYDRA-UMC-PHYSICS-REPLICA - src/jacobian.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0 - see LICENSE
// =============================================================================
//! The geometric Jacobian of a joint frame and a damped-least-squares inverse
//! kinematics for its position.
//!
//! The Jacobian has six rows (vx, vy, vz, wx, wy, wz, in the world frame) and
//! one column per free joint: a joint that follows another (`<mimic>`) has no
//! column of its own, its effect is added to the column of its master, scaled
//! by the multiplier. Like the rest of the crate this is a kinematic estimate
//! from the URDF geometry, not a measurement.

use std::collections::HashMap;

use crate::kinematics::{effective_positions, forward_kinematics_model, KinematicsError};
use crate::urdf::{JointType, Model};

#[derive(Debug, PartialEq)]
pub enum JacobianError {
    Kinematics(KinematicsError),
    UnknownJoint(String),
    /// The damped system could not be solved (it never happens with a positive damping).
    Singular,
    /// The iteration ended without reaching the goal: no pose of the arm reaches it, or it stopped moving.
    NotReached {
        residual: f64,
        iterations: usize,
    },
}

impl std::fmt::Display for JacobianError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            JacobianError::Kinematics(e) => write!(f, "{e}"),
            JacobianError::UnknownJoint(name) => write!(f, "the URDF has no joint '{name}'"),
            JacobianError::Singular => write!(f, "the inverse kinematics system is singular"),
            JacobianError::NotReached {
                residual,
                iterations,
            } => write!(
                f,
                "the goal was not reached after {iterations} iteration(s): {residual:.6} m away"
            ),
        }
    }
}

impl From<KinematicsError> for JacobianError {
    fn from(e: KinematicsError) -> Self {
        JacobianError::Kinematics(e)
    }
}

/// `rows[0..3]` are the linear part, `rows[3..6]` the angular part; `joints[c]` names column `c`.
#[derive(Debug, PartialEq)]
pub struct Jacobian {
    pub joints: Vec<String>,
    pub rows: [Vec<f64>; 6],
}

/// d(position of `joint`) / d(position of the free joint `master`): 1 for the joint itself, the product of
/// the multipliers along a chain of followers, 0 for the rest.
fn coupling(model: &Model, joint: usize, master: usize) -> f64 {
    let mut index = joint;
    let mut factor = 1.0;
    // the parser refuses circles, so this ends; the bound is only a guard
    for _ in 0..=model.chain.joints.len() {
        if index == master {
            return factor;
        }
        match &model.extras[index].mimic {
            Some(mimic) => match model.joint_index(&mimic.joint) {
                Some(next) => {
                    factor *= mimic.multiplier;
                    index = next;
                }
                None => return 0.0,
            },
            None => return 0.0,
        }
    }
    0.0
}

/// The Jacobian of the frame of the joint called `target`, at the given positions of the free joints.
pub fn jacobian(
    model: &Model,
    given: &HashMap<String, f64>,
    target: &str,
) -> Result<Jacobian, JacobianError> {
    let target_index = model
        .joint_index(target)
        .ok_or_else(|| JacobianError::UnknownJoint(target.to_string()))?;
    let worlds = forward_kinematics_model(model, given)?;
    let tip = worlds[target_index].1.translation_part();

    // the joints from the root down to the target
    let mut ancestors = Vec::new();
    let mut cursor = Some(target_index);
    while let Some(index) = cursor {
        ancestors.push(index);
        cursor = model.parents[index];
    }

    let free: Vec<usize> = (0..model.chain.joints.len())
        .filter(|&i| {
            model.chain.joints[i].joint_type != JointType::Fixed && model.extras[i].mimic.is_none()
        })
        .collect();

    let mut rows: [Vec<f64>; 6] = Default::default();
    let mut names = Vec::with_capacity(free.len());
    for &master in &free {
        let mut column = [0.0f64; 6];
        for &a in &ancestors {
            let joint = &model.chain.joints[a];
            if joint.joint_type == JointType::Fixed {
                continue;
            }
            let k = coupling(model, a, master);
            if k == 0.0 {
                continue;
            }
            let world = &worlds[a].1;
            let axis = joint.axis.normalized();
            let w = [
                world.m[0][0] * axis.x + world.m[0][1] * axis.y + world.m[0][2] * axis.z,
                world.m[1][0] * axis.x + world.m[1][1] * axis.y + world.m[1][2] * axis.z,
                world.m[2][0] * axis.x + world.m[2][1] * axis.y + world.m[2][2] * axis.z,
            ];
            match joint.joint_type {
                JointType::Prismatic => {
                    for i in 0..3 {
                        column[i] += k * w[i];
                    }
                }
                _ => {
                    let origin = world.translation_part();
                    let r = [tip.x - origin.x, tip.y - origin.y, tip.z - origin.z];
                    let linear = [
                        w[1] * r[2] - w[2] * r[1],
                        w[2] * r[0] - w[0] * r[2],
                        w[0] * r[1] - w[1] * r[0],
                    ];
                    for i in 0..3 {
                        column[i] += k * linear[i];
                        column[3 + i] += k * w[i];
                    }
                }
            }
        }
        names.push(model.chain.joints[master].name.clone());
        for (row, value) in rows.iter_mut().zip(column) {
            row.push(value);
        }
    }
    Ok(Jacobian {
        joints: names,
        rows,
    })
}

/// Settings of the iteration. The command line takes them as flags, with these defaults.
#[derive(Debug, Clone, Copy)]
pub struct IkOptions {
    pub max_iterations: usize,
    /// Distance in metres under which the goal counts as reached.
    pub tolerance: f64,
    /// Damping of the least-squares step: larger is steadier near singular poses and slower.
    pub damping: f64,
    /// Longest step of one iteration, in metres of the goal error, so a far goal is approached in several steps.
    pub max_step: f64,
}

impl Default for IkOptions {
    fn default() -> Self {
        IkOptions {
            max_iterations: 200,
            tolerance: 1e-4,
            damping: 0.05,
            max_step: 0.1,
        }
    }
}

/// Solves the 3x3 system `a x = b` by Gauss elimination with pivoting.
#[allow(clippy::needless_range_loop)]
fn solve3(mut a: [[f64; 3]; 3], mut b: [f64; 3]) -> Option<[f64; 3]> {
    for col in 0..3 {
        let pivot = (col..3).max_by(|&i, &j| a[i][col].abs().total_cmp(&a[j][col].abs()))?;
        if a[pivot][col].abs() < 1e-14 {
            return None;
        }
        a.swap(col, pivot);
        b.swap(col, pivot);
        for r in col + 1..3 {
            let f = a[r][col] / a[col][col];
            for c in col..3 {
                a[r][c] -= f * a[col][c];
            }
            b[r] -= f * b[col];
        }
    }
    let mut x = [0.0; 3];
    for r in (0..3).rev() {
        let mut s = b[r];
        for c in r + 1..3 {
            s -= a[r][c] * x[c];
        }
        x[r] = s / a[r][r];
    }
    Some(x)
}

/// What the iteration found: the positions of the free joints and how far the frame ended from the goal.
#[derive(Debug)]
pub struct IkSolution {
    pub positions: HashMap<String, f64>,
    pub residual: f64,
    pub iterations: usize,
}

/// Moves the frame of joint `target` to the position `goal` (world frame, metres) starting from `initial`, by damped
/// least squares on the position rows of the Jacobian. Positions stay inside the declared limits of each joint.
#[allow(clippy::needless_range_loop)]
pub fn ik_position(
    model: &Model,
    initial: &HashMap<String, f64>,
    target: &str,
    goal: [f64; 3],
    options: IkOptions,
) -> Result<IkSolution, JacobianError> {
    let target_index = model
        .joint_index(target)
        .ok_or_else(|| JacobianError::UnknownJoint(target.to_string()))?;
    let mut positions = initial.clone();
    // the starting positions must be valid before any step is taken
    effective_positions(model, &positions)?;
    let mut residual = f64::INFINITY;
    for iteration in 0..=options.max_iterations {
        let worlds = forward_kinematics_model(model, &positions)?;
        let tip = worlds[target_index].1.translation_part();
        let error = [goal[0] - tip.x, goal[1] - tip.y, goal[2] - tip.z];
        residual = (error[0] * error[0] + error[1] * error[1] + error[2] * error[2]).sqrt();
        if residual <= options.tolerance {
            return Ok(IkSolution {
                positions,
                residual,
                iterations: iteration,
            });
        }
        if iteration == options.max_iterations {
            break;
        }
        let scale = if residual > options.max_step {
            options.max_step / residual
        } else {
            1.0
        };
        let e = [error[0] * scale, error[1] * scale, error[2] * scale];
        let j = jacobian(model, &positions, target)?;
        let n = j.joints.len();
        // (J J^T + damping^2 I) y = e, then dq = J^T y
        let mut m = [[0.0; 3]; 3];
        for r in 0..3 {
            for c in 0..3 {
                m[r][c] = (0..n).map(|k| j.rows[r][k] * j.rows[c][k]).sum();
            }
            m[r][r] += options.damping * options.damping;
        }
        let y = solve3(m, e).ok_or(JacobianError::Singular)?;
        for (k, name) in j.joints.iter().enumerate() {
            let step: f64 = (0..3).map(|r| j.rows[r][k] * y[r]).sum();
            let index = model.joint_index(name).expect("a column names a joint");
            let mut value = positions[name] + step;
            if let Some((lower, upper)) = model.chain.joints[index].limit {
                value = value.clamp(lower, upper);
            }
            positions.insert(name.clone(), value);
        }
    }
    Err(JacobianError::NotReached {
        residual,
        iterations: options.max_iterations,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::urdf::parse_urdf_model;

    /// A shoulder (z), an elbow (y) and a prismatic reach with a finger that mirrors it.
    const ARM: &str = r#"<robot name="t">
      <link name="base"/><link name="a"/><link name="b"/><link name="c"/><link name="d"/>
      <joint name="shoulder" type="revolute"><parent link="base"/><child link="a"/>
        <origin xyz="0 0 0.3" rpy="0.1 0 0.2"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
      <joint name="elbow" type="revolute"><parent link="a"/><child link="b"/>
        <origin xyz="0.4 0 0"/><axis xyz="0 1 0"/><limit lower="-2" upper="2" effort="1" velocity="1"/></joint>
      <joint name="reach" type="prismatic"><parent link="b"/><child link="c"/>
        <origin xyz="0.3 0 0" rpy="0 0.3 0"/><axis xyz="1 0 0"/><limit lower="0" upper="0.2" effort="1" velocity="1"/></joint>
      <joint name="finger" type="prismatic"><parent link="c"/><child link="d"/>
        <origin xyz="0 0.05 0"/><axis xyz="0 1 0"/><limit lower="-0.1" upper="0.1" effort="1" velocity="1"/>
        <mimic joint="reach" multiplier="-0.5" offset="0"/></joint>
    </robot>"#;

    fn model() -> Model {
        parse_urdf_model(ARM).unwrap()
    }

    fn pose(shoulder: f64, elbow: f64, reach: f64) -> HashMap<String, f64> {
        HashMap::from([
            ("shoulder".to_string(), shoulder),
            ("elbow".to_string(), elbow),
            ("reach".to_string(), reach),
        ])
    }

    fn tip_of(model: &Model, q: &HashMap<String, f64>, joint: &str) -> [f64; 3] {
        let worlds = forward_kinematics_model(model, q).unwrap();
        let i = model.joint_index(joint).unwrap();
        let t = worlds[i].1.translation_part();
        [t.x, t.y, t.z]
    }

    /// The linear rows of the Jacobian must be the derivative of the position given by forward kinematics.
    #[test]
    fn linear_rows_match_finite_differences_of_forward_kinematics() {
        let model = model();
        let q = pose(0.4, -0.7, 0.1);
        for target in ["elbow", "reach", "finger"] {
            let j = jacobian(&model, &q, target).unwrap();
            assert_eq!(j.joints, vec!["shoulder", "elbow", "reach"]);
            for (c, name) in j.joints.iter().enumerate() {
                let h = 1e-6;
                let mut up = q.clone();
                let mut down = q.clone();
                *up.get_mut(name).unwrap() += h;
                *down.get_mut(name).unwrap() -= h;
                let a = tip_of(&model, &up, target);
                let b = tip_of(&model, &down, target);
                for r in 0..3 {
                    let numeric = (a[r] - b[r]) / (2.0 * h);
                    assert!(
                        (numeric - j.rows[r][c]).abs() < 1e-6,
                        "{target} column {name} row {r}: {numeric} against {}",
                        j.rows[r][c]
                    );
                }
            }
        }
    }

    #[test]
    fn a_joint_after_the_target_does_not_move_it() {
        let model = model();
        let j = jacobian(&model, &pose(0.2, 0.3, 0.1), "elbow").unwrap();
        // the prismatic reach is further down the arm than the elbow frame
        assert!(j.rows.iter().all(|row| row[2] == 0.0));
        // and the elbow frame turns with the shoulder about the world z of that joint
        assert!(j.rows[3..].iter().map(|r| r[0]).any(|v| v != 0.0));
    }

    #[test]
    fn the_follower_adds_its_share_to_the_column_of_its_master() {
        let model = model();
        let q = pose(0.0, 0.0, 0.05);
        let reach = jacobian(&model, &q, "reach").unwrap();
        let finger = jacobian(&model, &q, "finger").unwrap();
        // the same column, plus the finger sliding at -0.5 times the reach along its own axis
        let differs = (0..3).any(|r| (reach.rows[r][2] - finger.rows[r][2]).abs() > 1e-9);
        assert!(differs);
    }

    #[test]
    fn an_unknown_joint_is_refused() {
        let model = model();
        assert_eq!(
            jacobian(&model, &pose(0.0, 0.0, 0.0), "nope"),
            Err(JacobianError::UnknownJoint("nope".to_string()))
        );
    }

    #[test]
    fn a_missing_position_is_refused() {
        let model = model();
        let mut q = pose(0.0, 0.0, 0.0);
        q.remove("elbow");
        assert!(matches!(
            jacobian(&model, &q, "reach"),
            Err(JacobianError::Kinematics(
                KinematicsError::MissingJointPosition(_)
            ))
        ));
    }

    #[test]
    fn inverse_kinematics_reaches_a_position_the_arm_can_make() {
        let model = model();
        let goal = tip_of(&model, &pose(0.6, -0.5, 0.12), "reach");
        let solution = ik_position(
            &model,
            &pose(0.0, 0.0, 0.0),
            "reach",
            goal,
            IkOptions::default(),
        )
        .unwrap();
        assert!(solution.residual <= 1e-4);
        let reached = tip_of(&model, &solution.positions, "reach");
        for i in 0..3 {
            assert!((reached[i] - goal[i]).abs() < 2e-4);
        }
        for (name, value) in &solution.positions {
            let index = model.joint_index(name).unwrap();
            let (lower, upper) = model.chain.joints[index].limit.unwrap();
            assert!(*value >= lower && *value <= upper, "{name} out of range");
        }
    }

    #[test]
    fn inverse_kinematics_refuses_a_position_out_of_reach() {
        let model = model();
        let result = ik_position(
            &model,
            &pose(0.0, 0.0, 0.0),
            "reach",
            [5.0, 0.0, 0.0],
            IkOptions::default(),
        );
        assert!(matches!(result, Err(JacobianError::NotReached { .. })));
    }
}
