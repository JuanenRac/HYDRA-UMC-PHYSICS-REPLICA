// =============================================================================
// HYDRA-UMC-PHYSICS-REPLICA - src/dynamics.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0-or-later - see LICENSE
// =============================================================================
//! Inverse dynamics by the recursive Newton-Euler algorithm (RNEA) over the `<inertial>` blocks of the URDF: the torque (or force, for a
//! prismatic joint) each joint must give for the robot to follow a given position, velocity and acceleration under gravity.
//!
//! The recursion runs in the world frame over the tree of links: a forward pass for velocities and accelerations, a backward pass for
//! the forces and moments that each joint transmits. Gravity enters as the acceleration of the root link, `-g`. A joint that follows
//! another (`<mimic>`) moves with it: its torque is folded into the torque of its master (the generalised force on the one degree of
//! freedom), and only the free joints are reported.
//!
//! What it does not model: friction, motor and gearbox inertia, the elasticity of links and joints, contact forces, and the load carried
//! by the tool unless the URDF puts it in the inertia of the last link. A link without `<inertial>` is massless to this algorithm, which
//! UNDER-estimates the torques, so it is an error unless the caller says that it is acceptable; the links that were taken as massless are
//! always listed. The numbers are estimates from the model, not measurements of a real robot.

use std::collections::HashMap;

use crate::kinematics::{effective_positions, forward_kinematics_model};
use crate::transform::Mat4;
use crate::urdf::{JointType, Model};

type V = [f64; 3];
type M3 = [[f64; 3]; 3];

/// Standard gravity, the default acceleration of gravity along -z (metres per second squared).
pub const DEFAULT_GRAVITY: V = [0.0, 0.0, -9.81];

fn add(a: V, b: V) -> V {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
fn sub(a: V, b: V) -> V {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn scale(a: V, k: f64) -> V {
    [a[0] * k, a[1] * k, a[2] * k]
}
fn dot(a: V, b: V) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross(a: V, b: V) -> V {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn rot(m: &Mat4) -> M3 {
    let mut r = [[0.0; 3]; 3];
    for (i, row) in r.iter_mut().enumerate() {
        for (j, v) in row.iter_mut().enumerate() {
            *v = m.m[i][j];
        }
    }
    r
}
fn mat_mul(a: &M3, b: &M3) -> M3 {
    let mut out = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            out[i][j] = (0..3).map(|k| a[i][k] * b[k][j]).sum();
        }
    }
    out
}
fn transpose(a: &M3) -> M3 {
    let mut out = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            out[i][j] = a[j][i];
        }
    }
    out
}
fn mat_vec(a: &M3, v: V) -> V {
    [dot(a[0], v), dot(a[1], v), dot(a[2], v)]
}
fn trans(m: &Mat4) -> V {
    [m.m[0][3], m.m[1][3], m.m[2][3]]
}
fn pt(m: &Mat4, p: V) -> V {
    let t = m.transform_point(crate::transform::Vec3::new(p[0], p[1], p[2]));
    [t.x, t.y, t.z]
}

#[derive(Debug, Clone, Copy)]
pub struct Options {
    pub gravity: V,
    /// Accept links that have no `<inertial>` (they count as massless and are listed).
    pub allow_missing_inertial: bool,
}

impl Default for Options {
    fn default() -> Self {
        Options {
            gravity: DEFAULT_GRAVITY,
            allow_missing_inertial: false,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Torques {
    /// Torque (N m) of a revolute or continuous joint, force (N) of a prismatic one, by the name of the free joints.
    pub torques: HashMap<String, f64>,
    /// Links below the root that have no `<inertial>` (taken as massless).
    pub without_inertial: Vec<String>,
}

/// The velocity or acceleration of every joint: the ones given, and `multiplier * master` for each follower. The offset of a `<mimic>`
/// belongs to the position only.
fn resolve_rates(
    model: &Model,
    given: &HashMap<String, f64>,
    what: &str,
) -> Result<Vec<f64>, String> {
    let joints = &model.chain.joints;
    for name in given.keys() {
        let index = model
            .joint_index(name)
            .ok_or_else(|| format!("unknown joint '{name}' in the {what}"))?;
        if model.extras[index].mimic.is_some() {
            return Err(format!(
                "a {what} was given for joint '{name}', which follows another joint (<mimic>): it is derived"
            ));
        }
    }
    let mut out: Vec<Option<f64>> = vec![None; joints.len()];
    for (i, joint) in joints.iter().enumerate() {
        if joint.joint_type == JointType::Fixed {
            out[i] = Some(0.0);
        } else if model.extras[i].mimic.is_none() {
            let value = *given.get(&joint.name).ok_or_else(|| {
                format!(
                    "no {what} for joint '{}': an unknown value is never read as zero",
                    joint.name
                )
            })?;
            if !value.is_finite() {
                return Err(format!(
                    "the {what} of joint '{}' is not finite",
                    joint.name
                ));
            }
            out[i] = Some(value);
        }
    }
    loop {
        let mut changed = false;
        for i in 0..joints.len() {
            if out[i].is_some() {
                continue;
            }
            if let Some(mimic) = &model.extras[i].mimic {
                let master = model
                    .joint_index(&mimic.joint)
                    .ok_or("a mimic names no joint")?;
                if let Some(m) = out[master] {
                    out[i] = Some(mimic.multiplier * m);
                    changed = true;
                }
            }
        }
        if !changed {
            break;
        }
    }
    out.into_iter()
        .map(|v| v.ok_or_else(|| "a follower could not be resolved".to_string()))
        .collect()
}

struct Body {
    o: V,
    omega: V,
    alpha: V,
    v: V,
    a: V,
}

struct Pass {
    /// Centre of mass, force and moment (about the centre of mass) of every link below the root, in the world frame.
    com: Vec<V>,
    force: Vec<V>,
    moment: Vec<V>,
    origins: Vec<V>,
    axes: Vec<V>,
    kinetic: f64,
    potential: f64,
    without_inertial: Vec<String>,
}

fn pass(
    model: &Model,
    given: &HashMap<String, f64>,
    qd: &[f64],
    qdd: &[f64],
    options: &Options,
) -> Result<Pass, String> {
    let joints = &model.chain.joints;
    let worlds = forward_kinematics_model(model, given).map_err(|e| e.to_string())?;
    let mut bodies: Vec<Body> = Vec::with_capacity(joints.len());
    let root = Body {
        o: [0.0; 3],
        omega: [0.0; 3],
        alpha: [0.0; 3],
        v: [0.0; 3],
        a: scale(options.gravity, -1.0),
    };
    let mut out = Pass {
        com: Vec::new(),
        force: Vec::new(),
        moment: Vec::new(),
        origins: Vec::new(),
        axes: Vec::new(),
        kinetic: 0.0,
        potential: 0.0,
        without_inertial: Vec::new(),
    };
    for (i, joint) in joints.iter().enumerate() {
        let (base, parent) = match model.parents[i] {
            Some(p) => (worlds[p].1, &bodies[p]),
            None => (Mat4::identity(), &root),
        };
        let frame = base.mul(&joint.origin);
        let z = {
            let a = joint.axis.normalized();
            let r = rot(&frame);
            mat_vec(&r, [a.x, a.y, a.z])
        };
        let w = worlds[i].1;
        let o = trans(&w);
        let r = sub(o, parent.o);
        let vp = add(parent.v, cross(parent.omega, r));
        let ap = add(
            add(parent.a, cross(parent.alpha, r)),
            cross(parent.omega, cross(parent.omega, r)),
        );
        let (q1, q2) = (qd[i], qdd[i]);
        let body = match joint.joint_type {
            JointType::Fixed => Body {
                o,
                omega: parent.omega,
                alpha: parent.alpha,
                v: vp,
                a: ap,
            },
            JointType::Revolute | JointType::Continuous => Body {
                o,
                omega: add(parent.omega, scale(z, q1)),
                alpha: add(
                    add(parent.alpha, scale(z, q2)),
                    cross(parent.omega, scale(z, q1)),
                ),
                v: vp,
                a: ap,
            },
            JointType::Prismatic => Body {
                o,
                omega: parent.omega,
                alpha: parent.alpha,
                v: add(vp, scale(z, q1)),
                a: add(
                    add(ap, scale(cross(parent.omega, z), 2.0 * q1)),
                    scale(z, q2),
                ),
            },
        };
        let link = &model.child_links[i];
        let info = model.links.get(link);
        if let Some(problem) = info.and_then(|l| l.inertial_problem.as_ref()) {
            return Err(format!("link '{link}': {problem}"));
        }
        let (c, force, moment) = match info.and_then(|l| l.inertial.as_ref()) {
            Some(inertial) => {
                let c = pt(&w, trans(&inertial.origin));
                let rc = mat_mul(&rot(&w), &rot(&inertial.origin));
                let iw = mat_mul(&mat_mul(&rc, &inertial.inertia), &transpose(&rc));
                let rr = sub(c, o);
                let vc = add(body.v, cross(body.omega, rr));
                let ac = add(
                    add(body.a, cross(body.alpha, rr)),
                    cross(body.omega, cross(body.omega, rr)),
                );
                let iw_omega = mat_vec(&iw, body.omega);
                out.kinetic += 0.5 * inertial.mass * dot(vc, vc) + 0.5 * dot(body.omega, iw_omega);
                out.potential -= inertial.mass * dot(options.gravity, c);
                (
                    c,
                    scale(ac, inertial.mass),
                    add(mat_vec(&iw, body.alpha), cross(body.omega, iw_omega)),
                )
            }
            None => {
                out.without_inertial.push(link.clone());
                (o, [0.0; 3], [0.0; 3])
            }
        };
        out.com.push(c);
        out.force.push(force);
        out.moment.push(moment);
        out.origins.push(o);
        out.axes.push(z);
        bodies.push(body);
    }
    if !out.without_inertial.is_empty() && !options.allow_missing_inertial {
        return Err(format!(
            "link(s) {} have no <inertial>: they would count as massless and the torques would be under-estimated; allow it on purpose if that is acceptable",
            out.without_inertial.join(", ")
        ));
    }
    Ok(out)
}

/// The joint torques (forces, for prismatic joints) needed for the motion given. `positions`, `velocities` and `accelerations` name every
/// joint that moves by itself; a joint that follows another is derived.
pub fn inverse_dynamics(
    model: &Model,
    positions: &HashMap<String, f64>,
    velocities: &HashMap<String, f64>,
    accelerations: &HashMap<String, f64>,
    options: &Options,
) -> Result<Torques, String> {
    // the positions are checked here (a follower given, a joint missing) before the pass places the links
    effective_positions(model, positions).map_err(|e| e.to_string())?;
    let qd = resolve_rates(model, velocities, "velocity")?;
    let qdd = resolve_rates(model, accelerations, "acceleration")?;
    let joints = &model.chain.joints;
    let pass = pass(model, positions, &qd, &qdd, options)?;
    let n = joints.len();
    let mut f_acc = vec![[0.0; 3]; n];
    let mut n_acc = vec![[0.0; 3]; n];
    let mut tau = vec![0.0; n];
    for i in (0..n).rev() {
        let f = add(pass.force[i], f_acc[i]);
        let moment = add(
            add(
                pass.moment[i],
                cross(sub(pass.com[i], pass.origins[i]), pass.force[i]),
            ),
            n_acc[i],
        );
        tau[i] = match joints[i].joint_type {
            JointType::Fixed => 0.0,
            JointType::Revolute | JointType::Continuous => dot(pass.axes[i], moment),
            JointType::Prismatic => dot(pass.axes[i], f),
        };
        if let Some(p) = model.parents[i] {
            f_acc[p] = add(f_acc[p], f);
            n_acc[p] = add(
                n_acc[p],
                add(moment, cross(sub(pass.origins[i], pass.origins[p]), f)),
            );
        }
    }
    // fold each follower into its master, the ones that nothing follows first
    let mut pending: Vec<usize> = (0..n)
        .filter(|&i| model.extras[i].mimic.is_some())
        .collect();
    while !pending.is_empty() {
        let leaf = pending
            .iter()
            .position(|&i| {
                !pending.iter().any(|&j| {
                    model.extras[j]
                        .mimic
                        .as_ref()
                        .is_some_and(|m| model.joint_index(&m.joint) == Some(i))
                })
            })
            .ok_or("the followers follow each other in a circle")?;
        let i = pending.remove(leaf);
        if let Some(mimic) = &model.extras[i].mimic {
            let master = model
                .joint_index(&mimic.joint)
                .ok_or("a mimic names no joint")?;
            tau[master] += mimic.multiplier * tau[i];
        }
    }
    let torques = joints
        .iter()
        .enumerate()
        .filter(|(i, j)| j.joint_type != JointType::Fixed && model.extras[*i].mimic.is_none())
        .map(|(i, j)| (j.name.clone(), tau[i]))
        .collect();
    Ok(Torques {
        torques,
        without_inertial: pass.without_inertial,
    })
}

/// Kinetic and potential energy of the robot at a state (potential relative to the origin of the world).
#[cfg(test)]
pub fn energy(
    model: &Model,
    positions: &HashMap<String, f64>,
    velocities: &HashMap<String, f64>,
    options: &Options,
) -> Result<(f64, f64), String> {
    effective_positions(model, positions).map_err(|e| e.to_string())?;
    let qd = resolve_rates(model, velocities, "velocity")?;
    let zeros = vec![0.0; model.chain.joints.len()];
    let pass = pass(model, positions, &qd, &zeros, options)?;
    Ok((pass.kinetic, pass.potential))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::urdf::parse_urdf_model;

    fn map(pairs: &[(&str, f64)]) -> HashMap<String, f64> {
        pairs.iter().map(|(k, v)| (k.to_string(), *v)).collect()
    }

    const PENDULUM: &str = r#"<robot name="p">
      <link name="base"/>
      <link name="rod"><inertial><origin xyz="0.4 0 0"/><mass value="2"/>
        <inertia ixx="0" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0"/></inertial></link>
      <joint name="j" type="revolute"><parent link="base"/><child link="rod"/>
        <axis xyz="0 -1 0"/><limit lower="-3" upper="3" effort="50" velocity="5"/></joint>
    </robot>"#;

    #[test]
    fn a_pendulum_matches_the_closed_form() {
        let model = parse_urdf_model(PENDULUM).unwrap();
        let (m, l, i_com, g) = (2.0, 0.4, 0.05, 9.81);
        for theta in [0.0, 0.5, -1.0, 1.4] {
            // static: tau = m g l cos(theta)
            let t = inverse_dynamics(
                &model,
                &map(&[("j", theta)]),
                &map(&[("j", 0.0)]),
                &map(&[("j", 0.0)]),
                &Options::default(),
            )
            .unwrap();
            assert!((t.torques["j"] - m * g * l * theta.cos()).abs() < 1e-9);
            // moving: tau = (I + m l^2) qdd + m g l cos(theta); the centrifugal term does not act on one axis through the pivot
            let t = inverse_dynamics(
                &model,
                &map(&[("j", theta)]),
                &map(&[("j", 3.0)]),
                &map(&[("j", 2.0)]),
                &Options::default(),
            )
            .unwrap();
            let expected = (i_com + m * l * l) * 2.0 + m * g * l * theta.cos();
            assert!((t.torques["j"] - expected).abs() < 1e-9);
        }
    }

    const TWO_LINK: &str = r#"<robot name="t">
      <link name="base"/>
      <link name="l1"><inertial><origin xyz="0.3 0 0"/><mass value="3"/>
        <inertia ixx="0" ixy="0" ixz="0" iyy="0.04" iyz="0" izz="0"/></inertial></link>
      <link name="l2"><inertial><origin xyz="0.2 0 0"/><mass value="1.5"/>
        <inertia ixx="0" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0"/></inertial></link>
      <joint name="a" type="revolute"><parent link="base"/><child link="l1"/>
        <axis xyz="0 -1 0"/><limit lower="-3" upper="3" effort="50" velocity="5"/></joint>
      <joint name="b" type="revolute"><parent link="l1"/><child link="l2"/>
        <origin xyz="0.5 0 0"/><axis xyz="0 -1 0"/><limit lower="-3" upper="3" effort="50" velocity="5"/></joint>
    </robot>"#;

    /// The planar two-link arm in the vertical plane, with the textbook equations of motion.
    #[test]
    fn a_two_link_arm_matches_the_textbook_equations() {
        let model = parse_urdf_model(TWO_LINK).unwrap();
        let (m1, m2, l1, lc1, lc2, i1, i2, g) = (3.0, 1.5, 0.5, 0.3, 0.2, 0.04, 0.02, 9.81);
        for (q1, q2, d1, d2, dd1, dd2) in [
            (0.3, 0.8, 0.0, 0.0, 0.0, 0.0),
            (0.3, -0.8, 1.2, -0.7, 0.5, 1.1),
            (-1.1, 1.9, -2.0, 1.5, -0.4, 0.9),
        ] {
            let t = inverse_dynamics(
                &model,
                &map(&[("a", q1), ("b", q2)]),
                &map(&[("a", d1), ("b", d2)]),
                &map(&[("a", dd1), ("b", dd2)]),
                &Options::default(),
            )
            .unwrap();
            let (c2, s2) = (q2.cos(), q2.sin());
            let c1 = q1.cos();
            let c12 = (q1 + q2).cos();
            let m11 = m1 * lc1 * lc1 + m2 * (l1 * l1 + lc2 * lc2 + 2.0 * l1 * lc2 * c2) + i1 + i2;
            let m12 = m2 * (lc2 * lc2 + l1 * lc2 * c2) + i2;
            let m22 = m2 * lc2 * lc2 + i2;
            let tau1 = m11 * dd1 + m12 * dd2 - m2 * l1 * lc2 * s2 * (2.0 * d1 * d2 + d2 * d2)
                + (m1 * lc1 + m2 * l1) * g * c1
                + m2 * lc2 * g * c12;
            let tau2 = m12 * dd1 + m22 * dd2 + m2 * l1 * lc2 * s2 * d1 * d1 + m2 * lc2 * g * c12;
            assert!(
                (t.torques["a"] - tau1).abs() < 1e-9,
                "{} {}",
                t.torques["a"],
                tau1
            );
            assert!(
                (t.torques["b"] - tau2).abs() < 1e-9,
                "{} {}",
                t.torques["b"],
                tau2
            );
        }
    }

    #[test]
    fn a_vertical_slide_carries_the_weight_and_the_acceleration() {
        let xml = r#"<robot name="s"><link name="base"/>
          <link name="car"><inertial><mass value="4"/><inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/></inertial></link>
          <joint name="z" type="prismatic"><parent link="base"/><child link="car"/><axis xyz="0 0 1"/>
            <limit lower="0" upper="1" effort="100" velocity="1"/></joint></robot>"#;
        let model = parse_urdf_model(xml).unwrap();
        let t = inverse_dynamics(
            &model,
            &map(&[("z", 0.2)]),
            &map(&[("z", 0.3)]),
            &map(&[("z", 1.5)]),
            &Options::default(),
        )
        .unwrap();
        assert!((t.torques["z"] - 4.0 * (9.81 + 1.5)).abs() < 1e-9);
        // gravity is a setting: none in space
        let t = inverse_dynamics(
            &model,
            &map(&[("z", 0.2)]),
            &map(&[("z", 0.0)]),
            &map(&[("z", 1.5)]),
            &Options {
                gravity: [0.0; 3],
                ..Options::default()
            },
        )
        .unwrap();
        assert!((t.torques["z"] - 6.0).abs() < 1e-9);
    }

    /// A 3D arm with tilted axes and frames, an off-centre and rotated inertia, a prismatic joint and a follower.
    const ARM: &str = r#"<robot name="arm">
      <link name="base"/>
      <link name="turret"><inertial><origin xyz="0.02 0.01 0.05" rpy="0.3 0.2 0.1"/><mass value="2.5"/>
        <inertia ixx="0.03" ixy="0.002" ixz="0.001" iyy="0.04" iyz="0.003" izz="0.05"/></inertial></link>
      <link name="upper"><inertial><origin xyz="0.15 0.02 -0.01" rpy="0.1 -0.4 0.5"/><mass value="1.8"/>
        <inertia ixx="0.02" ixy="0.001" ixz="0.0005" iyy="0.03" iyz="0.002" izz="0.025"/></inertial></link>
      <link name="slider"><inertial><origin xyz="0.05 0 0"/><mass value="0.7"/>
        <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.006" iyz="0" izz="0.004"/></inertial></link>
      <link name="finger"><inertial><origin xyz="0 0.01 0"/><mass value="0.2"/>
        <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/></inertial></link>
      <joint name="yaw" type="revolute"><parent link="base"/><child link="turret"/>
        <origin xyz="0.05 0 0.1" rpy="0.1 0.05 0"/><axis xyz="0.1 0.2 1"/><limit lower="-3" upper="3" effort="50" velocity="5"/></joint>
      <joint name="pitch" type="revolute"><parent link="turret"/><child link="upper"/>
        <origin xyz="0.03 0.02 0.2" rpy="0.2 0 0.4"/><axis xyz="0 1 0.3"/><limit lower="-3" upper="3" effort="50" velocity="5"/></joint>
      <joint name="slide" type="prismatic"><parent link="upper"/><child link="slider"/>
        <origin xyz="0.3 0 0" rpy="0 0.3 0.2"/><axis xyz="1 0.1 0"/><limit lower="0" upper="0.3" effort="50" velocity="5"/></joint>
      <joint name="grip" type="prismatic"><parent link="slider"/><child link="finger"/>
        <origin xyz="0 0.05 0"/><axis xyz="0 1 0"/><limit lower="-0.1" upper="0.1" effort="5" velocity="1"/>
        <mimic joint="slide" multiplier="-0.5" offset="0.01"/></joint>
    </robot>"#;

    /// The power the joints deliver is the rate of change of the mechanical energy: it checks the whole recursion at once, with any
    /// axis, offset, rotated inertia, follower and gravity.
    #[test]
    fn the_power_of_the_joints_is_the_rate_of_change_of_the_energy() {
        let model = parse_urdf_model(ARM).unwrap();
        let q0 = [("yaw", 0.4), ("pitch", -0.7), ("slide", 0.12)];
        let d0 = [("yaw", 0.9), ("pitch", -1.3), ("slide", 0.4)];
        let dd = [("yaw", -0.6), ("pitch", 1.7), ("slide", 0.8)];
        let options = Options::default();
        let t = inverse_dynamics(&model, &map(&q0), &map(&d0), &map(&dd), &options).unwrap();
        assert_eq!(t.torques.len(), 3, "the follower is folded into its master");
        let power: f64 = d0.iter().map(|(n, v)| t.torques[*n] * v).sum();
        let energy_at = |time: f64| {
            let q: Vec<(&str, f64)> = (0..3)
                .map(|i| {
                    (
                        q0[i].0,
                        q0[i].1 + d0[i].1 * time + 0.5 * dd[i].1 * time * time,
                    )
                })
                .collect();
            let d: Vec<(&str, f64)> = (0..3)
                .map(|i| (d0[i].0, d0[i].1 + dd[i].1 * time))
                .collect();
            let (k, p) = energy(&model, &map(&q), &map(&d), &options).unwrap();
            k + p
        };
        let h = 1e-5;
        let rate = (energy_at(h) - energy_at(-h)) / (2.0 * h);
        assert!(
            (power - rate).abs() < 1e-6 * (1.0 + rate.abs()),
            "power {power} vs energy rate {rate}"
        );
        // and the torque of a joint is not the same without gravity
        let free = inverse_dynamics(
            &model,
            &map(&q0),
            &map(&d0),
            &map(&dd),
            &Options {
                gravity: [0.0; 3],
                ..options
            },
        )
        .unwrap();
        assert!((free.torques["pitch"] - t.torques["pitch"]).abs() > 1e-3);
    }

    #[test]
    fn a_link_without_inertial_is_refused_unless_allowed_and_is_listed() {
        let xml = r#"<robot name="m"><link name="base"/><link name="a"/>
          <joint name="j" type="revolute"><parent link="base"/><child link="a"/><axis xyz="0 0 1"/>
            <limit lower="-1" upper="1" effort="1" velocity="1"/></joint></robot>"#;
        let model = parse_urdf_model(xml).unwrap();
        let (q, d, a) = (map(&[("j", 0.0)]), map(&[("j", 0.0)]), map(&[("j", 0.0)]));
        assert!(inverse_dynamics(&model, &q, &d, &a, &Options::default())
            .unwrap_err()
            .contains("no <inertial>"));
        let t = inverse_dynamics(
            &model,
            &q,
            &d,
            &a,
            &Options {
                allow_missing_inertial: true,
                ..Options::default()
            },
        )
        .unwrap();
        assert_eq!(t.without_inertial, vec!["a".to_string()]);
        assert_eq!(t.torques["j"], 0.0);
    }

    #[test]
    fn missing_or_unknown_rates_are_errors() {
        let model = parse_urdf_model(PENDULUM).unwrap();
        let q = map(&[("j", 0.0)]);
        let ok = map(&[("j", 0.0)]);
        let o = Options::default();
        assert!(inverse_dynamics(&model, &q, &HashMap::new(), &ok, &o).is_err());
        assert!(inverse_dynamics(&model, &q, &ok, &map(&[("x", 1.0)]), &o).is_err());
        assert!(inverse_dynamics(&model, &q, &map(&[("j", f64::NAN)]), &ok, &o).is_err());
    }
}
