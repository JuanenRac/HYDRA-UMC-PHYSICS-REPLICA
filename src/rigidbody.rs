// =============================================================================
// HYDRA-UMC-PHYSICS-REPLICA - src/rigidbody.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0-or-later - see LICENSE
// =============================================================================
//! A small rigid-body engine for the hardware-in-the-loop bench: it moves a robot forward in time and drops free bodies on the ground.
//!
//! Two parts, both written here with no physics dependency:
//!
//! * **Articulated robots.** `forward_dynamics` gives the joint accelerations produced by given joint torques, from the same RNEA as
//!   `dynamics::inverse_dynamics`: the bias term (gravity, Coriolis and centrifugal) is the torque of a motion with no acceleration, and each
//!   column of the mass matrix is the torque of a unit acceleration minus the gravity torque. `rk4_step` integrates the state with the classical
//!   fourth-order Runge-Kutta rule. The link masses and inertias come from the URDF `<inertial>` blocks, as in the inverse dynamics.
//! * **Free bodies and contacts.** `Ball` is a sphere under gravity above a ground plane. A step is split at the exact instant of impact (the flight
//!   between two impacts is a parabola, so the instant is the root of a quadratic), the impact applies the restitution to the normal speed and a
//!   Coulomb impulse to the tangential one, and a body whose bounce is slower than a threshold comes to rest and slides to a stop.
//!
//! What it checks itself against (see the tests): the period and the energy of a pendulum, the equations of the planar two-link arm, free fall,
//! the height and the instant of the bounces, the loss of tangential speed of a frictional impact, and the rest on the ground. What it does not
//! model: contacts between bodies or with a link of the robot, spin of a ball, joint friction, motor dynamics and flexible parts. The numbers are
//! the ones of a model, not measurements of a robot: any use against a real arm is evidence=simulator until it is compared with one.

#![allow(dead_code)]
#![allow(clippy::needless_range_loop)]

use std::collections::HashMap;

use crate::dynamics::{inverse_dynamics, Options};
use crate::urdf::{JointType, Model};

/// The names of the joints that move by themselves (not fixed, not followers), in the order of the URDF.
pub fn free_joint_names(model: &Model) -> Vec<String> {
    model
        .chain
        .joints
        .iter()
        .enumerate()
        .filter(|(i, j)| j.joint_type != JointType::Fixed && model.extras[*i].mimic.is_none())
        .map(|(_, j)| j.name.clone())
        .collect()
}

fn to_map(names: &[String], values: &[f64]) -> HashMap<String, f64> {
    names.iter().cloned().zip(values.iter().copied()).collect()
}

/// Solve `a x = b` by Gaussian elimination with partial pivoting. `a` is the mass matrix, symmetric and positive definite for a robot with mass.
fn solve(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Result<Vec<f64>, String> {
    let n = b.len();
    for col in 0..n {
        let pivot = (col..n)
            .max_by(|&x, &y| a[x][col].abs().total_cmp(&a[y][col].abs()))
            .ok_or("empty system")?;
        if a[pivot][col].abs() < 1e-12 {
            return Err(
                "the mass matrix is singular: a joint carries no mass or inertia".to_string(),
            );
        }
        a.swap(col, pivot);
        b.swap(col, pivot);
        for row in col + 1..n {
            let factor = a[row][col] / a[col][col];
            for k in col..n {
                let delta = factor * a[col][k];
                a[row][k] -= delta;
            }
            let delta = factor * b[col];
            b[row] -= delta;
        }
    }
    let mut x = vec![0.0; n];
    for row in (0..n).rev() {
        let mut sum = b[row];
        for k in row + 1..n {
            sum -= a[row][k] * x[k];
        }
        x[row] = sum / a[row][row];
    }
    Ok(x)
}

/// The accelerations of the free joints (in the order of `names`) for the state and the torques given.
pub fn forward_accelerations(
    model: &Model,
    names: &[String],
    q: &[f64],
    qd: &[f64],
    tau: &[f64],
    options: &Options,
) -> Result<Vec<f64>, String> {
    let n = names.len();
    let zero = vec![0.0; n];
    let positions = to_map(names, q);
    let velocities = to_map(names, qd);
    let zeros = to_map(names, &zero);
    let torque = |velocity: &HashMap<String, f64>,
                  acceleration: &HashMap<String, f64>|
     -> Result<Vec<f64>, String> {
        let t = inverse_dynamics(model, &positions, velocity, acceleration, options)?;
        names
            .iter()
            .map(|name| {
                t.torques
                    .get(name)
                    .copied()
                    .ok_or_else(|| format!("no torque for joint {name}"))
            })
            .collect()
    };
    let bias = torque(&velocities, &zeros)?;
    let gravity = torque(&zeros, &zeros)?;
    let mut mass = vec![vec![0.0; n]; n];
    for j in 0..n {
        let mut unit = zero.clone();
        unit[j] = 1.0;
        let column = torque(&zeros, &to_map(names, &unit))?;
        for i in 0..n {
            mass[i][j] = column[i] - gravity[i];
        }
    }
    let rhs: Vec<f64> = (0..n).map(|i| tau[i] - bias[i]).collect();
    solve(mass, rhs)
}

/// The torque of a position motor: `kp * error - kd * speed`, saturated at `max_torque` in both directions (the proportional-derivative law, with the same
/// saturation, that the twin gives a rigid joint). `error` is the target minus the position.
pub fn pd_torque(kp: f64, kd: f64, max_torque: f64, error: f64, speed: f64) -> f64 {
    (kp * error - kd * speed).clamp(-max_torque, max_torque)
}

/// One step of the classical Runge-Kutta rule for the free joints. `tau` is held constant during the step.
pub fn rk4_step(
    model: &Model,
    names: &[String],
    q: &mut [f64],
    qd: &mut [f64],
    tau: &[f64],
    dt: f64,
    options: &Options,
) -> Result<(), String> {
    let n = names.len();
    let a1 = forward_accelerations(model, names, q, qd, tau, options)?;
    let v1 = qd.to_vec();
    let q2: Vec<f64> = (0..n).map(|i| q[i] + 0.5 * dt * v1[i]).collect();
    let v2: Vec<f64> = (0..n).map(|i| qd[i] + 0.5 * dt * a1[i]).collect();
    let a2 = forward_accelerations(model, names, &q2, &v2, tau, options)?;
    let q3: Vec<f64> = (0..n).map(|i| q[i] + 0.5 * dt * v2[i]).collect();
    let v3: Vec<f64> = (0..n).map(|i| qd[i] + 0.5 * dt * a2[i]).collect();
    let a3 = forward_accelerations(model, names, &q3, &v3, tau, options)?;
    let q4: Vec<f64> = (0..n).map(|i| q[i] + dt * v3[i]).collect();
    let v4: Vec<f64> = (0..n).map(|i| qd[i] + dt * a3[i]).collect();
    let a4 = forward_accelerations(model, names, &q4, &v4, tau, options)?;
    for i in 0..n {
        q[i] += dt / 6.0 * (v1[i] + 2.0 * v2[i] + 2.0 * v3[i] + v4[i]);
        qd[i] += dt / 6.0 * (a1[i] + 2.0 * a2[i] + 2.0 * a3[i] + a4[i]);
    }
    Ok(())
}

/// A sphere under gravity above a horizontal ground. Gravity pulls along -z.
#[derive(Debug, Clone, PartialEq)]
pub struct Ball {
    pub mass: f64,
    pub radius: f64,
    pub position: [f64; 3],
    pub velocity: [f64; 3],
    /// Fraction of the normal speed that the impact gives back (0 to 1).
    pub restitution: f64,
    /// Coulomb coefficient of the impact and of the sliding on the ground.
    pub friction: f64,
    /// Height of the ground plane.
    pub ground: f64,
    /// True while the ball lies on the ground.
    pub resting: bool,
    /// Impacts so far.
    pub impacts: usize,
}

/// A bounce whose normal speed before the impact is below this (m/s) is not a bounce: the ball comes to rest.
pub const REST_SPEED: f64 = 1e-3;

impl Ball {
    pub fn new(mass: f64, radius: f64, position: [f64; 3]) -> Ball {
        Ball {
            mass,
            radius,
            position,
            velocity: [0.0; 3],
            restitution: 0.8,
            friction: 0.0,
            ground: 0.0,
            resting: false,
            impacts: 0,
        }
    }

    /// Height of the lowest point of the ball above the ground.
    pub fn clearance(&self) -> f64 {
        self.position[2] - self.radius - self.ground
    }

    pub fn kinetic_energy(&self) -> f64 {
        0.5 * self.mass * self.velocity.iter().map(|v| v * v).sum::<f64>()
    }

    /// Advance by `dt` seconds under the gravity `g` (a positive number, metres per second squared).
    pub fn step(&mut self, dt: f64, g: f64) {
        let mut left = dt;
        // a bounce that loses a fixed fraction of its speed ends: the guard only stops a pathological input (a restitution of 1 with g = 0)
        for _ in 0..100_000 {
            if left <= 0.0 {
                return;
            }
            if self.resting {
                self.slide(&mut left, g);
                return;
            }
            let h = self.clearance().max(0.0);
            let vz = self.velocity[2];
            // z(t) = h + vz t - g t^2 / 2 reaches the ground at the positive root of g t^2 / 2 - vz t - h = 0
            let hit = if g > 0.0 {
                (vz + (vz * vz + 2.0 * g * h).sqrt()) / g
            } else {
                f64::INFINITY
            };
            if hit >= left || hit.is_nan() {
                self.fly(left, g);
                return;
            }
            self.fly(hit, g);
            left -= hit;
            self.position[2] = self.ground + self.radius;
            let normal = -self.velocity[2];
            if normal < REST_SPEED {
                self.velocity[2] = 0.0;
                self.resting = true;
                continue;
            }
            self.impacts += 1;
            self.velocity[2] = self.restitution * normal;
            let tangent =
                (self.velocity[0] * self.velocity[0] + self.velocity[1] * self.velocity[1]).sqrt();
            if tangent > 0.0 {
                // the impulse of the impact is m (1 + e) normal; Coulomb friction takes at most mu times that from the tangential momentum
                let lost = self.friction * (1.0 + self.restitution) * normal;
                let kept = (tangent - lost).max(0.0);
                let ratio = kept / tangent;
                self.velocity[0] *= ratio;
                self.velocity[1] *= ratio;
            }
        }
    }

    fn fly(&mut self, t: f64, g: f64) {
        for axis in 0..2 {
            self.position[axis] += self.velocity[axis] * t;
        }
        self.position[2] += self.velocity[2] * t - 0.5 * g * t * t;
        self.velocity[2] -= g * t;
    }

    /// On the ground: the sliding speed falls at mu g until it is zero.
    fn slide(&mut self, left: &mut f64, g: f64) {
        let speed =
            (self.velocity[0] * self.velocity[0] + self.velocity[1] * self.velocity[1]).sqrt();
        if speed <= 0.0 || self.friction <= 0.0 || g <= 0.0 {
            for axis in 0..2 {
                self.position[axis] += self.velocity[axis] * *left;
            }
            *left = 0.0;
            if self.friction > 0.0 && g > 0.0 {
                self.velocity = [0.0; 3];
            }
            return;
        }
        let decel = self.friction * g;
        let stop = speed / decel;
        if stop <= *left {
            for axis in 0..2 {
                self.position[axis] += self.velocity[axis] * 0.5 * stop;
                self.velocity[axis] = 0.0;
            }
        } else {
            let ratio = (speed - decel * *left) / speed;
            for axis in 0..2 {
                self.position[axis] += self.velocity[axis] * (1.0 + ratio) * 0.5 * *left;
                self.velocity[axis] *= ratio;
            }
        }
        *left = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dynamics::energy;
    use crate::urdf::parse_urdf_model;

    const G: f64 = 9.81;

    const PENDULUM: &str = r#"<robot name="p">
      <link name="base"/>
      <link name="rod"><inertial><origin xyz="0.4 0 0"/><mass value="2"/>
        <inertia ixx="0" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0"/></inertial></link>
      <joint name="j" type="revolute"><parent link="base"/><child link="rod"/>
        <axis xyz="0 -1 0"/><limit lower="-3" upper="3" effort="50" velocity="5"/></joint>
    </robot>"#;

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

    fn total_energy(model: &Model, names: &[String], q: &[f64], qd: &[f64]) -> f64 {
        let (k, u) = energy(
            model,
            &to_map(names, q),
            &to_map(names, qd),
            &Options::default(),
        )
        .unwrap();
        k + u
    }

    /// Hanging straight down is theta = -pi/2 for this axis; a small swing has the period 2 pi sqrt((I + m l^2) / (m g l)).
    #[test]
    fn a_small_swing_of_a_pendulum_follows_the_closed_form() {
        let model = parse_urdf_model(PENDULUM).unwrap();
        let names = free_joint_names(&model);
        assert_eq!(names, vec!["j".to_string()]);
        let (m, l, i_com) = (2.0, 0.4, 0.05);
        let omega = (m * G * l / (i_com + m * l * l)).sqrt();
        let amplitude = 0.01;
        let mut q = vec![-std::f64::consts::FRAC_PI_2 + amplitude];
        let mut qd = vec![0.0];
        let dt = 1e-3;
        for step in 1..=3000 {
            rk4_step(
                &model,
                &names,
                &mut q,
                &mut qd,
                &[0.0],
                dt,
                &Options::default(),
            )
            .unwrap();
            if step % 500 == 0 {
                let t = step as f64 * dt;
                let expected = -std::f64::consts::FRAC_PI_2 + amplitude * (omega * t).cos();
                assert!(
                    (q[0] - expected).abs() < 1e-5,
                    "t={t}: {} against {expected}",
                    q[0]
                );
            }
        }
    }

    #[test]
    fn a_large_swing_keeps_its_energy() {
        let model = parse_urdf_model(PENDULUM).unwrap();
        let names = free_joint_names(&model);
        let mut q = vec![0.3];
        let mut qd = vec![0.0];
        let start = total_energy(&model, &names, &q, &qd);
        for _ in 0..5000 {
            rk4_step(
                &model,
                &names,
                &mut q,
                &mut qd,
                &[0.0],
                1e-3,
                &Options::default(),
            )
            .unwrap();
        }
        let end = total_energy(&model, &names, &q, &qd);
        assert!((end - start).abs() < 1e-6, "{start} -> {end}");
        assert!(qd[0].abs() > 0.1 || q[0] != 0.3, "the pendulum moved");
    }

    #[test]
    fn a_torque_holds_a_pendulum_where_it_is() {
        let model = parse_urdf_model(PENDULUM).unwrap();
        let names = free_joint_names(&model);
        let theta = 0.7_f64;
        let hold = 2.0 * G * 0.4 * theta.cos();
        let a = forward_accelerations(
            &model,
            &names,
            &[theta],
            &[0.0],
            &[hold],
            &Options::default(),
        )
        .unwrap();
        assert!(a[0].abs() < 1e-9, "{}", a[0]);
        let free = forward_accelerations(
            &model,
            &names,
            &[theta],
            &[0.0],
            &[0.0],
            &Options::default(),
        )
        .unwrap();
        assert!((free[0] + 2.0 * G * 0.4 * theta.cos() / (0.05 + 2.0 * 0.16)).abs() < 1e-9);
    }

    /// Forward and inverse dynamics undo each other for the planar two-link arm in a moving state.
    #[test]
    fn forward_dynamics_inverts_the_inverse_dynamics_of_a_two_link_arm() {
        let model = parse_urdf_model(TWO_LINK).unwrap();
        let names = free_joint_names(&model);
        assert_eq!(names, vec!["a".to_string(), "b".to_string()]);
        for (q, qd, tau) in [
            ([0.3, 0.8], [0.0, 0.0], [1.0, -2.0]),
            ([-1.1, 1.9], [-2.0, 1.5], [0.0, 0.0]),
            ([0.5, -0.4], [1.2, -0.7], [5.0, 3.0]),
        ] {
            let acc =
                forward_accelerations(&model, &names, &q, &qd, &tau, &Options::default()).unwrap();
            let back = inverse_dynamics(
                &model,
                &to_map(&names, &q),
                &to_map(&names, &qd),
                &to_map(&names, &acc),
                &Options::default(),
            )
            .unwrap();
            assert!((back.torques["a"] - tau[0]).abs() < 1e-9);
            assert!((back.torques["b"] - tau[1]).abs() < 1e-9);
        }
    }

    #[test]
    fn a_two_link_arm_keeps_its_energy_while_it_falls() {
        let model = parse_urdf_model(TWO_LINK).unwrap();
        let names = free_joint_names(&model);
        let mut q = vec![0.4, 0.9];
        let mut qd = vec![0.0, 0.0];
        let start = total_energy(&model, &names, &q, &qd);
        for _ in 0..2000 {
            rk4_step(
                &model,
                &names,
                &mut q,
                &mut qd,
                &[0.0, 0.0],
                1e-3,
                &Options::default(),
            )
            .unwrap();
        }
        let end = total_energy(&model, &names, &q, &qd);
        assert!((end - start).abs() < 1e-5, "{start} -> {end}");
        assert!(qd[0].abs() + qd[1].abs() > 0.1, "the arm moved");
    }

    #[test]
    fn a_massless_joint_is_reported_not_divided_by() {
        let xml = r#"<robot name="m"><link name="base"/><link name="a"/>
          <joint name="j" type="revolute"><parent link="base"/><child link="a"/><axis xyz="0 0 1"/>
          <limit lower="-1" upper="1" effort="1" velocity="1"/></joint></robot>"#;
        let model = parse_urdf_model(xml).unwrap();
        let names = free_joint_names(&model);
        let options = Options {
            allow_missing_inertial: true,
            ..Options::default()
        };
        let error =
            forward_accelerations(&model, &names, &[0.0], &[0.0], &[1.0], &options).unwrap_err();
        assert!(error.contains("singular"), "{error}");
    }

    #[test]
    fn free_fall_follows_the_parabola_exactly() {
        let mut ball = Ball::new(1.0, 0.1, [0.0, 0.0, 2.1]);
        for _ in 0..50 {
            ball.step(0.01, G);
        }
        assert!((ball.position[2] - (2.1 - 0.5 * G * 0.25)).abs() < 1e-12);
        assert!((ball.velocity[2] + G * 0.5).abs() < 1e-12);
        assert_eq!(ball.impacts, 0);
    }

    #[test]
    fn the_first_impact_comes_when_the_drop_height_says() {
        let h: f64 = 1.0;
        let mut ball = Ball::new(1.0, 0.1, [0.0, 0.0, h + 0.1]);
        let dt = 1e-3;
        let mut time = 0.0;
        while ball.impacts == 0 {
            ball.step(dt, G);
            time += dt;
            assert!(time < 1.0);
        }
        let expected = (2.0 * h / G).sqrt();
        assert!(
            (time - expected).abs() <= dt + 1e-9,
            "{time} against {expected}"
        );
        // the rebound speed is e times the speed of the fall
        let fall = G * expected;
        assert!((ball.velocity[2] - 0.8 * fall).abs() <= G * dt + 1e-9);
    }

    #[test]
    fn a_bounce_rises_to_e_squared_times_the_drop_height() {
        let (h, e) = (1.0_f64, 0.8_f64);
        let mut ball = Ball::new(1.0, 0.1, [0.0, 0.0, h + 0.1]);
        ball.restitution = e;
        let dt = 1e-4;
        let mut apex = 0.0_f64;
        for _ in 0..30_000 {
            ball.step(dt, G);
            if ball.impacts == 1 {
                apex = apex.max(ball.clearance());
            }
            if ball.impacts == 2 {
                break;
            }
        }
        assert!(
            (apex - e * e * h).abs() < 1e-4,
            "{apex} against {}",
            e * e * h
        );
        // the second impact: the drop time, then the flight up and down at the rebound speed
        assert_eq!(ball.impacts, 2);
    }

    #[test]
    fn the_energy_after_a_bounce_is_e_squared_of_the_energy_before() {
        let mut ball = Ball::new(2.0, 0.05, [0.0, 0.0, 1.55]);
        ball.restitution = 0.5;
        let before = ball.mass * G * 1.5;
        while ball.impacts == 0 {
            ball.step(1e-4, G);
        }
        assert!((ball.kinetic_energy() - 0.25 * before).abs() < 1e-2 * before);
    }

    #[test]
    fn a_frictionless_impact_keeps_the_sideways_speed() {
        let mut ball = Ball::new(1.0, 0.1, [0.0, 0.0, 1.1]);
        ball.velocity = [2.0, -1.0, 0.0];
        ball.friction = 0.0;
        while ball.impacts == 0 {
            ball.step(1e-3, G);
        }
        assert!((ball.velocity[0] - 2.0).abs() < 1e-12);
        assert!((ball.velocity[1] + 1.0).abs() < 1e-12);
    }

    #[test]
    fn a_frictional_impact_takes_mu_times_the_impulse_from_the_sideways_speed() {
        let (e, mu) = (0.5_f64, 0.2_f64);
        let mut ball = Ball::new(1.0, 0.1, [0.0, 0.0, 1.1]);
        ball.restitution = e;
        ball.friction = mu;
        ball.velocity = [3.0, 0.0, 0.0];
        while ball.impacts == 0 {
            ball.step(1e-4, G);
        }
        // the normal speed at the impact comes from the fall; the sideways speed falls by mu (1 + e) times it
        let normal = (2.0 * G * 1.0_f64).sqrt();
        let expected = 3.0 - mu * (1.0 + e) * normal;
        assert!(
            (ball.velocity[0] - expected).abs() < 1e-9,
            "{} against {expected}",
            ball.velocity[0]
        );
        // the sideways speed never turns around
        ball.velocity = [0.1, 0.0, 0.0];
        ball.position = [0.0, 0.0, 1.1];
        ball.impacts = 0;
        ball.resting = false;
        while ball.impacts == 0 {
            ball.step(1e-3, G);
        }
        assert!(ball.velocity[0] >= 0.0);
    }

    #[test]
    fn a_ball_comes_to_rest_on_the_ground_and_stays() {
        let mut ball = Ball::new(1.0, 0.1, [0.0, 0.0, 1.1]);
        ball.restitution = 0.5;
        for _ in 0..10_000 {
            ball.step(1e-3, G);
        }
        assert!(ball.resting);
        assert_eq!(ball.velocity[2], 0.0);
        assert!((ball.position[2] - 0.1).abs() < 1e-12);
        assert!(ball.impacts >= 5, "{}", ball.impacts);
    }

    #[test]
    fn a_sliding_ball_stops_after_v_squared_over_two_mu_g() {
        let mu = 0.3;
        let mut ball = Ball::new(1.0, 0.1, [0.0, 0.0, 0.1]);
        ball.resting = true;
        ball.friction = mu;
        ball.velocity = [2.0, 0.0, 0.0];
        for _ in 0..2000 {
            ball.step(1e-3, G);
        }
        let distance = 2.0 * 2.0 / (2.0 * mu * G);
        assert!(
            (ball.position[0] - distance).abs() < 1e-9,
            "{} against {distance}",
            ball.position[0]
        );
        assert_eq!(ball.velocity[0], 0.0);
    }
}
