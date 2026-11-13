// =============================================================================
// HYDRA-UMC-PHYSICS-REPLICA - src/simulation_bench.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0-or-later - see LICENSE
// =============================================================================
//! The simulation bench of the dynamics: scenarios drawn from a fixed seed (a pendulum let go from many states, balls dropped and thrown with many
//! restitutions and frictions) run through the real integrator and the real contact model, and checked against what physics requires within a
//! stated tolerance: a pendulum keeps its energy and loses it faster the coarser the step, a ball never goes below the ground, never gains
//! mechanical energy, and ends at rest. Nothing here touches a robot: a pass means "passed in simulation". The numbers are those of a model.

use std::collections::HashMap;

use crate::dynamics::{energy, Options};
use crate::rigidbody::{free_joint_names, rk4_step, Ball};
use crate::urdf::{parse_urdf_model, Model};

const G: f64 = 9.81;

const PENDULUM: &str = r#"<robot name="p">
  <link name="base"/>
  <link name="rod"><inertial><origin xyz="0.4 0 0"/><mass value="2"/>
    <inertia ixx="0" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0"/></inertial></link>
  <joint name="j" type="revolute"><parent link="base"/><child link="rod"/>
    <axis xyz="0 -1 0"/><limit lower="-3" upper="3" effort="50" velocity="5"/></joint>
</robot>"#;

/// A small generator with a fixed seed.
struct Lcg(u64);

impl Lcg {
    fn unit(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((self.0 >> 11) as f64) / ((1u64 << 53) as f64)
    }
    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.unit()
    }
}

fn total_energy(model: &Model, names: &[String], q: &[f64], qd: &[f64]) -> f64 {
    let positions: HashMap<String, f64> = names.iter().cloned().zip(q.iter().copied()).collect();
    let velocities: HashMap<String, f64> = names.iter().cloned().zip(qd.iter().copied()).collect();
    let (kinetic, potential) = energy(model, &positions, &velocities, &Options::default()).unwrap();
    kinetic + potential
}

/// The energy a pendulum let go from `(q0, qd0)` has lost (or gained) after `seconds`, integrated with the step `dt`.
fn pendulum_drift(q0: f64, qd0: f64, dt: f64, seconds: f64) -> f64 {
    let model = parse_urdf_model(PENDULUM).unwrap();
    let names = free_joint_names(&model);
    let mut q = vec![q0];
    let mut qd = vec![qd0];
    let start = total_energy(&model, &names, &q, &qd);
    let steps = (seconds / dt).round() as usize;
    for _ in 0..steps {
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
    }
    (total_energy(&model, &names, &q, &qd) - start).abs()
}

#[test]
fn a_pendulum_let_go_from_many_states_keeps_its_energy() {
    let mut rng = Lcg(2024);
    for case in 0..12 {
        let q0 = rng.range(
            -std::f64::consts::FRAC_PI_2 - 1.2,
            -std::f64::consts::FRAC_PI_2 + 1.2,
        );
        let qd0 = rng.range(-2.0, 2.0);
        let drift = pendulum_drift(q0, qd0, 1e-3, 2.0);
        assert!(drift < 1e-5, "case {case} ({q0}, {qd0}): lost {drift} J");
    }
}

#[test]
fn a_coarser_step_loses_more_energy_than_a_finer_one() {
    let mut rng = Lcg(31);
    for case in 0..4 {
        let q0 = rng.range(-1.2, 0.2);
        let qd0 = rng.range(-1.0, 1.0);
        let fine = pendulum_drift(q0, qd0, 0.005, 2.0);
        let coarse = pendulum_drift(q0, qd0, 0.04, 2.0);
        assert!(
            coarse > fine,
            "case {case}: the step of 0.04 s lost {coarse} J, the step of 0.005 s lost {fine} J"
        );
    }
}

#[test]
fn the_same_start_gives_the_same_swing_bit_for_bit() {
    let first = pendulum_drift(-0.7, 0.4, 0.01, 1.0);
    let second = pendulum_drift(-0.7, 0.4, 0.01, 1.0);
    assert_eq!(first.to_bits(), second.to_bits());
}

#[test]
fn balls_dropped_and_thrown_never_go_below_the_ground_never_gain_energy_and_come_to_rest() {
    let mut rng = Lcg(7);
    for case in 0..40 {
        let mass = rng.range(0.1, 5.0);
        let radius = rng.range(0.02, 0.3);
        let height = rng.range(0.5, 3.0);
        let mut ball = Ball::new(mass, radius, [0.0, 0.0, radius + height]);
        ball.restitution = rng.range(0.2, 0.9);
        ball.friction = rng.range(0.0, 0.6);
        ball.velocity = [
            rng.range(0.0, 3.0),
            rng.range(-1.0, 1.0),
            rng.range(-2.0, 2.0),
        ];
        let mechanical = |b: &Ball| b.kinetic_energy() + b.mass * G * b.position[2];
        let start = mechanical(&ball);
        let mut last = start;
        for step in 0..8000 {
            ball.step(0.005, G);
            assert!(
                ball.clearance() >= -1e-9,
                "case {case}, step {step}: {} below the ground",
                ball.clearance()
            );
            let now = mechanical(&ball);
            assert!(
                now <= last + 1e-9 * (1.0 + start.abs()),
                "case {case}, step {step}: energy went from {last} to {now}"
            );
            last = now;
        }
        assert!(ball.resting, "case {case}: still bouncing after 40 s");
        assert_eq!(ball.velocity[2], 0.0, "case {case}");
        assert!(ball.impacts >= 1, "case {case}");
    }
}

#[test]
fn the_first_impact_comes_at_the_instant_the_height_says_for_any_drop() {
    let mut rng = Lcg(99);
    for case in 0..10 {
        let height = rng.range(0.2, 4.0);
        let radius = rng.range(0.02, 0.2);
        let mut ball = Ball::new(1.0, radius, [0.0, 0.0, radius + height]);
        let fall = (2.0 * height / G).sqrt();
        ball.step(fall - 1e-6, G);
        assert_eq!(ball.impacts, 0, "case {case}: before the instant");
        ball.step(2e-6, G);
        assert_eq!(ball.impacts, 1, "case {case}: after the instant");
    }
}

#[test]
fn every_bounce_returns_the_fraction_of_the_speed_the_restitution_says() {
    let mut rng = Lcg(5);
    for case in 0..10 {
        let restitution = rng.range(0.3, 0.9);
        let height = rng.range(0.5, 2.0);
        let mut ball = Ball::new(1.0, 0.1, [0.0, 0.0, 0.1 + height]);
        ball.restitution = restitution;
        let fall = (2.0 * height / G).sqrt();
        let arrival = G * fall;
        ball.step(fall + 1e-6, G);
        assert_eq!(ball.impacts, 1, "case {case}");
        let leaving = ball.velocity[2] + G * 1e-6;
        assert!(
            (leaving - restitution * arrival).abs() < 1e-6,
            "case {case}: left at {leaving}, expected {}",
            restitution * arrival
        );
    }
}
