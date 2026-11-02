// =============================================================================
// HYDRA-UMC-PHYSICS-REPLICA - src/selfcollision.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0-or-later - see LICENSE
// =============================================================================
//! Self-collision with capsules built from the `<collision>` shapes of the URDF.
//!
//! Every shape of every link is replaced by a capsule (two end points and a radius, in the frame of its link) that CONTAINS it, so the
//! check may report a touch that would not happen and never misses one that would:
//!
//! - a sphere is a capsule of length zero;
//! - a cylinder is the capsule with the same axis, between the centres of its two flat faces, and the same radius;
//! - a box is the capsule along its longest side, between the centres of the two faces at its ends, with the radius of the diagonal of
//!   the other two sides;
//! - a mesh has no capsule here (its triangles are not read): it is an error unless the configuration declares `mesh_radius_m`, the radius
//!   of a sphere around the origin of the mesh, which is then the operator's claim;
//! - a shape that cannot be read is an error, never skipped.
//!
//! Between two states of the trajectory the joints are interpolated linearly and sampled so that no point of the robot can have moved
//! further than the allowance from its place in the nearest sample (a bound from the offsets and the sizes of the capsules, the same
//! argument as in `collision.rs`); both capsules of a pair may move, so twice the allowance is added to the margin of the pair.
//!
//! Pairs of links that touch by construction (the same link, a link and its neighbour) are not compared: `min_link_distance` is the number
//! of joints between two links under which a pair is skipped (default 2: neighbours are skipped), and `ignore_pairs` names further pairs
//! that are allowed to touch. The capsules of the root link do not move. Not tried with a real robot.

use std::collections::HashMap;

use serde_json::{json, Value};

use crate::collision::node_distance;
use crate::kinematics::{effective_positions, forward_kinematics_model};
use crate::links::Shape;
use crate::transform::Mat4;
use crate::urdf::{JointType, Model};

type P3 = [f64; 3];

/// Added to every margin so that the rounding of the arithmetic cannot hide a touch.
const FP_SLACK_M: f64 = 1e-9;
const MAX_LISTED_HITS: usize = 32;
pub const MAX_STATES: usize = 4096;

fn sub(a: P3, b: P3) -> P3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn dot(a: P3, b: P3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn norm(a: P3) -> f64 {
    dot(a, a).sqrt()
}

#[derive(Debug, Clone, PartialEq)]
pub struct Config {
    pub margin_m: f64,
    pub step_m: f64,
    pub max_substeps: usize,
    pub min_link_distance: usize,
    pub ignore_pairs: Vec<(String, String)>,
    pub mesh_radius_m: Option<f64>,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            margin_m: 0.0,
            step_m: 0.02,
            max_substeps: 200,
            min_link_distance: 2,
            ignore_pairs: Vec::new(),
            mesh_radius_m: None,
        }
    }
}

const CONFIG_FIELDS: [&str; 6] = [
    "margin_m",
    "step_m",
    "max_substeps",
    "min_link_distance",
    "ignore_pairs",
    "mesh_radius_m",
];

fn finite(value: &Value, what: &str) -> Result<f64, String> {
    value
        .as_f64()
        .filter(|n| n.is_finite())
        .ok_or_else(|| format!("{what} must be a finite number, got {value}"))
}

/// Reads the configuration object; every field is optional and an unknown field is an error.
pub fn parse_config(raw: &Value) -> Result<Config, String> {
    let object = raw
        .as_object()
        .ok_or("the self-collision configuration must be a JSON object")?;
    let mut unknown: Vec<&str> = object
        .keys()
        .map(String::as_str)
        .filter(|k| !CONFIG_FIELDS.contains(k))
        .collect();
    if !unknown.is_empty() {
        unknown.sort_unstable();
        return Err(format!(
            "the self-collision configuration has unknown field(s): {}",
            unknown.join(", ")
        ));
    }
    let mut config = Config::default();
    if let Some(v) = object.get("margin_m") {
        config.margin_m = finite(v, "margin_m")?;
        if config.margin_m < 0.0 {
            return Err("margin_m must not be negative: a margin can only grow the robot".into());
        }
    }
    if let Some(v) = object.get("step_m") {
        config.step_m = finite(v, "step_m")?;
        if !(0.001..=10.0).contains(&config.step_m) {
            return Err("step_m must be between 0.001 and 10".into());
        }
    }
    if let Some(v) = object.get("max_substeps") {
        config.max_substeps =
            v.as_u64()
                .filter(|n| (1..=10_000).contains(n))
                .ok_or("max_substeps must be a whole number from 1 to 10000")? as usize;
    }
    if let Some(v) = object.get("min_link_distance") {
        config.min_link_distance = v
            .as_u64()
            .filter(|n| (0..=16).contains(n))
            .ok_or("min_link_distance must be a whole number from 0 to 16")?
            as usize;
    }
    if let Some(v) = object.get("mesh_radius_m") {
        let r = finite(v, "mesh_radius_m")?;
        if r <= 0.0 {
            return Err("mesh_radius_m must be above zero".into());
        }
        config.mesh_radius_m = Some(r);
    }
    if let Some(v) = object.get("ignore_pairs") {
        let list = v
            .as_array()
            .ok_or("ignore_pairs must be a list of [link, link]")?;
        for pair in list {
            match pair.as_array().map(Vec::as_slice) {
                Some([Value::String(a), Value::String(b)]) => {
                    config.ignore_pairs.push((a.clone(), b.clone()))
                }
                _ => return Err("every entry of ignore_pairs must be [\"link\", \"link\"]".into()),
            }
        }
    }
    Ok(config)
}

/// A capsule in the frame of its link.
#[derive(Debug, Clone, PartialEq)]
pub struct Capsule {
    pub link: String,
    /// 0 for the root link, joint index + 1 for the link a joint carries.
    pub node: usize,
    pub a: P3,
    pub b: P3,
    pub radius: f64,
}

fn point(m: &Mat4, p: P3) -> P3 {
    let t = m.transform_point(crate::transform::Vec3::new(p[0], p[1], p[2]));
    [t.x, t.y, t.z]
}

fn capsule_of(shape: &Shape, origin: &Mat4, config: &Config) -> Result<(P3, P3, f64), String> {
    let (a, b, radius) = match shape {
        Shape::Sphere { radius } => ([0.0; 3], [0.0; 3], *radius),
        Shape::Cylinder { radius, length } => {
            ([0.0, 0.0, -length / 2.0], [0.0, 0.0, length / 2.0], *radius)
        }
        Shape::Box { size } => {
            let half = [size[0] / 2.0, size[1] / 2.0, size[2] / 2.0];
            let long = (0..3)
                .max_by(|&i, &j| half[i].total_cmp(&half[j]))
                .unwrap_or(0);
            let others: f64 = (0..3)
                .filter(|&i| i != long)
                .map(|i| half[i] * half[i])
                .sum();
            let mut a = [0.0; 3];
            let mut b = [0.0; 3];
            a[long] = -half[long];
            b[long] = half[long];
            (a, b, others.sqrt())
        }
        Shape::Mesh { filename } => match config.mesh_radius_m {
            Some(r) => ([0.0; 3], [0.0; 3], r),
            None => {
                return Err(format!(
                    "the mesh '{filename}' has no capsule: its triangles are not read; declare mesh_radius_m to stand for it with a sphere"
                ))
            }
        },
        Shape::Unusable(reason) => return Err(reason.clone()),
    };
    Ok((point(origin, a), point(origin, b), radius))
}

/// The capsules of every link, in the frame of its link.
pub fn capsules(model: &Model, config: &Config) -> Result<Vec<Capsule>, String> {
    let mut names: Vec<(usize, &str)> = vec![(0, model.root_link.as_str())];
    for (i, link) in model.child_links.iter().enumerate() {
        names.push((i + 1, link.as_str()));
    }
    let mut out = Vec::new();
    for (node, name) in names {
        let Some(info) = model.links.get(name) else {
            continue;
        };
        for (k, collision) in info.collisions.iter().enumerate() {
            let (a, b, radius) = capsule_of(&collision.shape, &collision.origin, config)
                .map_err(|e| format!("link '{name}', collision {k}: {e}"))?;
            out.push(Capsule {
                link: name.to_string(),
                node,
                a,
                b,
                radius,
            });
        }
    }
    Ok(out)
}

/// Shortest distance between the segments `p1-q1` and `p2-q2`.
pub fn segment_distance(p1: P3, q1: P3, p2: P3, q2: P3) -> f64 {
    let d1 = sub(q1, p1);
    let d2 = sub(q2, p2);
    let r = sub(p1, p2);
    let a = dot(d1, d1);
    let e = dot(d2, d2);
    let f = dot(d2, r);
    const EPS: f64 = 1e-18;
    let (s, t);
    if a <= EPS && e <= EPS {
        return norm(r);
    }
    if a <= EPS {
        s = 0.0;
        t = (f / e).clamp(0.0, 1.0);
    } else {
        let c = dot(d1, r);
        if e <= EPS {
            t = 0.0;
            s = (-c / a).clamp(0.0, 1.0);
        } else {
            let b = dot(d1, d2);
            let denom = a * e - b * b;
            let mut s_try = if denom > EPS {
                ((b * f - c * e) / denom).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let mut t_try = (b * s_try + f) / e;
            if t_try < 0.0 {
                t_try = 0.0;
                s_try = (-c / a).clamp(0.0, 1.0);
            } else if t_try > 1.0 {
                t_try = 1.0;
                s_try = ((b - c) / a).clamp(0.0, 1.0);
            }
            s = s_try;
            t = t_try;
        }
    }
    let c1 = [p1[0] + d1[0] * s, p1[1] + d1[1] * s, p1[2] + d1[2] * s];
    let c2 = [p2[0] + d2[0] * t, p2[1] + d2[1] * t, p2[2] + d2[2] * t];
    norm(sub(c1, c2))
}

#[derive(Debug, Clone, PartialEq)]
pub struct PairHit {
    pub at: f64,
    pub link_a: String,
    pub link_b: String,
    pub clearance_m: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Report {
    pub capsule_count: usize,
    pub pairs_compared: usize,
    pub states: usize,
    pub samples: usize,
    pub max_allowance_m: f64,
    pub min_clearance_m: Option<f64>,
    pub hits_total: usize,
    pub hits: Vec<PairHit>,
}

impl Report {
    pub fn collision(&self) -> bool {
        self.hits_total > 0
    }

    pub fn to_json(&self) -> Value {
        let hits: Vec<Value> = self
            .hits
            .iter()
            .map(|h| json!({"at": h.at, "link": h.link_a, "other_link": h.link_b, "clearance_m": h.clearance_m}))
            .collect();
        json!({
            "collision": self.collision(),
            "capsules": self.capsule_count,
            "pairs_compared": self.pairs_compared,
            "states": self.states,
            "samples": self.samples,
            "max_motion_allowance_m": self.max_allowance_m,
            "min_clearance_m": self.min_clearance_m,
            "hits_total": self.hits_total,
            "hits": hits,
            "model": "capsules that contain the <collision> shapes of the URDF; overestimates by construction, not tried with a real robot",
        })
    }
}

/// For each moving joint, how far any point of a capsule can be moved by one unit of that joint, whatever the pose.
fn lever_arms(model: &Model, capsules: &[Capsule], travel: &HashMap<String, f64>) -> Vec<f64> {
    let joints = &model.chain.joints;
    let mut reach = vec![0.0f64; joints.len()];
    for c in capsules {
        if c.node > 0 {
            let extent = norm(c.a).max(norm(c.b)) + c.radius;
            reach[c.node - 1] = reach[c.node - 1].max(extent);
        }
    }
    for k in (0..joints.len()).rev() {
        if let Some(parent) = model.parents[k] {
            let t = joints[k].origin.translation_part();
            let offset = (t.x * t.x + t.y * t.y + t.z * t.z).sqrt()
                + if joints[k].joint_type == JointType::Prismatic {
                    travel.get(&joints[k].name).copied().unwrap_or(0.0)
                } else {
                    0.0
                };
            reach[parent] = reach[parent].max(offset + reach[k]);
        }
    }
    joints
        .iter()
        .enumerate()
        .map(|(i, j)| match j.joint_type {
            JointType::Fixed => 0.0,
            JointType::Prismatic => 1.0,
            JointType::Revolute | JointType::Continuous => reach[i],
        })
        .collect()
}

fn ignored(config: &Config, a: &str, b: &str) -> bool {
    config
        .ignore_pairs
        .iter()
        .any(|(x, y)| (x == a && y == b) || (x == b && y == a))
}

struct Pairs {
    list: Vec<(usize, usize)>,
}

fn comparable_pairs(model: &Model, caps: &[Capsule], config: &Config) -> Pairs {
    let mut list = Vec::new();
    for i in 0..caps.len() {
        for j in (i + 1)..caps.len() {
            let (a, b) = (&caps[i], &caps[j]);
            if a.node == b.node
                || ignored(config, &a.link, &b.link)
                || node_distance(model, a.node, b.node) < config.min_link_distance
            {
                continue;
            }
            list.push((i, j));
        }
    }
    Pairs { list }
}

/// Checks a trajectory (joints interpolated linearly between the states) for contact between the capsules of the robot.
pub fn check_states(
    model: &Model,
    config: &Config,
    states: &[HashMap<String, f64>],
) -> Result<Report, String> {
    if states.is_empty() {
        return Err("the trajectory has no states".into());
    }
    if states.len() > MAX_STATES {
        return Err(format!("the trajectory has more than {MAX_STATES} states"));
    }
    let caps = capsules(model, config)?;
    let pairs = comparable_pairs(model, &caps, config);
    let effective: Vec<HashMap<String, f64>> = states
        .iter()
        .enumerate()
        .map(|(i, s)| {
            for name in s.keys() {
                if model.joint_index(name).is_none() {
                    return Err(format!("state {i}: unknown joint {name:?}"));
                }
            }
            effective_positions(model, s).map_err(|e| format!("state {i}: {e}"))
        })
        .collect::<Result<_, _>>()?;
    let joints = &model.chain.joints;
    let moving: Vec<usize> = (0..joints.len())
        .filter(|&i| joints[i].joint_type != JointType::Fixed)
        .collect();

    let mut samples = 0usize;
    let mut max_allowance = 0.0f64;
    let mut min_clearance = f64::INFINITY;
    let mut hits_total = 0usize;
    let mut hits: Vec<PairHit> = Vec::new();
    let mut seen: std::collections::HashSet<(usize, usize)> = Default::default();

    let mut check =
        |given: &HashMap<String, f64>, at: f64, pair_allowance: f64| -> Result<(), String> {
            let worlds = forward_kinematics_model(model, given).map_err(|e| e.to_string())?;
            let world_of = |node: usize| -> Mat4 {
                if node == 0 {
                    Mat4::identity()
                } else {
                    worlds[node - 1].1
                }
            };
            let placed: Vec<(P3, P3)> = caps
                .iter()
                .map(|c| {
                    let w = world_of(c.node);
                    (point(&w, c.a), point(&w, c.b))
                })
                .collect();
            let margin = config.margin_m + pair_allowance + FP_SLACK_M;
            samples += 1;
            for &(i, j) in &pairs.list {
                let d = segment_distance(placed[i].0, placed[i].1, placed[j].0, placed[j].1);
                let clearance = d - caps[i].radius - caps[j].radius - margin;
                min_clearance = min_clearance.min(clearance);
                if clearance <= 0.0 {
                    hits_total += 1;
                    if hits.len() < MAX_LISTED_HITS && seen.insert((i, j)) {
                        hits.push(PairHit {
                            at,
                            link_a: caps[i].link.clone(),
                            link_b: caps[j].link.clone(),
                            clearance_m: clearance,
                        });
                    }
                }
            }
            Ok(())
        };

    if states.len() == 1 {
        check(&states[0], 0.0, 0.0)?;
    }
    for pair in 0..states.len().saturating_sub(1) {
        let (qa, qb) = (&effective[pair], &effective[pair + 1]);
        let travel: HashMap<String, f64> = moving
            .iter()
            .map(|&i| {
                let name = &joints[i].name;
                (
                    name.clone(),
                    qa.get(name)
                        .copied()
                        .unwrap_or(0.0)
                        .abs()
                        .max(qb.get(name).copied().unwrap_or(0.0).abs()),
                )
            })
            .collect();
        let arms = lever_arms(model, &caps, &travel);
        let total: f64 = moving
            .iter()
            .map(|&i| {
                let name = &joints[i].name;
                (qb[name] - qa[name]).abs() * arms[i]
            })
            .sum();
        let steps = ((total / config.step_m).ceil() as usize).clamp(1, config.max_substeps);
        // a point is at most total / (2 steps) from its place in the nearest sample; both capsules of a pair may move
        let pair_allowance = total / steps as f64;
        max_allowance = max_allowance.max(pair_allowance);
        let first = if pair == 0 { 0 } else { 1 };
        for k in first..=steps {
            let t = k as f64 / steps as f64;
            let mut given = HashMap::new();
            for (name, a) in &states[pair] {
                let b = states[pair + 1].get(name).copied().unwrap_or(*a);
                given.insert(name.clone(), a + (b - a) * t);
            }
            for (name, b) in &states[pair + 1] {
                given.entry(name.clone()).or_insert(*b);
            }
            check(&given, pair as f64 + t, pair_allowance)?;
        }
    }
    Ok(Report {
        capsule_count: caps.len(),
        pairs_compared: pairs.list.len(),
        states: states.len(),
        samples,
        max_allowance_m: max_allowance,
        min_clearance_m: Some(min_clearance).filter(|v| v.is_finite()),
        hits_total,
        hits,
    })
}

/// The capsules in the frame of their links, for a consumer that masks the robot out of what a sensor sees.
pub fn capsules_json(caps: &[Capsule]) -> Value {
    Value::Array(
        caps.iter()
            .map(|c| json!({"link": c.link, "a": c.a, "b": c.b, "radius_m": c.radius}))
            .collect(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::urdf::parse_urdf_model;

    /// A shoulder that swings a long arm (a cylinder along x) over a fixed post (a box) beside it.
    const ARM: &str = r#"<robot name="t">
      <link name="base">
        <collision><origin xyz="0 0 0"/><geometry><box size="0.1 0.1 0.1"/></geometry></collision>
      </link>
      <link name="arm">
        <collision><origin xyz="0.25 0 0" rpy="0 1.5707963267948966 0"/><geometry><cylinder radius="0.02" length="0.5"/></geometry></collision>
      </link>
      <link name="post">
        <collision><geometry><sphere radius="0.03"/></geometry></collision>
      </link>
      <joint name="shoulder" type="revolute"><parent link="base"/><child link="arm"/>
        <origin xyz="0 0 0.3"/><axis xyz="0 0 1"/><limit lower="-3.1" upper="3.1" effort="1" velocity="1"/></joint>
      <joint name="mount" type="fixed"><parent link="base"/><child link="post"/>
        <origin xyz="0 0.4 0.3"/></joint>
    </robot>"#;

    fn state(q: f64) -> HashMap<String, f64> {
        HashMap::from([("shoulder".to_string(), q)])
    }

    #[test]
    fn capsules_contain_their_shapes() {
        let model = parse_urdf_model(ARM).unwrap();
        let caps = capsules(&model, &Config::default()).unwrap();
        assert_eq!(caps.len(), 3);
        let base = &caps[0];
        // the box 0.1 cube: longest side ties to x, radius = diagonal of the other two half sides
        assert!((base.radius - (0.05f64 * 0.05 * 2.0).sqrt()).abs() < 1e-12);
        let arm = caps.iter().find(|c| c.link == "arm").unwrap();
        // the cylinder was turned onto x by the rpy, centred at x = 0.25
        let (lo, hi) = if arm.a[0] < arm.b[0] {
            (arm.a, arm.b)
        } else {
            (arm.b, arm.a)
        };
        assert!(lo[0].abs() < 1e-9 && (hi[0] - 0.5).abs() < 1e-9);
        assert_eq!(arm.radius, 0.02);
    }

    #[test]
    fn a_swing_into_the_post_is_a_contact_and_a_swing_away_is_not() {
        let model = parse_urdf_model(ARM).unwrap();
        let config = Config::default();
        // pointing along +x: far from the post at y = 0.4
        let clear = check_states(&model, &config, &[state(0.0)]).unwrap();
        assert!(!clear.collision());
        assert!(clear.min_clearance_m.unwrap() > 0.3);
        // quarter turn: the arm points at the post (y 0.4) and the tip passes through it
        let hit = check_states(&model, &config, &[state(std::f64::consts::FRAC_PI_2)]).unwrap();
        assert!(hit.collision());
        assert_eq!(hit.hits[0].link_a, "arm");
        assert_eq!(hit.hits[0].link_b, "post");
        // a swing between two clear states that crosses the post is caught between the samples
        let swing = check_states(&model, &config, &[state(0.3), state(1.9)]).unwrap();
        assert!(swing.collision());
        assert!(swing.samples > 2);
        let away = check_states(&model, &config, &[state(-0.2), state(-1.2)]).unwrap();
        assert!(!away.collision());
    }

    #[test]
    fn neighbours_and_ignored_pairs_are_not_compared() {
        let model = parse_urdf_model(ARM).unwrap();
        let mut config = Config::default();
        // base-arm are neighbours: skipped by the default; arm-post are two joints apart
        let caps = capsules(&model, &config).unwrap();
        let pairs = comparable_pairs(&model, &caps, &config);
        assert!(pairs
            .list
            .iter()
            .all(|&(i, j)| !(caps[i].link == "base" && caps[j].link == "arm")));
        config.ignore_pairs.push(("arm".into(), "post".into()));
        let pairs = comparable_pairs(&model, &caps, &config);
        assert!(pairs
            .list
            .iter()
            .all(|&(i, j)| !(caps[i].link == "arm" && caps[j].link == "post")));
        let hit = check_states(&model, &config, &[state(std::f64::consts::FRAC_PI_2)]).unwrap();
        assert!(!hit.collision());
    }

    #[test]
    fn a_mesh_or_a_bad_shape_is_an_error_unless_declared() {
        let xml = r#"<robot name="m"><link name="a"><collision><geometry><mesh filename="a.stl"/></geometry></collision></link>
          <link name="b"/><joint name="j" type="fixed"><parent link="a"/><child link="b"/></joint></robot>"#;
        let model = parse_urdf_model(xml).unwrap();
        assert!(capsules(&model, &Config::default())
            .unwrap_err()
            .contains("mesh_radius_m"));
        let config = Config {
            mesh_radius_m: Some(0.1),
            ..Config::default()
        };
        assert_eq!(capsules(&model, &config).unwrap()[0].radius, 0.1);
        let bad = r#"<robot name="m"><link name="a"><collision><geometry><cylinder radius="0" length="1"/></geometry></collision></link></robot>"#;
        let model = parse_urdf_model(bad).unwrap();
        assert!(capsules(&model, &Config::default()).is_err());
    }

    #[test]
    fn segment_distance_matches_known_cases() {
        // crossing in 3D, one unit apart
        let d = segment_distance(
            [-1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 1.0],
            [0.0, 1.0, 1.0],
        );
        assert!((d - 1.0).abs() < 1e-12);
        // parallel, offset 2
        let d = segment_distance([0.0; 3], [1.0, 0.0, 0.0], [0.0, 2.0, 0.0], [1.0, 2.0, 0.0]);
        assert!((d - 2.0).abs() < 1e-12);
        // end to end
        let d = segment_distance([0.0; 3], [1.0, 0.0, 0.0], [3.0, 0.0, 0.0], [4.0, 0.0, 0.0]);
        assert!((d - 2.0).abs() < 1e-12);
        // points
        let d = segment_distance([0.0; 3], [0.0; 3], [3.0, 4.0, 0.0], [3.0, 4.0, 0.0]);
        assert!((d - 5.0).abs() < 1e-12);
    }

    #[test]
    fn config_parsing_is_strict() {
        let c = parse_config(
            &json!({"margin_m": 0.01, "ignore_pairs": [["a", "b"]], "mesh_radius_m": 0.2}),
        )
        .unwrap();
        assert_eq!(c.ignore_pairs, vec![("a".to_string(), "b".to_string())]);
        assert!(parse_config(&json!({"nope": 1})).is_err());
        assert!(parse_config(&json!({"margin_m": -1})).is_err());
        assert!(parse_config(&json!({"ignore_pairs": [["a"]]})).is_err());
        assert!(parse_config(&json!([])).is_err());
    }
}
