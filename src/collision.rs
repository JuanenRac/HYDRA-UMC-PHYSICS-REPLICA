// =============================================================================
// HYDRA-UMC-PHYSICS-REPLICA - src/collision.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0-or-later - see LICENSE
// =============================================================================
//! Collision prediction by forward kinematics: does a short trajectory of the robot enter a protected zone or the robot itself?
//!
//! The robot is reduced to a skeleton: for every joint, the segment from the origin of the frame of its parent to the origin of its own
//! frame (a prismatic joint adds the segment that its travel draws), and a sphere at the end of every joint that nothing hangs from (the
//! tool). Each segment gets a radius that the CONFIGURATION declares: the URDF of this project carries no collision geometry, so how thick
//! a link is cannot be known here and is never guessed. The zones are the ones of the zones file of HYDRA-UMC-SAFETY-ZONES (boxes and,
//! when the file switches them on, oriented boxes, vertical cylinders and capsules).
//!
//! The rule of the whole module is that it may report a contact that would not happen and must never miss one that would:
//!
//! - a segment is cut into pieces and each piece is replaced by the sphere that contains it, so the sphere is larger than the piece;
//! - the distance from a point to a zone is the exact distance to the solid reduced by a nanometre, like in the zones service;
//! - between two states of the trajectory the joints are interpolated linearly and sampled so that no point of the robot can have moved
//!   further from the nearest sample than `motion_allowance`, a bound worked out from the sizes of the links (the triangle inequality, with
//!   no dependence on the pose), and that allowance is added to the margin of every sample;
//! - the configured margin is added on top.
//!
//! What it cannot know, it does not claim: a link whose real shape sticks out of its radius, a tool longer than `tool_extent_m`, a load
//! carried by the tool, the dynamics of the stop and the zones of the cell that are not in the file are all outside this model. Not tried
//! with a real robot.

use std::collections::HashMap;

use serde_json::{json, Value};

use crate::kinematics::{effective_positions, forward_kinematics_model};
use crate::transform::Mat4;
use crate::urdf::{JointType, Model};

/// Distances are reduced by this much (the zones service does the same): the rounding of the arithmetic can only make a point read closer.
const SHAPE_EPSILON_M: f64 = 1e-9;
/// Added to every margin so that the rounding of the forward kinematics cannot hide a touch.
const FP_SLACK_M: f64 = 1e-9;
/// The most states a trajectory may have, zones a file may have and pieces a segment is cut into.
pub const MAX_STATES: usize = 256;
pub const MAX_ZONES: usize = 1000;
const MAX_PIECES_PER_SEGMENT: usize = 256;
/// How many hits are listed (the count of all of them is always given).
const MAX_LISTED_HITS: usize = 32;

type P3 = [f64; 3];

fn sub(a: P3, b: P3) -> P3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn dist(a: P3, b: P3) -> f64 {
    let d = sub(a, b);
    (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
}

// ---------------------------------------------------------------------------------------------------------------- configuration

/// The declared sizes and the numeric choices of the prediction.
#[derive(Debug, Clone, PartialEq)]
pub struct Config {
    /// Radius of every link that has no entry in `link_radii`.
    pub link_radius_m: f64,
    /// How far the tool reaches beyond the frame of the last joint (a sphere of this radius, plus the link radius, is put there).
    pub tool_extent_m: f64,
    /// Radius by the name of the joint at the end of the link.
    pub link_radii: HashMap<String, f64>,
    /// Extra distance added to every comparison.
    pub margin_m: f64,
    /// A segment is cut into pieces no longer than this.
    pub resolution_m: f64,
    /// The most a point of the robot may move between two samples of the trajectory (before the allowance is added back as margin).
    pub step_m: f64,
    /// The most samples taken between two states; when it is reached the allowance grows instead of the sampling being thinner unnoticed.
    pub max_substeps: usize,
    pub self_collision: bool,
    /// Pairs of segments whose distance in the tree of joints is below this are not compared (1: only those that share a joint).
    pub self_skip: usize,
}

const CONFIG_FIELDS: [&str; 9] = [
    "link_radius_m",
    "tool_extent_m",
    "link_radii",
    "margin_m",
    "resolution_m",
    "step_m",
    "max_substeps",
    "self_collision",
    "self_skip",
];

fn number(value: &Value, what: &str) -> Result<f64, String> {
    match value.as_f64() {
        Some(n) if n.is_finite() => Ok(n),
        _ => Err(format!("{what} must be a finite number, got {value}")),
    }
}

pub fn parse_config(raw: &Value) -> Result<Config, String> {
    let object = raw
        .as_object()
        .ok_or("the collision configuration must be a JSON object")?;
    let mut unknown: Vec<&str> = object
        .keys()
        .map(String::as_str)
        .filter(|k| !CONFIG_FIELDS.contains(k))
        .collect();
    if !unknown.is_empty() {
        unknown.sort_unstable();
        return Err(format!(
            "the collision configuration has unknown field(s): {}",
            unknown.join(", ")
        ));
    }
    // The two sizes of the robot have no default on purpose: a default would be a guess, and a small one would hide a contact.
    let need = |key: &str| -> Result<f64, String> {
        number(
            object.get(key).ok_or_else(|| {
                format!("the collision configuration needs \"{key}\": the sizes of the robot are never assumed")
            })?,
            key,
        )
    };
    let link_radius_m = need("link_radius_m")?;
    if link_radius_m <= 0.0 {
        return Err("link_radius_m must be above zero".into());
    }
    let tool_extent_m = need("tool_extent_m")?;
    if tool_extent_m < 0.0 {
        return Err("tool_extent_m must not be negative".into());
    }
    let optional = |key: &str, default: f64| -> Result<f64, String> {
        object.get(key).map_or(Ok(default), |v| number(v, key))
    };
    let margin_m = optional("margin_m", 0.0)?;
    if margin_m < 0.0 {
        return Err("margin_m must not be negative: a margin can only grow the robot".into());
    }
    let resolution_m = optional("resolution_m", 0.05)?;
    if !(0.005..=10.0).contains(&resolution_m) {
        return Err("resolution_m must be between 0.005 and 10".into());
    }
    let step_m = optional("step_m", 0.02)?;
    if !(0.001..=10.0).contains(&step_m) {
        return Err("step_m must be between 0.001 and 10".into());
    }
    let whole = |key: &str, default: usize, low: usize, high: usize| -> Result<usize, String> {
        match object.get(key) {
            None => Ok(default),
            Some(v) => v
                .as_u64()
                .map(|n| n as usize)
                .filter(|n| (low..=high).contains(n))
                .ok_or_else(|| format!("{key} must be a whole number from {low} to {high}")),
        }
    };
    let max_substeps = whole("max_substeps", 200, 1, 10_000)?;
    let self_skip = whole("self_skip", 1, 1, 8)?;
    let self_collision = match object.get("self_collision") {
        None => true,
        Some(Value::Bool(b)) => *b,
        Some(other) => return Err(format!("self_collision must be true or false, got {other}")),
    };
    let mut link_radii = HashMap::new();
    if let Some(radii) = object.get("link_radii") {
        let radii = radii
            .as_object()
            .ok_or("link_radii must be an object of joint name to radius")?;
        for (name, value) in radii {
            let r = number(value, &format!("link_radii.{name}"))?;
            if r <= 0.0 {
                return Err(format!("link_radii.{name} must be above zero"));
            }
            link_radii.insert(name.clone(), r);
        }
    }
    Ok(Config {
        link_radius_m,
        tool_extent_m,
        link_radii,
        margin_m,
        resolution_m,
        step_m,
        max_substeps,
        self_collision,
        self_skip,
    })
}

// ---------------------------------------------------------------------------------------------------------------------------- zones

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Level {
    Warning,
    Danger,
}

impl Level {
    fn name(self) -> &'static str {
        match self {
            Level::Warning => "warning",
            Level::Danger => "danger",
        }
    }
}

#[derive(Debug, Clone)]
enum Volume {
    Box {
        min: P3,
        max: P3,
    },
    Obb {
        center: P3,
        half: P3,
        /// R = Rz(yaw) * Ry(pitch) * Rx(roll): maps local coordinates to the cell frame.
        r: [[f64; 3]; 3],
    },
    Cylinder {
        base: P3,
        radius: f64,
        height: f64,
    },
    Capsule {
        a: P3,
        b: P3,
        radius: f64,
    },
}

#[derive(Debug, Clone)]
pub struct Zone {
    pub id: String,
    pub level: Level,
    volume: Volume,
}

fn rotation(roll_deg: f64, pitch_deg: f64, yaw_deg: f64) -> [[f64; 3]; 3] {
    let (sr, cr) = roll_deg.to_radians().sin_cos();
    let (sp, cp) = pitch_deg.to_radians().sin_cos();
    let (sy, cy) = yaw_deg.to_radians().sin_cos();
    [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
}

impl Volume {
    /// The exact distance from the point to the solid, reduced by a nanometre (never below zero).
    fn distance(&self, p: P3) -> f64 {
        let exact = match self {
            Volume::Box { min, max } => {
                let mut sq = 0.0;
                for i in 0..3 {
                    let gap = (min[i] - p[i]).max(0.0).max(p[i] - max[i]);
                    sq += gap * gap;
                }
                sq.sqrt()
            }
            Volume::Obb { center, half, r } => {
                let d = sub(p, *center);
                let mut sq = 0.0;
                for i in 0..3 {
                    let local = r[0][i] * d[0] + r[1][i] * d[1] + r[2][i] * d[2];
                    let gap = (local.abs() - half[i]).max(0.0);
                    sq += gap * gap;
                }
                sq.sqrt()
            }
            Volume::Cylinder {
                base,
                radius,
                height,
            } => {
                let radial = ((p[0] - base[0]).hypot(p[1] - base[1]) - radius).max(0.0);
                let vertical = (base[2] - p[2]).max(0.0).max(p[2] - (base[2] + height));
                radial.hypot(vertical)
            }
            Volume::Capsule { a, b, radius } => {
                let ab = sub(*b, *a);
                let ap = sub(p, *a);
                let len_sq = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2];
                let t = if len_sq == 0.0 {
                    0.0
                } else {
                    ((ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / len_sq).clamp(0.0, 1.0)
                };
                let nearest = [a[0] + t * ab[0], a[1] + t * ab[1], a[2] + t * ab[2]];
                (dist(nearest, p) - radius).max(0.0)
            }
        };
        (exact - SHAPE_EPSILON_M).max(0.0)
    }

    /// The axis-aligned box that contains the whole solid (a safe one, not the tightest).
    fn bounding_box(&self) -> Volume {
        let e = SHAPE_EPSILON_M;
        match self {
            Volume::Box { .. } => self.clone(),
            Volume::Obb { center, half, r } => {
                let mut reach = [0.0; 3];
                for (i, row) in r.iter().enumerate() {
                    reach[i] = (0..3).map(|j| row[j].abs() * half[j]).sum::<f64>() + e;
                }
                Volume::Box {
                    min: [
                        center[0] - reach[0],
                        center[1] - reach[1],
                        center[2] - reach[2],
                    ],
                    max: [
                        center[0] + reach[0],
                        center[1] + reach[1],
                        center[2] + reach[2],
                    ],
                }
            }
            Volume::Cylinder {
                base,
                radius,
                height,
            } => Volume::Box {
                min: [base[0] - radius - e, base[1] - radius - e, base[2] - e],
                max: [
                    base[0] + radius + e,
                    base[1] + radius + e,
                    base[2] + height + e,
                ],
            },
            Volume::Capsule { a, b, radius } => Volume::Box {
                min: [
                    a[0].min(b[0]) - radius - e,
                    a[1].min(b[1]) - radius - e,
                    a[2].min(b[2]) - radius - e,
                ],
                max: [
                    a[0].max(b[0]) + radius + e,
                    a[1].max(b[1]) + radius + e,
                    a[2].max(b[2]) + radius + e,
                ],
            },
        }
    }
}

fn point(value: Option<&Value>, what: &str) -> Result<P3, String> {
    let object = value
        .and_then(Value::as_object)
        .ok_or_else(|| format!("{what} must be an object {{x, y, z}}"))?;
    let mut out = [0.0; 3];
    for (i, axis) in ["x", "y", "z"].iter().enumerate() {
        out[i] = number(
            object
                .get(*axis)
                .ok_or_else(|| format!("{what} is missing \"{axis}\""))?,
            &format!("{what}.{axis}"),
        )?;
    }
    Ok(out)
}

/// The zones of a zones file of HYDRA-UMC-SAFETY-ZONES. Only the geometry is read: its calibration, its separation model and its velocity
/// margins are NOT applied here (use `margin_m` of the configuration for the distance you want to keep).
pub fn parse_zones(raw: &Value) -> Result<Vec<Zone>, String> {
    let object = raw
        .as_object()
        .ok_or("the zones file must be a JSON object")?;
    let (shapes_on, bounding) = match object.get("geometry") {
        None => (false, false),
        Some(g) => {
            let g = g.as_object().ok_or("geometry must be an object")?;
            let shapes = match g.get("shapes") {
                None => false,
                Some(Value::Bool(b)) => *b,
                Some(_) => return Err("geometry.shapes must be true or false".into()),
            };
            let bounding = match g.get("mode").and_then(Value::as_str) {
                None | Some("exact") => false,
                Some("bounding") => true,
                Some(other) => {
                    return Err(format!(
                        "geometry.mode must be \"exact\" or \"bounding\", got {other:?}"
                    ))
                }
            };
            (shapes, bounding)
        }
    };
    let entries = object
        .get("zones")
        .and_then(Value::as_array)
        .ok_or("the zones file needs a \"zones\" list")?;
    if entries.len() > MAX_ZONES {
        return Err(format!("more than {MAX_ZONES} zones"));
    }
    let mut zones: Vec<Zone> = Vec::new();
    for entry in entries {
        let entry = entry.as_object().ok_or("a zone must be an object")?;
        let id = entry
            .get("id")
            .and_then(Value::as_str)
            .filter(|s| !s.is_empty() && s.len() <= 128)
            .ok_or("a zone needs an \"id\" of 1 to 128 characters")?
            .to_string();
        if zones.iter().any(|z| z.id == id) {
            return Err(format!("zone id {id:?} is used more than once"));
        }
        let level = match entry.get("level").and_then(Value::as_str) {
            Some("warning") => Level::Warning,
            Some("danger") => Level::Danger,
            other => {
                return Err(format!(
                    "zone {id:?}: level must be \"warning\" or \"danger\", got {other:?}"
                ))
            }
        };
        let shape = entry.get("shape").and_then(Value::as_str).unwrap_or("box");
        if entry.get("shape").is_some_and(|s| !s.is_string()) {
            return Err(format!("zone {id:?}: shape must be a string"));
        }
        let volume = match shape {
            "box" => {
                let min = point(entry.get("min"), &format!("zone {id:?} min"))?;
                let max = point(entry.get("max"), &format!("zone {id:?} max"))?;
                if (0..3).any(|i| min[i] > max[i]) {
                    return Err(format!("zone {id:?}: min is above max"));
                }
                Volume::Box { min, max }
            }
            "obb" | "cylinder" | "capsule" if !shapes_on => {
                return Err(format!(
                    "zone {id:?}: shape {shape:?} needs \"geometry\": {{\"shapes\": true}} in the zones file"
                ))
            }
            "obb" => {
                let center = point(entry.get("center"), &format!("zone {id:?} center"))?;
                let half = point(entry.get("half_extents"), &format!("zone {id:?} half_extents"))?;
                if half.iter().any(|h| *h < 0.0) {
                    return Err(format!("zone {id:?}: half_extents must not be negative"));
                }
                let mut angles = [0.0; 3];
                if let Some(rot) = entry.get("rotation_deg") {
                    let rot = rot
                        .as_object()
                        .ok_or_else(|| format!("zone {id:?}: rotation_deg must be an object"))?;
                    for (i, key) in ["roll", "pitch", "yaw"].iter().enumerate() {
                        if let Some(v) = rot.get(*key) {
                            angles[i] = number(v, &format!("zone {id:?} rotation_deg.{key}"))?;
                        }
                    }
                }
                Volume::Obb {
                    center,
                    half,
                    r: rotation(angles[0], angles[1], angles[2]),
                }
            }
            "cylinder" => {
                let base = point(entry.get("base_center"), &format!("zone {id:?} base_center"))?;
                let radius = number(entry.get("radius").unwrap_or(&Value::Null), "radius")?;
                let height = number(entry.get("height").unwrap_or(&Value::Null), "height")?;
                if radius <= 0.0 || height <= 0.0 {
                    return Err(format!("zone {id:?}: radius and height must be above zero"));
                }
                Volume::Cylinder {
                    base,
                    radius,
                    height,
                }
            }
            "capsule" => {
                let a = point(entry.get("start"), &format!("zone {id:?} start"))?;
                let b = point(entry.get("end"), &format!("zone {id:?} end"))?;
                let radius = number(entry.get("radius").unwrap_or(&Value::Null), "radius")?;
                if radius <= 0.0 {
                    return Err(format!("zone {id:?}: radius must be above zero"));
                }
                Volume::Capsule { a, b, radius }
            }
            other => {
                return Err(format!(
                    "zone {id:?}: shape must be one of box, obb, cylinder, capsule, got {other:?}"
                ))
            }
        };
        zones.push(Zone {
            id,
            level,
            volume: if bounding {
                volume.bounding_box()
            } else {
                volume
            },
        });
    }
    Ok(zones)
}

// ------------------------------------------------------------------------------------------------------------------------- skeleton

#[derive(Debug, Clone)]
struct Segment {
    a: P3,
    b: P3,
    radius: f64,
    /// The tree nodes it hangs between (0 is the base, joint `i` is node `i + 1`).
    nodes: (usize, usize),
    label: String,
}

impl Segment {
    fn length(&self) -> f64 {
        dist(self.a, self.b)
    }
    fn middle(&self) -> P3 {
        [
            (self.a[0] + self.b[0]) / 2.0,
            (self.a[1] + self.b[1]) / 2.0,
            (self.a[2] + self.b[2]) / 2.0,
        ]
    }
}

/// A sphere that contains a piece of a segment together with the radius of the link.
#[derive(Debug, Clone, Copy)]
struct Ball {
    center: P3,
    radius: f64,
}

fn link_radius(config: &Config, joint: &str) -> f64 {
    config
        .link_radii
        .get(joint)
        .copied()
        .unwrap_or(config.link_radius_m)
}

fn translation(m: &Mat4) -> P3 {
    let t = m.translation_part();
    [t.x, t.y, t.z]
}

fn skeleton(model: &Model, worlds: &[(String, Mat4)], config: &Config) -> Vec<Segment> {
    let joints = &model.chain.joints;
    let mut has_child = vec![false; joints.len()];
    for parent in model.parents.iter().flatten() {
        has_child[*parent] = true;
    }
    let mut segments = Vec::new();
    for (i, joint) in joints.iter().enumerate() {
        let (parent_point, parent_node, parent_world) = match model.parents[i] {
            Some(p) => (translation(&worlds[p].1), p + 1, worlds[p].1),
            None => ([0.0; 3], 0, Mat4::identity()),
        };
        let own = translation(&worlds[i].1);
        let radius = link_radius(config, &joint.name);
        if joint.joint_type == JointType::Prismatic {
            let anchor = translation(&parent_world.mul(&joint.origin));
            segments.push(Segment {
                a: parent_point,
                b: anchor,
                radius,
                nodes: (parent_node, i + 1),
                label: joint.name.clone(),
            });
            segments.push(Segment {
                a: anchor,
                b: own,
                radius,
                nodes: (parent_node, i + 1),
                label: joint.name.clone(),
            });
        } else {
            segments.push(Segment {
                a: parent_point,
                b: own,
                radius,
                nodes: (parent_node, i + 1),
                label: joint.name.clone(),
            });
        }
        if !has_child[i] {
            segments.push(Segment {
                a: own,
                b: own,
                radius: radius + config.tool_extent_m,
                nodes: (i + 1, i + 1),
                label: format!("tool:{}", joint.name),
            });
        }
    }
    segments
}

/// The spheres that cover one segment: pieces no longer than the resolution, each replaced by the sphere around its middle that
/// contains it.
fn pieces(segment: &Segment, resolution_m: f64) -> Vec<Ball> {
    let length = segment.length();
    let count = ((length / resolution_m).ceil() as usize).clamp(1, MAX_PIECES_PER_SEGMENT);
    let piece_length = length / count as f64;
    (0..count)
        .map(|k| {
            let t = (k as f64 + 0.5) / count as f64;
            Ball {
                center: [
                    segment.a[0] + t * (segment.b[0] - segment.a[0]),
                    segment.a[1] + t * (segment.b[1] - segment.a[1]),
                    segment.a[2] + t * (segment.b[2] - segment.a[2]),
                ],
                radius: segment.radius + piece_length / 2.0 + FP_SLACK_M,
            }
        })
        .collect()
}

/// The sphere around a whole segment, used to skip the pieces of one that is far from what it is compared with.
fn segment_ball(segment: &Segment) -> Ball {
    Ball {
        center: segment.middle(),
        // the length, not half of it: a piece is at most as long as the segment, so every sphere of a piece is within this one's reach
        radius: segment.radius + segment.length() + FP_SLACK_M,
    }
}

fn node_depth(model: &Model, mut node: usize) -> usize {
    let mut depth = 0;
    while node != 0 {
        node = model.parents[node - 1].map_or(0, |p| p + 1);
        depth += 1;
    }
    depth
}

/// The number of joints between two nodes of the tree.
fn node_distance(model: &Model, mut a: usize, mut b: usize) -> usize {
    let (mut da, mut db) = (node_depth(model, a), node_depth(model, b));
    let mut steps = 0;
    let up = |n: usize| model.parents[n - 1].map_or(0, |p| p + 1);
    while da > db {
        a = up(a);
        da -= 1;
        steps += 1;
    }
    while db > da {
        b = up(b);
        db -= 1;
        steps += 1;
    }
    while a != b {
        a = up(a);
        b = up(b);
        steps += 2;
    }
    steps
}

// ------------------------------------------------------------------------------------------------------------------------- motion

/// For each moving joint, how far any point of the robot can be moved by one unit of that joint (metres per radian or per metre),
/// whatever the pose: the distance of a point to the axis is at most its distance to the origin of the joint, which is at most the sum of
/// the offsets of the joints in between, plus the sizes of the link and the tool at the end.
fn lever_arms(model: &Model, config: &Config, travel: &HashMap<String, f64>) -> Vec<f64> {
    let joints = &model.chain.joints;
    let count = joints.len();
    let mut has_child = vec![false; count];
    for parent in model.parents.iter().flatten() {
        has_child[*parent] = true;
    }
    // reach[k]: the largest distance from the origin of joint k to a point of the robot at or below k (through the offsets, not the pose)
    let mut reach = vec![0.0f64; count];
    for k in (0..count).rev() {
        let own = link_radius(config, &joints[k].name)
            + if has_child[k] {
                0.0
            } else {
                config.tool_extent_m
            };
        reach[k] = reach[k].max(own);
        if let Some(parent) = model.parents[k] {
            let t = joints[k].origin.translation_part();
            let offset = (t.x * t.x + t.y * t.y + t.z * t.z).sqrt()
                + if joints[k].joint_type == JointType::Prismatic {
                    travel.get(&joints[k].name).copied().unwrap_or(0.0)
                } else {
                    0.0
                };
            // the segment of k itself hangs from the parent's origin: it is as far from the parent's axis as offset + its radius
            let from_parent = offset + reach[k];
            reach[parent] = reach[parent].max(from_parent);
        }
    }
    joints
        .iter()
        .enumerate()
        .map(|(i, j)| match j.joint_type {
            JointType::Fixed => 0.0,
            JointType::Prismatic => 1.0,
            // a point that rotates about an axis moves at most its distance to the origin of the joint times the angle; the segment that
            // ends at the joint does not turn with it, and the sphere of the joint itself is inside reach[i]
            JointType::Revolute | JointType::Continuous => reach[i],
        })
        .collect()
}

// ------------------------------------------------------------------------------------------------------------------------ prediction

#[derive(Debug, Clone, PartialEq)]
pub struct Hit {
    /// The place on the trajectory, in states (1.5 is half way from the second state to the third); counted from 0.
    pub at: f64,
    /// "zone" or "self"
    pub kind: &'static str,
    pub zone: Option<String>,
    pub level: Option<Level>,
    pub link: String,
    pub other_link: Option<String>,
    pub clearance_m: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Prediction {
    pub level: Option<Level>,
    pub self_collision: bool,
    pub states: usize,
    pub samples: usize,
    /// The largest allowance added to a sample for the motion between samples.
    pub max_motion_allowance_m: f64,
    /// The smallest clearance to a zone over every sample (None: there were no zones). Negative means a contact.
    pub min_zone_clearance_m: Option<f64>,
    /// The same between pieces of the robot that are compared with each other (None: self collision off or nothing to compare).
    pub min_self_clearance_m: Option<f64>,
    pub hits_total: usize,
    pub hits: Vec<Hit>,
    pub outside_limits: Vec<(usize, String, f64, f64, f64)>,
}

impl Prediction {
    pub fn collision(&self) -> bool {
        self.level.is_some() || self.self_collision
    }

    pub fn to_json(&self) -> Value {
        let hits: Vec<Value> = self
            .hits
            .iter()
            .map(|h| {
                json!({
                    "at": h.at,
                    "kind": h.kind,
                    "zone": h.zone,
                    "level": h.level.map(Level::name),
                    "link": h.link,
                    "other_link": h.other_link,
                    "clearance_m": h.clearance_m,
                })
            })
            .collect();
        let limits: Vec<Value> = self
            .outside_limits
            .iter()
            .map(|(state, joint, value, lower, upper)| {
                json!({"state": state, "joint": joint, "value": value, "lower": lower, "upper": upper})
            })
            .collect();
        json!({
            "collision": self.collision(),
            "level": self.level.map_or("clear", Level::name),
            "self_collision": self.self_collision,
            "states": self.states,
            "samples": self.samples,
            "max_motion_allowance_m": self.max_motion_allowance_m,
            "min_clearance_m": self.min_zone_clearance_m,
            "min_self_clearance_m": self.min_self_clearance_m,
            "hits_total": self.hits_total,
            "hits": hits,
            "outside_limits": limits,
            "model": "skeleton of the URDF joints with the configured link radii; overestimates by construction, not tried with a real robot",
        })
    }
}

/// `{"states": [{"joint": position, ...}, ...]}`.
pub fn parse_trajectory(raw: &Value) -> Result<Vec<HashMap<String, f64>>, String> {
    let list = raw
        .get("states")
        .and_then(Value::as_array)
        .ok_or("the trajectory needs a \"states\" list")?;
    if list.is_empty() {
        return Err("the trajectory has no states".into());
    }
    if list.len() > MAX_STATES {
        return Err(format!("the trajectory has more than {MAX_STATES} states"));
    }
    let mut out = Vec::with_capacity(list.len());
    for (i, state) in list.iter().enumerate() {
        let object = state
            .as_object()
            .ok_or_else(|| format!("state {i} must be an object of joint name to position"))?;
        let mut map = HashMap::new();
        for (name, value) in object {
            map.insert(
                name.clone(),
                number(value, &format!("state {i} joint {name}"))?,
            );
        }
        out.push(map);
    }
    Ok(out)
}

/// `{"joint": position, ...}`: where the joints are known to be (for example the last reports of the real robot).
pub fn parse_base(raw: &Value) -> Result<HashMap<String, f64>, String> {
    let object = raw
        .as_object()
        .ok_or("the base state must be an object of joint name to position")?;
    let mut out = HashMap::new();
    for (name, value) in object {
        out.insert(name.clone(), number(value, &format!("base joint {name}"))?);
    }
    Ok(out)
}

/// Gives every state the joints it does not mention, from `base`. Only joints that move by themselves are filled (a follower is derived,
/// a fixed joint has no position) and a name of `base` that the robot does not have is ignored, because the source of the base may follow
/// more joints than this robot has. A joint that neither the state nor the base has stays missing, and `predict` refuses the state: an
/// unknown position is never read as "zero".
pub fn fill_from_base(
    model: &Model,
    states: &[HashMap<String, f64>],
    base: &HashMap<String, f64>,
) -> Vec<HashMap<String, f64>> {
    states
        .iter()
        .map(|state| {
            let mut filled = state.clone();
            for (i, joint) in model.chain.joints.iter().enumerate() {
                if joint.joint_type == JointType::Fixed || model.extras[i].mimic.is_some() {
                    continue;
                }
                if let Some(value) = base.get(&joint.name) {
                    filled.entry(joint.name.clone()).or_insert(*value);
                }
            }
            filled
        })
        .collect()
}

struct Tracker {
    level: Option<Level>,
    self_collision: bool,
    min_zone: f64,
    min_self: f64,
    hits_total: usize,
    hits: Vec<Hit>,
    seen: std::collections::HashSet<String>,
}

impl Tracker {
    fn zone_clearance(&mut self, value: f64) {
        self.min_zone = self.min_zone.min(value);
    }
    fn self_clearance(&mut self, value: f64) {
        self.min_self = self.min_self.min(value);
    }
    fn record(&mut self, hit: Hit) {
        self.hits_total += 1;
        match hit.kind {
            "zone" => self.level = self.level.max(hit.level),
            _ => self.self_collision = true,
        }
        let key = format!(
            "{}|{}|{}",
            hit.kind,
            hit.zone.as_deref().unwrap_or(""),
            hit.link.clone() + hit.other_link.as_deref().unwrap_or("")
        );
        if self.hits.len() < MAX_LISTED_HITS && self.seen.insert(key) {
            self.hits.push(hit);
        }
    }
}

fn effective_given(
    model: &Model,
    given: &HashMap<String, f64>,
) -> Result<HashMap<String, f64>, String> {
    for name in given.keys() {
        if model.joint_index(name).is_none() {
            return Err(format!("unknown joint {name:?} in a state"));
        }
    }
    effective_positions(model, given).map_err(|e| e.to_string())
}

/// Predicts whether the trajectory, joints interpolated linearly between its states, brings the robot into a zone or into itself.
pub fn predict(
    model: &Model,
    zones: &[Zone],
    config: &Config,
    states: &[HashMap<String, f64>],
) -> Result<Prediction, String> {
    if states.is_empty() {
        return Err("the trajectory has no states".into());
    }
    // every state resolved first, so that a bad one is an error before any answer is given
    let effective: Vec<HashMap<String, f64>> = states
        .iter()
        .enumerate()
        .map(|(i, s)| effective_given(model, s).map_err(|e| format!("state {i}: {e}")))
        .collect::<Result<_, _>>()?;
    let mut outside = Vec::new();
    for (i, eff) in effective.iter().enumerate() {
        for v in crate::limits::validate_limits(&model.chain, eff) {
            outside.push((i, v.joint, v.value, v.lower, v.upper));
        }
    }
    // the node hop distances between segments are the same for every pose
    let joints = &model.chain.joints;
    let mut tracker = Tracker {
        level: None,
        self_collision: false,
        min_zone: f64::INFINITY,
        min_self: f64::INFINITY,
        hits_total: 0,
        hits: Vec::new(),
        seen: Default::default(),
    };
    let mut samples = 0usize;
    let mut max_allowance = 0.0f64;
    let moving: Vec<usize> = (0..joints.len())
        .filter(|&i| joints[i].joint_type != JointType::Fixed)
        .collect();

    let mut check = |given: &HashMap<String, f64>, at: f64, allowance: f64| -> Result<(), String> {
        let worlds = forward_kinematics_model(model, given).map_err(|e| e.to_string())?;
        let segments = skeleton(model, &worlds, config);
        let margin = config.margin_m + allowance + FP_SLACK_M;
        samples += 1;
        let balls: Vec<(Ball, Vec<Ball>)> = segments
            .iter()
            .map(|s| (segment_ball(s), pieces(s, config.resolution_m)))
            .collect();
        for zone in zones {
            for (s, (ball, parts)) in segments.iter().zip(&balls) {
                let far = zone.volume.distance(ball.center) - ball.radius - margin;
                if far > 0.0 && far >= tracker.min_zone {
                    continue;
                }
                for part in parts {
                    let c = zone.volume.distance(part.center) - part.radius - margin;
                    tracker.zone_clearance(c);
                    if c <= 0.0 {
                        tracker.record(Hit {
                            at,
                            kind: "zone",
                            zone: Some(zone.id.clone()),
                            level: Some(zone.level),
                            link: s.label.clone(),
                            other_link: None,
                            clearance_m: c,
                        });
                    }
                }
            }
        }
        if config.self_collision {
            for i in 0..segments.len() {
                for j in (i + 1)..segments.len() {
                    let (si, sj) = (&segments[i], &segments[j]);
                    // a prismatic joint's two segments are one link
                    if si.label == sj.label {
                        continue;
                    }
                    let hop = [
                        (si.nodes.0, sj.nodes.0),
                        (si.nodes.0, sj.nodes.1),
                        (si.nodes.1, sj.nodes.0),
                        (si.nodes.1, sj.nodes.1),
                    ]
                    .iter()
                    .map(|&(a, b)| node_distance(model, a, b))
                    .min()
                    .unwrap_or(0);
                    if hop < config.self_skip {
                        continue;
                    }
                    let (bi, pi) = &balls[i];
                    let (bj, pj) = &balls[j];
                    let far = dist(bi.center, bj.center) - bi.radius - bj.radius - margin;
                    if far > 0.0 && far >= tracker.min_self {
                        continue;
                    }
                    for a in pi {
                        for b in pj {
                            let c = dist(a.center, b.center) - a.radius - b.radius - margin;
                            tracker.self_clearance(c);
                            if c <= 0.0 {
                                tracker.record(Hit {
                                    at,
                                    kind: "self",
                                    zone: None,
                                    level: None,
                                    link: si.label.clone(),
                                    other_link: Some(sj.label.clone()),
                                    clearance_m: c,
                                });
                            }
                        }
                    }
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
        // the travel of the prismatic joints, for the lever arms
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
        let arms = lever_arms(model, config, &travel);
        let total: f64 = moving
            .iter()
            .map(|&i| {
                let name = &joints[i].name;
                (qb[name] - qa[name]).abs() * arms[i]
            })
            .sum();
        let steps = ((total / config.step_m).ceil() as usize).clamp(1, config.max_substeps);
        let allowance = total / (2.0 * steps as f64);
        max_allowance = max_allowance.max(allowance);
        let first = if pair == 0 { 0 } else { 1 };
        for k in first..=steps {
            let t = k as f64 / steps as f64;
            // interpolate what was given, so that the followers are worked out by the same rules as in a state
            let mut given = HashMap::new();
            for (name, a) in &states[pair] {
                let b = states[pair + 1].get(name).copied().unwrap_or(*a);
                given.insert(name.clone(), a + (b - a) * t);
            }
            for (name, b) in &states[pair + 1] {
                given.entry(name.clone()).or_insert(*b);
            }
            check(&given, pair as f64 + t, allowance)?;
        }
    }
    Ok(Prediction {
        level: tracker.level,
        self_collision: tracker.self_collision,
        states: states.len(),
        samples,
        max_motion_allowance_m: max_allowance,
        min_zone_clearance_m: Some(tracker.min_zone).filter(|v| v.is_finite()),
        min_self_clearance_m: Some(tracker.min_self).filter(|v| v.is_finite()),
        hits_total: tracker.hits_total,
        hits: tracker.hits,
        outside_limits: outside,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::urdf::parse_urdf_model;

    pub const PLANAR: &str = r#"<robot name="planar">
      <link name="base"/><link name="l1"/><link name="l2"/><link name="tool"/>
      <joint name="shoulder" type="revolute"><parent link="base"/><child link="l1"/>
        <axis xyz="0 0 1"/><limit lower="-3.1" upper="3.1" effort="10" velocity="2"/></joint>
      <joint name="elbow" type="revolute"><parent link="l1"/><child link="l2"/>
        <origin xyz="1 0 0"/><axis xyz="0 0 1"/><limit lower="-3.1" upper="3.1" effort="10" velocity="2"/></joint>
      <joint name="tip" type="fixed"><parent link="l2"/><child link="tool"/>
        <origin xyz="1 0 0"/></joint>
    </robot>"#;

    fn cfg() -> Config {
        parse_config(&json!({"link_radius_m": 0.05, "tool_extent_m": 0.05})).unwrap()
    }

    fn state(shoulder: f64, elbow: f64) -> HashMap<String, f64> {
        HashMap::from([
            ("shoulder".to_string(), shoulder),
            ("elbow".to_string(), elbow),
        ])
    }

    fn boxed(min: P3, max: P3, level: Level) -> Zone {
        Zone {
            id: "z".into(),
            level,
            volume: Volume::Box { min, max },
        }
    }

    #[test]
    fn the_configuration_never_assumes_the_size_of_the_robot() {
        assert!(parse_config(&json!({}))
            .unwrap_err()
            .contains("link_radius_m"));
        assert!(parse_config(&json!({"link_radius_m": 0.1}))
            .unwrap_err()
            .contains("tool_extent_m"));
        assert!(parse_config(&json!({"link_radius_m": 0.0, "tool_extent_m": 0})).is_err());
        assert!(
            parse_config(&json!({"link_radius_m": 0.1, "tool_extent_m": 0, "margin_m": -1}))
                .is_err()
        );
        assert!(
            parse_config(&json!({"link_radius_m": 0.1, "tool_extent_m": 0, "bogus": 1}))
                .unwrap_err()
                .contains("bogus")
        );
        assert!(
            parse_config(&json!({"link_radius_m": 0.1, "tool_extent_m": 0, "self_skip": 0}))
                .is_err()
        );
    }

    #[test]
    fn a_tool_in_a_box_is_a_danger_and_a_box_far_away_is_clear() {
        let model = parse_urdf_model(PLANAR).unwrap();
        // at zero the tool is at (2, 0, 0)
        let inside = boxed([1.9, -0.1, -0.1], [2.1, 0.1, 0.1], Level::Danger);
        let p = predict(&model, &[inside], &cfg(), &[state(0.0, 0.0)]).unwrap();
        assert_eq!(p.level, Some(Level::Danger));
        assert!(p.collision());
        let far = boxed([5.0, 5.0, 5.0], [6.0, 6.0, 6.0], Level::Danger);
        let p = predict(&model, &[far], &cfg(), &[state(0.0, 0.0)]).unwrap();
        assert!(!p.collision());
        assert!(p.min_zone_clearance_m.unwrap() > 0.0);
    }

    #[test]
    fn a_zone_reached_half_way_between_two_states_is_found() {
        let model = parse_urdf_model(PLANAR).unwrap();
        // the straight arm sweeps a quarter circle from +x to +y; the box is on the diagonal, where neither state is
        let diagonal = boxed([1.2, 1.2, -0.1], [1.6, 1.6, 0.1], Level::Warning);
        let ends = [state(0.0, 0.0), state(std::f64::consts::FRAC_PI_2, 0.0)];
        let p = predict(&model, std::slice::from_ref(&diagonal), &cfg(), &ends).unwrap();
        assert_eq!(p.level, Some(Level::Warning), "{:?}", p.hits);
        let at = p.hits[0].at;
        assert!(at > 0.0 && at < 1.0, "{at}");
        // the two end states alone do not touch it
        let p0 = predict(&model, std::slice::from_ref(&diagonal), &cfg(), &ends[..1]).unwrap();
        let p1 = predict(&model, &[diagonal], &cfg(), &ends[1..]).unwrap();
        assert!(!p0.collision() && !p1.collision());
    }

    #[test]
    fn the_margin_grows_the_robot_and_never_shrinks_it() {
        let model = parse_urdf_model(PLANAR).unwrap();
        let wall = boxed([2.3, -1.0, -1.0], [2.5, 1.0, 1.0], Level::Danger);
        let mut near = cfg();
        let clear = predict(
            &model,
            std::slice::from_ref(&wall),
            &near,
            &[state(0.0, 0.0)],
        )
        .unwrap();
        assert!(!clear.collision());
        near.margin_m = 0.3;
        let hit = predict(&model, &[wall], &near, &[state(0.0, 0.0)]).unwrap();
        assert!(hit.collision());
    }

    #[test]
    fn the_arm_folded_onto_itself_is_a_self_collision_but_not_when_straight() {
        let model = parse_urdf_model(PLANAR).unwrap();
        let straight = predict(&model, &[], &cfg(), &[state(0.0, 0.0)]).unwrap();
        assert!(!straight.self_collision);
        // elbow at pi: the forearm lies back on the upper arm
        let folded = predict(&model, &[], &cfg(), &[state(0.0, 3.1)]).unwrap();
        assert!(folded.self_collision, "{:?}", folded.min_self_clearance_m);
        let mut off = cfg();
        off.self_collision = false;
        assert!(
            !predict(&model, &[], &off, &[state(0.0, 3.1)])
                .unwrap()
                .self_collision
        );
    }

    #[test]
    fn a_state_with_a_joint_missing_or_unknown_or_not_finite_is_an_error_not_an_answer() {
        let model = parse_urdf_model(PLANAR).unwrap();
        let missing = HashMap::from([("shoulder".to_string(), 0.0)]);
        assert!(predict(&model, &[], &cfg(), &[missing]).is_err());
        let mut extra = state(0.0, 0.0);
        extra.insert("wrist".into(), 0.0);
        assert!(predict(&model, &[], &cfg(), &[extra])
            .unwrap_err()
            .contains("unknown joint"));
        assert!(predict(&model, &[], &cfg(), &[state(f64::NAN, 0.0)]).is_err());
        assert!(predict(&model, &[], &cfg(), &[]).is_err());
    }

    #[test]
    fn positions_outside_the_limits_are_listed_but_do_not_make_a_collision() {
        let model = parse_urdf_model(PLANAR).unwrap();
        let p = predict(&model, &[], &cfg(), &[state(3.12, 0.0)]).unwrap();
        assert_eq!(p.outside_limits.len(), 1);
        assert_eq!(p.outside_limits[0].1, "shoulder");
    }

    #[test]
    fn the_zone_shapes_measure_like_the_zones_service() {
        let on = json!({"geometry": {"shapes": true}, "zones": [
            {"id": "c", "level": "danger", "shape": "cylinder", "base_center": {"x": 0, "y": 0, "z": 0}, "radius": 1.0, "height": 2.0},
            {"id": "o", "level": "warning", "shape": "obb", "center": {"x": 5, "y": 0, "z": 0}, "half_extents": {"x": 1, "y": 0.5, "z": 0.5}, "rotation_deg": {"yaw": 90}},
            {"id": "k", "level": "warning", "shape": "capsule", "start": {"x": 0, "y": 0, "z": 0}, "end": {"x": 0, "y": 4, "z": 0}, "radius": 0.5}
        ]});
        let zones = parse_zones(&on).unwrap();
        let by = |id: &str| zones.iter().find(|z| z.id == id).unwrap().volume.clone();
        // cylinder: a point 3 m out is 2 m from the wall (less a nanometre)
        assert!((by("c").distance([3.0, 0.0, 1.0]) - 2.0).abs() < 1e-6);
        assert_eq!(by("c").distance([0.5, 0.0, 1.0]), 0.0);
        // obb rotated 90 degrees about z: its long half extent (1) lies along y, so (5, 1.2, 0) is 0.2 beyond and (5.6, 0, 0) is 0.1 beyond
        assert!((by("o").distance([5.0, 1.2, 0.0]) - 0.2).abs() < 1e-6);
        assert!((by("o").distance([5.6, 0.0, 0.0]) - 0.1).abs() < 1e-6);
        // capsule: 1.5 m from the axis is 1.0 m from the surface
        assert!((by("k").distance([1.5, 2.0, 0.0]) - 1.0).abs() < 1e-6);
        // without the switch the same file is refused
        let off = json!({"zones": on["zones"].clone()});
        assert!(parse_zones(&off).unwrap_err().contains("shapes"));
        // bounding mode replaces each shape by a box that contains it
        let bounded = parse_zones(&json!({"geometry": {"shapes": true, "mode": "bounding"}, "zones": on["zones"].clone()})).unwrap();
        for (a, b) in zones.iter().zip(&bounded) {
            for p in [
                [0.9, 0.9, 0.1],
                [5.4, 0.9, 0.3],
                [0.4, 3.9, 0.0],
                [-0.4, -0.4, 0.0],
            ] {
                assert!(b.volume.distance(p) <= a.volume.distance(p) + 1e-12);
            }
        }
    }

    #[test]
    fn bad_zone_files_are_refused() {
        for bad in [
            json!([]),
            json!({}),
            json!({"zones": [{"id": "a", "level": "red", "min": {"x":0,"y":0,"z":0}, "max": {"x":1,"y":1,"z":1}}]}),
            json!({"zones": [{"id": "a", "level": "danger", "min": {"x":2,"y":0,"z":0}, "max": {"x":1,"y":1,"z":1}}]}),
            json!({"zones": [{"id": "a", "level": "danger", "shape": "sphere"}]}),
            json!({"zones": [
                {"id": "a", "level": "danger", "min": {"x":0,"y":0,"z":0}, "max": {"x":1,"y":1,"z":1}},
                {"id": "a", "level": "danger", "min": {"x":0,"y":0,"z":0}, "max": {"x":1,"y":1,"z":1}}]}),
            json!({"geometry": {"mode": "loose"}, "zones": []}),
        ] {
            assert!(parse_zones(&bad).is_err(), "{bad}");
        }
    }

    /// The property that matters: a prediction never says "clear" for a configuration that touches. The test builds a point that lies inside
    /// the swept body by construction (a point of a link and a distance smaller than its radius), puts a tiny box there, and asks for a
    /// verdict at every sampled pose of a random trajectory. A seeded generator keeps it repeatable.
    #[test]
    fn property_a_zone_that_the_body_really_touches_is_never_reported_clear() {
        let model = parse_urdf_model(PLANAR).unwrap();
        let mut seed: u64 = 0x2545F4914F6CDD1D;
        let mut next = || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 11) as f64 / (1u64 << 53) as f64
        };
        let config = cfg();
        for _ in 0..300 {
            let a = state(next() * 6.0 - 3.0, next() * 6.0 - 3.0);
            let b = state(next() * 6.0 - 3.0, next() * 6.0 - 3.0);
            // a true point of the body at a random moment of the straight joint-space path between a and b
            let t = next();
            let along = state(
                a["shoulder"] + (b["shoulder"] - a["shoulder"]) * t,
                a["elbow"] + (b["elbow"] - a["elbow"]) * t,
            );
            let worlds = forward_kinematics_model(&model, &along).unwrap();
            let segments = skeleton(&model, &worlds, &config);
            let s = &segments[(next() * segments.len() as f64) as usize % segments.len()];
            let u = next();
            let on_axis = [
                s.a[0] + u * (s.b[0] - s.a[0]),
                s.a[1] + u * (s.b[1] - s.a[1]),
                s.a[2] + u * (s.b[2] - s.a[2]),
            ];
            // a cube of 2 cm around a point of the link's core: the body certainly overlaps it
            let zone = boxed(
                [on_axis[0] - 0.01, on_axis[1] - 0.01, on_axis[2] - 0.01],
                [on_axis[0] + 0.01, on_axis[1] + 0.01, on_axis[2] + 0.01],
                Level::Danger,
            );
            let p = predict(&model, &[zone], &config, &[a.clone(), b.clone()]).unwrap();
            assert!(
                p.collision(),
                "missed a touch at t={t}: {a:?} -> {b:?}, point {on_axis:?}"
            );
        }
    }

    /// And the sampling allowance holds: a body that is really at the pose half way between two samples is covered.
    #[test]
    fn property_the_allowance_covers_every_pose_between_the_samples() {
        let model = parse_urdf_model(PLANAR).unwrap();
        let config = Config {
            step_m: 0.5,
            ..cfg()
        };
        let (a, b) = (state(-1.0, 0.5), state(1.2, -1.0));
        let mut seed = 7u64;
        let mut next = || {
            seed = seed
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            (seed >> 33) as f64 / (1u64 << 31) as f64
        };
        for _ in 0..200 {
            let t = next();
            let along = state(-1.0 + 2.2 * t, 0.5 - 1.5 * t);
            let worlds = forward_kinematics_model(&model, &along).unwrap();
            let tip = translation(&worlds[2].1);
            let zone = boxed(
                [tip[0] - 0.005, tip[1] - 0.005, tip[2] - 0.005],
                [tip[0] + 0.005, tip[1] + 0.005, tip[2] + 0.005],
                Level::Warning,
            );
            let p = predict(&model, &[zone], &config, &[a.clone(), b.clone()]).unwrap();
            assert!(p.collision(), "t={t}");
        }
    }

    #[test]
    fn the_base_state_fills_what_a_state_leaves_out_and_nothing_else() {
        let model = parse_urdf_model(PLANAR).unwrap();
        let base = HashMap::from([
            ("shoulder".to_string(), 0.3),
            ("elbow".to_string(), 0.4),
            ("tip".to_string(), 9.0),
            ("not_a_joint".to_string(), 1.0),
        ]);
        let states = [HashMap::from([("elbow".to_string(), -1.0)])];
        let filled = fill_from_base(&model, &states, &base);
        // the given one wins, the missing one comes from the base, the fixed joint and the stranger are not copied
        assert_eq!(filled[0]["elbow"], -1.0);
        assert_eq!(filled[0]["shoulder"], 0.3);
        assert_eq!(filled[0].len(), 2);
        // a joint nobody knows is an error, not zero
        let nobody = fill_from_base(&model, &states, &HashMap::new());
        assert!(predict(&model, &[], &cfg(), &nobody).is_err());
        assert!(parse_base(&json!([1])).is_err());
        assert!(parse_base(&json!({"a": "x"})).is_err());
    }

    #[test]
    fn a_prismatic_joint_and_a_follower_are_part_of_the_skeleton() {
        let urdf = r#"<robot name="gantry">
          <link name="base"/><link name="slide"/><link name="finger"/>
          <joint name="x" type="prismatic"><parent link="base"/><child link="slide"/>
            <axis xyz="1 0 0"/><limit lower="0" upper="2" effort="1" velocity="1"/></joint>
          <joint name="f1" type="prismatic"><parent link="slide"/><child link="finger"/>
            <origin xyz="0 0.5 0"/><axis xyz="0 1 0"/><limit lower="0" upper="1" effort="1" velocity="1"/></joint>
        </robot>"#;
        let model = parse_urdf_model(urdf).unwrap();
        let zone = boxed([1.4, 0.9, -0.1], [1.6, 1.1, 0.1], Level::Danger);
        let go = |x: f64, f: f64| HashMap::from([("x".to_string(), x), ("f1".to_string(), f)]);
        // the carriage slides to x=1.5 and the finger reaches y = 0.5 + 0.5
        let p = predict(&model, std::slice::from_ref(&zone), &cfg(), &[go(1.5, 0.5)]).unwrap();
        assert!(p.collision());
        let p = predict(&model, &[zone], &cfg(), &[go(0.0, 0.0), go(1.5, 0.0)]).unwrap();
        assert!(!p.collision(), "{:?}", p.hits);
    }
}
