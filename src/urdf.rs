// =============================================================================
// HYDRA-UMC-PHYSICS-REPLICA - src/urdf.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0 - see LICENSE
// =============================================================================
//! A real, deliberately partial URDF reader.
//!
//! Found while auditing the code: this used to
//! parse `<joint>` elements (`type`, `<origin>`, `<axis>`, `<limit>`) in
//! raw XML document order and treat that order as the serial chain -
//! real URDF makes no such guarantee (a spec-valid file may declare its
//! `<joint>` elements in any order at all; only each joint's own
//! `<parent>`/`<child>` link names define the real tree). Fixed: every
//! joint's real `<parent link="...">`/`<child link="...">` is now parsed
//! and walked from the one real root link (the link that is never any
//! joint's own child) to produce `Chain.joints` in genuine root-to-leaf
//! order, regardless of how the source file happens to list them.
//!
//! A genuinely BRANCHING tree (a gripper's two fingers, a dual-arm robot
//! on one torso) is read by `parse_urdf_model`, which returns a `Model`:
//! the joints in parent-before-child order, the index of each joint's
//! parent joint, and the extras a joint can carry (`<mimic>`, the effort
//! and velocity of `<limit>`, the soft limits of `<safety_controller>`).
//! `parse_urdf` keeps its original contract, a flat serial `Chain`, and
//! refuses a branch with `UrdfError::Branching` rather than silently
//! keeping one branch and dropping the other.
//!
//! Units are those of URDF: metres and radians. Only the <joint> elements
//! that are direct children of <robot> are read (a <transmission> holds
//! joints of its own that are not part of the tree), a continuous or fixed
//! joint's `<limit>` is ignored for positions (only revolute and prismatic
//! joints have position limits), and a moving joint needs a non-zero axis.

use crate::transform::{Mat4, Vec3};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JointType {
    Revolute,
    Continuous,
    Prismatic,
    Fixed,
}

#[derive(Debug, Clone)]
pub struct Joint {
    pub name: String,
    pub joint_type: JointType,
    pub origin: Mat4,
    pub axis: Vec3,
    pub limit: Option<(f64, f64)>,
}

#[derive(Debug, Clone)]
pub struct Chain {
    pub joints: Vec<Joint>,
}

/// `<mimic joint="master" multiplier="m" offset="o"/>`: the joint has no position of its own, it is `m * master + o`.
#[derive(Debug, Clone, PartialEq)]
pub struct Mimic {
    pub joint: String,
    pub multiplier: f64,
    pub offset: f64,
}

/// `<safety_controller soft_lower_limit soft_upper_limit k_position k_velocity>`: the range a controller keeps
/// inside the hard `<limit>` of the joint.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SoftLimits {
    pub lower: f64,
    pub upper: f64,
    pub k_position: f64,
    pub k_velocity: f64,
}

/// What a joint carries beyond its motion: kept next to `Joint`, not in it, so that every place that builds a `Joint` by hand is untouched.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct JointExtras {
    pub mimic: Option<Mimic>,
    /// `<limit velocity>`: the most this joint may move per second (rad/s or m/s). A continuous joint has one too.
    pub velocity_limit: Option<f64>,
    /// `<limit effort>`: the most force or torque it may be asked for (N or N*m).
    pub effort_limit: Option<f64>,
    pub soft: Option<SoftLimits>,
}

/// The whole tree of a URDF.
#[derive(Debug, Clone)]
pub struct Model {
    /// The joints, every joint after the one that carries its parent link.
    pub chain: Chain,
    #[allow(dead_code)]
    pub root_link: String,
    /// For each joint, the index of the joint whose child link is its parent link (`None`: it hangs from the root link).
    pub parents: Vec<Option<usize>>,
    /// Index-aligned with `chain.joints`.
    pub extras: Vec<JointExtras>,
}

#[allow(dead_code)]
impl Model {
    /// True when every joint hangs from the previous one: the shape `Chain` alone can describe.
    pub fn is_serial(&self) -> bool {
        self.parents
            .iter()
            .enumerate()
            .all(|(i, parent)| *parent == if i == 0 { None } else { Some(i - 1) })
    }

    pub fn joint_index(&self, name: &str) -> Option<usize> {
        self.chain.joints.iter().position(|j| j.name == name)
    }
}

#[derive(Debug, PartialEq, Eq)]
pub enum UrdfError {
    Xml(String),
    MissingAttribute {
        joint: String,
        attribute: &'static str,
    },
    UnsupportedJointType {
        joint: String,
        joint_type: String,
    },
    InvalidNumber {
        joint: String,
        text: String,
    },
    /// Rust's own `f64::from_str` accepts "nan"/"inf"/"infinity" as
    /// syntactically valid floats, so a plain `.parse::<f64>()` alone
    /// never rejects a non-finite origin component or joint limit -
    /// `is_finite()` must be checked explicitly wherever a value must be
    /// a real, usable number. Kept distinct from `InvalidNumber` (a
    /// token that doesn't even parse as a float at all): one is a typo,
    /// the other is a value that parses cleanly but can never mean
    /// anything physical.
    NonFiniteNumber {
        joint: String,
        text: String,
    },
    /// A joint's `<limit lower="..." upper="...">` has `lower > upper` -
    /// no real, finite value can ever satisfy both bounds at once, so
    /// every motion check against this limit would either always pass
    /// or always fail depending on which bound it happened to compare
    /// against first, never a real constraint.
    InvertedLimit {
        joint: String,
        text: String,
    },
    /// No `<joint>` at all has a `<child>` link that is never any OTHER
    /// joint's own `<parent>` - real URDF's tree has exactly one such
    /// link (the root), so zero real candidates means something is
    /// genuinely malformed (e.g. every link participates in a cycle).
    NoRootLink,
    /// More than one link is never any joint's own child - a real URDF
    /// document describes exactly one connected tree; two or more real
    /// roots means this is genuinely two or more disconnected robots in
    /// one file, which this v0 refuses rather than silently picking one.
    MultipleRootLinks {
        roots: Vec<String>,
    },
    /// A real link has more than one `<joint>` naming it as `<parent>` -
    /// a genuine branch in the URDF tree. `Chain` is a flat serial
    /// `Vec<Joint>` with no way to represent two children faithfully, so
    /// this is refused rather than silently keeping one branch and
    /// dropping the other.
    #[allow(dead_code)]
    Branching {
        link: String,
        joint_count: usize,
    },
    /// The tree walk from the real root reached fewer joints than this
    /// document actually declares - a real joint whose own `<parent>`
    /// link is never reachable from the root (e.g. it names a link that
    /// belongs to a second, disconnected sub-tree not caught by
    /// `MultipleRootLinks` because its own root happens to coincide with
    /// a real link name elsewhere). Refused rather than silently
    /// returning a truncated chain.
    UnreachableJoints {
        count: usize,
    },
    /// Two `<joint>` elements share a name. Positions are given by name, so
    /// one entry would silently move both.
    DuplicateJointName {
        joint: String,
    },
    /// Two joints name the same link as their `<child>`: in a URDF every link
    /// has at most one parent. (Walking such a file from the root would never
    /// end when the second parent is reachable from the first.)
    LinkWithTwoParents {
        link: String,
    },
    /// A joint names a link that no `<link>` element declares.
    UnknownLink {
        joint: String,
        link: String,
    },
    /// A moving joint whose axis has length 0 has no direction to move along.
    ZeroAxis {
        joint: String,
    },
    /// A `<mimic>` that cannot work: it names no joint, a joint that does not exist, itself, a fixed joint, or it is part of a cycle of followers.
    InvalidMimic {
        joint: String,
        reason: String,
    },
    /// A `<safety_controller>` whose soft limits are inverted or lie outside the hard `<limit>` they are meant to sit inside.
    InvalidSoftLimits {
        joint: String,
        reason: String,
    },
}

impl std::fmt::Display for UrdfError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            UrdfError::Xml(msg) => write!(f, "XML parse error: {msg}"),
            UrdfError::MissingAttribute { joint, attribute } => {
                write!(f, "joint '{joint}' is missing required attribute '{attribute}'")
            }
            UrdfError::UnsupportedJointType { joint, joint_type } => write!(
                f,
                "joint '{joint}' has unsupported type '{joint_type}' (v0 supports revolute/continuous/prismatic/fixed only)"
            ),
            UrdfError::InvalidNumber { joint, text } => {
                write!(f, "joint '{joint}' has a non-numeric value: '{text}'")
            }
            UrdfError::NonFiniteNumber { joint, text } => {
                write!(f, "joint '{joint}' has a non-finite value (NaN/infinity is not a real, usable number here): '{text}'")
            }
            UrdfError::InvertedLimit { joint, text } => {
                write!(f, "joint '{joint}' has an inverted limit ({text}) - lower must not exceed upper")
            }
            UrdfError::NoRootLink => write!(
                f,
                "could not find a real root link (every link is some joint's own child - the tree may contain a cycle)"
            ),
            UrdfError::MultipleRootLinks { roots } => write!(
                f,
                "found {} real root links ({}) - this v0 supports exactly one connected robot per document",
                roots.len(),
                roots.join(", ")
            ),
            UrdfError::Branching { link, joint_count } => write!(
                f,
                "link '{link}' has {joint_count} child joints - this v0's Chain is a flat serial chain and cannot represent a real branching tree"
            ),
            UrdfError::UnreachableJoints { count } => write!(
                f,
                "{count} joint(s) declared in this document are never reached walking the tree from its own real root link"
            ),
            UrdfError::DuplicateJointName { joint } => {
                write!(f, "joint name '{joint}' is used by more than one joint")
            }
            UrdfError::LinkWithTwoParents { link } => write!(
                f,
                "link '{link}' is the child of more than one joint - a URDF link has at most one parent"
            ),
            UrdfError::UnknownLink { joint, link } => {
                write!(f, "joint '{joint}' names link '{link}', which no <link> declares")
            }
            UrdfError::ZeroAxis { joint } => {
                write!(f, "joint '{joint}' moves along an axis of length 0")
            }
            UrdfError::InvalidMimic { joint, reason } => {
                write!(f, "joint '{joint}' has an invalid <mimic>: {reason}")
            }
            UrdfError::InvalidSoftLimits { joint, reason } => {
                write!(f, "joint '{joint}' has invalid soft limits: {reason}")
            }
        }
    }
}

/// Parses a real, space-separated "x y z" attribute value.
///
/// Previously this used to be `.filter_map(|s| s.parse().ok())` - a malformed token
/// (e.g. "typo" in "typo 1 2 3") was silently DROPPED rather than
/// failing the parse, so a 4-token value with one garbage token and
/// three real numbers was silently accepted as if it were the valid
/// 3-token value. Every token must now parse (`.collect::<Option<Vec<_>>>()`
/// short-circuits to `None` on the first failure), and every parsed
/// component must be finite - the same non-finite-value class of bug as
/// the joint limits below, just for a position/orientation component.
fn parse_xyz(text: &str) -> Option<Vec3> {
    let parts: Vec<f64> = text
        .split_whitespace()
        .map(|s| s.parse::<f64>().ok())
        .collect::<Option<Vec<f64>>>()?;
    if parts.len() == 3 && parts.iter().all(|v| v.is_finite()) {
        Some(Vec3::new(parts[0], parts[1], parts[2]))
    } else {
        None
    }
}

/// Parses an optional Vec3-shaped attribute (`xyz`/`rpy`) on an element
/// already known to exist (`node`), applying `default` only when the
/// attribute itself is genuinely ABSENT. a PRESENT but
/// malformed value (wrong token count, a non-numeric or non-finite
/// token) is a real error, never a silent fallback to that same
/// default - the old `.and_then(parse_xyz).unwrap_or(default)` made a
/// corrupt "typo 1 2 3" origin indistinguishable from a real,
/// deliberately-absent one.
fn parse_optional_vec3(
    node: roxmltree::Node,
    attribute: &'static str,
    joint: &str,
    default: Vec3,
) -> Result<Vec3, UrdfError> {
    match node.attribute(attribute) {
        None => Ok(default),
        Some(text) => parse_xyz(text).ok_or_else(|| UrdfError::InvalidNumber {
            joint: joint.to_string(),
            text: text.to_string(),
        }),
    }
}

/// Parses a required-when-present numeric attribute (a joint limit
/// bound), defaulting to `0.0` only when the attribute is genuinely
/// absent. Rust's own `f64::from_str` accepts
/// "nan"/"inf"/"infinity" as syntactically valid floats, so
/// `.parse::<f64>()` alone never rejects a non-finite limit -
/// `is_finite()` is checked explicitly.
fn parse_finite_attribute(
    node: roxmltree::Node,
    attribute: &str,
    joint: &str,
) -> Result<f64, UrdfError> {
    let text = node.attribute(attribute).unwrap_or("0");
    let value: f64 = text.parse().map_err(|_| UrdfError::InvalidNumber {
        joint: joint.to_string(),
        text: text.to_string(),
    })?;
    if !value.is_finite() {
        return Err(UrdfError::NonFiniteNumber {
            joint: joint.to_string(),
            text: text.to_string(),
        });
    }
    Ok(value)
}

/// One joint's real, parsed state plus the two real link names
/// (`<parent link="..."/>`/`<child link="..."/>`) that place it in the
/// document's own real URDF tree - kept separate from `Joint` itself so
/// `Joint`'s own public shape (every existing construction site across
/// this crate's tests) never needed to change for this fix.
struct ParsedJoint {
    joint: Joint,
    parent_link: String,
    child_link: String,
    extras: JointExtras,
}

fn parse_one_joint(node: roxmltree::Node) -> Result<ParsedJoint, UrdfError> {
    let name = node
        .attribute("name")
        .ok_or(UrdfError::MissingAttribute {
            joint: "<unnamed>".into(),
            attribute: "name",
        })?
        .to_string();

    let type_str = node
        .attribute("type")
        .ok_or_else(|| UrdfError::MissingAttribute {
            joint: name.clone(),
            attribute: "type",
        })?;
    let joint_type = match type_str {
        "revolute" => JointType::Revolute,
        "continuous" => JointType::Continuous,
        "prismatic" => JointType::Prismatic,
        "fixed" => JointType::Fixed,
        other => {
            return Err(UrdfError::UnsupportedJointType {
                joint: name,
                joint_type: other.to_string(),
            })
        }
    };

    // Real URDF requires exactly one <parent>/<child> per joint - these
    // are what place this joint in the document's own real tree, unlike
    // every field below which only describes the joint's own motion.
    let parent_link = node
        .children()
        .find(|c| c.has_tag_name("parent"))
        .and_then(|c| c.attribute("link"))
        .ok_or_else(|| UrdfError::MissingAttribute {
            joint: name.clone(),
            attribute: "parent",
        })?
        .to_string();
    let child_link = node
        .children()
        .find(|c| c.has_tag_name("child"))
        .and_then(|c| c.attribute("link"))
        .ok_or_else(|| UrdfError::MissingAttribute {
            joint: name.clone(),
            attribute: "child",
        })?
        .to_string();

    let origin = match node.children().find(|c| c.has_tag_name("origin")) {
        Some(c) => {
            let xyz = parse_optional_vec3(c, "xyz", &name, Vec3::ZERO)?;
            let rpy = parse_optional_vec3(c, "rpy", &name, Vec3::ZERO)?;
            Mat4::translation(xyz).mul(&Mat4::from_rpy(rpy.x, rpy.y, rpy.z))
        }
        None => Mat4::identity(),
    };

    // URDF's own default axis when <axis> is absent is (1, 0, 0).
    let axis = match node.children().find(|c| c.has_tag_name("axis")) {
        Some(c) => parse_optional_vec3(c, "xyz", &name, Vec3::new(1.0, 0.0, 0.0))?,
        None => Vec3::new(1.0, 0.0, 0.0),
    };

    if joint_type != JointType::Fixed && axis.norm() < 1e-9 {
        return Err(UrdfError::ZeroAxis { joint: name });
    }

    // Only a revolute or prismatic joint has position limits. A continuous
    // joint's <limit> carries effort and velocity only (lower and upper
    // default to 0 and used to lock it at 0 rad), and a fixed joint cannot
    // move: both are read as having none.
    let has_position_limits = matches!(joint_type, JointType::Revolute | JointType::Prismatic);
    let limit = node
        .children()
        .filter(|_| has_position_limits)
        .find(|c| c.has_tag_name("limit"))
        .map(|c| {
            let lower = parse_finite_attribute(c, "lower", &name)?;
            let upper = parse_finite_attribute(c, "upper", &name)?;
            if lower > upper {
                return Err(UrdfError::InvertedLimit {
                    joint: name.clone(),
                    text: format!("lower={lower} upper={upper}"),
                });
            }
            Ok::<(f64, f64), UrdfError>((lower, upper))
        })
        .transpose()?;

    // effort and velocity live in <limit> for every joint type (a continuous joint has no position limit but has these)
    let mut extras = JointExtras::default();
    if let Some(c) = node.children().find(|c| c.has_tag_name("limit")) {
        extras.effort_limit = parse_optional_non_negative(c, "effort", &name)?;
        extras.velocity_limit = parse_optional_non_negative(c, "velocity", &name)?;
    }

    if let Some(c) = node.children().find(|c| c.has_tag_name("mimic")) {
        if joint_type == JointType::Fixed {
            return Err(UrdfError::InvalidMimic {
                joint: name,
                reason: "a fixed joint cannot follow another".to_string(),
            });
        }
        let master = c
            .attribute("joint")
            .filter(|m| !m.is_empty())
            .ok_or_else(|| UrdfError::InvalidMimic {
                joint: name.clone(),
                reason: "it names no joint to follow".to_string(),
            })?;
        let multiplier = parse_optional_number(c, "multiplier", &name, 1.0)?;
        let offset = parse_optional_number(c, "offset", &name, 0.0)?;
        extras.mimic = Some(Mimic {
            joint: master.to_string(),
            multiplier,
            offset,
        });
    }

    if let Some(c) = node
        .children()
        .find(|c| c.has_tag_name("safety_controller"))
    {
        let soft = SoftLimits {
            lower: parse_optional_number(c, "soft_lower_limit", &name, 0.0)?,
            upper: parse_optional_number(c, "soft_upper_limit", &name, 0.0)?,
            k_position: parse_optional_number(c, "k_position", &name, 0.0)?,
            k_velocity: parse_optional_number(c, "k_velocity", &name, 0.0)?,
        };
        if soft.lower > soft.upper {
            return Err(UrdfError::InvalidSoftLimits {
                joint: name,
                reason: format!(
                    "soft_lower_limit {} is above soft_upper_limit {}",
                    soft.lower, soft.upper
                ),
            });
        }
        if let Some((lower, upper)) = limit {
            if soft.lower < lower - 1e-9 || soft.upper > upper + 1e-9 {
                return Err(UrdfError::InvalidSoftLimits {
                    joint: name,
                    reason: format!(
                        "the soft range [{}, {}] is not inside the hard limit [{lower}, {upper}]",
                        soft.lower, soft.upper
                    ),
                });
            }
        }
        if has_position_limits {
            extras.soft = Some(soft);
        }
    }

    Ok(ParsedJoint {
        joint: Joint {
            name,
            joint_type,
            origin,
            axis,
            limit,
        },
        parent_link,
        child_link,
        extras,
    })
}

/// A numeric attribute that may be absent (`default`), but when present must be a finite number.
fn parse_optional_number(
    node: roxmltree::Node,
    attribute: &str,
    joint: &str,
    default: f64,
) -> Result<f64, UrdfError> {
    match node.attribute(attribute) {
        None => Ok(default),
        Some(_) => parse_finite_attribute(node, attribute, joint),
    }
}

/// Like `parse_optional_number` with no default, and a negative value is refused: a speed or effort limit is a magnitude.
fn parse_optional_non_negative(
    node: roxmltree::Node,
    attribute: &str,
    joint: &str,
) -> Result<Option<f64>, UrdfError> {
    if node.attribute(attribute).is_none() {
        return Ok(None);
    }
    let value = parse_finite_attribute(node, attribute, joint)?;
    if value < 0.0 {
        return Err(UrdfError::InvalidNumber {
            joint: joint.to_string(),
            text: format!("{attribute}=\"{value}\" (a limit is not negative)"),
        });
    }
    Ok(Some(value))
}

/// Reads the whole tree of a URDF: every branch, the mimic joints, and the effort, velocity and soft limits.
pub fn parse_urdf_model(xml: &str) -> Result<Model, UrdfError> {
    let doc = roxmltree::Document::parse(xml).map_err(|e| UrdfError::Xml(e.to_string()))?;

    // Only the <joint> elements that are direct children of <robot>: a
    // <transmission> (ros_control) holds <joint name="..."> elements of its
    // own, without a type, which are not part of the kinematic tree.
    let robot = doc.root_element();
    let parsed: Vec<ParsedJoint> = robot
        .children()
        .filter(|n| n.has_tag_name("joint"))
        .map(parse_one_joint)
        .collect::<Result<_, _>>()?;

    let mut names = std::collections::HashSet::new();
    for p in &parsed {
        if !names.insert(p.joint.name.as_str()) {
            return Err(UrdfError::DuplicateJointName {
                joint: p.joint.name.clone(),
            });
        }
    }
    let mut child_names = std::collections::HashSet::new();
    for p in &parsed {
        if !child_names.insert(p.child_link.as_str()) {
            return Err(UrdfError::LinkWithTwoParents {
                link: p.child_link.clone(),
            });
        }
    }
    // When the document declares its links, every link a joint names must be one of them.
    let declared_links: std::collections::HashSet<&str> = robot
        .children()
        .filter(|n| n.has_tag_name("link"))
        .filter_map(|n| n.attribute("name"))
        .collect();
    if !declared_links.is_empty() {
        for p in &parsed {
            for link in [p.parent_link.as_str(), p.child_link.as_str()] {
                if !declared_links.contains(link) {
                    return Err(UrdfError::UnknownLink {
                        joint: p.joint.name.clone(),
                        link: link.to_string(),
                    });
                }
            }
        }
    }

    if parsed.is_empty() {
        // A real, valid single-link URDF (no joints at all) - an empty
        // chain, not an error.
        return Ok(Model {
            chain: Chain { joints: Vec::new() },
            root_link: robot
                .children()
                .find(|n| n.has_tag_name("link"))
                .and_then(|n| n.attribute("name"))
                .unwrap_or("")
                .to_string(),
            parents: Vec::new(),
            extras: Vec::new(),
        });
    }

    // The real tree this document actually describes: every link that is
    // some joint's own <parent>, mapped to the joint(s) whose <parent>
    // names it - found by walking from the one real root, never by
    // trusting the order these <joint> elements happen to appear in.
    let mut children_of: std::collections::HashMap<&str, Vec<usize>> =
        std::collections::HashMap::new();
    for (i, p) in parsed.iter().enumerate() {
        children_of
            .entry(p.parent_link.as_str())
            .or_default()
            .push(i);
    }

    let child_links: std::collections::HashSet<&str> =
        parsed.iter().map(|p| p.child_link.as_str()).collect();
    let roots: Vec<&str> = children_of
        .keys()
        .copied()
        .filter(|link| !child_links.contains(link))
        .collect();

    let root = match roots.as_slice() {
        [] => return Err(UrdfError::NoRootLink),
        [single] => *single,
        _ => {
            let mut roots: Vec<String> = roots.iter().map(|s| s.to_string()).collect();
            roots.sort();
            return Err(UrdfError::MultipleRootLinks { roots });
        }
    };

    // Depth-first from the root, children in the order the document lists them: every joint comes after the joint that carries its parent link.
    let mut order: Vec<usize> = Vec::with_capacity(parsed.len());
    let mut parent_of_joint: std::collections::HashMap<usize, Option<usize>> =
        std::collections::HashMap::new();
    let mut stack: Vec<(usize, Option<usize>)> = children_of
        .get(root)
        .map(|v| v.iter().rev().map(|&i| (i, None)).collect())
        .unwrap_or_default();
    while let Some((index, parent)) = stack.pop() {
        // each link has one parent (checked above), so a joint can only be met twice through a cycle
        if parent_of_joint.insert(index, parent).is_some() {
            return Err(UrdfError::NoRootLink);
        }
        order.push(index);
        if let Some(children) = children_of.get(parsed[index].child_link.as_str()) {
            for &child in children.iter().rev() {
                stack.push((child, Some(index)));
            }
        }
    }

    if order.len() != parsed.len() {
        return Err(UrdfError::UnreachableJoints {
            count: parsed.len() - order.len(),
        });
    }

    let position_in_order: std::collections::HashMap<usize, usize> =
        order.iter().enumerate().map(|(pos, &i)| (i, pos)).collect();
    let parents: Vec<Option<usize>> = order
        .iter()
        .map(|i| parent_of_joint[i].map(|p| position_in_order[&p]))
        .collect();
    let mut joints = Vec::with_capacity(order.len());
    let mut extras = Vec::with_capacity(order.len());
    for &i in &order {
        joints.push(parsed[i].joint.clone());
        extras.push(parsed[i].extras.clone());
    }
    validate_mimics(&joints, &extras)?;

    Ok(Model {
        chain: Chain { joints },
        root_link: root.to_string(),
        parents,
        extras,
    })
}

/// A mimic must follow a joint that exists, is not itself, can move, and the followers must not follow each other in a circle.
fn validate_mimics(joints: &[Joint], extras: &[JointExtras]) -> Result<(), UrdfError> {
    let index_of = |name: &str| joints.iter().position(|j| j.name == name);
    for (i, extra) in extras.iter().enumerate() {
        let Some(mimic) = &extra.mimic else {
            continue;
        };
        let fail = |reason: String| UrdfError::InvalidMimic {
            joint: joints[i].name.clone(),
            reason,
        };
        let Some(master) = index_of(&mimic.joint) else {
            return Err(fail(format!("there is no joint named '{}'", mimic.joint)));
        };
        if master == i {
            return Err(fail("a joint cannot follow itself".to_string()));
        }
        if joints[master].joint_type == JointType::Fixed {
            return Err(fail(format!(
                "'{}' is a fixed joint: it has no position to follow",
                mimic.joint
            )));
        }
        let mut seen = vec![i];
        let mut here = master;
        while let Some(next) = &extras[here].mimic {
            let Some(next_index) = index_of(&next.joint) else {
                return Err(fail(format!(
                    "'{}' follows '{}', which does not exist",
                    joints[here].name, next.joint
                )));
            };
            if seen.contains(&next_index) || seen.contains(&here) {
                return Err(fail(
                    "the followers follow each other in a circle".to_string(),
                ));
            }
            seen.push(here);
            here = next_index;
        }
    }
    Ok(())
}

/// The serial chain of a URDF, in root-to-leaf order. A branching tree is refused (`parse_urdf_model` reads those).
#[allow(dead_code)]
pub fn parse_urdf(xml: &str) -> Result<Chain, UrdfError> {
    let model = parse_urdf_model(xml)?;
    let mut children_count: std::collections::HashMap<Option<usize>, usize> =
        std::collections::HashMap::new();
    for parent in &model.parents {
        *children_count.entry(*parent).or_default() += 1;
    }
    if let Some((parent, count)) = children_count.iter().find(|(_, &c)| c > 1) {
        // name the link they hang from, as the error always did
        let first_child = model.parents.iter().position(|p| p == parent).unwrap_or(0);
        return Err(UrdfError::Branching {
            link: first_parent_link(xml, &model.chain.joints[first_child].name),
            joint_count: *count,
        });
    }
    Ok(model.chain)
}

/// The `<parent link>` of a named joint, for the message of `UrdfError::Branching`.
fn first_parent_link(xml: &str, joint: &str) -> String {
    roxmltree::Document::parse(xml)
        .ok()
        .and_then(|doc| {
            doc.root_element()
                .children()
                .filter(|n| n.has_tag_name("joint") && n.attribute("name") == Some(joint))
                .find_map(|n| {
                    n.children()
                        .find(|c| c.has_tag_name("parent"))
                        .and_then(|c| c.attribute("link"))
                        .map(str::to_string)
                })
        })
        .unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_a_single_revolute_joint() {
        let xml = r#"
            <robot name="test">
              <joint name="j1" type="revolute">
                <parent link="base"/>
                <child link="link1"/>
                <origin xyz="0 0 0.1" rpy="0 0 0"/>
                <axis xyz="0 0 1"/>
                <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
              </joint>
            </robot>
        "#;
        let chain = parse_urdf(xml).unwrap();
        assert_eq!(chain.joints.len(), 1);
        let j = &chain.joints[0];
        assert_eq!(j.name, "j1");
        assert_eq!(j.joint_type, JointType::Revolute);
        assert_eq!(j.axis, Vec3::new(0.0, 0.0, 1.0));
        assert_eq!(j.limit, Some((-1.57, 1.57)));
    }

    #[test]
    fn defaults_origin_and_axis_when_absent() {
        let xml = r#"
            <robot>
              <joint name="j1" type="fixed">
                <parent link="base"/>
                <child link="link1"/>
              </joint>
            </robot>
        "#;
        let chain = parse_urdf(xml).unwrap();
        let j = &chain.joints[0];
        assert_eq!(j.axis, Vec3::new(1.0, 0.0, 0.0));
        assert_eq!(j.limit, None);
        assert_eq!(j.origin.translation_part(), Vec3::ZERO);
    }

    #[test]
    fn multiple_joints_parsed_in_document_order() {
        let xml = r#"
            <robot>
              <joint name="j1" type="revolute">
                <parent link="base"/><child link="link1"/>
                <axis xyz="0 0 1"/>
              </joint>
              <joint name="j2" type="prismatic">
                <parent link="link1"/><child link="link2"/>
                <axis xyz="1 0 0"/>
              </joint>
            </robot>
        "#;
        let chain = parse_urdf(xml).unwrap();
        assert_eq!(chain.joints.len(), 2);
        assert_eq!(chain.joints[0].name, "j1");
        assert_eq!(chain.joints[1].name, "j2");
    }

    #[test]
    fn joints_out_of_document_order_are_still_assembled_root_to_leaf() {
        // Real URDF makes no guarantee about <joint> element order - here
        // the file lists the tip joint first and the root joint last, and
        // the real tree (via <parent>/<child>, not document position)
        // must still walk out base -> link1 -> link2 -> link3.
        let xml = r#"
            <robot>
              <joint name="j3" type="revolute"><parent link="link2"/><child link="link3"/></joint>
              <joint name="j1" type="revolute"><parent link="base"/><child link="link1"/></joint>
              <joint name="j2" type="revolute"><parent link="link1"/><child link="link2"/></joint>
            </robot>
        "#;
        let chain = parse_urdf(xml).unwrap();
        let names: Vec<&str> = chain.joints.iter().map(|j| j.name.as_str()).collect();
        assert_eq!(names, vec!["j1", "j2", "j3"]);
    }

    #[test]
    fn missing_name_is_an_error() {
        let xml = r#"<robot><joint type="revolute"><parent link="base"/><child link="link1"/></joint></robot>"#;
        assert!(matches!(
            parse_urdf(xml),
            Err(UrdfError::MissingAttribute { .. })
        ));
    }

    #[test]
    fn missing_type_is_an_error() {
        let xml =
            r#"<robot><joint name="j1"><parent link="base"/><child link="link1"/></joint></robot>"#;
        assert!(matches!(
            parse_urdf(xml),
            Err(UrdfError::MissingAttribute { .. })
        ));
    }

    #[test]
    fn missing_parent_is_an_error() {
        let xml =
            r#"<robot><joint name="j1" type="revolute"><child link="link1"/></joint></robot>"#;
        assert!(matches!(
            parse_urdf(xml),
            Err(UrdfError::MissingAttribute {
                attribute: "parent",
                ..
            })
        ));
    }

    #[test]
    fn missing_child_is_an_error() {
        let xml =
            r#"<robot><joint name="j1" type="revolute"><parent link="base"/></joint></robot>"#;
        assert!(matches!(
            parse_urdf(xml),
            Err(UrdfError::MissingAttribute {
                attribute: "child",
                ..
            })
        ));
    }

    #[test]
    fn unsupported_joint_type_is_an_error() {
        let xml = r#"<robot><joint name="j1" type="floating"><parent link="base"/><child link="link1"/></joint></robot>"#;
        assert!(matches!(
            parse_urdf(xml),
            Err(UrdfError::UnsupportedJointType { .. })
        ));
    }

    #[test]
    fn malformed_xml_is_an_error() {
        assert!(matches!(
            parse_urdf("<robot><joint"),
            Err(UrdfError::Xml(_))
        ));
    }

    #[test]
    fn no_joints_at_all_is_an_empty_chain() {
        let chain = parse_urdf(r#"<robot><link name="base"/></robot>"#).unwrap();
        assert_eq!(chain.joints.len(), 0);
    }

    #[test]
    fn a_link_with_two_child_joints_is_a_real_branching_error() {
        let xml = r#"
            <robot>
              <joint name="j1" type="revolute"><parent link="base"/><child link="left"/></joint>
              <joint name="j2" type="revolute"><parent link="base"/><child link="right"/></joint>
            </robot>
        "#;
        assert!(matches!(
            parse_urdf(xml),
            Err(UrdfError::Branching { joint_count: 2, .. })
        ));
    }

    #[test]
    fn every_link_being_someone_elses_child_is_a_no_root_error() {
        // A real cycle: base <- j1 <- link1 <- j2 <- base. No link is ever
        // free of being some joint's own <child>, so there is no real root.
        let xml = r#"
            <robot>
              <joint name="j1" type="revolute"><parent link="link1"/><child link="base"/></joint>
              <joint name="j2" type="revolute"><parent link="base"/><child link="link1"/></joint>
            </robot>
        "#;
        assert!(matches!(parse_urdf(xml), Err(UrdfError::NoRootLink)));
    }

    #[test]
    fn two_disconnected_robots_in_one_file_is_a_multiple_roots_error() {
        let xml = r#"
            <robot>
              <joint name="j1" type="revolute"><parent link="base_a"/><child link="link_a"/></joint>
              <joint name="j2" type="revolute"><parent link="base_b"/><child link="link_b"/></joint>
            </robot>
        "#;
        assert!(matches!(
            parse_urdf(xml),
            Err(UrdfError::MultipleRootLinks { roots }) if roots == vec!["base_a".to_string(), "base_b".to_string()]
        ));
    }

    #[test]
    fn a_disconnected_cycle_alongside_the_real_tree_is_unreachable() {
        // A genuinely disconnected 2-cycle (x's <parent> is y's <child>
        // and vice-versa) never shows up in `roots` at all - both x and y
        // ARE some joint's own <child>, so neither looks like a second
        // root - yet neither is reachable by walking from the one real
        // root ("base"). This is the one real failure shape
        // `MultipleRootLinks`/`NoRootLink`/`Branching` cannot catch.
        let xml = r#"
            <robot>
              <joint name="j1" type="revolute"><parent link="base"/><child link="link1"/></joint>
              <joint name="j2" type="revolute"><parent link="x"/><child link="y"/></joint>
              <joint name="j3" type="revolute"><parent link="y"/><child link="x"/></joint>
            </robot>
        "#;
        assert!(matches!(
            parse_urdf(xml),
            Err(UrdfError::UnreachableJoints { count: 2 })
        ));
    }

    // the finding's own exact reproduction - a URDF with a
    // malformed `origin xyz` (a real token mixed with garbage) and a
    // non-finite joint limit used to be silently accepted, producing a
    // real, wrong pose (an FK check against a 1000-unit rail joint came
    // back with a position offset by the garbage-adjacent real tokens,
    // exit code 0) instead of a real, explicit parse error.

    #[test]
    fn an_origin_xyz_with_one_garbage_token_among_real_numbers_is_rejected_not_silently_truncated()
    {
        // Before the fix: filter_map silently dropped "typo", leaving
        // exactly 3 real numbers (1, 2, 3) - accepted as if the
        // attribute had genuinely been "1 2 3".
        let xml = r#"
            <robot>
              <joint name="j1" type="fixed">
                <parent link="base"/>
                <child link="link1"/>
                <origin xyz="typo 1 2 3" rpy="0 0 0"/>
              </joint>
            </robot>
        "#;
        assert!(matches!(
            parse_urdf(xml),
            Err(UrdfError::InvalidNumber { .. })
        ));
    }

    #[test]
    fn a_nan_joint_limit_is_rejected_not_silently_accepted() {
        // Rust's own f64::from_str accepts "NaN" as a syntactically
        // valid float - .parse::<f64>() alone never catches this.
        let xml = r#"
            <robot>
              <joint name="j1" type="revolute">
                <parent link="base"/>
                <child link="link1"/>
                <limit lower="NaN" upper="1.57" effort="10" velocity="1"/>
              </joint>
            </robot>
        "#;
        assert!(matches!(
            parse_urdf(xml),
            Err(UrdfError::NonFiniteNumber { .. })
        ));
    }

    #[test]
    fn an_inverted_joint_limit_is_rejected() {
        let xml = r#"
            <robot>
              <joint name="j1" type="revolute">
                <parent link="base"/>
                <child link="link1"/>
                <limit lower="1.0" upper="-1.0" effort="10" velocity="1"/>
              </joint>
            </robot>
        "#;
        assert!(matches!(
            parse_urdf(xml),
            Err(UrdfError::InvertedLimit { .. })
        ));
    }

    #[test]
    fn a_genuinely_absent_origin_still_defaults_cleanly() {
        // Confirms the fix distinguishes "attribute absent" (fine,
        // real default) from "attribute present but corrupt" (a real
        // error) - the genuinely-absent case above in
        // defaults_origin_and_axis_when_absent must keep working.
        let xml = r#"
            <robot>
              <joint name="j1" type="fixed">
                <parent link="base"/>
                <child link="link1"/>
                <origin rpy="0 0 0"/>
              </joint>
            </robot>
        "#;
        let chain = parse_urdf(xml).expect("an absent xyz must default, not error");
        assert_eq!(chain.joints[0].origin.translation_part(), Vec3::ZERO);
    }

    fn two_joint_urdf(extra: &str, second_child: &str) -> String {
        format!(
            r#"<robot name="t">
              <link name="base"/><link name="l1"/><link name="l2"/>
              <joint name="a" type="revolute"><parent link="base"/><child link="l1"/>
                <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
              <joint name="b" type="revolute"><parent link="l1"/><child link="{second_child}"/>
                <axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
              {extra}
            </robot>"#
        )
    }

    #[test]
    fn joints_inside_a_transmission_are_not_part_of_the_tree() {
        let xml = two_joint_urdf(
            r#"<transmission name="t1"><type>transmission_interface/SimpleTransmission</type>
                 <joint name="a"><hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface></joint>
               </transmission>"#,
            "l2",
        );
        let chain = parse_urdf(&xml).expect("a standard ros_control URDF must parse");
        assert_eq!(chain.joints.len(), 2);
    }

    #[test]
    fn a_link_with_two_parents_is_refused_instead_of_walking_for_ever() {
        // base->l1, l1->l2, and a third joint l2->l1: l1 would have two parents
        let xml = two_joint_urdf(
            r#"<joint name="c" type="fixed"><parent link="l2"/><child link="l1"/></joint>"#,
            "l2",
        );
        assert_eq!(
            parse_urdf(&xml).unwrap_err(),
            UrdfError::LinkWithTwoParents { link: "l1".into() }
        );
    }

    #[test]
    fn two_joints_with_the_same_name_are_refused() {
        let xml = two_joint_urdf("", "l2").replace("name=\"b\"", "name=\"a\"");
        assert_eq!(
            parse_urdf(&xml).unwrap_err(),
            UrdfError::DuplicateJointName { joint: "a".into() }
        );
    }

    #[test]
    fn a_joint_naming_a_link_nobody_declares_is_refused() {
        let xml = two_joint_urdf("", "ghost");
        assert_eq!(
            parse_urdf(&xml).unwrap_err(),
            UrdfError::UnknownLink {
                joint: "b".into(),
                link: "ghost".into()
            }
        );
    }

    #[test]
    fn a_moving_joint_with_a_zero_axis_is_refused_but_a_fixed_one_may_have_any() {
        let moving = r#"<robot name="t"><joint name="j" type="revolute"><parent link="a"/><child link="b"/>
            <axis xyz="0 0 0"/><limit lower="-1" upper="1"/></joint></robot>"#;
        assert_eq!(
            parse_urdf(moving).unwrap_err(),
            UrdfError::ZeroAxis { joint: "j".into() }
        );
        let fixed = moving.replace("revolute", "fixed");
        assert!(parse_urdf(&fixed).is_ok());
    }

    #[test]
    fn a_continuous_joint_with_effort_and_velocity_only_is_not_locked_at_zero() {
        let xml = r#"<robot name="t"><joint name="spin" type="continuous"><parent link="a"/><child link="b"/>
            <axis xyz="0 0 1"/><limit effort="10" velocity="2"/></joint></robot>"#;
        let chain = parse_urdf(xml).unwrap();
        assert_eq!(chain.joints[0].limit, None);
    }

    // ---- the tree, the followers and the limits beyond the position

    const GRIPPER: &str = r#"<robot name="g"><link name="base"/><link name="palm"/><link name="l"/><link name="r"/>
        <joint name="finger_right" type="prismatic"><parent link="palm"/><child link="r"/><axis xyz="1 0 0"/>
          <limit lower="-1" upper="0" effort="5" velocity="0.5"/><mimic joint="finger_left" multiplier="-1" offset="0.1"/></joint>
        <joint name="finger_left" type="prismatic"><parent link="palm"/><child link="l"/><axis xyz="1 0 0"/>
          <limit lower="0" upper="1" effort="5" velocity="0.5"/></joint>
        <joint name="wrist" type="revolute"><parent link="base"/><child link="palm"/><axis xyz="0 0 1"/>
          <limit lower="-1" upper="1" effort="9" velocity="2"/>
          <safety_controller soft_lower_limit="-0.8" soft_upper_limit="0.8" k_position="5" k_velocity="1"/></joint>
      </robot>"#;

    #[test]
    fn a_branching_tree_is_read_with_every_joint_after_the_joint_that_carries_it() {
        let model = parse_urdf_model(GRIPPER).unwrap();
        let names: Vec<&str> = model.chain.joints.iter().map(|j| j.name.as_str()).collect();
        assert_eq!(names, vec!["wrist", "finger_right", "finger_left"]);
        assert_eq!(model.parents, vec![None, Some(0), Some(0)]);
        assert_eq!(model.root_link, "base");
        assert!(!model.is_serial());
        assert_eq!(model.joint_index("finger_left"), Some(2));
        // the serial reader still refuses it, naming the link that branches
        assert_eq!(
            parse_urdf(GRIPPER).unwrap_err(),
            UrdfError::Branching {
                link: "palm".into(),
                joint_count: 2
            }
        );
    }

    #[test]
    fn a_serial_chain_is_serial_as_a_model_too() {
        let model = parse_urdf_model(&two_joint_urdf("", "l2")).unwrap();
        assert!(model.is_serial());
    }

    #[test]
    fn mimic_effort_velocity_and_soft_limits_are_read() {
        let model = parse_urdf_model(GRIPPER).unwrap();
        let follower = &model.extras[1];
        assert_eq!(
            follower.mimic,
            Some(Mimic {
                joint: "finger_left".into(),
                multiplier: -1.0,
                offset: 0.1
            })
        );
        assert_eq!(follower.velocity_limit, Some(0.5));
        assert_eq!(follower.effort_limit, Some(5.0));
        assert_eq!(
            model.extras[0].soft,
            Some(SoftLimits {
                lower: -0.8,
                upper: 0.8,
                k_position: 5.0,
                k_velocity: 1.0
            })
        );
        assert_eq!(model.extras[2].mimic, None);
    }

    #[test]
    fn a_continuous_joint_keeps_its_velocity_and_effort() {
        let xml = r#"<robot name="t"><joint name="spin" type="continuous"><parent link="a"/><child link="b"/>
            <axis xyz="0 0 1"/><limit effort="10" velocity="2"/></joint></robot>"#;
        let model = parse_urdf_model(xml).unwrap();
        assert_eq!(model.extras[0].velocity_limit, Some(2.0));
        assert_eq!(model.extras[0].effort_limit, Some(10.0));
        assert_eq!(model.chain.joints[0].limit, None);
    }

    fn with_extra(inside_second_joint: &str) -> String {
        format!(
            r#"<robot name="t"><joint name="a" type="revolute"><parent link="x"/><child link="y"/><axis xyz="0 0 1"/>
                <limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
               <joint name="b" type="revolute"><parent link="y"/><child link="z"/><axis xyz="0 0 1"/>
                <limit lower="-1" upper="1" effort="1" velocity="1"/>{inside_second_joint}</joint></robot>"#
        )
    }

    #[test]
    fn a_mimic_that_cannot_work_is_refused() {
        for (inside, reason_part) in [
            (r#"<mimic joint="ghost"/>"#, "no joint named"),
            (r#"<mimic joint="b"/>"#, "itself"),
            (r#"<mimic/>"#, "no joint"),
        ] {
            match parse_urdf_model(&with_extra(inside)).unwrap_err() {
                UrdfError::InvalidMimic { joint, reason } => {
                    assert_eq!(joint, "b");
                    assert!(reason.contains(reason_part), "{reason}");
                }
                other => panic!("{other:?}"),
            }
        }
        assert!(matches!(
            parse_urdf_model(&with_extra(r#"<mimic joint="a" multiplier="nan"/>"#)),
            Err(UrdfError::NonFiniteNumber { .. })
        ));
    }

    #[test]
    fn a_fixed_joint_cannot_follow_and_cannot_be_followed() {
        let xml = r#"<robot name="t"><joint name="a" type="fixed"><parent link="x"/><child link="y"/></joint>
            <joint name="b" type="revolute"><parent link="y"/><child link="z"/><axis xyz="0 0 1"/><limit lower="-1" upper="1"/>
              <mimic joint="a"/></joint></robot>"#;
        assert!(matches!(
            parse_urdf_model(xml),
            Err(UrdfError::InvalidMimic { .. })
        ));
        let follower_fixed = xml.replace(
            r#"<joint name="a" type="fixed">"#,
            r#"<joint name="a" type="fixed"><mimic joint="b"/>"#,
        );
        assert!(matches!(
            parse_urdf_model(&follower_fixed),
            Err(UrdfError::InvalidMimic { .. })
        ));
    }

    #[test]
    fn followers_that_follow_each_other_in_a_circle_are_refused() {
        let xml = r#"<robot name="t">
            <joint name="a" type="revolute"><parent link="x"/><child link="y"/><axis xyz="0 0 1"/><limit lower="-1" upper="1"/><mimic joint="b"/></joint>
            <joint name="b" type="revolute"><parent link="y"/><child link="z"/><axis xyz="0 0 1"/><limit lower="-1" upper="1"/><mimic joint="c"/></joint>
            <joint name="c" type="revolute"><parent link="z"/><child link="w"/><axis xyz="0 0 1"/><limit lower="-1" upper="1"/><mimic joint="a"/></joint>
          </robot>"#;
        match parse_urdf_model(xml).unwrap_err() {
            UrdfError::InvalidMimic { reason, .. } => {
                assert!(reason.contains("circle"), "{reason}")
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn a_follower_of_a_follower_is_fine() {
        let xml = r#"<robot name="t">
            <joint name="a" type="revolute"><parent link="x"/><child link="y"/><axis xyz="0 0 1"/><limit lower="-1" upper="1"/></joint>
            <joint name="b" type="revolute"><parent link="y"/><child link="z"/><axis xyz="0 0 1"/><limit lower="-1" upper="1"/><mimic joint="a" multiplier="2"/></joint>
            <joint name="c" type="revolute"><parent link="z"/><child link="w"/><axis xyz="0 0 1"/><limit lower="-1" upper="1"/><mimic joint="b" offset="1"/></joint>
          </robot>"#;
        assert!(parse_urdf_model(xml).is_ok());
    }

    #[test]
    fn soft_limits_must_be_ordered_and_inside_the_hard_limit() {
        let inverted =
            with_extra(r#"<safety_controller soft_lower_limit="0.5" soft_upper_limit="-0.5"/>"#);
        assert!(matches!(
            parse_urdf_model(&inverted),
            Err(UrdfError::InvalidSoftLimits { .. })
        ));
        let outside =
            with_extra(r#"<safety_controller soft_lower_limit="-2" soft_upper_limit="0.5"/>"#);
        assert!(matches!(
            parse_urdf_model(&outside),
            Err(UrdfError::InvalidSoftLimits { .. })
        ));
        let fine = with_extra(r#"<safety_controller soft_lower_limit="-1" soft_upper_limit="1"/>"#);
        assert!(parse_urdf_model(&fine).is_ok());
    }

    #[test]
    fn a_negative_or_unreadable_speed_or_effort_limit_is_refused() {
        for bad in [
            r#"effort="-1""#,
            r#"velocity="-0.1""#,
            r#"velocity="fast""#,
            r#"effort="inf""#,
        ] {
            let xml = format!(
                r#"<robot name="t"><joint name="a" type="revolute"><parent link="x"/><child link="y"/><axis xyz="0 0 1"/>
                    <limit lower="-1" upper="1" {bad}/></joint></robot>"#
            );
            assert!(parse_urdf_model(&xml).is_err(), "{bad}");
        }
    }
}
