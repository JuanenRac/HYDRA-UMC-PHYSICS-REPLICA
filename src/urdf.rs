// =============================================================================
// HYDRA-UMC-PHYSICS-REPLICA - src/urdf.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0 - see LICENSE
// =============================================================================
//! A real, deliberately partial URDF reader.
//!
//! Found in an ecosystem-wide software-improvements audit: this used to
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
//! Still an honest, deliberate v0 boundary: `Chain` itself remains a flat
//! `Vec<Joint>` - a genuinely BRANCHING tree (one link with more than one
//! child joint) has no single root-to-leaf order to flatten it into, and
//! `parse_urdf` refuses it with a real `UrdfError::Branching` rather than
//! silently picking one branch and discarding the rest. `HYDRA-UMC-EDITOR-URDF`'s
//! own catalog is itself entirely single serial arms today, which is
//! what makes a real, walked serial chain a genuinely useful v0 rather
//! than a half-measure - full tree/multi-branch support is real,
//! separate future work once a real branching robot exists to design
//! its own output shape against.

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
    /// PHYS-01 (found in an ecosystem-wide software-improvements audit,
    /// P0): Rust's own `f64::from_str` accepts "nan"/"inf"/"infinity" as
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
        }
    }
}

/// Parses a real, space-separated "x y z" attribute value. PHYS-01
/// (found in an ecosystem-wide software-improvements audit, P0): this
/// used to be `.filter_map(|s| s.parse().ok())` - a malformed token
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
/// attribute itself is genuinely ABSENT. PHYS-01: a PRESENT but
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
/// absent. PHYS-01: Rust's own `f64::from_str` accepts
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

    let limit = node
        .children()
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
    })
}

pub fn parse_urdf(xml: &str) -> Result<Chain, UrdfError> {
    let doc = roxmltree::Document::parse(xml).map_err(|e| UrdfError::Xml(e.to_string()))?;

    let parsed: Vec<ParsedJoint> = doc
        .descendants()
        .filter(|n| n.has_tag_name("joint"))
        .map(parse_one_joint)
        .collect::<Result<_, _>>()?;

    if parsed.is_empty() {
        // A real, valid single-link URDF (no joints at all) - an empty
        // chain, not an error.
        return Ok(Chain { joints: Vec::new() });
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

    let mut ordered: Vec<Joint> = Vec::with_capacity(parsed.len());
    let mut current_link = root;
    // A real leaf link (`children_of.get` returns `None`) ends the walk.
    while let Some(here) = children_of.get(current_link) {
        match here.as_slice() {
            [] => break,
            [only] => {
                let p = &parsed[*only];
                ordered.push(p.joint.clone());
                current_link = p.child_link.as_str();
            }
            many => {
                return Err(UrdfError::Branching {
                    link: current_link.to_string(),
                    joint_count: many.len(),
                })
            }
        }
    }

    if ordered.len() != parsed.len() {
        return Err(UrdfError::UnreachableJoints {
            count: parsed.len() - ordered.len(),
        });
    }

    Ok(Chain { joints: ordered })
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

    // PHYS-01 (found in an ecosystem-wide software-improvements audit,
    // P0): the finding's own exact reproduction - a URDF with a
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
}
