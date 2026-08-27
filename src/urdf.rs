// =============================================================================
// HYDRA-UMC-PHYSICS-REPLICA - src/urdf.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0 - see LICENSE
// =============================================================================
//! A real, deliberately partial URDF reader.
//!
//! Honest scope: this parses `<joint>` elements (`type`, `<origin>`,
//! `<axis>`, `<limit>`) in document order and treats that order as a single
//! serial chain. Real URDF describes a tree of links that can branch; a
//! branching robot would need every `<joint>`'s `parent`/`child` link names
//! actually walked from the root link, which this v0 does not do. That is
//! real, documented future work (see `mejoras_futuras.txt`), not a hidden
//! gap - `HYDRA-UMC-EDITOR-URDF`'s own catalog is itself mostly single
//! serial arms today, which is what makes this a genuinely useful v0.

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
        }
    }
}

fn parse_xyz(text: &str) -> Option<Vec3> {
    let parts: Vec<f64> = text
        .split_whitespace()
        .filter_map(|s| s.parse().ok())
        .collect();
    if parts.len() == 3 {
        Some(Vec3::new(parts[0], parts[1], parts[2]))
    } else {
        None
    }
}

pub fn parse_urdf(xml: &str) -> Result<Chain, UrdfError> {
    let doc = roxmltree::Document::parse(xml).map_err(|e| UrdfError::Xml(e.to_string()))?;

    let mut joints = Vec::new();
    for node in doc.descendants().filter(|n| n.has_tag_name("joint")) {
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

        let origin = node
            .children()
            .find(|c| c.has_tag_name("origin"))
            .map(|c| {
                let xyz = c.attribute("xyz").and_then(parse_xyz).unwrap_or(Vec3::ZERO);
                let rpy = c.attribute("rpy").and_then(parse_xyz).unwrap_or(Vec3::ZERO);
                Mat4::translation(xyz).mul(&Mat4::from_rpy(rpy.x, rpy.y, rpy.z))
            })
            .unwrap_or_else(Mat4::identity);

        // URDF's own default axis when <axis> is absent is (1, 0, 0).
        let axis = node
            .children()
            .find(|c| c.has_tag_name("axis"))
            .and_then(|c| c.attribute("xyz"))
            .and_then(parse_xyz)
            .unwrap_or(Vec3::new(1.0, 0.0, 0.0));

        let limit = node
            .children()
            .find(|c| c.has_tag_name("limit"))
            .map(|c| {
                let lower = c
                    .attribute("lower")
                    .unwrap_or("0")
                    .parse::<f64>()
                    .map_err(|_| UrdfError::InvalidNumber {
                        joint: name.clone(),
                        text: c.attribute("lower").unwrap_or("").into(),
                    })?;
                let upper = c
                    .attribute("upper")
                    .unwrap_or("0")
                    .parse::<f64>()
                    .map_err(|_| UrdfError::InvalidNumber {
                        joint: name.clone(),
                        text: c.attribute("upper").unwrap_or("").into(),
                    })?;
                Ok::<(f64, f64), UrdfError>((lower, upper))
            })
            .transpose()?;

        joints.push(Joint {
            name,
            joint_type,
            origin,
            axis,
            limit,
        });
    }

    Ok(Chain { joints })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_a_single_revolute_joint() {
        let xml = r#"
            <robot name="test">
              <joint name="j1" type="revolute">
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
        let xml = r#"<robot><joint name="j1" type="fixed"/></robot>"#;
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
              <joint name="j1" type="revolute"><axis xyz="0 0 1"/></joint>
              <joint name="j2" type="prismatic"><axis xyz="1 0 0"/></joint>
            </robot>
        "#;
        let chain = parse_urdf(xml).unwrap();
        assert_eq!(chain.joints.len(), 2);
        assert_eq!(chain.joints[0].name, "j1");
        assert_eq!(chain.joints[1].name, "j2");
    }

    #[test]
    fn missing_name_is_an_error() {
        let xml = r#"<robot><joint type="revolute"/></robot>"#;
        assert!(matches!(
            parse_urdf(xml),
            Err(UrdfError::MissingAttribute { .. })
        ));
    }

    #[test]
    fn missing_type_is_an_error() {
        let xml = r#"<robot><joint name="j1"/></robot>"#;
        assert!(matches!(
            parse_urdf(xml),
            Err(UrdfError::MissingAttribute { .. })
        ));
    }

    #[test]
    fn unsupported_joint_type_is_an_error() {
        let xml = r#"<robot><joint name="j1" type="floating"/></robot>"#;
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
}
