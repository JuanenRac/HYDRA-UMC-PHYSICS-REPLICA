// =============================================================================
// HYDRA-UMC-PHYSICS-REPLICA - src/mjcf.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0-or-later - see LICENSE
// =============================================================================
//! The URDF written as MJCF, the model format of MuJoCo: one `<body>` per link nested as the tree of joints, a `<joint>` per moving
//! joint (hinge or slide, with its position range), the `<collision>` shapes as `<geom>`, the `<inertial>` blocks as `<inertial>` and
//! the `<mimic>` joints as `<equality>` constraints.
//!
//! This is plain text and needs no physics engine: the file can be read by MuJoCo or by any other tool of the format. Whatever cannot be
//! written faithfully is listed in the warnings instead of being guessed: a mesh (its file is not read), a moving link without
//! `<inertial>` and without shapes (MuJoCo refuses a moving body with no mass).

use crate::links::Shape;
use crate::transform::Mat4;
use crate::urdf::{JointType, Model};

pub struct Mjcf {
    pub xml: String,
    pub warnings: Vec<String>,
}

fn escape(text: &str) -> String {
    text.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
}

fn pos(m: &Mat4) -> String {
    let t = m.translation_part();
    format!("{} {} {}", t.x, t.y, t.z)
}

fn quat(m: &Mat4) -> String {
    let (w, x, y, z) = m.quaternion();
    format!("{w} {x} {y} {z}")
}

fn geom(
    link: &str,
    k: usize,
    c: &crate::links::Collision,
    warnings: &mut Vec<String>,
) -> Result<Option<String>, String> {
    let placement = format!("pos=\"{}\" quat=\"{}\"", pos(&c.origin), quat(&c.origin));
    Ok(match &c.shape {
        Shape::Sphere { radius } => Some(format!(
            "<geom type=\"sphere\" size=\"{radius}\" {placement}/>"
        )),
        Shape::Cylinder { radius, length } => Some(format!(
            "<geom type=\"cylinder\" size=\"{radius} {}\" {placement}/>",
            length / 2.0
        )),
        Shape::Box { size } => Some(format!(
            "<geom type=\"box\" size=\"{} {} {}\" {placement}/>",
            size[0] / 2.0,
            size[1] / 2.0,
            size[2] / 2.0
        )),
        Shape::Mesh { filename } => {
            warnings.push(format!(
                "link '{link}', collision {k}: the mesh '{filename}' is not written (its file is not read)"
            ));
            None
        }
        Shape::Unusable(reason) => return Err(format!("link '{link}', collision {k}: {reason}")),
    })
}

/// Writes the model as MJCF.
pub fn to_mjcf(model: &Model, robot_name: &str) -> Result<Mjcf, String> {
    let joints = &model.chain.joints;
    let mut warnings = Vec::new();
    let mut children: Vec<Vec<usize>> = vec![Vec::new(); joints.len() + 1];
    for (i, parent) in model.parents.iter().enumerate() {
        children[parent.map_or(0, |p| p + 1)].push(i);
    }
    let mut out = String::new();
    out.push_str(&format!("<mujoco model=\"{}\">\n", escape(robot_name)));
    out.push_str("  <compiler angle=\"radian\"/>\n  <worldbody>\n");

    fn body(
        model: &Model,
        node: usize,
        children: &[Vec<usize>],
        depth: usize,
        out: &mut String,
        warnings: &mut Vec<String>,
    ) -> Result<(), String> {
        let pad = "  ".repeat(depth);
        let (name, header) = if node == 0 {
            (model.root_link.clone(), String::new())
        } else {
            let joint = &model.chain.joints[node - 1];
            (
                model.child_links[node - 1].clone(),
                format!(
                    " pos=\"{}\" quat=\"{}\"",
                    pos(&joint.origin),
                    quat(&joint.origin)
                ),
            )
        };
        out.push_str(&format!("{pad}<body name=\"{}\"{header}>\n", escape(&name)));
        let inner = format!("{pad}  ");
        let mut has_mass = false;
        if node > 0 {
            let i = node - 1;
            let joint = &model.chain.joints[i];
            let a = joint.axis.normalized();
            let range = joint
                .limit
                .map(|(lo, hi)| format!(" limited=\"true\" range=\"{lo} {hi}\""))
                .unwrap_or_default();
            let force = model.extras[i]
                .effort_limit
                .map(|e| format!(" actuatorfrcrange=\"{} {e}\"", -e))
                .unwrap_or_default();
            match joint.joint_type {
                JointType::Fixed => {}
                JointType::Revolute | JointType::Continuous => out.push_str(&format!(
                    "{inner}<joint name=\"{}\" type=\"hinge\" axis=\"{} {} {}\"{range}{force}/>\n",
                    escape(&joint.name),
                    a.x,
                    a.y,
                    a.z
                )),
                JointType::Prismatic => out.push_str(&format!(
                    "{inner}<joint name=\"{}\" type=\"slide\" axis=\"{} {} {}\"{range}{force}/>\n",
                    escape(&joint.name),
                    a.x,
                    a.y,
                    a.z
                )),
            }
        }
        if let Some(info) = model.links.get(&name) {
            if let Some(problem) = &info.inertial_problem {
                return Err(format!("link '{name}': {problem}"));
            }
            if let Some(inertial) = &info.inertial {
                if node > 0 {
                    has_mass = true;
                    // MuJoCo takes a full inertia matrix in the frame of the body (it finds the principal axes itself) and refuses an
                    // orientation next to it: the tensor of the URDF, given in the frame of the inertial origin, is turned into the frame
                    // of the link first.
                    let r = &inertial.origin.m;
                    let rot = |a: usize, b: usize| r[a][b];
                    let mut i = [[0.0f64; 3]; 3];
                    for (row, out_row) in i.iter_mut().enumerate() {
                        for (col, cell) in out_row.iter_mut().enumerate() {
                            *cell = (0..3)
                                .flat_map(|k| (0..3).map(move |l| (k, l)))
                                .map(|(k, l)| rot(row, k) * inertial.inertia[k][l] * rot(col, l))
                                .sum();
                        }
                    }
                    out.push_str(&format!(
                        "{inner}<inertial pos=\"{}\" mass=\"{}\" fullinertia=\"{} {} {} {} {} {}\"/>\n",
                        pos(&inertial.origin),
                        inertial.mass,
                        i[0][0],
                        i[1][1],
                        i[2][2],
                        i[0][1],
                        i[0][2],
                        i[1][2]
                    ));
                }
            }
            let mut written = 0;
            for (k, c) in info.collisions.iter().enumerate() {
                if let Some(g) = geom(&name, k, c, warnings)? {
                    out.push_str(&format!("{inner}{g}\n"));
                    written += 1;
                }
            }
            if node > 0
                && !has_mass
                && written == 0
                && model.chain.joints[node - 1].joint_type != JointType::Fixed
            {
                warnings.push(format!(
                    "link '{name}' moves but has no <inertial> and no shape: MuJoCo refuses a moving body without mass"
                ));
            }
        } else if node > 0 && model.chain.joints[node - 1].joint_type != JointType::Fixed {
            warnings.push(format!("link '{name}' is not declared in the file"));
        }
        for &child in &children[node] {
            body(model, child + 1, children, depth + 1, out, warnings)?;
        }
        out.push_str(&format!("{pad}</body>\n"));
        Ok(())
    }

    body(model, 0, &children, 2, &mut out, &mut warnings)?;
    out.push_str("  </worldbody>\n");
    let followers: Vec<String> = model
        .extras
        .iter()
        .enumerate()
        .filter_map(|(i, e)| {
            e.mimic.as_ref().map(|m| {
                format!(
                    "    <joint joint1=\"{}\" joint2=\"{}\" polycoef=\"{} {} 0 0 0\"/>\n",
                    escape(&joints[i].name),
                    escape(&m.joint),
                    m.offset,
                    m.multiplier
                )
            })
        })
        .collect();
    if !followers.is_empty() {
        out.push_str("  <equality>\n");
        for f in followers {
            out.push_str(&f);
        }
        out.push_str("  </equality>\n");
    }
    out.push_str("</mujoco>\n");
    Ok(Mjcf { xml: out, warnings })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::urdf::parse_urdf_model;

    const ARM: &str = r#"<robot name="arm">
      <link name="base"><collision><geometry><box size="0.2 0.2 0.1"/></geometry></collision></link>
      <link name="upper">
        <collision><origin xyz="0 0 0.1"/><geometry><cylinder radius="0.03" length="0.2"/></geometry></collision>
        <collision><geometry><mesh filename="x.stl"/></geometry></collision>
        <inertial><origin xyz="0 0 0.1"/><mass value="1.2"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.03"/></inertial></link>
      <link name="a"/><link name="b"/>
      <joint name="shoulder" type="revolute"><parent link="base"/><child link="upper"/>
        <origin xyz="0 0 0.05" rpy="0 0 1.5707963267948966"/><axis xyz="0 1 0"/><limit lower="-1" upper="2" effort="30" velocity="2"/></joint>
      <joint name="s1" type="prismatic"><parent link="upper"/><child link="a"/><axis xyz="1 0 0"/>
        <limit lower="0" upper="0.1" effort="5" velocity="1"/></joint>
      <joint name="s2" type="prismatic"><parent link="upper"/><child link="b"/><axis xyz="1 0 0"/>
        <limit lower="0" upper="0.1" effort="5" velocity="1"/><mimic joint="s1" multiplier="-1" offset="0.1"/></joint>
    </robot>"#;

    #[test]
    fn the_tree_joints_shapes_and_followers_are_written() {
        let model = parse_urdf_model(ARM).unwrap();
        let out = to_mjcf(&model, "arm").unwrap();
        let xml = &out.xml;
        assert!(roxmltree::Document::parse(xml).is_ok(), "{xml}");
        assert!(xml.contains("<body name=\"base\">"));
        assert!(xml.contains("<geom type=\"box\" size=\"0.1 0.1 0.05\""));
        assert!(xml.contains("<geom type=\"cylinder\" size=\"0.03 0.1\""));
        assert!(xml.contains("<joint name=\"shoulder\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-1 2\" actuatorfrcrange=\"-30 30\"/>"));
        assert!(xml.contains("type=\"slide\""));
        assert!(xml.contains("<inertial pos=\"0 0 0.1\""));
        assert!(xml.contains("mass=\"1.2\" fullinertia=\"0.01 0.02 0.03 0 0 0\""));
        assert!(xml.contains("<joint joint1=\"s2\" joint2=\"s1\" polycoef=\"0.1 -1 0 0 0\"/>"));
        // the follower and its master are siblings under the upper body
        let doc = roxmltree::Document::parse(xml).unwrap();
        let upper = doc
            .descendants()
            .find(|n| n.attribute("name") == Some("upper"))
            .unwrap();
        assert_eq!(
            upper.children().filter(|c| c.has_tag_name("body")).count(),
            2
        );
        // the quaternion of the quarter turn about z
        assert!(xml.contains("quat=\"0.7071"));
    }

    #[test]
    fn what_cannot_be_written_is_a_warning() {
        let model = parse_urdf_model(ARM).unwrap();
        let out = to_mjcf(&model, "arm").unwrap();
        assert!(out.warnings.iter().any(|w| w.contains("x.stl")));
        assert!(out.warnings.iter().any(|w| w.contains("'a' moves")));
    }

    #[test]
    fn a_bad_shape_is_an_error() {
        let xml = r#"<robot name="m"><link name="a"><collision><geometry><box size="0 1 1"/></geometry></collision></link></robot>"#;
        let model = parse_urdf_model(xml).unwrap();
        assert!(to_mjcf(&model, "m").is_err());
    }
}
