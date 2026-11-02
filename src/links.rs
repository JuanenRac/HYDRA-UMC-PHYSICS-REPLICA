// =============================================================================
// HYDRA-UMC-PHYSICS-REPLICA - src/links.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0-or-later - see LICENSE
// =============================================================================
//! What a URDF says about each link besides where it hangs: the `<collision>` shapes and the `<inertial>` block.
//!
//! Reading a link never fails the whole file: forward kinematics needs neither, so a file that was readable before stays readable. A
//! malformed shape or inertial block is kept as a problem and reported by the code that needs it (the capsules of the self-collision
//! check, the inverse dynamics), never replaced by a guess.

use crate::transform::{Mat4, Vec3};

/// A collision shape, in the frame of its own `<origin>` (z is the axis of a cylinder, as in URDF).
#[derive(Debug, Clone, PartialEq)]
pub enum Shape {
    Sphere {
        radius: f64,
    },
    Cylinder {
        radius: f64,
        length: f64,
    },
    Box {
        size: [f64; 3],
    },
    /// A mesh: its triangles are not read here, so no capsule can be built from it.
    Mesh {
        filename: String,
    },
    /// Anything else, or a shape whose numbers are wrong; the reason is kept.
    Unusable(String),
}

#[derive(Debug, Clone, PartialEq)]
pub struct Collision {
    /// Pose of the shape in the frame of its link.
    pub origin: Mat4,
    pub shape: Shape,
}

/// `<inertial>`: the mass, the centre of mass and the inertia tensor about it, in the frame of `origin`.
#[derive(Debug, Clone, PartialEq)]
pub struct Inertial {
    pub origin: Mat4,
    pub mass: f64,
    pub inertia: [[f64; 3]; 3],
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct LinkInfo {
    pub collisions: Vec<Collision>,
    /// None: the link has no `<inertial>`.
    pub inertial: Option<Inertial>,
    /// Why the `<inertial>` block could not be read (then `inertial` is None).
    pub inertial_problem: Option<String>,
}

fn numbers(text: &str, count: usize) -> Option<Vec<f64>> {
    let parts: Vec<f64> = text
        .split_whitespace()
        .map(|s| s.parse::<f64>().ok().filter(|v| v.is_finite()))
        .collect::<Option<Vec<f64>>>()?;
    (parts.len() == count).then_some(parts)
}

fn origin_of(node: roxmltree::Node) -> Result<Mat4, String> {
    let Some(origin) = node.children().find(|c| c.has_tag_name("origin")) else {
        return Ok(Mat4::identity());
    };
    let read = |attribute: &str| -> Result<Vec3, String> {
        match origin.attribute(attribute) {
            None => Ok(Vec3::ZERO),
            Some(text) => numbers(text, 3)
                .map(|v| Vec3::new(v[0], v[1], v[2]))
                .ok_or_else(|| {
                    format!("<origin {attribute}=\"{text}\"> is not three finite numbers")
                }),
        }
    };
    let xyz = read("xyz")?;
    let rpy = read("rpy")?;
    Ok(Mat4::translation(xyz).mul(&Mat4::from_rpy(rpy.x, rpy.y, rpy.z)))
}

fn positive(node: roxmltree::Node, attribute: &str) -> Result<f64, String> {
    let text = node
        .attribute(attribute)
        .ok_or_else(|| format!("<{}> has no {attribute}", node.tag_name().name()))?;
    text.parse::<f64>()
        .ok()
        .filter(|v| v.is_finite() && *v > 0.0)
        .ok_or_else(|| format!("{attribute}=\"{text}\" must be a finite number above zero"))
}

fn shape_of(geometry: roxmltree::Node) -> Shape {
    let Some(element) = geometry.children().find(|c| c.is_element()) else {
        return Shape::Unusable("<geometry> is empty".into());
    };
    let read = || -> Result<Shape, String> {
        match element.tag_name().name() {
            "sphere" => Ok(Shape::Sphere {
                radius: positive(element, "radius")?,
            }),
            "cylinder" => Ok(Shape::Cylinder {
                radius: positive(element, "radius")?,
                length: positive(element, "length")?,
            }),
            "box" => {
                let text = element.attribute("size").ok_or("<box> has no size")?;
                let v = numbers(text, 3)
                    .filter(|v| v.iter().all(|x| *x > 0.0))
                    .ok_or_else(|| format!("size=\"{text}\" must be three numbers above zero"))?;
                Ok(Shape::Box {
                    size: [v[0], v[1], v[2]],
                })
            }
            "mesh" => Ok(Shape::Mesh {
                filename: element.attribute("filename").unwrap_or("").to_string(),
            }),
            other => Err(format!("the geometry <{other}> is not supported")),
        }
    };
    read().unwrap_or_else(Shape::Unusable)
}

fn inertial_of(node: roxmltree::Node) -> Result<Inertial, String> {
    let origin = origin_of(node)?;
    let mass_node = node
        .children()
        .find(|c| c.has_tag_name("mass"))
        .ok_or("<inertial> has no <mass>")?;
    let mass_text = mass_node.attribute("value").ok_or("<mass> has no value")?;
    let mass = mass_text
        .parse::<f64>()
        .ok()
        .filter(|v| v.is_finite() && *v >= 0.0)
        .ok_or_else(|| format!("mass=\"{mass_text}\" must be a finite number, not negative"))?;
    let inertia_node = node
        .children()
        .find(|c| c.has_tag_name("inertia"))
        .ok_or("<inertial> has no <inertia>")?;
    let read = |name: &str| -> Result<f64, String> {
        let text = inertia_node
            .attribute(name)
            .ok_or_else(|| format!("<inertia> has no {name}"))?;
        text.parse::<f64>()
            .ok()
            .filter(|v| v.is_finite())
            .ok_or_else(|| format!("{name}=\"{text}\" is not a finite number"))
    };
    let (ixx, ixy, ixz, iyy, iyz, izz) = (
        read("ixx")?,
        read("ixy")?,
        read("ixz")?,
        read("iyy")?,
        read("iyz")?,
        read("izz")?,
    );
    if ixx < 0.0 || iyy < 0.0 || izz < 0.0 {
        return Err("the diagonal of the inertia tensor must not be negative".into());
    }
    Ok(Inertial {
        origin,
        mass,
        inertia: [[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]],
    })
}

/// Reads one `<link>` element.
pub fn parse_link(link: roxmltree::Node) -> LinkInfo {
    let mut info = LinkInfo::default();
    for node in link.children().filter(|n| n.is_element()) {
        match node.tag_name().name() {
            "collision" => {
                let shape = node
                    .children()
                    .find(|c| c.has_tag_name("geometry"))
                    .map_or_else(
                        || Shape::Unusable("<collision> has no <geometry>".into()),
                        shape_of,
                    );
                match origin_of(node) {
                    Ok(origin) => info.collisions.push(Collision { origin, shape }),
                    Err(reason) => info.collisions.push(Collision {
                        origin: Mat4::identity(),
                        shape: Shape::Unusable(reason),
                    }),
                }
            }
            "inertial" => match inertial_of(node) {
                Ok(inertial) => info.inertial = Some(inertial),
                Err(reason) => info.inertial_problem = Some(reason),
            },
            _ => {}
        }
    }
    info
}

#[cfg(test)]
mod tests {
    use super::*;

    fn link(xml: &str) -> LinkInfo {
        let doc = roxmltree::Document::parse(xml).unwrap();
        parse_link(doc.root_element())
    }

    #[test]
    fn shapes_and_inertial_are_read() {
        let info = link(
            r#"<link name="a">
              <collision><origin xyz="0 0 0.1" rpy="0 1.5 0"/><geometry><cylinder radius="0.05" length="0.2"/></geometry></collision>
              <collision><geometry><box size="0.1 0.2 0.3"/></geometry></collision>
              <collision><geometry><sphere radius="0.04"/></geometry></collision>
              <collision><geometry><mesh filename="a.stl"/></geometry></collision>
              <inertial><origin xyz="0.01 0 0.02"/><mass value="1.5"/>
                <inertia ixx="0.1" ixy="0.01" ixz="0" iyy="0.2" iyz="0" izz="0.3"/></inertial>
            </link>"#,
        );
        assert_eq!(info.collisions.len(), 4);
        assert_eq!(
            info.collisions[0].shape,
            Shape::Cylinder {
                radius: 0.05,
                length: 0.2
            }
        );
        assert_eq!(
            info.collisions[1].shape,
            Shape::Box {
                size: [0.1, 0.2, 0.3]
            }
        );
        assert!(matches!(info.collisions[3].shape, Shape::Mesh { .. }));
        let inertial = info.inertial.unwrap();
        assert_eq!(inertial.mass, 1.5);
        assert_eq!(inertial.inertia[0][1], 0.01);
        assert_eq!(inertial.inertia[1][0], 0.01);
    }

    #[test]
    fn wrong_numbers_are_kept_as_problems_not_guessed() {
        let info = link(
            r#"<link name="a">
              <collision><geometry><cylinder radius="-1" length="0.2"/></geometry></collision>
              <collision><geometry><capsule radius="1" length="0.2"/></geometry></collision>
              <collision/>
              <inertial><mass value="-2"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
            </link>"#,
        );
        assert!(info
            .collisions
            .iter()
            .all(|c| matches!(c.shape, Shape::Unusable(_))));
        assert!(info.inertial.is_none());
        assert!(info.inertial_problem.unwrap().contains("mass"));
    }

    #[test]
    fn a_link_with_nothing_has_nothing() {
        let info = link(r#"<link name="a"/>"#);
        assert!(info.collisions.is_empty());
        assert!(info.inertial.is_none() && info.inertial_problem.is_none());
    }
}
