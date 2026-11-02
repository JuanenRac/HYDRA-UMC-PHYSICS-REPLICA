// =============================================================================
// HYDRA-UMC-PHYSICS-REPLICA - src/mujoco_bridge.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0-or-later - see LICENSE
// =============================================================================
//! The optional bridge to MuJoCo (feature `mujoco`, off by default; the rest of the program neither builds nor needs it).
//!
//! The URDF is written as MJCF (`mjcf.rs`) and loaded by `mujoco-rs`. Two things are offered:
//!
//! - `mujoco_inverse`: the joint torques that MuJoCo's own inverse dynamics gives for a state, to put next to the torques of the RNEA of
//!   `dynamics.rs` (the `mujoco-check` command prints both and the largest difference);
//! - `mujoco_step`: the joint positions after a number of simulation steps from a state, with MuJoCo's contacts off (a rigid-body
//!   playback of the robot under gravity, not a model of what it touches).
//!
//! Contacts are switched off in the model that is loaded, so that the torques compared are those of the free robot. A robot with a
//! `<mimic>` joint is refused: MuJoCo holds it with an equality constraint whose force is reported on its own joint, which is not
//! comparable with the torque folded onto the master.

use std::collections::HashMap;

use mujoco_rs::prelude::*;

use crate::mjcf;
use crate::urdf::Model;

fn load(model: &Model, name: &str) -> Result<(MjModel, Vec<String>), String> {
    if model.extras.iter().any(|e| e.mimic.is_some()) {
        return Err("a robot with a <mimic> joint is not compared with MuJoCo (its constraint force is on the follower)".into());
    }
    let out = mjcf::to_mjcf(model, name)?;
    // the free robot: no contacts between its own shapes
    let xml = out.xml.replace(
        "<compiler angle=\"radian\"/>",
        "<compiler angle=\"radian\"/>\n  <option><flag contact=\"disable\"/></option>",
    );
    let mj = MjModel::from_xml_string(&xml)
        .map_err(|e| format!("MuJoCo could not load the model: {e:?}"))?;
    let names = model
        .chain
        .joints
        .iter()
        .filter(|j| j.joint_type != crate::urdf::JointType::Fixed)
        .map(|j| j.name.clone())
        .collect();
    Ok((mj, names))
}

fn set_state(
    data: &mut MjData<&MjModel>,
    names: &[String],
    positions: &HashMap<String, f64>,
    velocities: &HashMap<String, f64>,
    accelerations: &HashMap<String, f64>,
) -> Result<(), String> {
    for name in names {
        let info = data
            .joint(name)
            .ok_or_else(|| format!("MuJoCo has no joint '{name}'"))?;
        let mut view = info.view_mut(data);
        let get = |m: &HashMap<String, f64>, what: &str| {
            m.get(name)
                .copied()
                .ok_or_else(|| format!("no {what} for joint '{name}'"))
        };
        view.qpos[0] = get(positions, "position")?;
        view.qvel[0] = get(velocities, "velocity")?;
        view.qacc[0] = get(accelerations, "acceleration")?;
    }
    Ok(())
}

/// The torques of MuJoCo's inverse dynamics for a state, by joint name.
pub fn mujoco_inverse(
    model: &Model,
    robot_name: &str,
    positions: &HashMap<String, f64>,
    velocities: &HashMap<String, f64>,
    accelerations: &HashMap<String, f64>,
) -> Result<HashMap<String, f64>, String> {
    let (mj, names) = load(model, robot_name)?;
    let mut data = mj.make_data();
    set_state(&mut data, &names, positions, velocities, accelerations)?;
    data.inverse();
    let mut out = HashMap::new();
    for name in &names {
        let info = data.joint(name).ok_or("a joint went missing")?;
        out.insert(name.clone(), info.view(&data).qfrc_inverse[0]);
    }
    Ok(out)
}

/// The joint positions after `steps` steps of the simulation from a state (velocities given, contacts off).
pub fn mujoco_step(
    model: &Model,
    robot_name: &str,
    positions: &HashMap<String, f64>,
    velocities: &HashMap<String, f64>,
    steps: usize,
) -> Result<HashMap<String, f64>, String> {
    let (mj, names) = load(model, robot_name)?;
    let mut data = mj.make_data();
    let zero: HashMap<String, f64> = names.iter().map(|n| (n.clone(), 0.0)).collect();
    set_state(&mut data, &names, positions, velocities, &zero)?;
    for _ in 0..steps {
        data.step();
    }
    let mut out = HashMap::new();
    for name in &names {
        let info = data.joint(name).ok_or("a joint went missing")?;
        out.insert(name.clone(), info.view(&data).qpos[0]);
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dynamics;
    use crate::urdf::parse_urdf_model;

    /// A 3D arm with tilted axes and frames, an off-centre and rotated inertia, and a prismatic joint (no follower: see the module notes).
    const ARM: &str = r#"<robot name="arm">
      <link name="base"/>
      <link name="turret"><inertial><origin xyz="0.02 0.01 0.05" rpy="0.3 0.2 0.1"/><mass value="2.5"/>
        <inertia ixx="0.03" ixy="0.002" ixz="0.001" iyy="0.04" iyz="0.003" izz="0.05"/></inertial></link>
      <link name="upper"><inertial><origin xyz="0.15 0.02 -0.01" rpy="0.1 -0.4 0.5"/><mass value="1.8"/>
        <inertia ixx="0.02" ixy="0.001" ixz="0.0005" iyy="0.03" iyz="0.002" izz="0.025"/></inertial></link>
      <link name="slider"><inertial><origin xyz="0.05 0 0"/><mass value="0.7"/>
        <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.006" iyz="0" izz="0.004"/></inertial></link>
      <joint name="yaw" type="revolute"><parent link="base"/><child link="turret"/>
        <origin xyz="0.05 0 0.1" rpy="0.1 0.05 0"/><axis xyz="0.1 0.2 1"/><limit lower="-3" upper="3" effort="50" velocity="5"/></joint>
      <joint name="pitch" type="revolute"><parent link="turret"/><child link="upper"/>
        <origin xyz="0.03 0.02 0.2" rpy="0.2 0 0.4"/><axis xyz="0 1 0.3"/><limit lower="-3" upper="3" effort="50" velocity="5"/></joint>
      <joint name="slide" type="prismatic"><parent link="upper"/><child link="slider"/>
        <origin xyz="0.3 0 0" rpy="0 0.3 0.2"/><axis xyz="1 0.1 0"/><limit lower="0" upper="0.3" effort="50" velocity="5"/></joint>
    </robot>"#;

    fn map(pairs: &[(&str, f64)]) -> HashMap<String, f64> {
        pairs.iter().map(|(k, v)| (k.to_string(), *v)).collect()
    }

    /// The RNEA of this program and the inverse dynamics of MuJoCo agree on the torques of the same robot.
    #[test]
    fn the_rnea_agrees_with_the_inverse_dynamics_of_mujoco() {
        let model = parse_urdf_model(ARM).unwrap();
        let q = map(&[("yaw", 0.4), ("pitch", -0.7), ("slide", 0.12)]);
        let qd = map(&[("yaw", 0.9), ("pitch", -1.3), ("slide", 0.4)]);
        let qdd = map(&[("yaw", -0.6), ("pitch", 1.7), ("slide", 0.8)]);
        let ours = dynamics::inverse_dynamics(&model, &q, &qd, &qdd, &dynamics::Options::default())
            .unwrap();
        let theirs = mujoco_inverse(&model, "arm", &q, &qd, &qdd).unwrap();
        for (name, tau) in &ours.torques {
            assert!(
                (tau - theirs[name]).abs() < 1e-6 * (1.0 + tau.abs()),
                "{name}: rnea {tau} vs mujoco {}",
                theirs[name]
            );
        }
    }

    #[test]
    fn a_free_fall_step_moves_a_vertical_slide_down() {
        let xml = r#"<robot name="s"><link name="base"/>
          <link name="car"><inertial><mass value="4"/><inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/></inertial></link>
          <joint name="z" type="prismatic"><parent link="base"/><child link="car"/><axis xyz="0 0 1"/>
            <limit lower="-10" upper="10" effort="100" velocity="10"/></joint></robot>"#;
        let model = parse_urdf_model(xml).unwrap();
        let after =
            mujoco_step(&model, "s", &map(&[("z", 0.0)]), &map(&[("z", 0.0)]), 100).unwrap();
        assert!(after["z"] < -0.0);
    }
}
