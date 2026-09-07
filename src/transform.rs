// =============================================================================
// HYDRA-UMC-PHYSICS-REPLICA - src/transform.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0 - see LICENSE
// =============================================================================
//! Real, dependency-free 3D transform math: `Vec3` and a row-major 4x4
//! homogeneous transform `Mat4`. This is the actual linear algebra behind
//! forward kinematics - no physics-engine crate involved, since none of it
//! needs anything beyond translation, axis-angle rotation, and matrix
//! composition.

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub const ZERO: Vec3 = Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Vec3 { x, y, z }
    }

    pub fn norm(&self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    pub fn normalized(&self) -> Vec3 {
        let n = self.norm();
        if n == 0.0 {
            *self
        } else {
            Vec3::new(self.x / n, self.y / n, self.z / n)
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mat4 {
    // Row-major: m[row][col].
    pub m: [[f64; 4]; 4],
}

impl Mat4 {
    pub fn identity() -> Mat4 {
        let mut m = [[0.0; 4]; 4];
        for (index, row) in m.iter_mut().enumerate() {
            row[index] = 1.0;
        }
        Mat4 { m }
    }

    pub fn translation(t: Vec3) -> Mat4 {
        let mut mat = Mat4::identity();
        mat.m[0][3] = t.x;
        mat.m[1][3] = t.y;
        mat.m[2][3] = t.z;
        mat
    }

    /// Rodrigues' rotation formula around a (not necessarily normalized)
    /// axis, by `angle` radians. A zero-length axis returns the identity -
    /// real behavior for a `fixed` joint's default axis, never a NaN.
    pub fn rotation_axis_angle(axis: Vec3, angle: f64) -> Mat4 {
        let a = axis.normalized();
        if a.norm() == 0.0 {
            return Mat4::identity();
        }
        let (s, c) = angle.sin_cos();
        let t = 1.0 - c;
        let (x, y, z) = (a.x, a.y, a.z);
        Mat4 {
            m: [
                [t * x * x + c, t * x * y - s * z, t * x * z + s * y, 0.0],
                [t * x * y + s * z, t * y * y + c, t * y * z - s * x, 0.0],
                [t * x * z - s * y, t * y * z + s * x, t * z * z + c, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        }
    }

    /// URDF's `rpy` convention: extrinsic XYZ Euler angles, composed as
    /// `Rz(yaw) * Ry(pitch) * Rx(roll)` per the URDF/ROS specification.
    pub fn from_rpy(roll: f64, pitch: f64, yaw: f64) -> Mat4 {
        let rz = Mat4::rotation_axis_angle(Vec3::new(0.0, 0.0, 1.0), yaw);
        let ry = Mat4::rotation_axis_angle(Vec3::new(0.0, 1.0, 0.0), pitch);
        let rx = Mat4::rotation_axis_angle(Vec3::new(1.0, 0.0, 0.0), roll);
        rz.mul(&ry).mul(&rx)
    }

    /// Standard row-major 4x4 matrix product: `self * other`.
    pub fn mul(&self, other: &Mat4) -> Mat4 {
        let mut out = [[0.0; 4]; 4];
        for (row_index, out_row) in out.iter_mut().enumerate() {
            for (column_index, out_cell) in out_row.iter_mut().enumerate() {
                *out_cell = self.m[row_index]
                    .iter()
                    .enumerate()
                    .map(|(inner_index, value)| value * other.m[inner_index][column_index])
                    .sum();
            }
        }
        Mat4 { m: out }
    }

    /// Applies this transform to a point (implicit homogeneous w = 1).
    ///
    /// Not yet called from `main.rs` - `fk`/`validate-limits` only report
    /// each joint's own origin (`translation_part`). Kept as real, tested
    /// public API: any future consumer transforming an arbitrary point
    /// (a collision-mesh vertex, a tool-tip offset) through a joint's
    /// world transform needs exactly this, not a link origin.
    #[allow(dead_code)]
    pub fn transform_point(&self, p: Vec3) -> Vec3 {
        Vec3::new(
            self.m[0][0] * p.x + self.m[0][1] * p.y + self.m[0][2] * p.z + self.m[0][3],
            self.m[1][0] * p.x + self.m[1][1] * p.y + self.m[1][2] * p.z + self.m[1][3],
            self.m[2][0] * p.x + self.m[2][1] * p.y + self.m[2][2] * p.z + self.m[2][3],
        )
    }

    /// The translation column - this transform's world-frame position.
    pub fn translation_part(&self) -> Vec3 {
        Vec3::new(self.m[0][3], self.m[1][3], self.m[2][3])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_vec3_close(a: Vec3, b: Vec3) {
        assert!((a.x - b.x).abs() < 1e-6, "{:?} != {:?}", a, b);
        assert!((a.y - b.y).abs() < 1e-6, "{:?} != {:?}", a, b);
        assert!((a.z - b.z).abs() < 1e-6, "{:?} != {:?}", a, b);
    }

    #[test]
    fn identity_leaves_points_unchanged() {
        let p = Vec3::new(1.0, 2.0, 3.0);
        assert_vec3_close(Mat4::identity().transform_point(p), p);
    }

    #[test]
    fn translation_moves_point() {
        let t = Mat4::translation(Vec3::new(1.0, 0.0, 0.0));
        assert_vec3_close(t.transform_point(Vec3::ZERO), Vec3::new(1.0, 0.0, 0.0));
    }

    #[test]
    fn rotation_90deg_about_z_maps_x_to_y() {
        let r = Mat4::rotation_axis_angle(Vec3::new(0.0, 0.0, 1.0), std::f64::consts::FRAC_PI_2);
        assert_vec3_close(
            r.transform_point(Vec3::new(1.0, 0.0, 0.0)),
            Vec3::new(0.0, 1.0, 0.0),
        );
    }

    #[test]
    fn rotation_zero_axis_is_identity() {
        let r = Mat4::rotation_axis_angle(Vec3::ZERO, 1.0);
        assert_vec3_close(
            r.transform_point(Vec3::new(1.0, 2.0, 3.0)),
            Vec3::new(1.0, 2.0, 3.0),
        );
    }

    #[test]
    fn mul_composes_translation_then_rotation_in_expected_order() {
        let t = Mat4::translation(Vec3::new(1.0, 0.0, 0.0));
        let r = Mat4::rotation_axis_angle(Vec3::new(0.0, 0.0, 1.0), std::f64::consts::FRAC_PI_2);
        // t.mul(&r): first rotate, then translate (world = T * R * local).
        let combined = t.mul(&r);
        assert_vec3_close(
            combined.transform_point(Vec3::new(1.0, 0.0, 0.0)),
            Vec3::new(1.0, 1.0, 0.0),
        );
    }

    #[test]
    fn from_rpy_zero_is_identity() {
        let m = Mat4::from_rpy(0.0, 0.0, 0.0);
        assert_vec3_close(
            m.transform_point(Vec3::new(1.0, 2.0, 3.0)),
            Vec3::new(1.0, 2.0, 3.0),
        );
    }

    #[test]
    fn translation_part_extracts_position() {
        let t = Mat4::translation(Vec3::new(1.0, 2.0, 3.0));
        assert_vec3_close(t.translation_part(), Vec3::new(1.0, 2.0, 3.0));
    }
}
