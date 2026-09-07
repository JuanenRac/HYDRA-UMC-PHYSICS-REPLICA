# Contributing to HYDRA-UMC-PHYSICS-REPLICA 🦾

We welcome contributions to the core physical simulation engine of the HYDRA-UMC platform.

## Technology Stack
- **Languages**: C++20, Rust 1.80+.
- **Solvers**: MuJoCo (Real-time), NVIDIA PhysX (Contact stability).
- **Math**: GLM, Eigen, SIMD-optimized kinematics.
- **Formats**: URDF, MJCF (MuJoCo XML).

## Guidelines
1. **Mathematical Determinism**: Ensure that all dynamic calculations are deterministic across different CPU architectures.
2. **Solver Optimization**: Use MuJoCo's native primitives whenever possible to minimize collision mesh complexity.
3. **Thermal Modeling**: When contributing to the thermal simulation, use verified heat dissipation constants for the HYDRA-UMC toolheads.
4. **Testing**: Validate joint limit constraints against the physical hardware specifications defined in the `HYDRA-UMC` core firmware.
