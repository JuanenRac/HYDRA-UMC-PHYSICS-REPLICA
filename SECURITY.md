# Security Policy 🔒 (HYDRA-UMC-PHYSICS-REPLICA)

## Supported Versions

| Version | Supported          |
| ------- | ------------------ |
| 0.x.x  | ✅ Yes             |

## Reporting a Vulnerability

**CRITICAL: Do not report safety-critical vulnerabilities through public GitHub issues.**

In a physics-based safety validator, a security flaw can lead to undetected collision risks. If you discover a vulnerability affecting the **solver precision**, **URDF mesh injection**, or **state buffer overflows**:

1. **Email**: Send a detailed report to `electrohobby3d@gmail.com`.
2. **Impact**: Describe if the bug allows bypassing physical constraints, spoofing contact forces, or causing remote code execution via malicious simulation files.
3. **Response**: Initial acknowledgment within 48 hours.

We follow a coordinated disclosure policy to ensure hardware safety before public release.
