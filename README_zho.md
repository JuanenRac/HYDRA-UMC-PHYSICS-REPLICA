<p align="center">
  <img src="images/HYDRA_UMC_BANNER.svg" alt="HYDRA-UMC-PHYSICS-REPLICA banner" width="100%">
</p>

# 🏗️ HYDRA-UMC-PHYSICS-REPLICA

<p align="center"><a href="README.md">🇺🇸 English</a> | <a href="README_spa.md">🇪🇸 Español</a> | <a href="README_fra.md">🇫🇷 Français</a> | <a href="README_ita.md">🇮🇹 Italiano</a> | <a href="README_deu.md">🇩🇪 Deutsch</a> | 🇨🇳 <b>简体中文</b> | <a href="README_jpn.md">🇯🇵 日本語</a></p>

### 📐 针对 URDF 运动学链的高保真 MuJoCo/PhysX 仿真

<p align="left">
  <img src="https://img.shields.io/badge/Licencia-GPL%203.0-blue.svg" alt="GPL 3.0">
  <img src="https://img.shields.io/badge/Solver-MuJoCo%20%2F%20PhysX-blue.svg" alt="Solver">
  <img src="https://img.shields.io/badge/Language-C++%20%2F%20Rust-orange.svg" alt="Tech">
  <img src="https://img.shields.io/badge/Stage-Established%20v0-brightgreen.svg" alt="Established v0 stage">
</p>

---

## 1. 🛠️ 技术概述

**HYDRA-UMC-PHYSICS-REPLICA** 是数字孪生系统的核心物理仿真模块。它专门
负责为整个机器人目录进行刚体动力学、关节约束和接触力的底层计算。

通过集成 MuJoCo 或 NVIDIA PhysX 等最先进的求解器，它为逼真的行为提供了
数学基础，包括重力、负载惯性以及抓取放置任务中的表面摩擦力。

### 关键特性：
* 📐 **运动学验证（v0）：** 在一个（已文档说明、部分支持的）URDF 子集上实现真实的正向运动学和真实的关节限位检查——下方"诚实说明"给出了今天到底能跑什么。
* 🔒 **真实 v0 —— 带限位检查的 FK：** `fk-checked` 会拒绝为超出 URDF 声明限位的关节位置计算世界坐标位姿，背后有一份真实、可复用的关节限位语料库和针对两侧边界及严重超限输入的回归测试作为支撑——填补了普通 `fk` 会默默报告一个物理上不可达位姿的漏洞。
* 🏗️ **URDF 转物理模型（v0，部分）：** 从 URDF 文件中读取真实的 `<joint>` 元素（类型/原点/轴/限位）构建成一条链。*尚未真实：* 碰撞网格生成——那仍是这项特性中「物理」的那一半。
* ⚡ **实时性能（计划中）：** 面向多机器人工作空间的并行化求解——依赖于先有一个真实的物理引擎集成。
* 🌡️ **热仿真（计划中）：** 用于模拟刀头（T12/激光）散热的实验性支持。

**诚实说明——今天实际运行的内容：** `fk --urdf 路径 --joints "j1=0.5,..."` 通过串联 URDF 关节变换，计算每个关节真实的世界坐标位置，而不管某个位置是否在其声明的限位之内；`fk-checked` 执行同样的计算，但会先针对每个位置调用 `validate-limits` 的真实限位检查，只要有任何一项超出范围就拒绝计算（或报告）任何位姿；`validate-limits --urdf 路径 --joints "..."` 则单独报告真实的超限关节。三者都是纯运动学——没有刚体动力学，没有接触力，尚未接入任何 MuJoCo/PhysX 求解器，而且 URDF 读取器只支持单一串联链（具体原因见 `urdf.rs` 自身的模块文档）。具体已交付内容请参见 [`CHANGELOG.md`](CHANGELOG.md)，尚待完成的内容请参见下方路线图。

---

## 2. 🔄 物理流水线

`URDF`（解析）以及一个真实、独立的运动学步骤（`fk`/`validate-limits`，
由于它在 v0 中承担了 `SOLVE` 的角色，下面没有把它单独画成一个方框）今天
已是真实的。`MESH`、真正的 `SOLVE` 步骤（一个真实的 MuJoCo/PhysX 求解
器）、`DYN` 以及 `TWIN` 仍是未来工作。

```mermaid
flowchart LR
    URDF["Visual URDF - 真实 v0（部分：单一串联链）"] --> MESH["Collision Mesh Simplification - 计划中"]
    MESH --> SOLVE["Physics Solver (MuJoCo) - 计划中"]
    SOLVE --> DYN["Dynamic State (Pos/Vel/Acc) - 计划中"]
    DYN --> TWIN["HYDRA-UMC-TWIN Viewport - 计划中"]
```

---

## 3. 🧱 架构与设计决策

* **为何本仿真模块没有 `hardware/`/`firmware/`/`os/` 文件夹。** 纯软件——没有自己的板卡，因此这些文件夹被直接省略而非留空。
* **为何它是 HYDRA-UMC-TWIN 的兄弟项目，而非子模块。** 物理求解器以自己的节拍运行，独立于渲染——将其保持为独立进程意味着一次缓慢的物理步进不会拖慢 HYDRA-UMC-TWIN 自身的帧率，并且两者中的任何一个都可以被替换/升级（例如 MuJoCo 与 PhysX 之间切换）而不影响另一个。
* **这如何融入生态系统的其余部分。** 为 HYDRA-UMC-TWIN 自身的渲染器提供真实的刚体/接触仿真——这正是"在孪生系统中可行，在实际车间中同样可行"这一理念背后的物理合理性检验。
* **为何 v0 按文档顺序解析 `<joint>` 元素，而不是遍历真实的 URDF 连杆树。** 真实的 URDF 是一棵可以在任意关节处分叉的连杆树；正确遍历它需要从根连杆开始，跟随每个关节的 `parent`/`child` 连杆名称。v0 转而把文档顺序当作单一串联链处理——这对 `HYDRA-UMC-EDITOR-URDF` 自身的目录来说是诚实的（其中今天大多数都是单一串联臂），但对任何会分叉的结构来说都是一个真实的局限（详见 `urdf.rs`）。
* **为何 `roxmltree` 是唯一的依赖，还不是完整的物理引擎绑定。** 真实的正向运动学和真实的限位检查所需要的，仅仅是读取 XML 属性和手写的矩阵运算（`transform.rs`）——在真正开始构建刚体/接触仿真之前，为此添加一个 MuJoCo/PhysX 的 FFI 绑定只会增加依赖负担而没有真实收益。
* **为何 `fk-checked` 是一个新子命令，而不是就地修改 `fk`。** `fk` 是已有的底层纯数学工具——有些调用方（例如调整限位本身）确实想要针对超限值获得未经检查的位姿。`fk-checked` 增加了那个真正的调用方应该使用的、带限位检查、故障安全的入口点，而不会悄悄改变 `fk` 一直以来的含义。
* **为何关节限位语料库（`corpus.rs`）仅用于测试。** 它存在的唯一目的，就是给 `limits.rs` 和 `kinematics.rs` 的回归测试提供一份共享的真实 fixture 集合，而不是每个测试模块各自重复的临时字面量——它没有理由被打进 release 二进制文件，因此被置于 `#[cfg(test)]` 之后。

---

## 📂 目录结构

纯软件仿真引擎，没有自己的硬件设计；源代码文件夹仅在实现需要时才包含，
因此本项目不携带 `hardware/`、`firmware/` 或 `os/` 文件夹。

```text
HYDRA-UMC-PHYSICS-REPLICA/
├── src/
│   ├── transform.rs      # 真实的 Vec3/Mat4（平移、轴角旋转、rpy）
│   ├── urdf.rs           # 真实的、部分支持的 URDF 读取器（单一串联链）
│   ├── kinematics.rs     # 真实的 forward_kinematics() + forward_kinematics_checked()
│   ├── limits.rs         # 真实的 validate_limits()
│   ├── corpus.rs         # 仅用于测试的限位 fixture 语料库
│   └── main.rs           # 入口点 + 真实的 `fk`/`fk-checked`/`validate-limits` 子命令
├── docs/                # 文档与优化指南
├── build/               # 构建笔记/产物（cargo 自身的输出位于 target/，已被 gitignore）
├── images/              # 媒体与图表
├── scripts/             # 实用脚本
├── tools/
│   ├── build_test.py    # 不递增版本号的构建检查
│   └── ci_validate.py   # CI 使用的清单/CHANGELOG/文档校验
├── Cargo.toml           # 包元数据、依赖项（roxmltree）、里程表版本号
├── bump_version.py      # 里程表式版本递增（由 build.sh/.bat 使用）
├── build.sh / build.bat # 递增版本号、`cargo test`，然后执行 `cargo build --release`
├── build-test.sh / build-test.bat # 不递增版本号的构建检查
└── run.sh / run.bat     # 运行编译后的 release 二进制文件（转发参数）
```

---

## 🏗️ 构建与运行

需要 Rust 工具链（`cargo`/`rustc`，通过 [rustup](https://rustup.rs) 安装）
以及 Python 3.10+（仅供 `bump_version.py` 使用）。

```bash
# Linux / macOS
./build.sh   # 里程表式版本递增、`cargo test`（33 个测试），然后执行 `cargo build --release`
./run.sh     # 运行 target/release/hydra-umc-physics-replica，打印名称 + 版本 + 角色
```

```bat
:: Windows
build.bat
run.bat
```

`build.sh`/`build.bat` 会按照生态系统的"里程表"规则（PATCH+1，超过 9
时进位到 MINOR）递增本项目自身的 `Cargo.toml` 版本号，运行真实的测试
套件，然后构建一个 release 二进制文件。

真实的 `fk` 和 `validate-limits` 子命令需要一个 URDF 文件：

```bash
./run.sh fk --urdf arm.urdf --joints "shoulder=0,elbow=0"
# shoulder: x=0.000000 y=0.000000 z=0.200000
# elbow: x=0.300000 y=0.000000 z=0.200000

./run.sh validate-limits --urdf arm.urdf --joints "shoulder=3.0,elbow=0.2"
# LIMIT VIOLATION: joint 'shoulder' = 3.000000 (allowed [-1.570000, 1.570000])
```

真实的 `fk-checked` 子命令会在关节超限时拒绝计算位姿——而普通 `fk`
则仍会照常计算：

```bash
./run.sh fk-checked --urdf arm.urdf --joints "shoulder=0.5,elbow=0.2"
# shoulder: x=0.000000 y=0.000000 z=0.100000
# elbow: x=0.438791 y=0.239713 z=0.100000

./run.sh fk-checked --urdf arm.urdf --joints "shoulder=0.5,elbow=5.0"
# LIMIT VIOLATION: joint 'elbow' = 5.000000 (allowed [-2.000000, 2.000000]) - refusing to compute an unreachable pose

./run.sh fk --urdf arm.urdf --joints "shoulder=0.5,elbow=5.0"
# elbow: x=0.438791 y=0.239713 z=0.100000   <- 仍会照常计算；这正是 fk-checked 所填补的漏洞
```

`fk` 成功时退出码为 `0`，`--urdf`/`--joints` 值无效时为 `2`。`fk-checked`
返回真实位姿时为 `0`，出现限位违规时为 `1`，输入无效时为 `2`。
`validate-limits` 无违规时退出码为 `0`，发现违规时为 `1`，输入无效时
为 `2`。

---

## 🚀 路线图
* **第一阶段：** 数字孪生与实时硬件遥测的同步，延迟低于 10ms。
* **第二阶段：** 物理复制品与工业级仿真器（Isaac Sim）的集成，以及可变形体支持。
* **第三阶段：** 用于去中心化故障转移和早期传感器退化检测的节点自愈自动化恢复模式。
* **第四阶段：** 支持可变形体仿真（线缆和真空管）以及照片级真实合成数据生成。

---

## 🔗 相关项目

本项目是同一作者（JuanenRac / Electro Hobby 3D）打造的更大规模机器人生态
系统的一部分，涵盖固件、控制软件、AI 节点和车队工具。值得了解，因为某个
需求实际上可能是关于这些项目之一，而非本仓库。

### 项目族

**父项目：** **[HYDRA-UMC-TWIN](https://github.com/JuanenRac/HYDRA-UMC-TWIN)** —— 本仿真模块所供给的集成父项目。

**同族项目：**
- **[HYDRA-UMC-HIL-BRIDGE](https://github.com/JuanenRac/HYDRA-UMC-HIL-BRIDGE)** —— 同级仿真服务，同一父项目。
- **[HYDRA-UMC-SYNTHETIC-DATA-GEN](https://github.com/JuanenRac/HYDRA-UMC-SYNTHETIC-DATA-GEN)** —— 同级仿真服务，同一父项目。

### 直接相关（项目族之外）

- **[HYDRA-UMC-EDITOR-URDF](https://github.com/JuanenRac/HYDRA-UMC-EDITOR-URDF)** —— 消费在此编写的 URDF 模型。

### 生态系统的其余部分

**HYDRA-UMC 平台** —— 多机器人微工厂单元
- **[HYDRA-UMC](https://github.com/JuanenRac/HYDRA-UMC)** —— 协调最多 8 条机械臂的 CM5 + STM32H745 主板。
- **[HYDRA-UMC-SERVER](https://github.com/JuanenRac/HYDRA-UMC-SERVER)** —— 每个控制客户端所对接的 Express/WebSocket 后端。
- **[HYDRA-UMC-STUDIO](https://github.com/JuanenRac/HYDRA-UMC-STUDIO)** —— 基于 Web 的控制仪表盘，多机器人 3D 可视化。
- **[HYDRA-UMC-ANDROID-CONTROL](https://github.com/JuanenRac/HYDRA-UMC-ANDROID-CONTROL)** —— 通过 Wi-Fi/蓝牙的 Android 控制应用。
- **[HYDRA-UMC-IOS-CONTROL](https://github.com/JuanenRac/HYDRA-UMC-IOS-CONTROL)** —— 基于 Flutter 构建的 iOS/iPadOS 控制应用。
- **[HYDRA-UMC-SUITE](https://github.com/JuanenRac/HYDRA-UMC-SUITE)** —— 桌面端集群指挥中心（Python/PySide6）。
- **[HYDRA-UMC-EDITOR-URDF](https://github.com/JuanenRac/HYDRA-UMC-EDITOR-URDF)** —— 用于机器人目录的桌面端 URDF 模型编辑器。
- **[HYDRA-UMC-DSI](https://github.com/JuanenRac/HYDRA-UMC-DSI)** —— 机载 DSI 触摸屏的原生触控 UI。

**URTC 平台** —— 每台 HYDRA-UMC 机械臂搭载的工具头控制器
- **[URTC](https://github.com/JuanenRac/URTC)** —— CAN 总线工具头控制器，25 种工具配置。
- **[URTC-FLASHER](https://github.com/JuanenRac/URTC-FLASHER)** —— 桌面端 CAN-OTA + SWD/JTAG 刷写工具。
- **[URTC-TESTER](https://github.com/JuanenRac/URTC-TESTER)** —— 桌面端实时 CAN 总线诊断工具。
- **[URTC-WEB-STUDIO](https://github.com/JuanenRac/URTC-WEB-STUDIO)** —— 通过 Web Serial API 的浏览器端替代方案。

**🎥 视觉 AI 节点（Hailo-8）**
- [HYDRA-UMC-VISION-NODE](https://github.com/JuanenRac/HYDRA-UMC-VISION-NODE)
- [HYDRA-UMC-VISION-STREAMER](https://github.com/JuanenRac/HYDRA-UMC-VISION-STREAMER)
- [HYDRA-UMC-DETECTION-HEF](https://github.com/JuanenRac/HYDRA-UMC-DETECTION-HEF)
- [HYDRA-UMC-SAFETY-ZONES](https://github.com/JuanenRac/HYDRA-UMC-SAFETY-ZONES)
- [HYDRA-UMC-VISUAL-SERVOING-API](https://github.com/JuanenRac/HYDRA-UMC-VISUAL-SERVOING-API)

**🧠 认知 AI 节点（Hailo-10）**
- [HYDRA-UMC-COGNITIVE-NODE](https://github.com/JuanenRac/HYDRA-UMC-COGNITIVE-NODE)
- [HYDRA-UMC-VLA-ENGINE](https://github.com/JuanenRac/HYDRA-UMC-VLA-ENGINE)
- [HYDRA-UMC-VOICE-UI](https://github.com/JuanenRac/HYDRA-UMC-VOICE-UI)
- [HYDRA-UMC-SEMANTIC-PLANNER](https://github.com/JuanenRac/HYDRA-UMC-SEMANTIC-PLANNER)
- [HYDRA-UMC-DOCS-QA](https://github.com/JuanenRac/HYDRA-UMC-DOCS-QA)

**🐝 编排与集群**
- [HYDRA-UMC-ORCHESTRATOR](https://github.com/JuanenRac/HYDRA-UMC-ORCHESTRATOR)
- [HYDRA-UMC-SWARM-SYNC](https://github.com/JuanenRac/HYDRA-UMC-SWARM-SYNC)
- [HYDRA-UMC-PATH-PLANNER-3D](https://github.com/JuanenRac/HYDRA-UMC-PATH-PLANNER-3D)
- [HYDRA-UMC-JOB-DISPATCHER](https://github.com/JuanenRac/HYDRA-UMC-JOB-DISPATCHER)
- [HYDRA-UMC-NODE-HEALING](https://github.com/JuanenRac/HYDRA-UMC-NODE-HEALING)

**📊 数据与分析**
- [HYDRA-UMC-DATALAKE](https://github.com/JuanenRac/HYDRA-UMC-DATALAKE)
- [HYDRA-UMC-TELEMETRY-COLLECTOR](https://github.com/JuanenRac/HYDRA-UMC-TELEMETRY-COLLECTOR)
- [HYDRA-UMC-ANOMALY-DETECTOR](https://github.com/JuanenRac/HYDRA-UMC-ANOMALY-DETECTOR)
- [HYDRA-UMC-PRODUCTION-REPORTS](https://github.com/JuanenRac/HYDRA-UMC-PRODUCTION-REPORTS)

**🏭 工业网关**
- [HYDRA-UMC-GATEWAY-INDUSTRIAL](https://github.com/JuanenRac/HYDRA-UMC-GATEWAY-INDUSTRIAL)
- [HYDRA-UMC-OPCUA-SERVER](https://github.com/JuanenRac/HYDRA-UMC-OPCUA-SERVER)
- [HYDRA-UMC-MQTT-BROKER](https://github.com/JuanenRac/HYDRA-UMC-MQTT-BROKER)
- [HYDRA-UMC-MTCONNECT-ADAPTER](https://github.com/JuanenRac/HYDRA-UMC-MTCONNECT-ADAPTER)

**🛠️ 配套工具**
- [URTC-SMART-RACK](https://github.com/JuanenRac/URTC-SMART-RACK)
- [URTC-VISION-TOOL](https://github.com/JuanenRac/URTC-VISION-TOOL)
- [HYDRA-UMC-WATCH](https://github.com/JuanenRac/HYDRA-UMC-WATCH)
- [HYDRA-UMC-TOOL-CLI](https://github.com/JuanenRac/HYDRA-UMC-TOOL-CLI)
- [HYDRA-UMC-DASHBOARD-AI](https://github.com/JuanenRac/HYDRA-UMC-DASHBOARD-AI)


## 👤 作者
**JuanenRac**（Electro Hobby 3D）
📧 electrohobby3d@gmail.com

## 📜 许可证
GPL-3.0 —— 详见 LICENSE。

## 🛠️ BUILD & RUN

请在发布构建前使用不改动版本的构建检查：

| 操作 | Windows | Linux / macOS |
|---|---|---|
| 构建检查（不修改版本或 CHANGELOG） | `build-test.bat` | `./build-test.sh` |
| 运行 / 开发（如提供） | `run*.bat` 或 `dev*.bat` | `./run*.sh` 或 `./dev*.sh` |

`build-test.bat` 和 `build-test.sh` 会编译或验证项目技术栈，但不会递增 `hydra-umc.project.json`，也不会修改 `CHANGELOG.md`。它们仅可能生成正常的编译器输出。现有的 `build*.bat`、`build*.sh`、`run*` 和 `dev*` 脚本保留各自的版本化或运行时行为；需要该行为时请使用它们。