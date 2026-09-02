<p align="center">
  <img src="images/HYDRA_UMC_BANNER.svg" alt="HYDRA-UMC-PHYSICS-REPLICA banner" width="100%">
</p>

# 🏗️ HYDRA-UMC-PHYSICS-REPLICA

<p align="center"><a href="README.md">🇺🇸 English</a> | <a href="README_spa.md">🇪🇸 Español</a> | <a href="README_fra.md">🇫🇷 Français</a> | <a href="README_ita.md">🇮🇹 Italiano</a> | <a href="README_deu.md">🇩🇪 Deutsch</a> | <a href="README_zho.md">🇨🇳 简体中文</a> | 🇯🇵 <b>日本語</b></p>

### 📐 URDF 運動学チェーンの高忠実度 MuJoCo/PhysX シミュレーション

<p align="left">
  <img src="https://img.shields.io/badge/Licencia-GPL%203.0-blue.svg" alt="GPL 3.0">
  <img src="https://img.shields.io/badge/Solver-MuJoCo%20%2F%20PhysX-blue.svg" alt="Solver">
  <img src="https://img.shields.io/badge/Language-C++%20%2F%20Rust-orange.svg" alt="Tech">
  <img src="https://img.shields.io/badge/Stage-Established%20v0-brightgreen.svg" alt="Established v0 stage">
</p>

---

## 1. 🛠️ 技術概要

**HYDRA-UMC-PHYSICS-REPLICA** は、デジタルツインの中核となる物理
シミュレーションモジュールです。ロボットカタログ全体にわたる剛体力学、
関節拘束、接触力の低レベル計算に特化しています。

MuJoCo や NVIDIA PhysX のような最先端のソルバーを統合することで、重力、
ペイロード慣性、ピック＆プレースタスクにおける表面摩擦を含む、リアル
な挙動のための数学的基盤を提供します。

### 主な機能：
* 📐 **運動学的検証（v0）：** （文書化された部分的な）URDF サブセット上での実際の順運動学と実際の関節可動域チェック——今日実際に何が動くのかは下記「正直な現状確認」を参照してください。
* 🔒 **実装済み v0 —— 可動域を考慮した FK：** `fk-checked` は、URDF で宣言された可動域を超える関節位置に対してワールド座標のポーズを計算することを拒否します。実際の再利用可能な関節可動域コーパスと、両方の境界および大幅に範囲外の入力に対する回帰テストに裏付けられています——プレーンな `fk` が物理的に到達不能なポーズを黙って報告してしまうギャップを塞ぎます。
* 🏗️ **URDF から物理モデルへ（v0、部分的）：** URDF ファイルから実際の `<joint>` 要素（タイプ/原点/軸/可動域）を読み取ってチェーンを構築します。*まだ実際には存在しないもの：* 衝突メッシュの生成——それはこの機能の「物理」側半分として、今も未実装のままです。
* ⚡ **リアルタイム性能（計画中）：** マルチロボットワークスペース向けの並列化された求解——まず実際の物理エンジン統合が存在することが前提です。
* 🌡️ **熱シミュレーション（計画中）：** 工具ヘッド（T12/レーザー）における放熱をエミュレートする実験的サポート。

**正直な現状確認 —— 今日実際に動くもの：** `fk --urdf パス --joints "j1=0.5,..."` は、URDF の関節変換を連鎖させることで、その位置が宣言された可動域内にあるかどうかに関わらず、各関節の実際のワールド座標位置を計算します。`fk-checked` は同じ計算を行いますが、まず各位置を `validate-limits` の実際のチェックに通し、何かが範囲外であればポーズの計算（および報告）自体を拒否します。`validate-limits --urdf パス --joints "..."` は、実際に可動域を超えた関節を単独で報告します。3 つとも純粋な運動学です——剛体力学も、接触力も、MuJoCo/PhysX ソルバーの接続もまだなく、URDF リーダーは単一の直列チェーンのみをサポートします（理由は `urdf.rs` 自身のモジュールドキュメントを参照）。実際に出荷済みの内容は [`CHANGELOG.md`](CHANGELOG.md) を、まだ残っている作業は下記のロードマップを参照してください。

---

## 2. 🔄 物理パイプライン

`URDF`（パース）と、実際の独立した運動学ステップ（`fk`/
`validate-limits`。v0 では `SOLVE` の役割を代替するため、下図では
独立したボックスとして示していません）は今日すでに実際に動作します。
`MESH`、実際の `SOLVE` ステップ（本物の MuJoCo/PhysX ソルバー）、`DYN`、
`TWIN` はいずれも今後の課題のままです。

```mermaid
flowchart LR
    URDF["Visual URDF - 実際の v0（部分的：単一の直列チェーン）"] --> MESH["Collision Mesh Simplification - 計画中"]
    MESH --> SOLVE["Physics Solver (MuJoCo) - 計画中"]
    SOLVE --> DYN["Dynamic State (Pos/Vel/Acc) - 計画中"]
    DYN --> TWIN["HYDRA-UMC-TWIN Viewport - 計画中"]
```

---

## 3. 🧱 アーキテクチャと設計上の決定

* **本シミュレーションに `hardware/`/`firmware/`/`os/` フォルダがない理由。** 純粋なソフトウェアであり、独自の基板を持たないため、これらのフォルダは空のまま残すのではなく意図的に省略されています。
* **HYDRA-UMC-TWIN のサブモジュールではなく兄弟プロジェクトである理由。** 物理ソルバーはレンダリングとは独立した独自のティックレートで動作します——独立したプロセスとして保つことで、遅い物理ステップが HYDRA-UMC-TWIN 自身のフレームレートを停滞させることはなく、どちらか一方を（例えば MuJoCo と PhysX の間で）入れ替え/アップグレードしても、もう一方に影響を与えません。
* **エコシステムの他の部分との関係。** HYDRA-UMC-TWIN 自身のレンダラーに実際の剛体/接触シミュレーションを供給します——これは「ツイン上で動作すれば、現場でも動作する」という理念を支える物理的妥当性の検証です。
* **v0 が実際の URDF リンクツリーを辿る代わりに、文書順で `<joint>` 要素をパースする理由。** 実際の URDF は、どの関節でも分岐しうるリンクのツリーです。これを正しく辿るには、各関節の `parent`/`child` リンク名をルートリンクから実際に辿る必要があります。v0 はその代わりに文書順を単一の直列チェーンとして扱います——今日ほとんどが単一の直列アームである `HYDRA-UMC-EDITOR-URDF` 自身のカタログに対しては正直な扱いですが、分岐するものに対しては実際の制約となります（`urdf.rs` を参照）。
* **`roxmltree` が唯一の依存関係であり、まだ完全な物理エンジンバインディングではない理由。** 実際の順運動学と実際の可動域チェックに必要なのは、XML 属性の読み取りと手書きの行列演算（`transform.rs`）だけです——実際の剛体/接触シミュレーションの構築が始まるまでは、そのために MuJoCo/PhysX の FFI バインディングを追加しても、実際の見返りのない依存関係の重みが増えるだけです。
* **`fk-checked` が `fk` をその場で変更するのではなく新しいサブコマンドである理由。** `fk` は既存の低レベルな純粋数学ユーティリティです——一部の呼び出し元（例えば可動域自体を調整する場合）は、範囲外の値に対する未チェックのポーズを実際に必要とします。`fk-checked` は、`fk` がこれまで意味してきたものを黙って変更することなく、実際の呼び出し元が使うべきフェイルセーフで可動域チェック付きのエントリポイントを追加します。
* **関節可動域コーパス（`corpus.rs`）がテスト専用である理由。** これは、`limits.rs` と `kinematics.rs` の回帰テストに、重複したその場しのぎのリテラルではなく、単一の実際の共有フィクスチャセットを与えるためだけに存在します——リリースバイナリに含める理由がないため、`#[cfg(test)]` の背後で保護されています。

---

## 📂 リポジトリ構成

純粋なソフトウェアシミュレーションエンジンで独自のハードウェア設計を
持たず、ソースフォルダは実装に必要な場合だけ含めます。そのため本プロジェクトは
`hardware/`、`firmware/`、`os/` フォルダを携えていません。

```text
HYDRA-UMC-PHYSICS-REPLICA/
├── src/
│   ├── transform.rs      # 実際の Vec3/Mat4（平行移動、軸角回転、rpy）
│   ├── urdf.rs           # 実際の、部分的な URDF リーダー（単一の直列チェーン）
│   ├── kinematics.rs     # 実際の forward_kinematics() + forward_kinematics_checked()
│   ├── limits.rs         # 実際の validate_limits()
│   ├── corpus.rs         # テスト専用の可動域フィクスチャコーパス
│   └── main.rs           # エントリポイント + 実際の `fk`/`fk-checked`/`validate-limits` サブコマンド
├── docs/                # ドキュメントと最適化ガイド
├── build/               # ビルドノート/成果物（cargo 自身の出力は target/ にあり、gitignore 対象）
├── images/              # メディアと図表
├── tools/
│   ├── build_test.py    # バージョンを増やさないビルドチェック
│   └── ci_validate.py   # CI が使用するマニフェスト/CHANGELOG/ドキュメント検証
├── Cargo.toml           # パッケージメタデータ、依存関係（roxmltree）、オドメーターバージョン
├── bump_version.py      # ネイティブバージョンのオドメーター式インクリメント（build.sh/.bat が使用）
├── bump_manifest_version.py # hydra-umc.project.json のバージョンをネイティブ版と同期(--sync)
├── build.sh / build.bat # バージョンを増加させ、`cargo test`、その後 `cargo build --release` を実行
├── build-test.sh / build-test.bat # バージョンを増やさないビルドチェック
└── run.sh / run.bat     # コンパイル済みの release バイナリを実行（引数を転送）
```

---

## 🏗️ ビルドと実行

Rust ツールチェーン（`cargo`/`rustc`、[rustup](https://rustup.rs) 経由で
インストール）と Python 3.10+（`bump_version.py` のみに使用）が必要です。

```bash
# Linux / macOS
./build.sh   # オドメーター式バージョンインクリメント、`cargo test`（35 件のテスト）、その後 `cargo build --release`
./run.sh     # target/release/hydra-umc-physics-replica を実行し、名前 + バージョン + 役割を表示
```

```bat
:: Windows
build.bat
run.bat
```

`build.sh`/`build.bat` は、エコシステムの「オドメーター」規則
（PATCH+1、9 を超えると MINOR に繰り上がる）に従って本プロジェクト
自身の `Cargo.toml` のバージョンを増加させ、実際のテストスイートを
実行し、その後 release バイナリをビルドします。

実際の `fk` および `validate-limits` サブコマンドには URDF ファイルが
必要です：

```bash
./run.sh fk --urdf arm.urdf --joints "shoulder=0,elbow=0"
# shoulder: x=0.000000 y=0.000000 z=0.200000
# elbow: x=0.300000 y=0.000000 z=0.200000

./run.sh validate-limits --urdf arm.urdf --joints "shoulder=3.0,elbow=0.2"
# LIMIT VIOLATION: joint 'shoulder' = 3.000000 (allowed [-1.570000, 1.570000])
```

実際の `fk-checked` サブコマンドは、関節が範囲外の場合はポーズの計算を
拒否します——それでも計算してしまうプレーンな `fk` とは異なります：

```bash
./run.sh fk-checked --urdf arm.urdf --joints "shoulder=0.5,elbow=0.2"
# shoulder: x=0.000000 y=0.000000 z=0.100000
# elbow: x=0.438791 y=0.239713 z=0.100000

./run.sh fk-checked --urdf arm.urdf --joints "shoulder=0.5,elbow=5.0"
# LIMIT VIOLATION: joint 'elbow' = 5.000000 (allowed [-2.000000, 2.000000]) - refusing to compute an unreachable pose

./run.sh fk --urdf arm.urdf --joints "shoulder=0.5,elbow=5.0"
# elbow: x=0.438791 y=0.239713 z=0.100000   <- それでも計算されてしまう；これが fk-checked が塞ぐギャップです
```

`fk` は成功時に終了コード `0`、`--urdf`/`--joints` の値が不正な場合は
`2` で終了します。`fk-checked` は実際のポーズを返す場合は `0`、可動域
違反の場合は `1`、不正な入力の場合は `2` で終了します。`validate-limits`
は違反なしで `0`、違反ありで `1`、不正な入力で `2` を返します。

---

## 🚀 ロードマップ
* **フェーズ 1：** リアルタイムハードウェアテレメトリとのデジタルツイン同期、サブ 10ms の遅延。
* **フェーズ 2：** 産業グレードのシミュレーター（Isaac Sim）との Physics Replica 統合、変形体サポート。
* **フェーズ 3：** 分散型フェイルオーバーと早期センサー劣化検知のためのノード自己修復自動化パターン。
* **フェーズ 4：** 変形体シミュレーション（ケーブルと真空チューブ）のサポートとフォトリアリスティックな合成データ生成。

---

## 🔗 関連プロジェクト

本プロジェクトは、同一著者（JuanenRac / Electro Hobby 3D）による、
ファームウェア、制御ソフトウェア、AI ノード、フリート管理ツールにまたがる、
より大きなロボティクスエコシステムの一部です。ご要望が実際にはこれらの
プロジェクトのいずれかに関するものであり、本リポジトリのものではない
可能性もあるため、知っておく価値があります。

### プロジェクトファミリー

**親プロジェクト：** **[HYDRA-UMC-TWIN](https://github.com/JuanenRac/HYDRA-UMC-TWIN)** —— 本シミュレーションモジュールが供給する統合親プロジェクト。

**兄弟プロジェクト：**
- **[HYDRA-UMC-HIL-BRIDGE](https://github.com/JuanenRac/HYDRA-UMC-HIL-BRIDGE)** —— 同じ親プロジェクトを持つ兄弟シミュレーションサービス。
- **[HYDRA-UMC-SYNTHETIC-DATA-GEN](https://github.com/JuanenRac/HYDRA-UMC-SYNTHETIC-DATA-GEN)** —— 同じ親プロジェクトを持つ兄弟シミュレーションサービス。

### 直接関連（ファミリー外）

- **[HYDRA-UMC-EDITOR-URDF](https://github.com/JuanenRac/HYDRA-UMC-EDITOR-URDF)** —— ここで作成された URDF モデルを消費します。

### エコシステムのその他のプロジェクト

**HYDRA-UMC プラットフォーム** — マルチロボット・マイクロファクトリーセル
- **[HYDRA-UMC](https://github.com/JuanenRac/HYDRA-UMC)** — 最大 8 台のロボットアームを統括する CM5 + STM32H745 マザーボード。
- **[HYDRA-UMC-SERVER](https://github.com/JuanenRac/HYDRA-UMC-SERVER)** — すべての制御クライアントが接続する Express/WebSocket バックエンド。
- **[HYDRA-UMC-STUDIO](https://github.com/JuanenRac/HYDRA-UMC-STUDIO)** — Web ベースの制御ダッシュボード、マルチロボット 3D 可視化。
- **[HYDRA-UMC-ANDROID-CONTROL](https://github.com/JuanenRac/HYDRA-UMC-ANDROID-CONTROL)** — Wi-Fi/Bluetooth 経由の Android 制御アプリ。
- **[HYDRA-UMC-IOS-CONTROL](https://github.com/JuanenRac/HYDRA-UMC-IOS-CONTROL)** — Flutter で構築された iOS/iPadOS 制御アプリ。
- **[HYDRA-UMC-SUITE](https://github.com/JuanenRac/HYDRA-UMC-SUITE)** — デスクトップ版群制御コマンドセンター（Python/PySide6）。
- **[HYDRA-UMC-EDITOR-URDF](https://github.com/JuanenRac/HYDRA-UMC-EDITOR-URDF)** — ロボットカタログ向けのデスクトップ版 URDF モデルエディター。
- **[HYDRA-UMC-DSI](https://github.com/JuanenRac/HYDRA-UMC-DSI)** — 機載 DSI タッチスクリーン用のネイティブタッチ UI。

**URTC プラットフォーム** — すべての HYDRA-UMC ロボットアームが搭載するツールヘッドコントローラー
- **[URTC](https://github.com/JuanenRac/URTC)** — CAN バスツールヘッドコントローラー、25 種類のツールプロファイル。
- **[URTC-FLASHER](https://github.com/JuanenRac/URTC-FLASHER)** — デスクトップ版 CAN-OTA + SWD/JTAG フラッシュツール。
- **[URTC-TESTER](https://github.com/JuanenRac/URTC-TESTER)** — デスクトップ版ライブ CAN バス診断ツール。
- **[URTC-WEB-STUDIO](https://github.com/JuanenRac/URTC-WEB-STUDIO)** — Web Serial API によるブラウザベースの代替版。

**🎥 ビジョン AI ノード（Hailo-8）**
- [HYDRA-UMC-VISION-NODE](https://github.com/JuanenRac/HYDRA-UMC-VISION-NODE)
- [HYDRA-UMC-VISION-STREAMER](https://github.com/JuanenRac/HYDRA-UMC-VISION-STREAMER)
- [HYDRA-UMC-DETECTION-HEF](https://github.com/JuanenRac/HYDRA-UMC-DETECTION-HEF)
- [HYDRA-UMC-SAFETY-ZONES](https://github.com/JuanenRac/HYDRA-UMC-SAFETY-ZONES)
- [HYDRA-UMC-VISUAL-SERVOING-API](https://github.com/JuanenRac/HYDRA-UMC-VISUAL-SERVOING-API)

**🧠 認知 AI ノード（Hailo-10）**
- [HYDRA-UMC-COGNITIVE-NODE](https://github.com/JuanenRac/HYDRA-UMC-COGNITIVE-NODE)
- [HYDRA-UMC-VLA-ENGINE](https://github.com/JuanenRac/HYDRA-UMC-VLA-ENGINE)
- [HYDRA-UMC-VOICE-UI](https://github.com/JuanenRac/HYDRA-UMC-VOICE-UI)
- [HYDRA-UMC-SEMANTIC-PLANNER](https://github.com/JuanenRac/HYDRA-UMC-SEMANTIC-PLANNER)
- [HYDRA-UMC-DOCS-QA](https://github.com/JuanenRac/HYDRA-UMC-DOCS-QA)

**🐝 オーケストレーションと群制御**
- [HYDRA-UMC-ORCHESTRATOR](https://github.com/JuanenRac/HYDRA-UMC-ORCHESTRATOR)
- [HYDRA-UMC-SWARM-SYNC](https://github.com/JuanenRac/HYDRA-UMC-SWARM-SYNC)
- [HYDRA-UMC-PATH-PLANNER-3D](https://github.com/JuanenRac/HYDRA-UMC-PATH-PLANNER-3D)
- [HYDRA-UMC-JOB-DISPATCHER](https://github.com/JuanenRac/HYDRA-UMC-JOB-DISPATCHER)
- [HYDRA-UMC-NODE-HEALING](https://github.com/JuanenRac/HYDRA-UMC-NODE-HEALING)

**📊 データと分析**
- [HYDRA-UMC-DATALAKE](https://github.com/JuanenRac/HYDRA-UMC-DATALAKE)
- [HYDRA-UMC-TELEMETRY-COLLECTOR](https://github.com/JuanenRac/HYDRA-UMC-TELEMETRY-COLLECTOR)
- [HYDRA-UMC-ANOMALY-DETECTOR](https://github.com/JuanenRac/HYDRA-UMC-ANOMALY-DETECTOR)
- [HYDRA-UMC-PRODUCTION-REPORTS](https://github.com/JuanenRac/HYDRA-UMC-PRODUCTION-REPORTS)

**🏭 産業用ゲートウェイ**
- [HYDRA-UMC-GATEWAY-INDUSTRIAL](https://github.com/JuanenRac/HYDRA-UMC-GATEWAY-INDUSTRIAL)
- [HYDRA-UMC-OPCUA-SERVER](https://github.com/JuanenRac/HYDRA-UMC-OPCUA-SERVER)
- [HYDRA-UMC-MQTT-BROKER](https://github.com/JuanenRac/HYDRA-UMC-MQTT-BROKER)
- [HYDRA-UMC-MTCONNECT-ADAPTER](https://github.com/JuanenRac/HYDRA-UMC-MTCONNECT-ADAPTER)

**🛠️ 補完ツール**
- [URTC-SMART-RACK](https://github.com/JuanenRac/URTC-SMART-RACK)
- [URTC-VISION-TOOL](https://github.com/JuanenRac/URTC-VISION-TOOL)
- [HYDRA-UMC-WATCH](https://github.com/JuanenRac/HYDRA-UMC-WATCH)
- [HYDRA-UMC-TOOL-CLI](https://github.com/JuanenRac/HYDRA-UMC-TOOL-CLI)
- [HYDRA-UMC-DASHBOARD-AI](https://github.com/JuanenRac/HYDRA-UMC-DASHBOARD-AI)


## 👤 作者
**JuanenRac** (Electro Hobby 3D)
📧 electrohobby3d@gmail.com
📺 [youtube.com/@electrohobby3d](https://youtube.com/@electrohobby3d)

## 📜 ライセンス
GPL-3.0 —— 詳細は LICENSE を参照してください。
