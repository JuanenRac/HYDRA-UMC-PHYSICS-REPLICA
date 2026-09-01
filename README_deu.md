<p align="center">
  <img src="images/HYDRA_UMC_BANNER.svg" alt="HYDRA-UMC-PHYSICS-REPLICA banner" width="100%">
</p>

# 🏗️ HYDRA-UMC-PHYSICS-REPLICA

<p align="center"><a href="README.md">🇺🇸 English</a> | <a href="README_spa.md">🇪🇸 Español</a> | <a href="README_fra.md">🇫🇷 Français</a> | <a href="README_ita.md">🇮🇹 Italiano</a> | 🇩🇪 <b>Deutsch</b> | <a href="README_zho.md">🇨🇳 简体中文</a> | <a href="README_jpn.md">🇯🇵 日本語</a></p>

### 📐 High-Fidelity MuJoCo/PhysX-Simulation von URDF-Kinematikketten

<p align="left">
  <img src="https://img.shields.io/badge/Lizenz-GPL%203.0-blue.svg" alt="GPL 3.0">
  <img src="https://img.shields.io/badge/Solver-MuJoCo%20%2F%20PhysX-blue.svg" alt="Solver">
  <img src="https://img.shields.io/badge/Sprache-C++%20%2F%20Rust-orange.svg" alt="Tech">
  <img src="https://img.shields.io/badge/Stufe-Etabliert%20v0-brightgreen.svg" alt="Etablierte v0-Stufe">
</p>

---

## 1. 🛠️ TECHNISCHER ÜBERBLICK

**HYDRA-UMC-PHYSICS-REPLICA** ist das zentrale physikalische Simulationsmodul des Digital Twin. Es ist spezialisiert auf die Low-Level-Berechnung von Starrkörperdynamik, Gelenkbeschränkungen und Kontaktkräften für den gesamten Roboterkatalog.

Durch die Integration modernster Solver wie MuJoCo oder NVIDIA PhysX bietet es die mathematische Grundlage für realistisches Verhalten, einschließlich Schwerkraft, Nutzlastträgheit und Oberflächenreibung für Pick-and-Place-Aufgaben.

### Hauptmerkmale:
* 📐 **Kinematische Validierung (v0):** echte Vorwärtskinematik und echte Gelenkgrenzenprüfung über eine (dokumentiert-partielle) URDF-Teilmenge - siehe "Ehrlichkeitscheck" unten für das, was heute genau läuft.
* 🔒 **Echtes v0 - Grenzen-bewusste FK:** `fk-checked` verweigert die Berechnung einer Weltraum-Pose für eine Gelenkposition außerhalb ihrer deklarierten URDF-Grenze, gestützt auf einen echten, wiederverwendbaren Gelenkgrenzen-Korpus und Regressionstests für beide Grenzen sowie stark außerhalb liegende Eingaben - schließt die Lücke, in der reines `fk` still eine physikalisch unerreichbare Pose melden würde.
* 🏗️ **URDF zu Physik (v0, teilweise):** liest echte `<joint>`-Elemente (Typ/Ursprung/Achse/Grenze) aus einer URDF-Datei in eine Kette ein. *Noch nicht real:* die Erzeugung von Kollisionsnetzen - das bleibt die "physikalische" Hälfte dieses Features.
* ⚡ **Echtzeit-Performance (geplant):** parallelisierte Berechnung für Multi-Roboter-Arbeitsbereiche - setzt voraus, dass zuerst eine echte Physik-Engine-Integration existiert.
* 🌡️ **Thermische Simulation (geplant):** experimentelle Unterstützung für die Emulation der Wärmeableitung in Werkzeugköpfen (T12/Laser).

**Ehrlichkeitscheck - was heute wirklich läuft:** `fk --urdf PFAD --joints "j1=0.5,..."` berechnet echte Weltraum-Positionen pro Gelenk durch Verketten der URDF-Gelenktransformationen, unabhängig davon, ob eine Position innerhalb ihrer deklarierten Grenze liegt; `fk-checked` führt dieselbe Berechnung durch, prüft aber zuerst jede Position gegen die echte Prüfung von `validate-limits` und verweigert die Berechnung (oder Meldung) jeglicher Pose, falls etwas außerhalb des Bereichs liegt; `validate-limits --urdf PFAD --joints "..."` meldet echte Gelenke außerhalb ihres Bereichs eigenständig. Alle drei sind reine Kinematik - keine Starrkörperdynamik, keine Kontaktkräfte, noch kein MuJoCo/PhysX-Solver angebunden, und der URDF-Reader unterstützt nur eine einzelne serielle Kette (siehe die eigene Dokumentation des `urdf.rs`-Moduls für das Warum). Siehe [`CHANGELOG.md`](CHANGELOG.md) für genau das, was geliefert wurde, und die Roadmap unten für das, was noch aussteht.

---

## 2. 🔄 PHYSIK-PIPELINE

`URDF` (Parsing) und ein echter, eigenständiger Kinematik-Schritt
(`fk`/`validate-limits`, unten nicht als eigene Box dargestellt, da er
für v0 die Rolle von `SOLVE` übernimmt) sind heute real. `MESH`, der
echte `SOLVE`-Schritt (ein echter MuJoCo/PhysX-Solver), `DYN` und `TWIN`
bleiben zukünftige Arbeit.

```mermaid
flowchart LR
    URDF["Visuelles URDF - real v0 (teilweise: einzelne serielle Kette)"] --> MESH["Kollisionsnetz-Vereinfachung - geplant"]
    MESH --> SOLVE["Physik-Solver (MuJoCo) - geplant"]
    SOLVE --> DYN["Dynamischer Zustand (Pos/Vel/Acc) - geplant"]
    DYN --> TWIN["HYDRA-UMC-TWIN Viewport - geplant"]
```

---

## 3. 🧱 ARCHITEKTUR & DESIGNENTSCHEIDUNGEN

* **Warum diese Simulation keine `hardware/`/`firmware/`/`os/`-Ordner hat.** Reine Software - keine eigene Platine, daher wurden diese Ordner entfernt statt leer gelassen.
* **Warum sie Geschwister, kein Submodul, von HYDRA-UMC-TWIN ist.** Der Physik-Solver läuft mit eigener Tick-Rate, unabhängig vom Rendering - ihn als separaten Prozess zu halten bedeutet, dass ein langsamer Physikschritt nicht die eigene Framerate von HYDRA-UMC-TWIN blockiert, und beide können ausgetauscht/aktualisiert werden (z. B. MuJoCo gegen PhysX), ohne den anderen zu berühren.
* **Wie sich das ins restliche Ökosystem einfügt.** Speist die eigene Rendering-Engine von HYDRA-UMC-TWIN mit echter Starrkörper-/Kontaktsimulation - die Prüfung physikalischer Plausibilität hinter 'wenn es im Zwilling funktioniert, funktioniert es in der Fabrik'.
* **Warum v0 `<joint>`-Elemente in Dokumentreihenfolge parst, statt den echten URDF-Link-Baum zu durchlaufen.** Ein echtes URDF ist ein Baum von Links, der sich an jedem Gelenk verzweigen kann; ihn korrekt zu durchlaufen erfordert, den `parent`/`child`-Linknamen jedes Gelenks ausgehend vom Wurzel-Link zu folgen. v0 behandelt stattdessen die Dokumentreihenfolge als eine einzelne serielle Kette - ehrlich für den eigenen Katalog von `HYDRA-UMC-EDITOR-URDF`, der heute größtenteils aus einzelnen seriellen Armen besteht, aber eine echte Einschränkung für alles, was sich verzweigt (siehe `urdf.rs`).
* **Warum `roxmltree` die einzige Abhängigkeit ist, noch kein vollständiges Physik-Engine-Binding.** Echte Vorwärtskinematik und echte Grenzenprüfung brauchen nichts weiter als das Lesen von XML-Attributen und handgeschriebene Matrixalgebra (`transform.rs`) - ein MuJoCo/PhysX-FFI-Binding dafür hinzuzufügen wäre Abhängigkeitsgewicht ohne echten Nutzen, bis tatsächlich eine echte Starrkörper-/Kontaktsimulation gebaut wird.
* **Warum `fk-checked` ein neuer Subbefehl ist, statt `fk` an Ort und Stelle zu ändern.** `fk` ist das bestehende Low-Level-Dienstprogramm, reine Mathematik - manche Aufrufer (z. B. das Anpassen einer Grenze selbst) wollen tatsächlich die ungeprüfte Pose für einen außerhalb liegenden Wert. `fk-checked` fügt den fehlersicheren, grenzengeprüften Einstiegspunkt hinzu, den echte Aufrufer nutzen sollten, ohne stillschweigend zu ändern, was `fk` schon immer bedeutet hat.
* **Warum der Gelenkgrenzen-Korpus (`corpus.rs`) nur für Tests ist.** Er existiert einzig, um den Regressionstests von `limits.rs` und `kinematics.rs` einen einzigen, echten, gemeinsamen Fixture-Satz zu geben, statt duplizierter Ad-hoc-Literale - er hat keinen Grund, in der Release-Binärdatei enthalten zu sein, daher ist er hinter `#[cfg(test)]` geschützt.

---

## 📂 VERZEICHNISSTRUKTUR

Reine Software-Simulations-Engine ohne eigenes Hardware-Design; Quellordner
werden nur aufgenommen, wenn ihre Implementierung sie erfordert. Daher hat
dieses Projekt keine Ordner `hardware/`, `firmware/` oder `os/`.

```text
HYDRA-UMC-PHYSICS-REPLICA/
├── src/
│   ├── transform.rs      # Echte Vec3/Mat4 (Translation, Achsen-Winkel-Rotation, rpy)
│   ├── urdf.rs           # Echter, partieller URDF-Reader (einzelne serielle Kette)
│   ├── kinematics.rs     # Echtes forward_kinematics() + forward_kinematics_checked()
│   ├── limits.rs         # Echtes validate_limits()
│   ├── corpus.rs         # Fixture-Korpus für Gelenkgrenzen, nur für Tests
│   └── main.rs           # Einstiegspunkt + echte `fk`/`fk-checked`/`validate-limits`-Subbefehle
├── docs/                # Dokumentation und Optimierungsleitfäden
├── build/               # Build-Notizen/Artefakte (die eigentliche cargo-Ausgabe liegt in target/, per .gitignore ausgeschlossen)
├── images/              # Medien und Diagramme
├── scripts/             # Utility-Skripte
├── tools/
│   ├── build_test.py    # Nicht-versionierender Build-Check
│   └── ci_validate.py   # Manifest/CHANGELOG/Docs-Validierung, von CI genutzt
├── Cargo.toml           # Paket-Metadaten, Abhängigkeiten (roxmltree), Kilometerzähler-Version
├── bump_version.py      # Kilometerzähler-artiger Versions-Bump (von build.sh/.bat verwendet)
├── build.sh / build.bat # Erhöht die Version, `cargo test`, dann `cargo build --release`
├── build-test.sh / build-test.bat # Nicht-versionierender Build-Check
└── run.sh / run.bat     # Führt die kompilierte Release-Binärdatei aus (leitet Argumente weiter)
```

---

## 🏗️ BUILD UND RUN

Erfordert die Rust-Toolchain (`cargo`/`rustc`, Installation via [rustup](https://rustup.rs)) und Python 3.10+ (nur für `bump_version.py`).

```bash
# Linux / macOS
./build.sh   # Kilometerzähler-Versions-Bump, `cargo test` (33 Tests), dann `cargo build --release`
./run.sh     # führt target/release/hydra-umc-physics-replica aus, gibt Name + Version + Rolle aus
```

```bat
:: Windows
build.bat
run.bat
```

`build.sh`/`build.bat` erhöhen die Version der eigenen `Cargo.toml` dieses Projekts nach der "Kilometerzähler"-Regel des Ökosystems (PATCH+1, mit Übertrag auf MINOR nach 9), führen die echte Testsuite aus und bauen dann eine Release-Binärdatei.

Die echten Subbefehle `fk` und `validate-limits` benötigen eine URDF-Datei:

```bash
./run.sh fk --urdf arm.urdf --joints "shoulder=0,elbow=0"
# shoulder: x=0.000000 y=0.000000 z=0.200000
# elbow: x=0.300000 y=0.000000 z=0.200000

./run.sh validate-limits --urdf arm.urdf --joints "shoulder=3.0,elbow=0.2"
# LIMIT VIOLATION: joint 'shoulder' = 3.000000 (allowed [-1.570000, 1.570000])
```

Der echte Subbefehl `fk-checked` verweigert die Berechnung einer Pose, wenn ein Gelenk außerhalb des Bereichs liegt - anders als reines `fk`, das sie trotzdem berechnet:

```bash
./run.sh fk-checked --urdf arm.urdf --joints "shoulder=0.5,elbow=0.2"
# shoulder: x=0.000000 y=0.000000 z=0.100000
# elbow: x=0.438791 y=0.239713 z=0.100000

./run.sh fk-checked --urdf arm.urdf --joints "shoulder=0.5,elbow=5.0"
# LIMIT VIOLATION: joint 'elbow' = 5.000000 (allowed [-2.000000, 2.000000]) - refusing to compute an unreachable pose

./run.sh fk --urdf arm.urdf --joints "shoulder=0.5,elbow=5.0"
# elbow: x=0.438791 y=0.239713 z=0.100000   <- wird trotzdem berechnet; das ist die Lücke, die fk-checked schließt
```

`fk` beendet sich mit `0` bei Erfolg, `2` bei ungültigem `--urdf`/`--joints`-Wert. `fk-checked` beendet sich mit `0` (echte Pose), `1` (Grenzverletzung) oder `2` (ungültige Eingabe). `validate-limits` beendet sich mit `0` (keine Verletzungen), `1` (Verletzungen gefunden) oder `2` (ungültige Eingabe).

---

## 🚀 FAHRPLAN
* **Phase 1:** Digital-Twin-Synchronisation mit Echtzeit-Hardware-Telemetrie und Sub-10ms-Latenz.
* **Phase 2:** Physics Replica-Integration mit industriellen Simulatoren (Isaac Sim) und Unterstützung für verformbare Körper.
* **Phase 3:** Automatisierte Wiederherstellungsmuster von Node Healing für dezentrales Failover und frühzeitige Erkennung von Sensordegradation.
* **Phase 4:** Unterstützung für die Simulation deformierbarer Körper (Kabel und Vakuumschläuche) und fotorealistische Erzeugung synthetischer Daten.

---

## 🔗 Verwandte Projekte

Dieses Projekt ist Teil eines größeren Robotik-Ökosystems desselben Autors (JuanenRac / Electro Hobby 3D), das Firmware, Steuerungssoftware, KI-Knoten und Flotten-Tools umfasst. Gut zu wissen, denn eine Anfrage könnte tatsächlich eines dieser Projekte betreffen statt dieses Repository.

### Familie

**Elternteil:** **[HYDRA-UMC-TWIN](https://github.com/JuanenRac/HYDRA-UMC-TWIN)** — der Integrations-Elternteil, den diese Simulation speist.

**Geschwister:**
- **[HYDRA-UMC-HIL-BRIDGE](https://github.com/JuanenRac/HYDRA-UMC-HIL-BRIDGE)** — Geschwister-Simulationsdienst, gleicher Elternteil.
- **[HYDRA-UMC-SYNTHETIC-DATA-GEN](https://github.com/JuanenRac/HYDRA-UMC-SYNTHETIC-DATA-GEN)** — Geschwister-Simulationsdienst, gleicher Elternteil.

### Direkte Beziehung (außerhalb der Familie)

- **[HYDRA-UMC-EDITOR-URDF](https://github.com/JuanenRac/HYDRA-UMC-EDITOR-URDF)** — verwendet die hier erstellten URDF-Modelle.

### Restliches Ökosystem

**HYDRA-UMC-Plattform** — die Multi-Roboter-Mikrofabrikzelle
- **[HYDRA-UMC](https://github.com/JuanenRac/HYDRA-UMC)** — das CM5 + STM32H745-Motherboard, das bis zu 8 Roboterarme orchestriert.
- **[HYDRA-UMC-SERVER](https://github.com/JuanenRac/HYDRA-UMC-SERVER)** — das Express/WebSocket-Backend, mit dem jeder Steuerungsclient spricht.
- **[HYDRA-UMC-STUDIO](https://github.com/JuanenRac/HYDRA-UMC-STUDIO)** — webbasiertes Steuerungs-Dashboard, Multi-Roboter-3D-Visualisierung.
- **[HYDRA-UMC-ANDROID-CONTROL](https://github.com/JuanenRac/HYDRA-UMC-ANDROID-CONTROL)** — Android-Steuerungs-App über Wi-Fi/Bluetooth.
- **[HYDRA-UMC-IOS-CONTROL](https://github.com/JuanenRac/HYDRA-UMC-IOS-CONTROL)** — iOS/iPadOS-Steuerungs-App, gebaut in Flutter.
- **[HYDRA-UMC-SUITE](https://github.com/JuanenRac/HYDRA-UMC-SUITE)** — Desktop-Schwarm-Kommandozentrale (Python/PySide6).
- **[HYDRA-UMC-EDITOR-URDF](https://github.com/JuanenRac/HYDRA-UMC-EDITOR-URDF)** — Desktop-URDF-Modelleditor für den Roboterkatalog.
- **[HYDRA-UMC-DSI](https://github.com/JuanenRac/HYDRA-UMC-DSI)** — native Touch-UI für den eingebauten DSI-Touchscreen.

**URTC-Plattform** — der Werkzeugkopf-Controller, den jeder HYDRA-UMC-Roboterarm trägt
- **[URTC](https://github.com/JuanenRac/URTC)** — CAN-Bus-Werkzeugkopf-Controller, 25 Werkzeugprofile.
- **[URTC-FLASHER](https://github.com/JuanenRac/URTC-FLASHER)** — Desktop-Tool für CAN-OTA + SWD/JTAG-Flashing.
- **[URTC-TESTER](https://github.com/JuanenRac/URTC-TESTER)** — Desktop-Tool für Live-CAN-Bus-Diagnose.
- **[URTC-WEB-STUDIO](https://github.com/JuanenRac/URTC-WEB-STUDIO)** — browserbasierte Alternative über die Web-Serial-API.

**🎥 Vision-KI-Knoten (Hailo-8)**
- [HYDRA-UMC-VISION-NODE](https://github.com/JuanenRac/HYDRA-UMC-VISION-NODE)
- [HYDRA-UMC-VISION-STREAMER](https://github.com/JuanenRac/HYDRA-UMC-VISION-STREAMER)
- [HYDRA-UMC-DETECTION-HEF](https://github.com/JuanenRac/HYDRA-UMC-DETECTION-HEF)
- [HYDRA-UMC-SAFETY-ZONES](https://github.com/JuanenRac/HYDRA-UMC-SAFETY-ZONES)
- [HYDRA-UMC-VISUAL-SERVOING-API](https://github.com/JuanenRac/HYDRA-UMC-VISUAL-SERVOING-API)

**🧠 Kognitiver KI-Knoten (Hailo-10)**
- [HYDRA-UMC-COGNITIVE-NODE](https://github.com/JuanenRac/HYDRA-UMC-COGNITIVE-NODE)
- [HYDRA-UMC-VLA-ENGINE](https://github.com/JuanenRac/HYDRA-UMC-VLA-ENGINE)
- [HYDRA-UMC-VOICE-UI](https://github.com/JuanenRac/HYDRA-UMC-VOICE-UI)
- [HYDRA-UMC-SEMANTIC-PLANNER](https://github.com/JuanenRac/HYDRA-UMC-SEMANTIC-PLANNER)
- [HYDRA-UMC-DOCS-QA](https://github.com/JuanenRac/HYDRA-UMC-DOCS-QA)

**🐝 Orchestrierung & Schwarm**
- [HYDRA-UMC-ORCHESTRATOR](https://github.com/JuanenRac/HYDRA-UMC-ORCHESTRATOR)
- [HYDRA-UMC-SWARM-SYNC](https://github.com/JuanenRac/HYDRA-UMC-SWARM-SYNC)
- [HYDRA-UMC-PATH-PLANNER-3D](https://github.com/JuanenRac/HYDRA-UMC-PATH-PLANNER-3D)
- [HYDRA-UMC-JOB-DISPATCHER](https://github.com/JuanenRac/HYDRA-UMC-JOB-DISPATCHER)
- [HYDRA-UMC-NODE-HEALING](https://github.com/JuanenRac/HYDRA-UMC-NODE-HEALING)

**📊 Daten & Analytik**
- [HYDRA-UMC-DATALAKE](https://github.com/JuanenRac/HYDRA-UMC-DATALAKE)
- [HYDRA-UMC-TELEMETRY-COLLECTOR](https://github.com/JuanenRac/HYDRA-UMC-TELEMETRY-COLLECTOR)
- [HYDRA-UMC-ANOMALY-DETECTOR](https://github.com/JuanenRac/HYDRA-UMC-ANOMALY-DETECTOR)
- [HYDRA-UMC-PRODUCTION-REPORTS](https://github.com/JuanenRac/HYDRA-UMC-PRODUCTION-REPORTS)

**🏭 Industrielles Gateway**
- [HYDRA-UMC-GATEWAY-INDUSTRIAL](https://github.com/JuanenRac/HYDRA-UMC-GATEWAY-INDUSTRIAL)
- [HYDRA-UMC-OPCUA-SERVER](https://github.com/JuanenRac/HYDRA-UMC-OPCUA-SERVER)
- [HYDRA-UMC-MQTT-BROKER](https://github.com/JuanenRac/HYDRA-UMC-MQTT-BROKER)
- [HYDRA-UMC-MTCONNECT-ADAPTER](https://github.com/JuanenRac/HYDRA-UMC-MTCONNECT-ADAPTER)

**🛠️ Ergänzende Werkzeuge**
- [URTC-SMART-RACK](https://github.com/JuanenRac/URTC-SMART-RACK)
- [URTC-VISION-TOOL](https://github.com/JuanenRac/URTC-VISION-TOOL)
- [HYDRA-UMC-WATCH](https://github.com/JuanenRac/HYDRA-UMC-WATCH)
- [HYDRA-UMC-TOOL-CLI](https://github.com/JuanenRac/HYDRA-UMC-TOOL-CLI)
- [HYDRA-UMC-DASHBOARD-AI](https://github.com/JuanenRac/HYDRA-UMC-DASHBOARD-AI)


## 👤 AUTOR
**JuanenRac** (Electro Hobby 3D)
📧 electrohobby3d@gmail.com
📺 [youtube.com/@electrohobby3d](https://youtube.com/@electrohobby3d)

## 📜 LIZENZ
GPL-3.0 - Siehe LICENSE für Details.
