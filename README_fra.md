<p align="center">
  <img src="images/HYDRA_UMC_BANNER.svg" alt="HYDRA-UMC-PHYSICS-REPLICA banner" width="100%">
</p>

# 🏗️ HYDRA-UMC-PHYSICS-REPLICA

<p align="center"><a href="README.md">🇺🇸 English</a> | <a href="README_spa.md">🇪🇸 Español</a> | 🇫🇷 <b>Français</b> | <a href="README_ita.md">🇮🇹 Italiano</a> | <a href="README_deu.md">🇩🇪 Deutsch</a> | <a href="README_zho.md">🇨🇳 简体中文</a> | <a href="README_jpn.md">🇯🇵 日本語</a></p>

### 📐 Simulation MuJoCo/PhysX haute fidélité des chaînes cinématiques URDF

<p align="left">
  <img src="https://img.shields.io/badge/Licence-GPL%203.0-blue.svg" alt="GPL 3.0">
  <img src="https://img.shields.io/badge/Solver-MuJoCo%20%2F%20PhysX-blue.svg" alt="Solver">
  <img src="https://img.shields.io/badge/Langage-C++%20%2F%20Rust-orange.svg" alt="Tech">
  <img src="https://img.shields.io/badge/%C3%89tape-%C3%89tabli%20v0-brightgreen.svg" alt="Étape établi v0">
</p>

---

## 1. 🛠️ APERÇU TECHNIQUE

**HYDRA-UMC-PHYSICS-REPLICA** est le module de simulation physique central du jumeau numérique. Il se spécialise dans le calcul de bas niveau de la dynamique des corps rigides, des contraintes d'articulation et des forces de contact pour l'ensemble du catalogue de robots.

En intégrant des solveurs de pointe tels que MuJoCo ou NVIDIA PhysX, il fournit la base mathématique d'un comportement réaliste, y compris la gravité, l'inertie de la charge utile et le frottement de surface pour les tâches de Pick-and-Place.

### Caractéristiques principales :
* 📐 **Validation cinématique (v0) :** cinématique directe réelle et vérification réelle des limites d'articulation sur un sous-ensemble URDF (documenté et partiel) - voir « Vérification d'honnêteté » ci-dessous pour ce qui fonctionne exactement aujourd'hui.
* 🔒 **Réel v0 - FK avec vérification de limites :** `fk-checked` refuse de calculer une pose dans le monde pour une position d'articulation en dehors de sa limite déclarée dans l'URDF, appuyé par un vrai corpus réutilisable de limites d'articulation et des tests de régression pour les deux limites et des entrées largement hors limites - comblant l'écart où `fk` seul rapporterait silencieusement une pose physiquement inatteignable.
* 🏗️ **URDF vers Physique (v0, partiel) :** lit de vrais éléments `<joint>` (type/origine/axe/limite) d'un fichier URDF dans une chaîne. *Pas encore réel :* la génération de maillages de collision - cela reste la moitié « physique » de cette fonctionnalité.
* ⚡ **Performance en temps réel (prévu) :** résolution parallélisée pour les espaces de travail multi-robots - dépend qu'une véritable intégration de moteur physique existe d'abord.
* 🌡️ **Simulation thermique (prévu) :** prise en charge expérimentale de l'émulation de la dissipation thermique dans les têtes d'outils (T12/Laser).

**Vérification d'honnêteté - ce qui fonctionne réellement aujourd'hui :** `fk --urdf CHEMIN --joints "j1=0.5,..."` calcule de vraies positions monde par articulation en chaînant les transformations d'articulation du URDF, peu importe qu'une position soit dans sa limite déclarée ; `fk-checked` effectue le même calcul mais vérifie d'abord chaque position par rapport à la vraie vérification de `validate-limits`, refusant de calculer (ou de signaler) une quelconque pose si quelque chose est hors limites ; `validate-limits --urdf CHEMIN --joints "..."` signale de vraies articulations hors limites de son côté. Les trois sont de la cinématique pure - pas de dynamique de corps rigide, pas de forces de contact, aucun solveur MuJoCo/PhysX encore branché, et le lecteur URDF ne prend en charge qu'une seule chaîne sérielle (voir la documentation propre du module `urdf.rs` pour le pourquoi). Voir [`CHANGELOG.md`](CHANGELOG.md) pour ce qui a été livré exactement, et la Roadmap ci-dessous pour ce qui reste à venir.

---

## 2. 🔄 FLUX DE PHYSIQUE (PIPELINE)

`URDF` (analyse) et une étape réelle et autonome de cinématique
(`fk`/`validate-limits`, non montrée comme case propre ci-dessous car
elle remplace le rôle de `SOLVE` pour v0) sont réelles aujourd'hui.
`MESH`, la vraie étape `SOLVE` (un vrai solveur MuJoCo/PhysX), `DYN` et
`TWIN` restent du travail futur.

```mermaid
flowchart LR
    URDF["URDF visuel - réel v0 (partiel : chaîne sérielle unique)"] --> MESH["Simplification du maillage de collision - prévu"]
    MESH --> SOLVE["Solveur physique (MuJoCo) - prévu"]
    SOLVE --> DYN["État dynamique (Pos/Vel/Acc) - prévu"]
    DYN --> TWIN["Fenêtre HYDRA-UMC-TWIN - prévu"]
```

---

## 3. 🧱 ARCHITECTURE & DÉCISIONS DE CONCEPTION

* **Pourquoi cette simulation n'a pas de dossiers `hardware/`/`firmware/`/`os/`.** Logiciel pur - aucune carte propre, donc ces dossiers ont été supprimés plutôt que laissés vides.
* **Pourquoi c'est une sœur, pas un sous-module, de HYDRA-UMC-TWIN.** Le solveur physique tourne à sa propre fréquence de tick, indépendamment du rendu - le garder comme processus séparé signifie qu'un pas physique lent ne bloque pas le propre framerate de HYDRA-UMC-TWIN, et l'un ou l'autre peut être remplacé/mis à niveau (ex. MuJoCo contre PhysX) sans toucher à l'autre.
* **Comment cela s'intègre dans le reste de l'écosystème.** Alimente le propre moteur de rendu de HYDRA-UMC-TWIN avec une vraie simulation de corps rigides/contacts - la vérification de plausibilité physique derrière 'si ça marche dans le Jumeau, ça marche sur le terrain'.
* **Pourquoi v0 analyse les éléments `<joint>` dans l'ordre du document plutôt que de parcourir le vrai arbre de liaisons du URDF.** Un vrai URDF est un arbre de liaisons qui peut se ramifier à n'importe quelle articulation ; le parcourir correctement nécessite de suivre les noms de liaison `parent`/`child` de chaque articulation depuis la liaison racine. v0 traite plutôt l'ordre du document comme une chaîne sérielle unique - honnête pour le propre catalogue de `HYDRA-UMC-EDITOR-URDF`, qui est aujourd'hui surtout composé de bras sériels uniques, mais une vraie limitation pour tout ce qui se ramifie (voir `urdf.rs`).
* **Pourquoi `roxmltree` est la seule dépendance, pas encore un binding complet de moteur physique.** La cinématique directe et la vérification de limites réelles n'ont besoin de rien de plus que lire des attributs XML et faire de l'algèbre matricielle à la main (`transform.rs`) - ajouter un binding FFI MuJoCo/PhysX pour cela serait du poids de dépendance sans réel bénéfice avant qu'une vraie simulation de corps rigides/contacts ne soit en cours de construction.
* **Pourquoi `fk-checked` est une nouvelle sous-commande plutôt qu'une modification de `fk` sur place.** `fk` est l'utilitaire de bas niveau existant, mathématique pur - certains appelants (ex. ajuster une limite elle-même) veulent vraiment la pose non vérifiée pour une valeur hors limites. `fk-checked` ajoute le point d'entrée sécurisé, avec vérification de limites, que les appelants réels devraient utiliser, sans changer silencieusement ce que `fk` a toujours signifié.
* **Pourquoi le corpus de limites d'articulation (`corpus.rs`) est réservé aux tests.** Il existe uniquement pour donner aux tests de régression de `limits.rs` et `kinematics.rs` un seul ensemble de fixtures réel et partagé au lieu de littéraux ad hoc dupliqués - il n'a aucune raison d'être livré dans le binaire de release, donc il est protégé derrière `#[cfg(test)]`.

---

## 📂 STRUCTURE DES RÉPERTOIRES

Moteur de simulation purement logiciel, sans conception matérielle propre -
les dossiers de code ne sont inclus que lorsque leur implémentation les
requiert; ce projet ne comporte donc pas `hardware/`, `firmware/` ni `os/`.

```text
HYDRA-UMC-PHYSICS-REPLICA/
├── src/
│   ├── transform.rs      # Vec3/Mat4 réels (translation, rotation axe-angle, rpy)
│   ├── urdf.rs           # Lecteur URDF réel et partiel (chaîne sérielle unique)
│   ├── kinematics.rs     # forward_kinematics() réel + forward_kinematics_checked()
│   ├── limits.rs         # validate_limits() réel
│   ├── corpus.rs         # Corpus de fixtures de limites, réservé aux tests
│   └── main.rs           # Point d'entrée + sous-commandes réelles `fk`/`fk-checked`/`validate-limits`
├── docs/                # Documentation et guides d'optimisation
├── build/               # Notes/artefacts de build (la sortie réelle de cargo vit dans target/, ignoré par git)
├── images/              # Médias et diagrammes
├── scripts/             # Scripts utilitaires
├── tools/
│   ├── build_test.py    # Vérification de build sans versionnage
│   └── ci_validate.py   # Validation manifeste/CHANGELOG/docs utilisée par CI
├── Cargo.toml           # Métadonnées du paquet, dépendances (roxmltree), version compteur kilométrique
├── bump_version.py      # Incrément de version type compteur kilométrique (utilisé par build.sh/.bat)
├── build.sh / build.bat # Incrémente la version, `cargo test`, puis `cargo build --release`
├── build-test.sh / build-test.bat # Vérification de build sans versionnage
└── run.sh / run.bat     # Exécute le binaire release compilé (relaie les arguments)
```

---

## 🏗️ BUILD ET RUN

Nécessite la chaîne d'outils Rust (`cargo`/`rustc`, à installer via [rustup](https://rustup.rs)) et Python 3.10+ (uniquement pour `bump_version.py`).

```bash
# Linux / macOS
./build.sh   # incrément de version compteur kilométrique, `cargo test` (33 tests), puis `cargo build --release`
./run.sh     # exécute target/release/hydra-umc-physics-replica, affiche nom + version + rôle
```

```bat
:: Windows
build.bat
run.bat
```

`build.sh`/`build.bat` incrémentent la version du propre `Cargo.toml` de ce projet selon la règle "compteur kilométrique" de l'écosystème (PATCH+1, avec retenue vers MINOR au-delà de 9), exécutent la vraie suite de tests, puis construisent un binaire release.

Les vraies sous-commandes `fk` et `validate-limits` ont besoin d'un fichier URDF :

```bash
./run.sh fk --urdf arm.urdf --joints "shoulder=0,elbow=0"
# shoulder: x=0.000000 y=0.000000 z=0.200000
# elbow: x=0.300000 y=0.000000 z=0.200000

./run.sh validate-limits --urdf arm.urdf --joints "shoulder=3.0,elbow=0.2"
# LIMIT VIOLATION: joint 'shoulder' = 3.000000 (allowed [-1.570000, 1.570000])
```

La vraie sous-commande `fk-checked` refuse de calculer une pose lorsqu'une articulation est hors limites - contrairement à `fk` seul, qui la calcule quand même :

```bash
./run.sh fk-checked --urdf arm.urdf --joints "shoulder=0.5,elbow=0.2"
# shoulder: x=0.000000 y=0.000000 z=0.100000
# elbow: x=0.438791 y=0.239713 z=0.100000

./run.sh fk-checked --urdf arm.urdf --joints "shoulder=0.5,elbow=5.0"
# LIMIT VIOLATION: joint 'elbow' = 5.000000 (allowed [-2.000000, 2.000000]) - refusing to compute an unreachable pose

./run.sh fk --urdf arm.urdf --joints "shoulder=0.5,elbow=5.0"
# elbow: x=0.438791 y=0.239713 z=0.100000   <- calculé quand même ; c'est l'écart que fk-checked comble
```

`fk` se termine avec `0` en cas de succès, `2` si `--urdf`/`--joints` est invalide. `fk-checked` se termine avec `0` (vraie pose), `1` (violation de limite), ou `2` (entrée invalide). `validate-limits` se termine avec `0` (aucune violation), `1` (violations trouvées), ou `2` (entrée invalide).

---

## 🚀 ROADMAP
* **Phase 1 :** Synchronisation du jumeau numérique avec la télémétrie matérielle en temps réel et latence inférieure à 10 ms.
* **Phase 2 :** Intégration de Physics Replica avec des simulateurs de classe industrielle (Isaac Sim) et prise en charge des corps déformables.
* **Phase 3 :** Modèles de récupération automatisés de Node Healing pour un basculement décentralisé et détection précoce de la dégradation des capteurs.
* **Phase 4 :** Prise en charge de la simulation de corps déformables (câbles et tubes à vide) et génération de données synthétiques photoréalistes.

---

## 🔗 Projets Liés

Ce projet fait partie d'un écosystème robotique plus large du même auteur (JuanenRac / Electro Hobby 3D), couvrant firmware, logiciel de contrôle, nœuds IA et outillage de flotte. Bon à savoir, car une demande pourrait en réalité concerner l'un de ces projets plutôt que ce dépôt.

### Famille

**Parent :** **[HYDRA-UMC-TWIN](https://github.com/JuanenRac/HYDRA-UMC-TWIN)** — le parent d'intégration qu'alimente cette simulation.

**Frères et sœurs :**
- **[HYDRA-UMC-HIL-BRIDGE](https://github.com/JuanenRac/HYDRA-UMC-HIL-BRIDGE)** — service de simulation frère, même parent.
- **[HYDRA-UMC-SYNTHETIC-DATA-GEN](https://github.com/JuanenRac/HYDRA-UMC-SYNTHETIC-DATA-GEN)** — service de simulation frère, même parent.

### Relation Directe (hors de la famille)

- **[HYDRA-UMC-EDITOR-URDF](https://github.com/JuanenRac/HYDRA-UMC-EDITOR-URDF)** — consomme les modèles URDF créés ici.

### Reste de l'Écosystème

**Plateforme HYDRA-UMC** — la cellule de micro-usine multi-robot
- **[HYDRA-UMC](https://github.com/JuanenRac/HYDRA-UMC)** — la carte mère CM5 + STM32H745 orchestrant jusqu'à 8 bras robotiques.
- **[HYDRA-UMC-SERVER](https://github.com/JuanenRac/HYDRA-UMC-SERVER)** — le backend Express/WebSocket auquel parle chaque client de contrôle.
- **[HYDRA-UMC-STUDIO](https://github.com/JuanenRac/HYDRA-UMC-STUDIO)** — tableau de bord de contrôle web, visualisation 3D multi-robot.
- **[HYDRA-UMC-ANDROID-CONTROL](https://github.com/JuanenRac/HYDRA-UMC-ANDROID-CONTROL)** — application de contrôle Android via Wi-Fi/Bluetooth.
- **[HYDRA-UMC-IOS-CONTROL](https://github.com/JuanenRac/HYDRA-UMC-IOS-CONTROL)** — application de contrôle iOS/iPadOS construite en Flutter.
- **[HYDRA-UMC-SUITE](https://github.com/JuanenRac/HYDRA-UMC-SUITE)** — centre de commande d'essaim de bureau (Python/PySide6).
- **[HYDRA-UMC-EDITOR-URDF](https://github.com/JuanenRac/HYDRA-UMC-EDITOR-URDF)** — éditeur de modèles URDF de bureau pour le catalogue de robots.
- **[HYDRA-UMC-DSI](https://github.com/JuanenRac/HYDRA-UMC-DSI)** — interface tactile native pour l'écran DSI embarqué.

**Plateforme URTC** — le contrôleur de tête d'outil que porte chaque bras HYDRA-UMC
- **[URTC](https://github.com/JuanenRac/URTC)** — contrôleur de tête d'outil sur bus CAN, 25 profils d'outil.
- **[URTC-FLASHER](https://github.com/JuanenRac/URTC-FLASHER)** — outil de bureau de flashage CAN-OTA + SWD/JTAG.
- **[URTC-TESTER](https://github.com/JuanenRac/URTC-TESTER)** — outil de bureau de diagnostic CAN en direct.
- **[URTC-WEB-STUDIO](https://github.com/JuanenRac/URTC-WEB-STUDIO)** — alternative basée navigateur via l'API Web Serial.

**🎥 Vision AI Node (Hailo-8)**
- [HYDRA-UMC-VISION-NODE](https://github.com/JuanenRac/HYDRA-UMC-VISION-NODE)
- [HYDRA-UMC-VISION-STREAMER](https://github.com/JuanenRac/HYDRA-UMC-VISION-STREAMER)
- [HYDRA-UMC-DETECTION-HEF](https://github.com/JuanenRac/HYDRA-UMC-DETECTION-HEF)
- [HYDRA-UMC-SAFETY-ZONES](https://github.com/JuanenRac/HYDRA-UMC-SAFETY-ZONES)
- [HYDRA-UMC-VISUAL-SERVOING-API](https://github.com/JuanenRac/HYDRA-UMC-VISUAL-SERVOING-API)

**🧠 Cognitive AI Node (Hailo-10)**
- [HYDRA-UMC-COGNITIVE-NODE](https://github.com/JuanenRac/HYDRA-UMC-COGNITIVE-NODE)
- [HYDRA-UMC-VLA-ENGINE](https://github.com/JuanenRac/HYDRA-UMC-VLA-ENGINE)
- [HYDRA-UMC-VOICE-UI](https://github.com/JuanenRac/HYDRA-UMC-VOICE-UI)
- [HYDRA-UMC-SEMANTIC-PLANNER](https://github.com/JuanenRac/HYDRA-UMC-SEMANTIC-PLANNER)
- [HYDRA-UMC-DOCS-QA](https://github.com/JuanenRac/HYDRA-UMC-DOCS-QA)

**🐝 Orchestration & Swarm**
- [HYDRA-UMC-ORCHESTRATOR](https://github.com/JuanenRac/HYDRA-UMC-ORCHESTRATOR)
- [HYDRA-UMC-SWARM-SYNC](https://github.com/JuanenRac/HYDRA-UMC-SWARM-SYNC)
- [HYDRA-UMC-PATH-PLANNER-3D](https://github.com/JuanenRac/HYDRA-UMC-PATH-PLANNER-3D)
- [HYDRA-UMC-JOB-DISPATCHER](https://github.com/JuanenRac/HYDRA-UMC-JOB-DISPATCHER)
- [HYDRA-UMC-NODE-HEALING](https://github.com/JuanenRac/HYDRA-UMC-NODE-HEALING)

**📊 Data & Analytics**
- [HYDRA-UMC-DATALAKE](https://github.com/JuanenRac/HYDRA-UMC-DATALAKE)
- [HYDRA-UMC-TELEMETRY-COLLECTOR](https://github.com/JuanenRac/HYDRA-UMC-TELEMETRY-COLLECTOR)
- [HYDRA-UMC-ANOMALY-DETECTOR](https://github.com/JuanenRac/HYDRA-UMC-ANOMALY-DETECTOR)
- [HYDRA-UMC-PRODUCTION-REPORTS](https://github.com/JuanenRac/HYDRA-UMC-PRODUCTION-REPORTS)

**🏭 Industrial Gateway**
- [HYDRA-UMC-GATEWAY-INDUSTRIAL](https://github.com/JuanenRac/HYDRA-UMC-GATEWAY-INDUSTRIAL)
- [HYDRA-UMC-OPCUA-SERVER](https://github.com/JuanenRac/HYDRA-UMC-OPCUA-SERVER)
- [HYDRA-UMC-MQTT-BROKER](https://github.com/JuanenRac/HYDRA-UMC-MQTT-BROKER)
- [HYDRA-UMC-MTCONNECT-ADAPTER](https://github.com/JuanenRac/HYDRA-UMC-MTCONNECT-ADAPTER)

**🛠️ Complementary Tools**
- [URTC-SMART-RACK](https://github.com/JuanenRac/URTC-SMART-RACK)
- [URTC-VISION-TOOL](https://github.com/JuanenRac/URTC-VISION-TOOL)
- [HYDRA-UMC-WATCH](https://github.com/JuanenRac/HYDRA-UMC-WATCH)
- [HYDRA-UMC-TOOL-CLI](https://github.com/JuanenRac/HYDRA-UMC-TOOL-CLI)
- [HYDRA-UMC-DASHBOARD-AI](https://github.com/JuanenRac/HYDRA-UMC-DASHBOARD-AI)


## 👤 AUTEUR
**JuanenRac** (Electro Hobby 3D)
📧 electrohobby3d@gmail.com

## 📜 LICENCE
GPL-3.0 - Voir le fichier LICENSE pour plus de détails.
