// =============================================================================
// HYDRA-UMC-PHYSICS-REPLICA - src/service_api.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0-or-later - see LICENSE
// =============================================================================
//! The computing routes of `serve`: `POST /fk`, `POST /ik`, `POST /check-trajectory`, `POST /inverse-dynamics` and `GET|POST /capsules`.
//! Pure functions from (method, path, body) to (status, JSON), so they are tested without a socket; `server.rs` only carries the bytes.
//!
//! The robot is the URDF that `serve --urdf` loaded at the start. A request may carry its own URDF text (`"urdf"`) only when the
//! configuration says `allow_inline_urdf: true` (off by default: a path or a file is never read from a request). Every limit and tolerance
//! comes from the configuration file (`serve --config`), with the defaults of `ServiceConfig::default()`.
//!
//! Every answer carries a stable `code`, an English `summary` or `error`, and a `message` in the language asked for (see `messages.rs`).

use std::collections::HashMap;

use serde_json::{json, Map, Value};

use crate::dynamics;
use crate::jacobian::{self, IkOptions};
use crate::kinematics;
use crate::messages::{Lang, Msg};
use crate::selfcollision;
use crate::trajectory;
use crate::urdf::{self, Model};

pub const NOTE: &str =
    "kinematic estimate from the URDF geometry only; not calibrated against a physical robot";

#[derive(Debug, Clone)]
pub struct IkLimits {
    /// `POST /ik` answers only with this (`ik.enabled` of the configuration file; off by default)
    pub enabled: bool,
    pub max_iterations_cap: usize,
    /// Longest time one request may spend iterating, in milliseconds (`ik.max_millis`)
    pub max_millis: u64,
    pub defaults: IkOptions,
}

#[derive(Debug, Clone)]
pub struct ServiceConfig {
    pub language: Lang,
    pub max_body_bytes: usize,
    pub max_samples: usize,
    pub allow_inline_urdf: bool,
    pub ik: IkLimits,
    pub trajectory: trajectory::Config,
    /// `GET /metrics` answers only with this (the `metrics` section of the configuration file; off by default)
    pub metrics: bool,
}

impl Default for ServiceConfig {
    fn default() -> Self {
        ServiceConfig {
            language: Lang::En,
            max_body_bytes: 1024 * 1024,
            max_samples: 2000,
            allow_inline_urdf: false,
            ik: IkLimits {
                enabled: false,
                max_iterations_cap: 500,
                max_millis: 2000,
                defaults: IkOptions::default(),
            },
            trajectory: trajectory::Config::default(),
            metrics: false,
        }
    }
}

const CONFIG_FIELDS: [&str; 8] = [
    "language",
    "max_body_bytes",
    "max_samples",
    "allow_inline_urdf",
    "ik",
    "trajectory",
    "collision",
    "metrics",
];
const IK_FIELDS: [&str; 6] = [
    "enabled",
    "max_iterations_cap",
    "max_millis",
    "tolerance",
    "damping",
    "max_step",
];

/// Reads the configuration file of the service; every field is optional and an unknown field is an error.
pub fn parse_config(raw: &Value) -> Result<ServiceConfig, String> {
    let object = raw
        .as_object()
        .ok_or("the service configuration must be a JSON object")?;
    let mut unknown: Vec<&str> = object
        .keys()
        .map(String::as_str)
        .filter(|k| !CONFIG_FIELDS.contains(k))
        .collect();
    if !unknown.is_empty() {
        unknown.sort_unstable();
        return Err(format!(
            "the service configuration has unknown field(s): {}",
            unknown.join(", ")
        ));
    }
    let mut config = ServiceConfig::default();
    if let Some(v) = object.get("language") {
        config.language = v
            .as_str()
            .and_then(Lang::from_tag)
            .ok_or("language must be one of en, es, de, fr, it, ja, zh, pt")?;
    }
    if let Some(v) = object.get("max_body_bytes") {
        config.max_body_bytes = v
            .as_u64()
            .filter(|n| (1024..=64 * 1024 * 1024).contains(n))
            .ok_or("max_body_bytes must be a whole number from 1024 to 67108864")?
            as usize;
    }
    if let Some(v) = object.get("max_samples") {
        config.max_samples = v
            .as_u64()
            .filter(|n| (1..=selfcollision::MAX_STATES as u64).contains(n))
            .ok_or_else(|| {
                format!(
                    "max_samples must be a whole number from 1 to {}",
                    selfcollision::MAX_STATES
                )
            })? as usize;
    }
    if let Some(v) = object.get("allow_inline_urdf") {
        config.allow_inline_urdf = v
            .as_bool()
            .ok_or("allow_inline_urdf must be true or false")?;
    }
    if let Some(v) = object.get("metrics") {
        let section = v.as_object().ok_or("metrics must be an object")?;
        if let Some(k) = section.keys().find(|k| k.as_str() != "enabled") {
            return Err(format!(
                "the metrics configuration has an unknown field: {k}"
            ));
        }
        if let Some(on) = section.get("enabled") {
            config.metrics = on
                .as_bool()
                .ok_or("metrics.enabled must be true or false")?;
        }
    }
    if let Some(v) = object.get("ik") {
        let ik = v.as_object().ok_or("ik must be an object")?;
        if let Some(k) = ik.keys().find(|k| !IK_FIELDS.contains(&k.as_str())) {
            return Err(format!("the ik configuration has an unknown field: {k}"));
        }
        let positive = |key: &str| -> Result<Option<f64>, String> {
            match ik.get(key) {
                None => Ok(None),
                Some(v) => v
                    .as_f64()
                    .filter(|n| n.is_finite() && *n > 0.0)
                    .map(Some)
                    .ok_or_else(|| format!("ik.{key} must be a number above zero")),
            }
        };
        if let Some(v) = ik.get("enabled") {
            config.ik.enabled = v.as_bool().ok_or("ik.enabled must be true or false")?;
        }
        if let Some(n) = ik.get("max_millis") {
            config.ik.max_millis = n
                .as_u64()
                .filter(|n| (1..=60_000).contains(n))
                .ok_or("ik.max_millis must be a whole number from 1 to 60000")?;
        }
        if let Some(n) = ik.get("max_iterations_cap") {
            config.ik.max_iterations_cap = n
                .as_u64()
                .filter(|n| (1..=100_000).contains(n))
                .ok_or("ik.max_iterations_cap must be a whole number from 1 to 100000")?
                as usize;
        }
        if let Some(n) = positive("tolerance")? {
            config.ik.defaults.tolerance = n;
        }
        if let Some(n) = positive("damping")? {
            config.ik.defaults.damping = n;
        }
        if let Some(n) = positive("max_step")? {
            config.ik.defaults.max_step = n;
        }
        config.ik.defaults.max_iterations = config
            .ik
            .defaults
            .max_iterations
            .min(config.ik.max_iterations_cap);
    }
    if let Some(v) = object.get("trajectory") {
        config.trajectory = trajectory::parse_config(v)?;
    }
    if let Some(v) = object.get("collision") {
        config.trajectory.collision = selfcollision::parse_config(v)?;
    }
    Ok(config)
}

/// What a request is answered with.
pub struct Context {
    pub config: ServiceConfig,
    pub model: Option<Model>,
}

#[derive(Debug)]
pub struct ApiError {
    pub status: u16,
    pub msg: Msg,
    pub detail: Option<String>,
}

fn bad(detail: impl Into<String>) -> ApiError {
    ApiError {
        status: 400,
        msg: Msg::BadRequest,
        detail: Some(detail.into()),
    }
}

impl ApiError {
    pub fn body(&self, lang: Lang) -> Value {
        let mut v = json!({
            "error": self.msg.text(Lang::En),
            "code": self.msg.code(),
            "message": self.msg.text(lang),
        });
        if let Some(d) = &self.detail {
            v["detail"] = json!(d);
        }
        v
    }
}

/// True for the routes this module answers (any method).
pub fn is_compute_path(path: &str) -> bool {
    matches!(
        path,
        "/fk" | "/ik" | "/check-trajectory" | "/inverse-dynamics" | "/capsules"
    )
}

fn stamp(mut value: Value, msg: Msg, lang: Lang) -> Value {
    value["code"] = json!(msg.code());
    value["summary"] = json!(msg.text(Lang::En));
    value["message"] = json!(msg.text(lang));
    value
}

fn joint_map(value: Option<&Value>, what: &str) -> Result<HashMap<String, f64>, ApiError> {
    let Some(value) = value else {
        return Ok(HashMap::new());
    };
    let object = value
        .as_object()
        .ok_or_else(|| bad(format!("{what} must be an object of joint name to number")))?;
    let mut out = HashMap::new();
    for (name, v) in object {
        let n = v
            .as_f64()
            .filter(|n| n.is_finite())
            .ok_or_else(|| bad(format!("{what}.{name} must be a finite number")))?;
        out.insert(name.clone(), n);
    }
    Ok(out)
}

fn check_names(model: &Model, given: &HashMap<String, f64>, what: &str) -> Result<(), ApiError> {
    let mut unknown: Vec<&str> = given
        .keys()
        .filter(|n| model.joint_index(n).is_none())
        .map(String::as_str)
        .collect();
    if unknown.is_empty() {
        return Ok(());
    }
    unknown.sort_unstable();
    Err(bad(format!(
        "unknown joint(s) in {what}: {}",
        unknown.join(", ")
    )))
}

fn pose_json(name: &str, m: &crate::transform::Mat4) -> Value {
    let p = m.translation_part();
    let (w, x, y, z) = m.quaternion();
    json!({
        "joint": name,
        "position": {"x": p.x, "y": p.y, "z": p.z},
        "quaternion": {"w": w, "x": x, "y": y, "z": z},
    })
}

fn sorted_map(map: &HashMap<String, f64>) -> Value {
    let mut pairs: Vec<(&String, &f64)> = map.iter().collect();
    pairs.sort_by(|a, b| a.0.cmp(b.0));
    Value::Object(
        pairs
            .into_iter()
            .map(|(k, v)| (k.clone(), json!(v)))
            .collect::<Map<String, Value>>(),
    )
}

fn route_fk(model: &Model, body: &Value) -> Result<Value, ApiError> {
    let joints = joint_map(body.get("joints"), "joints")?;
    check_names(model, &joints, "joints")?;
    let checked = body
        .get("checked")
        .and_then(Value::as_bool)
        .unwrap_or(false);
    let poses = if checked {
        kinematics::forward_kinematics_checked_model(model, &joints).map_err(|e| match e {
            kinematics::CheckedKinematicsError::LimitViolation(v) => ApiError {
                status: 422,
                msg: Msg::BadRequest,
                detail: Some(
                    v.iter()
                        .map(|v| {
                            format!(
                                "joint '{}' = {} outside [{}, {}]",
                                v.joint, v.value, v.lower, v.upper
                            )
                        })
                        .collect::<Vec<_>>()
                        .join("; "),
                ),
            },
            other => bad(other.to_string()),
        })?
    } else {
        kinematics::forward_kinematics_model(model, &joints).map_err(|e| bad(e.to_string()))?
    };
    Ok(json!({
        "poses": poses.iter().map(|(n, m)| pose_json(n, m)).collect::<Vec<_>>(),
        "checked": checked,
        "note": NOTE,
    }))
}

fn route_ik(model: &Model, body: &Value, ctx: &Context, lang: Lang) -> Result<Value, ApiError> {
    let target = body
        .get("target")
        .and_then(Value::as_str)
        .ok_or_else(|| bad("the request needs a \"target\" joint name"))?;
    let goal: Vec<f64> = body
        .get("goal")
        .and_then(Value::as_array)
        .map(|a| {
            a.iter()
                .filter_map(Value::as_f64)
                .filter(|n| n.is_finite())
                .collect()
        })
        .unwrap_or_default();
    if goal.len() != 3 || body["goal"].as_array().map_or(0, Vec::len) != 3 {
        return Err(bad(
            "\"goal\" must be [x, y, z], three finite numbers in metres",
        ));
    }
    let initial = joint_map(body.get("joints"), "joints")?;
    check_names(model, &initial, "joints")?;
    let mut options = ctx.config.ik.defaults;
    options.time_limit = Some(std::time::Duration::from_millis(ctx.config.ik.max_millis));
    for (key, slot) in [
        ("tolerance", &mut options.tolerance),
        ("damping", &mut options.damping),
        ("max_step", &mut options.max_step),
    ] {
        if let Some(v) = body.get(key) {
            *slot = v
                .as_f64()
                .filter(|n| n.is_finite() && *n > 0.0)
                .ok_or_else(|| bad(format!("{key} must be a number above zero")))?;
        }
    }
    if let Some(v) = body.get("max_iterations") {
        let n = v
            .as_u64()
            .filter(|n| *n >= 1)
            .ok_or_else(|| bad("max_iterations must be a whole number above zero"))?
            as usize;
        // the service bounds the work one request can ask for
        options.max_iterations = n.min(ctx.config.ik.max_iterations_cap);
    }
    match jacobian::ik_position(
        model,
        &initial,
        target,
        [goal[0], goal[1], goal[2]],
        options,
    ) {
        Ok(solution) => Ok(stamp(
            json!({
                "reached": true,
                "residual": solution.residual,
                "iterations": solution.iterations,
                "timed_out": false,
                "joints": sorted_map(&solution.positions),
                "note": NOTE,
            }),
            Msg::IkReached,
            lang,
        )),
        Err(jacobian::JacobianError::NotReached {
            residual,
            iterations,
        }) => Ok(stamp(
            json!({"reached": false, "residual": residual, "iterations": iterations, "timed_out": iterations < options.max_iterations, "note": NOTE}),
            Msg::IkNotReached,
            lang,
        )),
        Err(jacobian::JacobianError::Singular) => Ok(stamp(
            json!({"reached": false, "residual": null, "iterations": 0, "detail": "the system is singular", "note": NOTE}),
            Msg::IkNotReached,
            lang,
        )),
        Err(e) => Err(bad(e.to_string())),
    }
}

fn route_check_trajectory(
    model: &Model,
    body: &Value,
    ctx: &Context,
    lang: Lang,
) -> Result<Value, ApiError> {
    let samples = trajectory::parse_samples(body, ctx.config.max_samples).map_err(bad)?;
    let outcome = trajectory::check(model, &ctx.config.trajectory, &samples).map_err(bad)?;
    let msg = if outcome.ok {
        Msg::TrajectoryOk
    } else {
        Msg::TrajectoryViolations
    };
    let mut answer = stamp(outcome.to_json(), msg, lang);
    if !outcome.complete {
        answer["incomplete_message"] = json!(Msg::IncompleteCheck.text(lang));
        answer["incomplete_summary"] = json!(Msg::IncompleteCheck.text(Lang::En));
    }
    Ok(answer)
}

fn route_inverse_dynamics(model: &Model, body: &Value, ctx: &Context) -> Result<Value, ApiError> {
    let positions = joint_map(body.get("joints"), "joints")?;
    let velocities = joint_map(body.get("velocities"), "velocities")?;
    let accelerations = joint_map(body.get("accelerations"), "accelerations")?;
    check_names(model, &positions, "joints")?;
    let mut options = dynamics::Options {
        gravity: ctx.config.trajectory.gravity,
        allow_missing_inertial: ctx.config.trajectory.allow_missing_inertial,
    };
    if let Some(g) = body.get("gravity") {
        let list = g
            .as_array()
            .filter(|l| l.len() == 3)
            .ok_or_else(|| bad("gravity must be [x, y, z]"))?;
        for (slot, item) in options.gravity.iter_mut().zip(list) {
            *slot = item
                .as_f64()
                .filter(|n| n.is_finite())
                .ok_or_else(|| bad("gravity must be three finite numbers"))?;
        }
    }
    let result =
        dynamics::inverse_dynamics(model, &positions, &velocities, &accelerations, &options)
            .map_err(bad)?;
    let violations = crate::limits::effort_violations(model, &result.torques);
    Ok(json!({
        "torques": sorted_map(&result.torques),
        "within_effort_limits": violations.is_empty(),
        "effort_violations": violations.iter().map(|v| json!({"joint": v.joint, "value": v.value, "limit": v.limit})).collect::<Vec<_>>(),
        "links_without_inertial": result.without_inertial,
        "note": "torque (N m) for revolute joints, force (N) for prismatic ones, from the <inertial> blocks and gravity only; no friction, no motor inertia",
    }))
}

fn route_capsules(model: &Model, ctx: &Context) -> Result<Value, ApiError> {
    let caps = selfcollision::capsules(model, &ctx.config.trajectory.collision).map_err(bad)?;
    Ok(json!({
        "capsules": selfcollision::capsules_json(&caps),
        "frame": "the frame of each link; end points a and b and the radius, in metres",
    }))
}

/// The first value of `name` in a query string.
fn query_value<'a>(query: &'a str, name: &str) -> Option<&'a str> {
    query
        .split('&')
        .filter_map(|pair| pair.split_once('='))
        .find(|(k, _)| *k == name)
        .map(|(_, v)| v)
}

/// The language of a request: `?lang=`, then `Accept-Language`, then the configuration.
pub fn request_language(
    config: &ServiceConfig,
    query: &str,
    accept_language: Option<&str>,
) -> Lang {
    query_value(query, "lang")
        .and_then(Lang::from_tag)
        .or_else(|| accept_language.and_then(Lang::from_accept_language))
        .unwrap_or(config.language)
}

/// Answers a request for one of the computing routes. `body` is the raw body, already bounded in size by the caller.
pub fn handle(method: &str, path: &str, body: &[u8], ctx: &Context, lang: Lang) -> (u16, Value) {
    match handle_inner(method, path, body, ctx, lang) {
        Ok(value) => (200, value),
        Err(e) => (e.status, e.body(lang)),
    }
}

fn handle_inner(
    method: &str,
    path: &str,
    body: &[u8],
    ctx: &Context,
    lang: Lang,
) -> Result<Value, ApiError> {
    if body.len() > ctx.config.max_body_bytes {
        return Err(ApiError {
            status: 413,
            msg: Msg::PayloadTooLarge,
            detail: Some(format!("the limit is {} bytes", ctx.config.max_body_bytes)),
        });
    }
    if ctx.model.is_none() && !ctx.config.allow_inline_urdf {
        return Err(ApiError {
            status: 404,
            msg: Msg::ComputeOff,
            detail: None,
        });
    }
    if path == "/ik" && !ctx.config.ik.enabled {
        return Err(ApiError {
            status: 404,
            msg: Msg::IkOff,
            detail: None,
        });
    }
    let allowed = if path == "/capsules" {
        method == "GET" || method == "POST"
    } else {
        method == "POST"
    };
    if !allowed {
        return Err(ApiError {
            status: 405,
            msg: Msg::MethodNotAllowed,
            detail: Some(
                if path == "/capsules" {
                    "GET or POST"
                } else {
                    "POST"
                }
                .to_string(),
            ),
        });
    }
    let request: Value = if body.iter().all(u8::is_ascii_whitespace) {
        json!({})
    } else {
        serde_json::from_slice(body).map_err(|e| bad(format!("the body is not valid JSON: {e}")))?
    };
    if !request.is_object() {
        return Err(bad("the body must be a JSON object"));
    }
    let inline;
    let model: &Model = match request.get("urdf") {
        Some(Value::String(text)) => {
            if !ctx.config.allow_inline_urdf {
                return Err(ApiError {
                    status: 403,
                    msg: Msg::InlineUrdfOff,
                    detail: None,
                });
            }
            inline = urdf::parse_urdf_model(text).map_err(|e| bad(format!("urdf: {e}")))?;
            &inline
        }
        Some(_) => return Err(bad("\"urdf\" must be the text of a URDF")),
        None => ctx.model.as_ref().ok_or_else(|| {
            bad("this service has no URDF loaded: send one in \"urdf\" (allowed by the configuration)")
        })?,
    };
    match path {
        "/fk" => route_fk(model, &request),
        "/ik" => route_ik(model, &request, ctx, lang),
        "/check-trajectory" => route_check_trajectory(model, &request, ctx, lang),
        "/inverse-dynamics" => route_inverse_dynamics(model, &request, ctx),
        "/capsules" => route_capsules(model, ctx),
        _ => Err(ApiError {
            status: 404,
            msg: Msg::NotFound,
            detail: None,
        }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const ARM: &str = r#"<robot name="planar">
      <link name="base"><collision><geometry><sphere radius="0.05"/></geometry></collision></link>
      <link name="l1"><inertial><origin xyz="0.5 0 0"/><mass value="1"/>
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial></link>
      <link name="l2"><inertial><origin xyz="0.5 0 0"/><mass value="1"/>
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>
        <collision><origin xyz="0.5 0 0"/><geometry><cylinder radius="0.03" length="1"/></geometry></collision></link>
      <link name="tool"/>
      <joint name="shoulder" type="revolute"><parent link="base"/><child link="l1"/>
        <axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="40" velocity="2"/></joint>
      <joint name="elbow" type="revolute"><parent link="l1"/><child link="l2"/>
        <origin xyz="1 0 0"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="40" velocity="2"/></joint>
      <joint name="tip" type="fixed"><parent link="l2"/><child link="tool"/><origin xyz="1 0 0"/></joint>
    </robot>"#;

    fn ctx(config: ServiceConfig) -> Context {
        Context {
            config,
            model: Some(urdf::parse_urdf_model(ARM).unwrap()),
        }
    }

    fn ik_on() -> ServiceConfig {
        let mut config = ServiceConfig::default();
        config.ik.enabled = true;
        config
    }

    fn post(path: &str, body: Value, ctx: &Context) -> (u16, Value) {
        handle("POST", path, body.to_string().as_bytes(), ctx, Lang::En)
    }

    #[test]
    fn fk_answers_with_poses_and_checks_limits_when_asked() {
        let ctx = ctx(ServiceConfig::default());
        let (status, v) = post(
            "/fk",
            json!({"joints": {"shoulder": 0.0, "elbow": std::f64::consts::FRAC_PI_2}}),
            &ctx,
        );
        assert_eq!(status, 200, "{v}");
        let tip = v["poses"]
            .as_array()
            .unwrap()
            .iter()
            .find(|p| p["joint"] == "tip")
            .unwrap();
        assert!((tip["position"]["x"].as_f64().unwrap() - 1.0).abs() < 1e-9);
        assert!((tip["position"]["y"].as_f64().unwrap() - 1.0).abs() < 1e-9);
        let (status, v) = post(
            "/fk",
            json!({"joints": {"shoulder": 4.0, "elbow": 0.0}, "checked": true}),
            &ctx,
        );
        assert_eq!(status, 422);
        assert!(v["detail"].as_str().unwrap().contains("shoulder"));
        // a typo in a joint is a 400, not a pose for another configuration
        let (status, _) = post("/fk", json!({"joints": {"shoulderr": 0.0}}), &ctx);
        assert_eq!(status, 400);
        // a missing joint is never read as zero
        let (status, _) = post("/fk", json!({"joints": {"shoulder": 0.0}}), &ctx);
        assert_eq!(status, 400);
    }

    #[test]
    fn ik_reaches_a_goal_and_says_so_when_it_cannot() {
        let ctx = ctx(ik_on());
        let (status, v) = post(
            "/ik",
            json!({"target": "tip", "goal": [1.2, 0.9, 0.0], "joints": {"shoulder": 0.2, "elbow": 0.3}}),
            &ctx,
        );
        assert_eq!(status, 200, "{v}");
        assert_eq!(v["reached"], json!(true));
        assert_eq!(v["code"], "ik_reached");
        // verify with fk
        let (_, fk) = post("/fk", json!({"joints": v["joints"].clone()}), &ctx);
        let tip = fk["poses"]
            .as_array()
            .unwrap()
            .iter()
            .find(|p| p["joint"] == "tip")
            .unwrap();
        assert!((tip["position"]["x"].as_f64().unwrap() - 1.2).abs() < 1e-3);
        // out of reach: 200 with reached false
        let (status, v) = post(
            "/ik",
            json!({"target": "tip", "goal": [5.0, 0.0, 0.0], "joints": {"shoulder": 0.2, "elbow": 0.3}}),
            &ctx,
        );
        assert_eq!(status, 200);
        assert_eq!(v["reached"], json!(false));
        assert_eq!(v["code"], "ik_not_reached");
        // a goal of two numbers is a 400
        let (status, _) = post(
            "/ik",
            json!({"target": "tip", "goal": [1.0, 2.0], "joints": {"shoulder": 0.0, "elbow": 0.0}}),
            &ctx,
        );
        assert_eq!(status, 400);
    }

    #[test]
    fn the_work_one_request_can_ask_for_is_bounded_by_the_configuration() {
        let mut config = ik_on();
        config.ik.max_iterations_cap = 3;
        let ctx = ctx(config);
        let (_, v) = post(
            "/ik",
            json!({"target": "tip", "goal": [-1.9, 0.0, 0.0], "joints": {"shoulder": 0.0, "elbow": 0.0}, "max_iterations": 100000, "max_step": 0.001}),
            &ctx,
        );
        assert_eq!(v["reached"], json!(false));
        assert!(v["iterations"].as_u64().unwrap() <= 3);
    }

    #[test]
    fn the_ik_route_is_off_until_the_configuration_opens_it() {
        let off = ctx(ServiceConfig::default());
        let body = json!({"target": "tip", "goal": [1.2, 0.9, 0.0], "joints": {"shoulder": 0.2, "elbow": 0.3}});
        let (status, v) = post("/ik", body.clone(), &off);
        assert_eq!(status, 404);
        assert_eq!(v["code"], "ik_off");
        // the other routes are not touched by the switch
        assert_eq!(
            post(
                "/fk",
                json!({"joints": {"shoulder": 0.0, "elbow": 0.0}}),
                &off
            )
            .0,
            200
        );
        let config = parse_config(&json!({"ik": {"enabled": true, "max_millis": 50}})).unwrap();
        assert!(config.ik.enabled);
        assert_eq!(config.ik.max_millis, 50);
        let (status, v) = post("/ik", body, &ctx(config));
        assert_eq!(status, 200, "{v}");
        assert_eq!(v["reached"], json!(true));
        assert_eq!(v["timed_out"], json!(false));
        assert!(parse_config(&json!({"ik": {"enabled": "yes"}})).is_err());
        assert!(parse_config(&json!({"ik": {"max_millis": 0}})).is_err());
    }

    #[test]
    fn the_time_of_one_ik_request_is_bounded() {
        let mut config = ik_on();
        config.ik.max_millis = 1;
        config.ik.max_iterations_cap = 100_000;
        let ctx = ctx(config);
        // an unreachable goal with tiny steps would take 100000 iterations: the time limit ends it first
        let (status, v) = post(
            "/ik",
            json!({"target": "tip", "goal": [5.0, 0.0, 0.0], "joints": {"shoulder": 0.2, "elbow": 0.3},
                   "max_iterations": 100000, "max_step": 0.00001, "tolerance": 0.0000001}),
            &ctx,
        );
        assert_eq!(status, 200, "{v}");
        assert_eq!(v["reached"], json!(false));
        assert_eq!(v["timed_out"], json!(true), "{v}");
    }

    #[test]
    fn check_trajectory_returns_the_violations_and_the_language() {
        let ctx = ctx(ServiceConfig::default());
        let good = json!({"samples": [
            {"t": 0.0, "joints": {"shoulder": 0.0, "elbow": 0.5}},
            {"t": 0.5, "joints": {"shoulder": 0.2, "elbow": 0.5}},
            {"t": 1.0, "joints": {"shoulder": 0.5, "elbow": 0.5}}]});
        let (status, v) = post("/check-trajectory", good, &ctx);
        assert_eq!(status, 200, "{v}");
        assert_eq!(v["ok"], json!(true), "{v}");
        let fast = json!({"samples": [
            {"t": 0.0, "joints": {"shoulder": 0.0, "elbow": 0.5}},
            {"t": 0.1, "joints": {"shoulder": 1.0, "elbow": 0.5}}]});
        let (status, v) = handle(
            "POST",
            "/check-trajectory",
            fast.to_string().as_bytes(),
            &ctx,
            Lang::Pt,
        );
        assert_eq!(status, 200);
        assert_eq!(v["ok"], json!(false));
        assert_eq!(v["code"], "trajectory_violations");
        assert_eq!(v["summary"], "the trajectory breaks the limits");
        assert_eq!(v["message"], "a trajetória viola os limites");
        assert_eq!(v["violations"][0]["kind"], "velocity");
        assert_eq!(v["violations"][0]["joint"], "shoulder");
    }

    #[test]
    fn inverse_dynamics_and_capsules_answer() {
        let ctx = ctx(ServiceConfig::default());
        let (status, v) = post(
            "/inverse-dynamics",
            json!({"joints": {"shoulder": 0.0, "elbow": 0.0}, "velocities": {"shoulder": 0.0, "elbow": 0.0}, "accelerations": {"shoulder": 0.0, "elbow": 0.0}}),
            &ctx,
        );
        // link tool has no inertial: refused by default
        assert_eq!(status, 400, "{v}");
        assert!(v["detail"].as_str().unwrap().contains("tool"));
        let mut config = ServiceConfig::default();
        config.trajectory.allow_missing_inertial = true;
        let ctx2 = self::ctx(config);
        let (status, v) = post(
            "/inverse-dynamics",
            json!({"joints": {"shoulder": 0.0, "elbow": 0.0}, "velocities": {"shoulder": 0.0, "elbow": 0.0}, "accelerations": {"shoulder": 0.0, "elbow": 0.0}}),
            &ctx2,
        );
        assert_eq!(status, 200, "{v}");
        // elbow carries 1 kg at 0.5 m: 1 * 9.81 * 0.5 about z... gravity is along z, the axis is z: no torque
        assert!(v["torques"]["elbow"].as_f64().unwrap().abs() < 1e-9);
        assert_eq!(v["links_without_inertial"][0], "tool");
        let (status, v) = handle("GET", "/capsules", b"", &ctx, Lang::En);
        assert_eq!(status, 200, "{v}");
        assert_eq!(v["capsules"].as_array().unwrap().len(), 2);
    }

    #[test]
    fn methods_bodies_and_inline_urdf_are_policed() {
        let ctx = ctx(ServiceConfig::default());
        assert_eq!(handle("GET", "/fk", b"", &ctx, Lang::En).0, 405);
        assert_eq!(handle("POST", "/fk", b"{not json", &ctx, Lang::En).0, 400);
        assert_eq!(handle("POST", "/fk", b"[1]", &ctx, Lang::En).0, 400);
        let (status, v) = post("/fk", json!({"urdf": ARM, "joints": {}}), &ctx);
        assert_eq!(status, 403);
        assert_eq!(v["code"], "inline_urdf_off");
        let small = ServiceConfig {
            max_body_bytes: 1024,
            ..ServiceConfig::default()
        };
        let ctx_small = self::ctx(small);
        let big = vec![b' '; 2048];
        assert_eq!(handle("POST", "/fk", &big, &ctx_small, Lang::En).0, 413);
        // with the switch on, the request brings its own robot; with no robot loaded the routes are off otherwise
        let inline = Context {
            config: ServiceConfig {
                allow_inline_urdf: true,
                ..ServiceConfig::default()
            },
            model: None,
        };
        let (status, v) = post(
            "/fk",
            json!({"urdf": ARM, "joints": {"shoulder": 0.0, "elbow": 0.0}}),
            &inline,
        );
        assert_eq!(status, 200, "{v}");
        assert_eq!(post("/fk", json!({"joints": {}}), &inline).0, 400);
        let off = Context {
            config: ServiceConfig::default(),
            model: None,
        };
        assert_eq!(post("/fk", json!({}), &off).0, 404);
        assert_eq!(post("/fk", json!({}), &off).1["code"], "compute_off");
        // a broken urdf is a 400
        assert_eq!(
            post("/fk", json!({"urdf": "<robot", "joints": {}}), &inline).0,
            400
        );
    }

    #[test]
    fn the_configuration_is_strict_and_the_language_is_negotiated() {
        let c = parse_config(&json!({
            "language": "pt", "max_body_bytes": 2048, "max_samples": 10, "allow_inline_urdf": true,
            "ik": {"max_iterations_cap": 50, "tolerance": 0.001},
            "trajectory": {"default_acceleration_limit": 5},
            "collision": {"margin_m": 0.01}
        }))
        .unwrap();
        assert_eq!(c.language, Lang::Pt);
        assert_eq!(c.max_body_bytes, 2048);
        assert!(c.allow_inline_urdf);
        assert_eq!(c.ik.defaults.tolerance, 0.001);
        assert_eq!(c.ik.defaults.max_iterations, 50);
        assert_eq!(c.trajectory.default_acceleration_limit, Some(5.0));
        assert_eq!(c.trajectory.collision.margin_m, 0.01);
        assert!(parse_config(&json!({"language": "ru"})).is_err());
        assert!(parse_config(&json!({"max_body_bytes": 1})).is_err());
        assert!(parse_config(&json!({"ik": {"nope": 1}})).is_err());
        assert!(parse_config(&json!({"nope": 1})).is_err());
        assert!(!ServiceConfig::default().allow_inline_urdf);
        assert_eq!(request_language(&c, "lang=de", Some("fr")), Lang::De);
        assert_eq!(request_language(&c, "", Some("fr-FR,en;q=0.5")), Lang::Fr);
        assert_eq!(request_language(&c, "", None), Lang::Pt);
    }
}
