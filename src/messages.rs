// =============================================================================
// HYDRA-UMC-PHYSICS-REPLICA - src/messages.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0-or-later - see LICENSE
// =============================================================================
//! The fixed sentences of the HTTP service in the eight languages of the project (en, es, de, fr, it, ja, zh, pt of Portugal).
//!
//! Every answer carries a stable `code` and the English `error` or `summary` (what a program reads), plus a `message` in the language
//! that was asked for with `?lang=xx` or `Accept-Language`, or that the configuration names. The detail of a bad request (which field,
//! which joint) stays in English, in `detail`.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Lang {
    En,
    Es,
    De,
    Fr,
    It,
    Ja,
    Zh,
    Pt,
}

impl Lang {
    pub const ALL: [Lang; 8] = [
        Lang::En,
        Lang::Es,
        Lang::De,
        Lang::Fr,
        Lang::It,
        Lang::Ja,
        Lang::Zh,
        Lang::Pt,
    ];

    /// `es`, `es-ES`, `pt_PT`... (the part before the first `-` or `_`, in any case).
    pub fn from_tag(tag: &str) -> Option<Lang> {
        let primary = tag
            .trim()
            .split(['-', '_'])
            .next()
            .unwrap_or("")
            .to_ascii_lowercase();
        Lang::ALL.into_iter().find(|l| l.code() == primary)
    }

    pub fn code(self) -> &'static str {
        match self {
            Lang::En => "en",
            Lang::Es => "es",
            Lang::De => "de",
            Lang::Fr => "fr",
            Lang::It => "it",
            Lang::Ja => "ja",
            Lang::Zh => "zh",
            Lang::Pt => "pt",
        }
    }

    /// The first language of an `Accept-Language` header that the service has.
    pub fn from_accept_language(header: &str) -> Option<Lang> {
        header
            .split(',')
            .filter_map(|part| Lang::from_tag(part.split(';').next().unwrap_or("")))
            .next()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Msg {
    NotFound,
    ComputeOff,
    Unauthorized,
    BadRequest,
    PayloadTooLarge,
    MethodNotAllowed,
    InlineUrdfOff,
    TrajectoryOk,
    TrajectoryViolations,
    IkReached,
    IkNotReached,
    IncompleteCheck,
}

impl Msg {
    #[cfg(test)]
    pub const ALL: [Msg; 12] = [
        Msg::NotFound,
        Msg::ComputeOff,
        Msg::Unauthorized,
        Msg::BadRequest,
        Msg::PayloadTooLarge,
        Msg::MethodNotAllowed,
        Msg::InlineUrdfOff,
        Msg::TrajectoryOk,
        Msg::TrajectoryViolations,
        Msg::IkReached,
        Msg::IkNotReached,
        Msg::IncompleteCheck,
    ];

    /// The stable code of the sentence.
    pub fn code(self) -> &'static str {
        match self {
            Msg::NotFound => "not_found",
            Msg::ComputeOff => "compute_off",
            Msg::Unauthorized => "unauthorized",
            Msg::BadRequest => "bad_request",
            Msg::PayloadTooLarge => "payload_too_large",
            Msg::MethodNotAllowed => "method_not_allowed",
            Msg::InlineUrdfOff => "inline_urdf_off",
            Msg::TrajectoryOk => "trajectory_ok",
            Msg::TrajectoryViolations => "trajectory_violations",
            Msg::IkReached => "ik_reached",
            Msg::IkNotReached => "ik_not_reached",
            Msg::IncompleteCheck => "incomplete_check",
        }
    }

    pub fn text(self, lang: Lang) -> &'static str {
        let row: [&'static str; 8] = match self {
            Msg::NotFound => [
                "not found",
                "no encontrado",
                "nicht gefunden",
                "introuvable",
                "non trovato",
                "見つかりません",
                "未找到",
                "não encontrado",
            ],
            Msg::ComputeOff => [
                "the computing routes are off: start serve with --urdf",
                "las rutas de cálculo están apagadas: arranque serve con --urdf",
                "Die Rechenrouten sind ausgeschaltet: serve mit --urdf starten",
                "les routes de calcul sont désactivées : lancez serve avec --urdf",
                "le rotte di calcolo sono disattivate: avviare serve con --urdf",
                "計算用のルートはオフです。--urdf を付けて serve を起動してください",
                "计算路由已关闭：请使用 --urdf 启动 serve",
                "as rotas de cálculo estão desligadas: inicie o serve com --urdf",
            ],
            Msg::Unauthorized => [
                "a valid token is required",
                "se necesita un token válido",
                "Ein gültiges Token ist erforderlich",
                "un jeton valide est requis",
                "è richiesto un token valido",
                "有効なトークンが必要です",
                "需要有效的令牌",
                "é necessário um token válido",
            ],
            Msg::BadRequest => [
                "the request is not valid",
                "la petición no es válida",
                "Die Anfrage ist ungültig",
                "la requête n'est pas valide",
                "la richiesta non è valida",
                "リクエストが無効です",
                "请求无效",
                "o pedido não é válido",
            ],
            Msg::PayloadTooLarge => [
                "the request body is too large",
                "el cuerpo de la petición es demasiado grande",
                "Der Anfragekörper ist zu groß",
                "le corps de la requête est trop volumineux",
                "il corpo della richiesta è troppo grande",
                "リクエスト本文が大きすぎます",
                "请求正文过大",
                "o corpo do pedido é demasiado grande",
            ],
            Msg::MethodNotAllowed => [
                "method not allowed for this route",
                "método no permitido para esta ruta",
                "Methode für diese Route nicht erlaubt",
                "méthode non autorisée pour cette route",
                "metodo non consentito per questa rotta",
                "このルートでは許可されていないメソッドです",
                "此路由不允许该方法",
                "método não permitido para esta rota",
            ],
            Msg::InlineUrdfOff => [
                "an inline URDF is not accepted by this service",
                "este servicio no acepta un URDF en la petición",
                "Dieser Dienst akzeptiert kein URDF in der Anfrage",
                "ce service n'accepte pas d'URDF dans la requête",
                "questo servizio non accetta un URDF nella richiesta",
                "このサービスはリクエスト内の URDF を受け付けません",
                "此服务不接受请求中的 URDF",
                "este serviço não aceita um URDF no pedido",
            ],
            Msg::TrajectoryOk => [
                "the trajectory is within the limits",
                "la trayectoria cumple los límites",
                "Die Trajektorie hält die Grenzen ein",
                "la trajectoire respecte les limites",
                "la traiettoria rispetta i limiti",
                "軌道は制限内です",
                "轨迹在限制范围内",
                "a trajetória respeita os limites",
            ],
            Msg::TrajectoryViolations => [
                "the trajectory breaks the limits",
                "la trayectoria incumple los límites",
                "Die Trajektorie verletzt die Grenzen",
                "la trajectoire enfreint les limites",
                "la traiettoria viola i limiti",
                "軌道が制限を超えています",
                "轨迹超出了限制",
                "a trajetória viola os limites",
            ],
            Msg::IkReached => [
                "the goal was reached",
                "se alcanzó el objetivo",
                "Das Ziel wurde erreicht",
                "l'objectif a été atteint",
                "l'obiettivo è stato raggiunto",
                "目標に到達しました",
                "已到达目标",
                "o objetivo foi alcançado",
            ],
            Msg::IkNotReached => [
                "the goal was not reached",
                "no se alcanzó el objetivo",
                "Das Ziel wurde nicht erreicht",
                "l'objectif n'a pas été atteint",
                "l'obiettivo non è stato raggiunto",
                "目標に到達できませんでした",
                "未到达目标",
                "o objetivo não foi alcançado",
            ],
            Msg::IncompleteCheck => [
                "a check could not run; the answer is incomplete",
                "una comprobación no pudo ejecutarse; la respuesta está incompleta",
                "Eine Prüfung konnte nicht ausgeführt werden; die Antwort ist unvollständig",
                "une vérification n'a pas pu s'exécuter ; la réponse est incomplète",
                "un controllo non ha potuto essere eseguito; la risposta è incompleta",
                "実行できなかったチェックがあり、回答は不完全です",
                "有一项检查无法运行，答复不完整",
                "uma verificação não pôde ser executada; a resposta está incompleta",
            ],
        };
        row[Lang::ALL.iter().position(|l| *l == lang).unwrap_or(0)]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_sentence_exists_in_all_eight_languages_and_only_english_is_english() {
        for msg in Msg::ALL {
            let english = msg.text(Lang::En);
            for lang in Lang::ALL {
                let text = msg.text(lang);
                assert!(!text.is_empty());
                if lang != Lang::En {
                    assert_ne!(text, english, "{} in {}", msg.code(), lang.code());
                }
            }
            // Portugal, not Brazil
            assert_ne!(msg.text(Lang::Pt), msg.text(Lang::Es));
        }
        assert!(Msg::Unauthorized.text(Lang::Pt).contains("é necessário"));
        assert!(Msg::BadRequest.text(Lang::Pt).contains("pedido"));
    }

    #[test]
    fn the_language_is_read_from_a_tag_or_a_header() {
        assert_eq!(Lang::from_tag("pt-PT"), Some(Lang::Pt));
        assert_eq!(Lang::from_tag("ZH_cn"), Some(Lang::Zh));
        assert_eq!(Lang::from_tag("xx"), None);
        assert_eq!(
            Lang::from_accept_language("ru;q=0.9, de-DE;q=0.8, en;q=0.5"),
            Some(Lang::De)
        );
        assert_eq!(Lang::from_accept_language("ru"), None);
    }
}
