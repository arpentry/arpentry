//! The synthetic network: a `--segments` spec instead of a parquet.
//!
//! `--terrain` accepts `flat|ramp|hill|step` so a step's output on the
//! ground is an assertion rather than a distribution. The network gets the
//! same dial: a handful of ways drawn in the local frame with the class and
//! subclass the data would carry, so a ribbon's area, a junction's region
//! count and a pavement's edge have a known answer.
//!
//! ```text
//! net:straight[?len=200&class=residential]   one way along x
//!   [&span=0.35,0.65&kind=bridge|tunnel&level=1]  with a mapped span over that fraction of it
//! net:tee[?d=8][&hook=5]                      two halves of it and a leg from the north, meeting at the origin; with `d`, a sidewalk `d` m off both axes wrapping the north-west corner; with `hook`, road-e turns back north on that radius
//!   [&span=0.3&kind=bridge|tunnel&level=1]    the fraction of every leg nearest the origin mapped as a structure: the junction itself stands on one
//! net:cross                                   four legs meeting at the origin
//! net:overpass[?span=0.35,0.65&level=1]       the way along x and a leg along y crossing it, mapped as a bridge over the crossing: interiors crossing, no shared connector
//!   [&leg=CLASS]                              the leg of another class: `leg=standard_gauge` is a rail bridge over the road, `class=standard_gauge&leg=residential` a road bridge over the railway
//! net:underpass[?span=0.35,0.65&level=1]      the same with the leg mapped as a tunnel
//! net:level[?rail=standard_gauge]             the way along x and a railway along y through one connector in the interior of both: a level crossing, as Overture maps every one
//! net:hairpin?angle=20                        one way bent by `angle` degrees at the origin
//! net:dual?gap=4                              two ways, their kerbs `gap` m apart
//! net:sidewalk?d=6[&gap=0]                    the way and a sidewalk `d` m off its axis, in two halves `gap` m apart if `gap`
//! net:corner?d=5[&split=1]                    the way turning north, a sidewalk wrapping the outside on a chamfer, cut at its midpoint if `split`
//! net:stub?d=0.5                              the way and a footway from the north ending `d` m short of its kerb (under the asphalt if negative)
//! net:driveway?d=6[&short=0]                  the way, a sidewalk `d` m off its axis in two halves, and a service way from the north ending on their joint, or `short` m before it
//! net:crossing?d=6                            sidewalks either side, each cut at the connector, and a crosswalk between them
//! net:roundabout?r=15&d=5                     a ring road in four arcs, four legs, a sidewalk ring `d` m outside its kerb
//! ```
//!
//! Every parameter has a default, so `net:cross` is a complete spec. The
//! way is `residential` unless `class=` says otherwise, and a specimen's
//! ids are stable words (`road`, `walk-n`, …) so a test can pick one out.
//! Ways meet the way Overture's do: cut at every connector, so a junction
//! is always a meeting of way ends and never a crossing of interiors — two
//! ways whose interiors cross are a bridge over a road, not a junction.

use crate::world::{Kind, Span, Way};

/// Whether `s` is a spec rather than a path.
pub fn is_spec(s: &str) -> bool {
    s.starts_with("net:")
}

/// The ways of `spec`, in local metres, each whole and carrying its own
/// span table — the same shape the parquet reader hands on.
pub fn parse(spec: &str) -> Result<Vec<Way>, String> {
    let rest = spec.strip_prefix("net:").ok_or_else(|| format!("not a network spec: {spec}"))?;
    let (name, query) = rest.split_once('?').unwrap_or((rest, ""));
    let params = Params::parse(query)?;
    let len = params.num("len", 200.0)?;
    let class = params.get("class").unwrap_or("residential");
    let half = len / 2.0;
    let road = |id: &str, pts: Vec<[f64; 2]>| line(id, class, "", pts);
    let walk = |id: &str, subclass: &str, pts: Vec<[f64; 2]>| line(id, "footway", subclass, pts);
    let x_road = |id: &str| road(id, vec![[-half, 0.0], [half, 0.0]]);
    let ways = match name {
        "straight" => spanned("road", class, "", 0.0, half, span_of(&params)?),
        "tee" => {
            // With `hook`, road-e runs 10 m east then turns back on an arc
            // of that radius and ends short of the leg: a bend within one
            // leg whose inside lies in the junction's mask.
            let east = match params.get("hook") {
                None => vec![[0.0, 0.0], [half, 0.0]],
                Some(r) => {
                    let r: f64 = r.parse().map_err(|_| format!("invalid hook: {r}"))?;
                    let mut pts = vec![[0.0, 0.0], [10.0, 0.0]];
                    pts.extend((1..=12).map(|k| {
                        let a = -std::f64::consts::FRAC_PI_2 + k as f64 * std::f64::consts::PI / 12.0;
                        [10.0 + r * a.cos(), r + r * a.sin()]
                    }));
                    pts.push([6.0, 2.0 * r]);
                    pts
                }
            };
            let mut ways = vec![
                road("road-w", vec![[-half, 0.0], [0.0, 0.0]]),
                road("road-e", east),
                road("leg", vec![[0.0, 0.0], [0.0, half]]),
            ];
            // **With `span`, the junction itself stands on a structure.** The
            // fraction of every leg nearest the origin is mapped as one, so
            // the three legs meet *on* the deck and each handover to the
            // ground falls out along its own leg.
            //
            // No other spec states that, which is why this one exists
            // (`data/plans/one-surface-at-a-junction-2026-09-14.md`, step 0):
            // `straight?span=` is a span with no junction on it, `tee` a
            // junction with no span, and `overpass`/`underpass` two interiors
            // crossing with **no connector between them** — the case the
            // surface rule deliberately keeps apart. Every leg takes the same
            // ordinal, because they meet: a level orders what crosses.
            if let Some(f) = params.get("span") {
                let f: f64 = f.parse().map_err(|_| format!("invalid span: {f}"))?;
                if !(0.0..=1.0).contains(&f) {
                    return Err(format!("invalid span: {f}, expected a fraction of a leg"));
                }
                let kind = kind_of(&params)?;
                ways = ways
                    .into_iter()
                    // road-w ends at the origin; the other two begin there.
                    .map(|w| {
                        let at_start = w.id != "road-w";
                        spanned_at(w, f, kind, at_start)
                    })
                    .collect();
            }
            // A sidewalk wrapping the corner between road-w and the leg on
            // a chamfer, `d` m off both axes: its stations either side of
            // the chamfer project onto different legs.
            if let Some(d) = params.get("d") {
                let d: f64 = d.parse().map_err(|_| format!("invalid d: {d}"))?;
                ways.push(walk("walk", "sidewalk", vec![[-half, d], [-1.5 * d, d], [-d, 1.5 * d], [-d, half]]));
            }
            ways
        }
        // The one pair of ways the model couples: their interiors cross,
        // they share no connector, and the leg carries the level ordinal
        // that says which of the two is meant to leave the ground. The
        // span is the annotation, not the answer — on flat ground its
        // chord starts at grade and the crossing step is what lifts it.
        "overpass" | "underpass" => {
            let level = params.num("level", 1.0)?.abs().max(1.0) as i64;
            let kind = if name == "overpass" { Kind::Bridge(level) } else { Kind::Tunnel(-level) };
            let span = match params.get("span") {
                None => (0.35, 0.65),
                Some(_) => span_of(&params)?.expect("span= is set").0,
            };
            let mut ways = vec![x_road("road")];
            let leg = params.get("leg").unwrap_or(class);
            for mut w in spanned("leg", leg, "", 0.0, half, Some((span, kind))) {
                for p in &mut w.pts {
                    *p = [p[1], p[0]];
                }
                ways.push(w);
            }
            ways
        }
        // A railway crossing the road at grade. Overture does not cut
        // either way at a level crossing's connector — every one of the
        // loop box's 48 lies in the interior of both — so each way is
        // whole and has a vertex there, which is all that joins them.
        "level" => {
            let rail = params.get("rail").unwrap_or("standard_gauge");
            vec![
                road("road", vec![[-half, 0.0], [0.0, 0.0], [half, 0.0]]),
                line("rail", rail, "", vec![[0.0, -half], [0.0, 0.0], [0.0, half]]),
            ]
        }
        "cross" => vec![
            road("road-w", vec![[-half, 0.0], [0.0, 0.0]]),
            road("road-e", vec![[0.0, 0.0], [half, 0.0]]),
            road("leg-s", vec![[0.0, -half], [0.0, 0.0]]),
            road("leg-n", vec![[0.0, 0.0], [0.0, half]]),
        ],
        "hairpin" => {
            let a = params.num("angle", 20.0)?.to_radians() / 2.0;
            let (dx, dy) = (half * a.cos(), half * a.sin());
            vec![road("road", vec![[dx, dy], [0.0, 0.0], [dx, -dy]])]
        }
        "dual" => {
            let gap = params.num("gap", 4.0)?;
            let y = (crate::width::of(class, "") + gap) / 2.0;
            vec![
                road("road-n", vec![[-half, y], [half, y]]),
                road("road-s", vec![[half, -y], [-half, -y]]),
            ]
        }
        "sidewalk" => {
            // With `gap`, the sidewalk is two halves that stop `gap` m
            // short of each other at the origin: a break in the data.
            let d = params.num("d", 6.0)?;
            let gap = params.num("gap", 0.0)?;
            if gap > 0.0 {
                vec![
                    x_road("road"),
                    walk("walk-w", "sidewalk", vec![[-half, d], [-gap / 2.0, d]]),
                    walk("walk-e", "sidewalk", vec![[gap / 2.0, d], [half, d]]),
                ]
            } else {
                // With `span`, both the road and its separated sidewalk
                // are mapped as their own bridge over the same stretch,
                // which is how 22.7 % of the extract's footbridges are
                // drawn: one structure, two ways on it.
                let span = span_of(&params)?;
                let mut ways = spanned("road", class, "", 0.0, half, span);
                ways.extend(spanned("walk-n", "footway", "sidewalk", d, half, span));
                ways
            }
        }
        "corner" => {
            // The sidewalk cuts the corner on a chamfer from `(0, -d)` to
            // `(d, 0)`, at 45° to both legs, as a mapper draws it round a
            // building's corner; `split=1` cuts it at the chamfer's
            // midpoint, where Overture would put the crossing's connector,
            // into two ways that meet there.
            let d = params.num("d", 5.0)?;
            let split = params.num("split", 0.0)? != 0.0;
            let mid = [d / 2.0, -d / 2.0];
            let mut ways = vec![road("road", vec![[-half, 0.0], [0.0, 0.0], [0.0, half]])];
            if split {
                ways.push(walk("walk-w", "sidewalk", vec![[-half, -d], [0.0, -d], mid]));
                ways.push(walk("walk-n", "sidewalk", vec![mid, [d, 0.0], [d, half]]));
            } else {
                ways.push(walk("walk", "sidewalk", vec![[-half, -d], [0.0, -d], [d, 0.0], [d, half]]));
            }
            ways
        }
        "driveway" => {
            let d = params.num("d", 6.0)?;
            let short = params.num("short", 0.0)?;
            vec![
                x_road("road"),
                walk("walk-w", "sidewalk", vec![[-half, d], [0.0, d]]),
                walk("walk-e", "sidewalk", vec![[0.0, d], [half, d]]),
                line("drive", "service", "driveway", vec![[0.0, 20.0], [0.0, d + short]]),
            ]
        }
        "stub" => {
            let d = params.num("d", 0.5)?;
            let kerb = crate::width::of(class, "") / 2.0;
            vec![x_road("road"), walk("stub", "", vec![[0.0, 20.0], [0.0, kerb + d]])]
        }
        "crossing" => {
            // Each sidewalk is cut at the crossing's connector, as Overture
            // cuts a way at every connector.
            let d = params.num("d", 6.0)?;
            vec![
                x_road("road"),
                walk("walk-nw", "sidewalk", vec![[-half, d], [0.0, d]]),
                walk("walk-ne", "sidewalk", vec![[0.0, d], [half, d]]),
                walk("walk-sw", "sidewalk", vec![[-half, -d], [0.0, -d]]),
                walk("walk-se", "sidewalk", vec![[0.0, -d], [half, -d]]),
                walk("crossing", "crosswalk", vec![[0.0, -d], [0.0, d]]),
            ]
        }
        "roundabout" => {
            let r = params.num("r", 15.0)?;
            let d = params.num("d", 5.0)?;
            let ring = |radius: f64| -> Vec<[f64; 2]> {
                (0..=36)
                    .map(|k| {
                        let a = (k % 36) as f64 * std::f64::consts::TAU / 36.0;
                        [radius * a.cos(), radius * a.sin()]
                    })
                    .collect()
            };
            // Four arcs, split at the legs: Overture cuts a way at every
            // connector, so a junction is always a meeting of way ends.
            let full = ring(r);
            let mut ways: Vec<Way> = (0..4)
                .map(|q| road(&format!("arc-{q}"), full[q * 9..=(q + 1) * 9].to_vec()))
                .collect();
            for (id, dir) in [("leg-e", [1.0, 0.0]), ("leg-n", [0.0, 1.0]), ("leg-w", [-1.0, 0.0]), ("leg-s", [0.0, -1.0])] {
                ways.push(road(id, vec![[dir[0] * r, dir[1] * r], [dir[0] * half, dir[1] * half]]));
            }
            let kerb = r + crate::width::of(class, "") / 2.0;
            ways.push(walk("walk", "sidewalk", ring(kerb + d)));
            ways
        }
        other => return Err(format!("unknown network `{other}` in {spec}")),
    };
    Ok(ways)
}

/// The mapped span of a spec, as `((from, to), kind)` over the way's
/// length; `None` if it has none.
fn span_of(params: &Params) -> Result<Option<((f64, f64), Kind)>, String> {
    let Some(span) = params.get("span") else {
        return Ok(None);
    };
    let (a, b) = span
        .split_once(',')
        .and_then(|(a, b)| Some((a.parse::<f64>().ok()?, b.parse::<f64>().ok()?)))
        .filter(|(a, b)| 0.0 <= *a && a < b && *b <= 1.0)
        .ok_or_else(|| format!("invalid span: {span}"))?;
    Ok(Some(((a, b), kind_of(params)?)))
}

/// The structure a spec's `kind` and `level` name.
fn kind_of(params: &Params) -> Result<Kind, String> {
    let level = params.num("level", 1.0)? as i64;
    Ok(match params.get("kind").unwrap_or("bridge") {
        "bridge" => Kind::Bridge(level.abs().max(1)),
        "tunnel" => Kind::Tunnel(-level.abs().max(1)),
        other => return Err(format!("invalid kind: {other}")),
    })
}

/// `w` with a mapped span of `kind` over the fraction `f` of its length
/// nearest one end — the end its points begin at when `at_start`.
///
/// [`spanned`] does the same for the x-aligned specs, from the pair of
/// fractions they take. This one measures from a *chosen end*, which is what
/// a junction needs: its legs run away from the connector they share in as
/// many directions as there are legs, so one pair of fractions cannot say
/// "the part next to the junction" for all of them.
fn spanned_at(mut w: Way, f: f64, kind: Kind, at_start: bool) -> Way {
    let len = crate::roads::length(&w.pts);
    if f <= 0.0 || len <= 0.0 {
        return w;
    }
    let (a0, a1) =
        if at_start { (0.0, (f * len).min(len)) } else { (((1.0 - f).max(0.0)) * len, len) };
    let mut spans = Vec::new();
    if a0 > 0.0 {
        spans.push(Span { a0: 0.0, a1: a0, kind: Kind::Ground });
    }
    spans.push(Span { a0, a1, kind });
    if a1 < len {
        spans.push(Span { a0: a1, a1: len, kind: Kind::Ground });
    }
    w.spans = spans;
    w
}

/// A way along x at `y`, from `-half` to `half`, cut at the boundaries of
/// its mapped span into the pieces the reader would produce: consecutive
/// pieces share their end vertex and their id.
/// One way along x at `y`, with `span` as its annotation: the fractions the
/// spec speaks in, converted to arc, with the ground either side of them.
/// The way is **not** cut — the partition step does that, once.
fn spanned(id: &str, class: &str, subclass: &str, y: f64, half: f64, span: Option<((f64, f64), Kind)>) -> Vec<Way> {
    let mut w = line(id, class, subclass, vec![[-half, y], [half, y]]);
    let Some(((a, b), kind)) = span else {
        return vec![w];
    };
    let len = 2.0 * half;
    let (a0, a1) = (a * len, b * len);
    let mut spans = Vec::new();
    if a0 > 0.0 {
        spans.push(Span { a0: 0.0, a1: a0, kind: Kind::Ground });
    }
    spans.push(Span { a0, a1, kind });
    if a1 < len {
        spans.push(Span { a0: a1, a1: len, kind: Kind::Ground });
    }
    w.spans = spans;
    vec![w]
}

/// A way on the ground end to end.
fn line(id: &str, class: &str, subclass: &str, pts: Vec<[f64; 2]>) -> Way {
    let len = crate::roads::length(&pts);
    Way {
        id: id.into(),
        class: class.into(),
        subclass: subclass.into(),
        width_m: crate::width::of(class, subclass),
        pts,
        spans: vec![Span { a0: 0.0, a1: len, kind: Kind::Ground }],
        layers: Vec::new(),
    }
}

/// The `k=v&k=v` part of a spec.
pub(crate) struct Params(Vec<(String, String)>);

impl Params {
    pub(crate) fn parse(query: &str) -> Result<Params, String> {
        let mut out = Vec::new();
        for pair in query.split('&').filter(|s| !s.is_empty()) {
            let (k, v) = pair.split_once('=').ok_or_else(|| format!("expected k=v, got `{pair}`"))?;
            out.push((k.to_string(), v.to_string()));
        }
        Ok(Params(out))
    }

    pub(crate) fn get(&self, key: &str) -> Option<&str> {
        self.0.iter().find(|(k, _)| k == key).map(|(_, v)| v.as_str())
    }

    pub(crate) fn num(&self, key: &str, default: f64) -> Result<f64, String> {
        Ok(self.opt(key)?.unwrap_or(default))
    }

    /// The number under `key`, or `None` where the spec does not say.
    pub(crate) fn opt(&self, key: &str) -> Result<Option<f64>, String> {
        self.get(key).map(|v| v.parse().map_err(|_| format!("invalid {key}: {v}"))).transpose()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_specimen_parses_with_defaults() {
        for (spec, n) in [
            ("net:straight", 1),
            ("net:tee", 3),
            ("net:cross", 4),
            // Two ways, not four pieces: the leg carries its bridge or its
            // bore as a span table, and the partition step is what cuts it.
            ("net:overpass", 2),
            ("net:underpass", 2),
            ("net:hairpin", 1),
            ("net:dual", 2),
            ("net:sidewalk", 2),
            ("net:corner", 2),
            ("net:crossing", 6),
            ("net:stub", 2),
            ("net:driveway", 4),
            ("net:roundabout", 9),
            ("net:level", 2),
        ] {
            let ways = parse(spec).unwrap();
            assert_eq!(ways.len(), n, "{spec}");
            assert!(ways.iter().all(|w| w.pts.len() >= 2), "{spec}");
            // And every way arrives with a span table that partitions it.
            for w in &ways {
                assert!(!w.spans.is_empty(), "{spec}: {} has no spans", w.id);
                assert!(w.spans[0].a0.abs() < 1e-9, "{spec}");
                assert!((w.spans[w.spans.len() - 1].a1 - w.len()).abs() < 1e-6, "{spec}");
            }
        }
    }

    #[test]
    fn parameters_are_read() {
        let ways = parse("net:straight?len=100&class=primary").unwrap();
        assert_eq!(ways[0].class, "primary");
        assert_eq!(ways[0].pts, vec![[-50.0, 0.0], [50.0, 0.0]]);
        let ways = parse("net:corner?d=5&split=1").unwrap();
        assert_eq!(ways.len(), 3);
        assert_eq!(ways[1].pts.last(), ways[2].pts.first(), "the two halves meet");
        assert_eq!(ways[1].pts.last(), Some(&[2.5, -2.5]), "at the chamfer's midpoint");
        let ways = parse("net:stub?d=-1").unwrap();
        assert_eq!(ways[1].pts[1], [0.0, 1.75], "a metre under the asphalt of a 5.5 m road");
        let ways = parse("net:sidewalk?d=3.5").unwrap();
        assert_eq!(ways[1].subclass, "sidewalk");
        assert_eq!(ways[1].pts[0][1], 3.5);
        // A dual's kerbs are `gap` apart: axes at ±(w + gap)/2.
        let ways = parse("net:dual?gap=4").unwrap();
        assert!((ways[0].pts[0][1] - 4.75).abs() < 1e-12);
        assert!((ways[1].pts[0][1] + 4.75).abs() < 1e-12);
    }

    /// A span is an *attribute* of one whole way, in arc — not a cut. The
    /// way that used to arrive as three pieces arrives as one line with a
    /// three-row table, and the partition step is what turns the table into
    /// geometry.
    #[test]
    fn a_span_is_a_table_on_one_whole_way() {
        let ways = parse("net:straight?len=200&span=0.35,0.65").unwrap();
        assert_eq!(ways.len(), 1, "one way");
        let w = &ways[0];
        assert_eq!(w.id, "road");
        assert_eq!(w.pts, vec![[-100.0, 0.0], [100.0, 0.0]], "uncut");
        assert_eq!(w.spans.iter().map(|s| s.kind).collect::<Vec<_>>(), [
            Kind::Ground,
            Kind::Bridge(1),
            Kind::Ground
        ]);
        // The table is in arc from the way's start, and partitions it.
        let arcs: Vec<(f64, f64)> = w.spans.iter().map(|s| (s.a0, s.a1)).collect();
        assert_eq!(arcs, vec![(0.0, 70.0), (70.0, 130.0), (130.0, 200.0)]);
        let ways = parse("net:straight?span=0,1&kind=tunnel&level=2").unwrap();
        assert_eq!(ways.len(), 1);
        assert_eq!(ways[0].spans.len(), 1, "no ground either side");
        assert_eq!(ways[0].spans[0].kind, Kind::Tunnel(-2));
        assert!(parse("net:straight?span=0.7,0.3").is_err());
        assert!(parse("net:straight?span=0.3,0.7&kind=viaduct").is_err());
    }

    #[test]
    fn an_overpass_crosses_the_road_without_meeting_it() {
        let ways = parse("net:overpass?len=200").unwrap();
        assert_eq!(ways.len(), 2, "the road and the leg, each whole");
        assert_eq!(ways[0].pts, vec![[-100.0, 0.0], [100.0, 0.0]], "the road along x");
        // The leg runs along y through the origin, with the bridge in the
        // middle of its table — and neither way has a vertex at the crossing,
        // which is what makes it a grade separation rather than a junction.
        let leg = ways.iter().find(|w| w.id == "leg").expect("the leg");
        assert_eq!(leg.pts, vec![[0.0, -100.0], [0.0, 100.0]]);
        assert_eq!(leg.spans.iter().map(|s| s.kind).collect::<Vec<_>>(), [
            Kind::Ground,
            Kind::Bridge(1),
            Kind::Ground
        ]);
        assert!(ways.iter().all(|w| w.pts.iter().all(|p| *p != [0.0, 0.0])), "no connector at the crossing");
        let ways = parse("net:underpass?level=2").unwrap();
        let leg = ways.iter().find(|w| w.has_structure()).expect("the leg");
        assert_eq!(leg.spans.iter().find(|s| s.kind != Kind::Ground).unwrap().kind, Kind::Tunnel(-2));
    }

    #[test]
    fn a_hairpin_opens_by_its_angle() {
        let ways = parse("net:hairpin?angle=90&len=200").unwrap();
        let [p, q, r] = ways[0].pts[..] else { panic!() };
        assert_eq!(q, [0.0, 0.0]);
        let dot = p[0] * r[0] + p[1] * r[1];
        assert!(dot.abs() < 1e-9, "not a right angle: {p:?} {r:?}");
    }

    #[test]
    fn bad_specs_are_errors() {
        assert!(parse("net:spaghetti").is_err());
        assert!(parse("net:straight?len=abc").is_err());
        assert!(parse("net:straight?len").is_err());
        assert!(parse("flat").is_err());
        assert!(is_spec("net:cross") && !is_spec("segment.parquet"));
    }
}
