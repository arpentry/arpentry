//! Way centrelines from the transportation source — roads and pedestrian
//! ways alike — in the local frame, clipped to the bbox.
//!
//! This is a reader, not a model: no joining into corridors, no heights.
//! It decides each way's width once, from its `class`, `subclass`, one-way
//! flag and measured width ([`crate::width::of_way`]), and carries `class`
//! and `subclass` on for the steps keyed on them, and `id` so a line in the
//! output can be traced to its source feature.
//!
//! **A way carries its spans; it is not cut by them.** Overture encodes a
//! bridge or a tunnel as a span of a segment — `level_rules`, or the
//! `is_bridge`/`is_tunnel` flags in `road_flags`, over a `[start, end]`
//! fraction of its length — and a way inside a building as an `is_indoor`
//! span. The reader converts those fractions to **arc** and hands the way on
//! whole, with its spans as an attribute ([`Way::spans`]).
//!
//! Cut there, a mapper's split point would become a survey point: a piece
//! end is a connector, and a connector is where the profile pins a height to
//! the ground, so a bridge annotated short of the gorge lip would have its
//! deck pinned to the DEM inside the approach. The cut happens once the
//! heights are solved, in [`crate::partition`], which is also where the
//! surface steps get their ground pieces — so what they union is where the
//! world says the ground is rather than where a segment was split.

use std::path::Path;

use arpentry_server::geoparquet::{GeoParquet, ReadError};
use arpentry_server::levels::LevelRun;
use arpentry_server::project::Bounds;
use arpentry_server::value::{str_of, width_rules_m, Props, Value};

use crate::{line, width};
use geo_types::{Geometry, LineString};

use crate::frame::{Frame, Rect};
use crate::world::{Kind, Span, Way};

/// The columns read. `subtype` and `class` decide admission.
///
/// `subclass` is the scalar column, which Overture fills only when the value
/// is uniform along the segment: a footway that is a sidewalk over part of its
/// length and a crossing over the rest has `subclass = NULL` (docs/SOURCES.md).
/// `subclass_rules` is not read, so such a way is an anonymous footway.
const COLUMNS: &[&str] = &[
    "id",
    "class",
    "subtype",
    "subclass",
    "level_rules",
    "road_flags",
    "rail_flags",
    "width_rules",
    "access_restrictions",
];

/// The road classes kept: every class [`crate::width::of`] gives a width.
/// Water is out, as is anything with no surface a person or a car stands on;
/// the railways kept are [`width::RAIL_CLASSES`], under `subtype = rail`.
const CLASSES: &[&str] = &[
    "motorway",
    "trunk",
    "primary",
    "secondary",
    "tertiary",
    "unclassified",
    "residential",
    "living_street",
    "service",
    "pedestrian",
    "footway",
    "steps",
    "path",
    "track",
    "cycleway",
    "bridleway",
    "unknown",
];

/// What [`read`] found.
#[derive(Debug, Default)]
pub struct Read {
    /// The whole ways, clipped to the rect, each with its span table.
    pub ways: Vec<Way>,
    /// Features decoded from the row groups touching the bbox.
    pub features: usize,
    /// Of those, the ways kept.
    pub kept: usize,
    /// Of the kept, the ways with a span above or below the ground, or
    /// indoors.
    pub structures: usize,
    /// Of those, the ways off the ground or indoors end to end, with no
    /// ground span at all: kept whole, like every other.
    pub aloft: usize,
    /// Of the kept, the ways whose width is measured (`width_rules`).
    pub measured: usize,
    /// Of the kept, the one-way carriageways.
    pub oneway: usize,
    /// Of the kept, the railways.
    pub rail: usize,
    /// Railways not kept: street-running rail and `unknown`, which have no
    /// formation of their own ([`width::RAIL_CLASSES`]).
    pub street_rail: usize,
    /// Of the kept, the ways the source put at a level somewhere without a
    /// bridge or a tunnel there: ordered, not raised or buried.
    pub layered: usize,
}

/// What the source's levels and flags make of a segment, as fractions of
/// its length: the **structures** — every `is_bridge` or `is_tunnel` stretch,
/// with the ordinal of the level rule of its own sign that overlaps it most,
/// or ±1 where none does — and the **layers**: every stretch of a level
/// rule no flag of its sign covers, with its level.
///
/// **A level is an ordering, not a structure.** Overture takes a segment's
/// level from OSM's `layer`, which says only what is drawn over what, and its
/// flags from `bridge` and `tunnel`. Read as one signal, a level −1 would
/// make a tunnel: a street mapped at −1 because it runs under a viaduct, with
/// no tunnel flag, would be buried. A stretch with a level and no flag is
/// therefore ground. A structure the flags do not
/// claim can still be one: the terrain derives it ([`crate::partition`]), and
/// there the geometry is the evidence rather than the ordinal.
pub fn structures(rules: &[LevelRun], flags: &[LevelRun]) -> (Vec<(f64, f64, Kind)>, Vec<(f64, f64, i64)>) {
    let same = |a: i64, b: i64| (a > 0) == (b > 0);
    let overlap = |a: &LevelRun, b: &LevelRun| (a.end.min(b.end) - a.start.max(b.start)).max(0.0);
    let off = flags
        .iter()
        .filter(|f| f.level != 0)
        .map(|f| {
            let ordinal = rules
                .iter()
                .filter(|r| r.level != 0 && same(r.level, f.level) && overlap(r, f) > 0.0)
                .max_by(|a, b| overlap(a, f).total_cmp(&overlap(b, f)))
                .map_or(f.level, |r| r.level);
            (f.start, f.end, if f.level > 0 { Kind::Bridge(ordinal) } else { Kind::Tunnel(ordinal) })
        })
        .collect();
    let mut layers = Vec::new();
    for r in rules.iter().filter(|r| r.level != 0) {
        let mut cut: Vec<(f64, f64)> =
            flags.iter().filter(|f| f.level != 0 && same(f.level, r.level)).map(|f| (f.start, f.end)).collect();
        cut.sort_by(|a, b| a.0.total_cmp(&b.0));
        let mut at = r.start;
        for (s, e) in cut {
            if e <= at || s >= r.end {
                continue;
            }
            if s - at > SPAN_EPS {
                layers.push((at, s, r.level));
            }
            at = at.max(e);
        }
        if r.end - at > SPAN_EPS {
            layers.push((at, r.end, r.level));
        }
    }
    (off, layers)
}

/// Whether a feature's properties name a way with a surface: a road class
/// under `subtype = road` (or none), or an independent railway under
/// `subtype = rail`.
pub fn keep(props: &Props) -> bool {
    let class = str_of(props, "class");
    match str_of(props, "subtype") {
        None | Some("road") => class.is_some_and(|c| CLASSES.contains(&c)),
        Some("rail") => class.is_some_and(|c| width::RAIL_CLASSES.contains(&c)),
        _ => false,
    }
}

/// Reads the ways of `path` touching `bbox`, projected into `frame` and
/// clipped to `rect`.
pub fn read(path: &Path, bbox: &Bounds, frame: &Frame, rect: &Rect) -> Result<Read, ReadError> {
    let gp = GeoParquet::open(path)?;
    let row_groups = gp.row_groups_intersecting((bbox.west, bbox.south, bbox.east, bbox.north));
    let mut out = Read::default();
    for feature in gp.features(row_groups, COLUMNS)? {
        let f = feature?;
        out.features += 1;
        if !keep(&f.properties) {
            out.street_rail += (str_of(&f.properties, "subtype") == Some("rail")) as usize;
            continue;
        }
        out.kept += 1;
        let id = str_of(&f.properties, "id").unwrap_or_default().to_string();
        let class = str_of(&f.properties, "class").unwrap_or_default().to_string();
        let subclass = str_of(&f.properties, "subclass").unwrap_or_default().to_string();
        let measured = width::measured(&class, width_rules_m(&f.properties));
        let oneway = f.properties.iter().any(|(k, v)| k == "oneway" && matches!(v, Value::Bool(true)));
        out.measured += measured.is_some() as usize;
        out.oneway += oneway as usize;
        out.rail += (width::family(&class) == width::Family::Rail) as usize;
        let width_m = width::of_way(&class, &subclass, oneway, measured);
        let (structures, layers) = structures(&f.rule_runs, &f.flag_runs);
        out.layered += !layers.is_empty() as usize;
        let off: Vec<(f64, f64, Kind)> = structures
            .into_iter()
            .chain(f.indoor_runs.iter().map(|&(s, e)| (s, e, Kind::Indoor)))
            .collect();
        let pieces = pieces_of(&off);
        if !off.is_empty() {
            out.structures += 1;
            if pieces.iter().all(|p| p.2 != Kind::Ground) {
                out.aloft += 1;
            }
        }
        for line in lines_of(&f.geometry) {
            let pts: Vec<[f64; 2]> = line.0.iter().map(|c| frame.to_local(c.x, c.y)).collect();
            let total = line::length(&pts);
            // The fractions the source speaks in, as arc along this line.
            let spans: Vec<Span> = pieces
                .iter()
                .map(|&(s, e, kind)| Span { a0: s * total, a1: e * total, kind })
                .collect();
            let layers = layers.iter().map(|&(s, e, level)| (s * total, e * total, level)).collect();
            let way = Way { id: id.clone(), class: class.clone(), subclass: subclass.clone(), width_m, pts, spans, layers };
            out.ways.extend(clip_way(&way, rect));
        }
    }
    Ok(out)
}

/// `way` clipped to `rect`: one way per run that survives, each carrying the
/// part of the span table that falls inside it, re-based on the run's own
/// start. A run with no span left is dropped — it has no geometry to name.
///
/// The clip only cuts, so a run's arc is the original's less its start, and a
/// span's arc translates by the same amount. That is the whole of it, and it
/// is why the span table is kept in arc rather than in fractions: a fraction
/// is of a length that the clip changes.
pub fn clip_way(way: &Way, rect: &Rect) -> Vec<Way> {
    let mut out = Vec::new();
    for (pts, at) in clip_runs(&way.pts, rect) {
        let len = line::length(&pts);
        let spans: Vec<Span> = way
            .spans
            .iter()
            .filter_map(|s| {
                let (a0, a1) = ((s.a0 - at).max(0.0), (s.a1 - at).min(len));
                (a1 - a0 > SPAN_EPS_M).then_some(Span { a0, a1, kind: s.kind })
            })
            .collect();
        if spans.is_empty() {
            continue;
        }
        let layers = way
            .layers
            .iter()
            .filter_map(|&(a0, a1, level)| {
                let (a0, a1) = ((a0 - at).max(0.0), (a1 - at).min(len));
                (a1 - a0 > SPAN_EPS_M).then_some((a0, a1, level))
            })
            .collect();
        out.push(Way {
            id: way.id.clone(),
            class: way.class.clone(),
            subclass: way.subclass.clone(),
            width_m: way.width_m,
            pts,
            spans,
            layers,
        });
    }
    out
}

/// Shortest span worth keeping after a clip, in metres.
const SPAN_EPS_M: f64 = 1e-9;

/// The pieces of a segment, as fractions of its length with their kind:
/// `[0, 1]` partitioned by the spans in `off`, in order, with the ground
/// between them. Where two spans overlap the earlier one holds the overlap
/// (a mapper's slop, not a stacked structure); nothing shorter than
/// `SPAN_EPS` is kept.
pub fn pieces_of(off: &[(f64, f64, Kind)]) -> Vec<(f64, f64, Kind)> {
    let mut off: Vec<(f64, f64, Kind)> =
        off.iter().map(|&(s, e, k)| (s.clamp(0.0, 1.0), e.clamp(0.0, 1.0), k)).collect();
    off.sort_by(|a, b| (a.0, a.1).partial_cmp(&(b.0, b.1)).expect("finite"));
    let mut out = Vec::new();
    let mut at = 0.0f64;
    for (s, e, kind) in off {
        if s - at > SPAN_EPS {
            out.push((at, s, Kind::Ground));
        }
        let s = s.max(at);
        if e - s > SPAN_EPS {
            out.push((s, e, kind));
        }
        at = at.max(e);
    }
    if 1.0 - at > SPAN_EPS {
        out.push((at, 1.0, Kind::Ground));
    }
    out
}

/// Shortest fraction of a segment worth a run.
const SPAN_EPS: f64 = 1e-6;

/// The line strings of a geometry; anything else is not a centreline.
fn lines_of(g: &Geometry) -> Vec<&LineString> {
    match g {
        Geometry::LineString(l) => vec![l],
        Geometry::MultiLineString(m) => m.0.iter().collect(),
        _ => Vec::new(),
    }
}

/// `pts` clipped to `rect`: the runs that remain inside, each with its **arc
/// along the original polyline** at its first vertex — what a span table has to be re-based on. Each segment is clipped
/// with Liang–Barsky; consecutive segments whose clipped parts meet are
/// joined into one run.
fn clip_runs(pts: &[[f64; 2]], rect: &Rect) -> Vec<(Vec<[f64; 2]>, f64)> {
    let mut runs: Vec<(Vec<[f64; 2]>, f64)> = Vec::new();
    let mut run: Vec<[f64; 2]> = Vec::new();
    let mut run_at = 0.0f64;
    let mut flush = |run: &mut Vec<[f64; 2]>, at: f64| {
        if run.len() >= 2 {
            runs.push((std::mem::take(run), at));
        } else {
            run.clear();
        }
    };
    let mut at = 0.0f64; // arc at `p`, along the original
    for pair in pts.windows(2) {
        let (p, q) = (pair[0], pair[1]);
        let seg = (q[0] - p[0]).hypot(q[1] - p[1]);
        let Some((t0, t1)) = liang_barsky(p, q, rect) else {
            flush(&mut run, run_at);
            at += seg;
            continue;
        };
        let a = line::lerp(p, q, t0);
        let b = line::lerp(p, q, t1);
        if run.is_empty() {
            run_at = at + t0 * seg;
            run.push(a);
        } else if t0 > 0.0 {
            flush(&mut run, run_at);
            run_at = at + t0 * seg;
            run.push(a);
        }
        if a != b {
            run.push(b);
        }
        if t1 < 1.0 {
            flush(&mut run, run_at);
        }
        at += seg;
    }
    flush(&mut run, run_at);
    runs
}

/// The parameter range `[t0, t1] ⊆ [0, 1]` of `p→q` inside `rect`, if any.
fn liang_barsky(p: [f64; 2], q: [f64; 2], rect: &Rect) -> Option<(f64, f64)> {
    let (dx, dy) = (q[0] - p[0], q[1] - p[1]);
    let (mut t0, mut t1) = (0.0f64, 1.0f64);
    for (num, den) in [
        (p[0] - rect.x0, -dx),
        (rect.x1 - p[0], dx),
        (p[1] - rect.y0, -dy),
        (rect.y1 - p[1], dy),
    ] {
        if den == 0.0 {
            if num < 0.0 {
                return None;
            }
            continue;
        }
        let t = num / den;
        if den < 0.0 {
            t0 = t0.max(t);
        } else {
            t1 = t1.min(t);
        }
    }
    (t0 <= t1).then_some((t0, t1))
}

#[cfg(test)]
mod tests {
    use arpentry_server::value::Value;

    use super::*;

    fn props(pairs: &[(&str, &str)]) -> Vec<(String, Value)> {
        pairs.iter().map(|(k, v)| (k.to_string(), Value::String(v.to_string()))).collect()
    }

    #[test]
    fn keeps_ways_with_a_surface_only() {
        assert!(keep(&props(&[("class", "residential"), ("subtype", "road")])));
        assert!(keep(&props(&[("class", "motorway")])));
        assert!(keep(&props(&[("class", "footway"), ("subtype", "road")])));
        assert!(keep(&props(&[("class", "steps"), ("subtype", "road")])));
        assert!(!keep(&props(&[("class", "rail"), ("subtype", "rail")])));
        assert!(!keep(&props(&[("subtype", "road")])));
        // The independent railways are kept; street rail and an `unknown`
        // railway are not, and a rail class is not a road class.
        assert!(keep(&props(&[("class", "standard_gauge"), ("subtype", "rail")])));
        assert!(keep(&props(&[("class", "funicular"), ("subtype", "rail")])));
        assert!(!keep(&props(&[("class", "tram"), ("subtype", "rail")])));
        assert!(!keep(&props(&[("class", "unknown"), ("subtype", "rail")])));
        assert!(!keep(&props(&[("class", "narrow_gauge"), ("subtype", "road")])));
        assert!(!keep(&props(&[("class", "primary"), ("subtype", "water")])));
    }

    /// **A flag builds; a level orders**: a street mapped under a viaduct
    /// is two layers and no structure, and the viaduct's tunnel is a tunnel
    /// at its rule's ordinal.
    #[test]
    fn a_flag_builds_and_a_level_orders() {
        let run = |start: f64, end: f64, level: i64| LevelRun { start, end, level };
        // Avenue de Naye: at +1 over a few metres and −1 for 500 m, under
        // the Viaduc de Chillon, with no flag at all. No structure: two
        // layers, which the crossing step reads and nothing else does.
        let (off, layers) = structures(&[run(0.179, 0.188, 1), run(0.188, 1.0, -1)], &[]);
        assert!(off.is_empty(), "{off:?}");
        assert_eq!(layers, vec![(0.179, 0.188, 1), (0.188, 1.0, -1)]);
        // The viaduct's Glion tunnel: flagged, and at level −5 by its rule —
        // the flag builds the tunnel, the rule gives it its ordinal.
        let (off, layers) = structures(&[run(0.058, 0.259, -5)], &[run(0.058, 0.259, -1)]);
        assert_eq!(off, vec![(0.058, 0.259, Kind::Tunnel(-5))]);
        assert!(layers.is_empty(), "{layers:?}");
        // A flag with no rule is a structure at ±1.
        let (off, _) = structures(&[], &[run(0.2, 0.4, 1)]);
        assert_eq!(off, vec![(0.2, 0.4, Kind::Bridge(1))]);
        // A rule longer than its flag: the flag's stretch is the tunnel and
        // the rest of the rule is a layer either side of it.
        let (off, layers) = structures(&[run(0.0, 1.0, -1)], &[run(0.3, 0.6, -1)]);
        assert_eq!(off, vec![(0.3, 0.6, Kind::Tunnel(-1))]);
        assert_eq!(layers, vec![(0.0, 0.3, -1), (0.6, 1.0, -1)]);
        // A bridge flag says nothing about a level below the ground.
        let (off, layers) = structures(&[run(0.0, 1.0, -1)], &[run(0.3, 0.6, 1)]);
        assert_eq!(off, vec![(0.3, 0.6, Kind::Bridge(1))]);
        assert_eq!(layers, vec![(0.0, 1.0, -1)]);
    }

    #[test]
    fn lines_of_flattens_multilines() {
        let l = LineString::from(vec![(0.0, 0.0), (1.0, 1.0)]);
        assert_eq!(lines_of(&Geometry::LineString(l.clone())).len(), 1);
        let m = geo_types::MultiLineString(vec![l.clone(), l]);
        assert_eq!(lines_of(&Geometry::MultiLineString(m)).len(), 2);
        assert!(lines_of(&Geometry::Point((0.0, 0.0).into())).is_empty());
    }

    fn unit() -> Rect {
        Rect { x0: 0.0, y0: 0.0, x1: 10.0, y1: 10.0 }
    }

    /// The runs of `pts` inside `rect`, without their arcs.
    fn clip(pts: &[[f64; 2]], rect: &Rect) -> Vec<Vec<[f64; 2]>> {
        clip_runs(pts, rect).into_iter().map(|(run, _)| run).collect()
    }

    #[test]
    fn inside_stays_one_run() {
        let pts = [[1.0, 1.0], [5.0, 2.0], [9.0, 9.0]];
        assert_eq!(clip(&pts, &unit()), vec![pts.to_vec()]);
    }

    #[test]
    fn crossing_is_cut_at_the_edge() {
        let runs = clip(&[[-5.0, 5.0], [5.0, 5.0], [15.0, 5.0]], &unit());
        assert_eq!(runs, vec![vec![[0.0, 5.0], [5.0, 5.0], [10.0, 5.0]]]);
    }

    #[test]
    fn leaving_and_returning_makes_two_runs() {
        let runs = clip(&[[2.0, 2.0], [2.0, 15.0], [8.0, 15.0], [8.0, 2.0]], &unit());
        assert_eq!(runs, vec![vec![[2.0, 2.0], [2.0, 10.0]], vec![[8.0, 10.0], [8.0, 2.0]]]);
    }

    #[test]
    fn outside_yields_nothing() {
        assert!(clip(&[[-5.0, -5.0], [-1.0, 20.0]], &unit()).is_empty());
        assert!(clip(&[[20.0, 0.0], [30.0, 0.0]], &unit()).is_empty());
    }

    #[test]
    fn a_way_is_cut_into_pieces_by_kind() {
        use Kind::*;
        assert_eq!(pieces_of(&[]), vec![(0.0, 1.0, Ground)]);
        assert_eq!(pieces_of(&[(0.0, 1.0, Tunnel(-1))]), vec![(0.0, 1.0, Tunnel(-1))]);
        assert_eq!(
            pieces_of(&[(0.3, 0.6, Bridge(1))]),
            vec![(0.0, 0.3, Ground), (0.3, 0.6, Bridge(1)), (0.6, 1.0, Ground)]
        );
        // Overlapping spans: the earlier holds the overlap; touching ones
        // leave no ground between them; order does not matter.
        assert_eq!(
            pieces_of(&[(0.5, 0.7, Tunnel(-1)), (0.2, 0.55, Bridge(1)), (0.9, 1.0, Indoor)]),
            vec![(0.0, 0.2, Ground), (0.2, 0.55, Bridge(1)), (0.55, 0.7, Tunnel(-1)), (0.7, 0.9, Ground), (0.9, 1.0, Indoor)]
        );
        assert_eq!(
            pieces_of(&[(0.0, 0.5, Bridge(1)), (0.5, 1.0, Bridge(2))]),
            vec![(0.0, 0.5, Bridge(1)), (0.5, 1.0, Bridge(2))]
        );
        // A span swallowed by an earlier one leaves no piece.
        assert_eq!(pieces_of(&[(0.2, 0.8, Bridge(1)), (0.3, 0.4, Tunnel(-1))]).len(), 3);
    }

    #[test]
    fn a_corner_touch_is_not_a_run() {
        // Passes through the corner (10, 10) exactly: one point, no length.
        assert!(clip(&[[5.0, 15.0], [15.0, 5.0]], &unit()).is_empty());
    }
}
