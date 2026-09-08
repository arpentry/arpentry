//! Way centrelines from the transportation source — roads and pedestrian
//! ways alike — in the local frame, clipped to the bbox.
//!
//! This is a reader, not a model: no joining into corridors, no widths.
//! Those are later steps, each of which will be built against what this one
//! returns. It carries `class` and `subclass` because the width function
//! ([`crate::width`]) is keyed on them, and `id` so a line in the output can
//! be traced to its source feature.
//!
//! **Every way is cut by kind.** Overture encodes a bridge or a tunnel as a
//! span of a segment — `level_rules`, or the `is_bridge`/`is_tunnel` flags in
//! `road_flags`, over a `[start, end]` fraction of its length — and a way
//! inside a building as an `is_indoor` span. The reader cuts the way at
//! every span boundary and emits every piece with its [`Kind`]: the ground
//! pieces into [`Read::lines`], from which every surface is built, and the
//! rest into [`Read::spans`], which the profile chords across. A segment
//! that climbs onto a viaduct keeps the stretch before the abutment on the
//! ground and the deck off it, and the two share the abutment's vertex.
//! The surface steps see only the ground, so they union freely: nothing
//! they see crosses anything else at a different level.

use std::path::Path;

use arpentry_server::geoparquet::{GeoParquet, ReadError};
use arpentry_server::project::Bounds;
use arpentry_server::value::{str_of, width_rules_m, Props, Value};

use crate::width;
use geo_types::{Geometry, LineString};

use crate::frame::{Frame, Rect};
use crate::world::{Kind, Polyline2};

/// The columns read. `subtype` and `class` decide admission.
///
/// `subclass` is the scalar column, which Overture fills only when the value
/// is uniform along the segment: a footway that is a sidewalk over part of its
/// length and a crossing over the rest has `subclass = NULL` (docs/SOURCES.md).
/// Reading `subclass_rules` is a later step; until then such a way is an
/// anonymous footway.
pub const COLUMNS: &[&str] = &[
    "id",
    "class",
    "subtype",
    "subclass",
    "level_rules",
    "road_flags",
    "width_rules",
    "access_restrictions",
];

/// The way classes kept: every class [`crate::width::of`] gives a width. Rail
/// and water are out, as is anything with no surface a person or a car
/// stands on.
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
    /// The pieces on the ground.
    pub lines: Vec<Polyline2>,
    /// The pieces above or below it, or indoors.
    pub spans: Vec<Polyline2>,
    /// Features decoded from the row groups touching the bbox.
    pub features: usize,
    /// Of those, the ways kept.
    pub kept: usize,
    /// Of the kept, the ways with a span above or below the ground, or
    /// indoors.
    pub structures: usize,
    /// Of those, the ways with no ground left at all.
    pub dropped: usize,
    /// Of the kept, the ways whose width is measured (`width_rules`).
    pub measured: usize,
    /// Of the kept, the one-way carriageways.
    pub oneway: usize,
}

/// Whether a feature's properties name a way with a surface.
pub fn keep(props: &Props) -> bool {
    let subtype_ok = matches!(str_of(props, "subtype"), None | Some("road"));
    subtype_ok && str_of(props, "class").is_some_and(|c| CLASSES.contains(&c))
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
        let width_m = width::of_way(&class, &subclass, oneway, measured);
        let off: Vec<(f64, f64, Kind)> = f
            .level_runs
            .iter()
            .map(|r| (r.start, r.end, if r.level > 0 { Kind::Bridge(r.level) } else { Kind::Tunnel(r.level) }))
            .chain(f.indoor_runs.iter().map(|&(s, e)| (s, e, Kind::Indoor)))
            .collect();
        let pieces = pieces_of(&off);
        if !off.is_empty() {
            out.structures += 1;
            if pieces.iter().all(|p| p.2 != Kind::Ground) {
                out.dropped += 1;
            }
        }
        for line in lines_of(&f.geometry) {
            let pts: Vec<[f64; 2]> = line.0.iter().map(|c| frame.to_local(c.x, c.y)).collect();
            for &(s, e, kind) in &pieces {
                for run in clip(&cut(&pts, s, e), rect) {
                    let piece = Polyline2 {
                        id: id.clone(),
                        class: class.clone(),
                        subclass: subclass.clone(),
                        width_m,
                        kind,
                        pts: run,
                    };
                    if kind == Kind::Ground {
                        out.lines.push(piece);
                    } else {
                        out.spans.push(piece);
                    }
                }
            }
        }
    }
    Ok(out)
}

/// The pieces of a segment, as fractions of its length with their kind:
/// `[0, 1]` partitioned by the spans in `off`, in order, with the ground
/// between them. Where two spans overlap the earlier one holds the overlap
/// (a mapper's slop, not a stacked structure); nothing shorter than
/// [`SPAN_EPS`] is kept.
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

/// The part of `pts` between the fractions `s` and `e` of its length: the
/// linear referencing Overture's `between` speaks in, measured along the
/// polyline. The whole line for `[0, 1]`; empty for an empty span.
pub fn cut(pts: &[[f64; 2]], s: f64, e: f64) -> Vec<[f64; 2]> {
    if pts.len() < 2 || e - s <= SPAN_EPS {
        return Vec::new();
    }
    if s <= 0.0 && e >= 1.0 {
        return pts.to_vec();
    }
    let lens: Vec<f64> = pts.windows(2).map(|p| (p[1][0] - p[0][0]).hypot(p[1][1] - p[0][1])).collect();
    let total: f64 = lens.iter().sum();
    if total <= 0.0 {
        return Vec::new();
    }
    let (d0, d1) = (s * total, e * total);
    let mut out: Vec<[f64; 2]> = Vec::new();
    let mut at = 0.0;
    for (i, &len) in lens.iter().enumerate() {
        let (p, q) = (pts[i], pts[i + 1]);
        let next = at + len;
        if next < d0 - 1e-12 {
            at = next;
            continue;
        }
        if at > d1 + 1e-12 {
            break;
        }
        if out.is_empty() {
            let t = if len > 0.0 { ((d0 - at) / len).clamp(0.0, 1.0) } else { 0.0 };
            out.push(lerp(p, q, t));
        }
        if next <= d1 + 1e-12 {
            if out.last() != Some(&q) {
                out.push(q);
            }
        } else {
            let t = if len > 0.0 { ((d1 - at) / len).clamp(0.0, 1.0) } else { 1.0 };
            let end = lerp(p, q, t);
            if out.last() != Some(&end) {
                out.push(end);
            }
            break;
        }
        at = next;
    }
    if out.len() < 2 {
        Vec::new()
    } else {
        out
    }
}

/// The line strings of a geometry; anything else is not a centreline.
pub fn lines_of(g: &Geometry) -> Vec<&LineString> {
    match g {
        Geometry::LineString(l) => vec![l],
        Geometry::MultiLineString(m) => m.0.iter().collect(),
        _ => Vec::new(),
    }
}

/// Clips a polyline to `rect`, returning the runs that remain inside. Each
/// segment is clipped with Liang–Barsky; consecutive segments whose clipped
/// parts meet are joined into one run.
pub fn clip(pts: &[[f64; 2]], rect: &Rect) -> Vec<Vec<[f64; 2]>> {
    let mut runs: Vec<Vec<[f64; 2]>> = Vec::new();
    let mut run: Vec<[f64; 2]> = Vec::new();
    let mut flush = |run: &mut Vec<[f64; 2]>| {
        if run.len() >= 2 {
            runs.push(std::mem::take(run));
        } else {
            run.clear();
        }
    };
    for pair in pts.windows(2) {
        let (p, q) = (pair[0], pair[1]);
        let Some((t0, t1)) = liang_barsky(p, q, rect) else {
            flush(&mut run);
            continue;
        };
        let a = lerp(p, q, t0);
        let b = lerp(p, q, t1);
        if run.is_empty() {
            run.push(a);
        } else if t0 > 0.0 {
            flush(&mut run);
            run.push(a);
        }
        if a != b {
            run.push(b);
        }
        if t1 < 1.0 {
            flush(&mut run);
        }
    }
    flush(&mut run);
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

fn lerp(p: [f64; 2], q: [f64; 2], t: f64) -> [f64; 2] {
    [p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t]
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
        assert!(!keep(&props(&[("class", "primary"), ("subtype", "water")])));
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
    fn a_cut_measures_along_the_line() {
        // Two legs of 10 m: fractions are of the 20 m total.
        let pts = [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0]];
        assert_eq!(cut(&pts, 0.0, 1.0), pts.to_vec());
        assert_eq!(cut(&pts, 0.25, 0.75), vec![[5.0, 0.0], [10.0, 0.0], [10.0, 5.0]]);
        assert_eq!(cut(&pts, 0.0, 0.5), vec![[0.0, 0.0], [10.0, 0.0]]);
        assert_eq!(cut(&pts, 0.5, 1.0), vec![[10.0, 0.0], [10.0, 10.0]]);
        assert_eq!(cut(&pts, 0.6, 0.8), vec![[10.0, 2.0], [10.0, 6.0]]);
        assert!(cut(&pts, 0.5, 0.5).is_empty());
        assert!(cut(&[[0.0, 0.0]], 0.0, 1.0).is_empty());
    }

    #[test]
    fn a_corner_touch_is_not_a_run() {
        // Passes through the corner (10, 10) exactly: one point, no length.
        assert!(clip(&[[5.0, 15.0], [15.0, 5.0]], &unit()).is_empty());
    }
}
