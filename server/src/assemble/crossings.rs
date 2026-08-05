//! Geometric crossing detection (docs/GENERATION.md D4, scenario S4).
//!
//! No link in the data connects an overpass to the road it crosses; the
//! crossing must be found geometrically. The *bridge* spans of every corridor
//! are indexed in a uniform grid, and a second pass over the transportation
//! input tests every feature's edges against the nearby span edges. A plan
//! intersection on a bridge span whose level exceeds the feature's is a
//! [`Crossing`]: the deck must clear it.
//!
//! Two intersections are neither:
//! - **Junctions**: features that share a connector meet, they don't pass
//!   over each other (a ramp joining a viaduct touches it in plan).
//! - **Self-crossings**: a corridor looping over itself (rare ramp loops) is
//!   deferred; skipped by corridor identity.

use std::collections::HashSet;
use std::path::Path;

use geo_types::{Coord, Geometry};

use crate::geoparquet::{GeoParquet, ReadError};
use crate::levels::LevelRun;
use crate::priors::Kind;
use crate::scene::{run_cos_lat, Crossing, SceneGraph, SpanKind};
use crate::value::Value;

use super::grid::GridIndex;

/// One indexed bridge-span edge.
struct SpanEdge {
    corridor: u32,
    /// Corridor node index; the edge spans `nodes[i]..nodes[i+1]`.
    node: usize,
    level: i64,
}

/// The corridors' bridge-span edges in a grid, shared by the road and water
/// passes.
struct SpanIndex {
    edges: Vec<SpanEdge>,
    grid: GridIndex,
}

fn index_spans(scene: &SceneGraph) -> SpanIndex {
    let mut edges: Vec<SpanEdge> = Vec::new();
    let mut grid = GridIndex::new();
    for c in &scene.corridors {
        for span in c.spans.iter().filter(|s| s.kind == SpanKind::Bridge) {
            for i in 0..c.nodes.len() - 1 {
                // Edge overlaps the span's arc interval.
                if c.arc[i + 1] <= span.arc0 || c.arc[i] >= span.arc1 {
                    continue;
                }
                let (a, b) = (c.nodes[i], c.nodes[i + 1]);
                let bb = (a.x.min(b.x), a.y.min(b.y), a.x.max(b.x), a.y.max(b.y));
                grid.insert(bb, edges.len() as u32);
                edges.push(SpanEdge { corridor: c.id, node: i, level: span.level });
            }
        }
    }
    SpanIndex { edges, grid }
}

/// Detects crossings — bridge spans passing over features — streaming the
/// input a second time.
pub fn detect(
    path: &Path,
    bbox: (f64, f64, f64, f64),
    scene: &SceneGraph,
) -> Result<Vec<Crossing>, ReadError> {
    let SpanIndex { edges, grid } = index_spans(scene);
    if grid.is_empty() {
        return Ok(Vec::new());
    }

    let gp = GeoParquet::open(path)?;
    let row_groups = gp.row_groups_intersecting(bbox);
    let mut crossings: Vec<Crossing> = Vec::new();
    let mut seen: HashSet<(u32, u64, i64, i64)> = HashSet::new();
    let mut candidates: Vec<u32> = Vec::new();
    for feature in gp.features(row_groups, super::ATTRS)? {
        let f = feature?;
        let Geometry::LineString(ref line) = f.geometry else {
            continue;
        };
        if line.0.len() < 2 {
            continue;
        }
        let source = prop_str(&f.properties, "id").map(crate::scene::source_hash);
        let other_corridor = source.and_then(|h| scene.lookup(h)).map(|(c, _)| c.id);
        // The crossed feature's own §9 key: it is *its* `clearance_over_m`
        // the deck above owes, so the prior is read from the thing being
        // crossed, not from the thing crossing.
        let other_kind = Kind::parse(
            prop_str(&f.properties, "subtype").as_deref(),
            prop_str(&f.properties, "class").as_deref(),
            prop_str(&f.properties, "subclass").as_deref(),
        );
        // Fraction positions along the feature, for reading its level at an
        // intersection. Computed lazily — most features never near a span.
        let mut cum: Option<(Vec<f64>, f64)> = None;

        for (si, w) in line.0.windows(2).enumerate() {
            let (c0, c1) = (w[0], w[1]);
            let bb = (c0.x.min(c1.x), c0.y.min(c1.y), c0.x.max(c1.x), c0.y.max(c1.y));
            grid.query(bb, &mut candidates);
            for &ei in candidates.iter() {
                let e = &edges[ei as usize];
                let upper = &scene.corridors[e.corridor as usize];
                if other_corridor == Some(upper.id) {
                    continue; // its own corridor: adjacency, not a crossing
                }
                let (a, b) = (upper.nodes[e.node], upper.nodes[e.node + 1]);
                let Some((t_span, t_seg)) = seg_intersect(a, b, c0, c1, upper.cos_lat) else {
                    continue;
                };
                // Junction, not a crossing: the two share a graph node.
                if f.connectors.iter().any(|c| upper.connectors.binary_search(&c.id).is_ok()) {
                    continue;
                }
                let (cum_d, total) = cum.get_or_insert_with(|| cumulative(&line.0));
                let frac = if *total > 0.0 {
                    (cum_d[si] + t_seg * (cum_d[si + 1] - cum_d[si])) / *total
                } else {
                    0.0
                };
                let feature_level = level_at(&f.level_runs, frac);
                // A bridge span passes over lower-level features. A same-level
                // braid, or the reverse pair's report, is not a crossing.
                if feature_level >= e.level {
                    continue;
                }
                let point = Coord { x: a.x + (b.x - a.x) * t_span, y: a.y + (b.y - a.y) * t_span };
                let span_arc =
                    upper.arc[e.node] + t_span * (upper.arc[e.node + 1] - upper.arc[e.node]);
                // One record per (span, feature, level pair): a shared vertex
                // of two adjacent edges would otherwise report twice.
                let key = (upper.id, source.unwrap_or(0), e.level, feature_level);
                if !seen.insert(key) {
                    continue;
                }
                crossings.push(Crossing {
                    upper: upper.id,
                    upper_arc: span_arc,
                    point,
                    lower: other_corridor,
                    lower_kind: other_kind,
                    upper_level: e.level,
                    lower_level: feature_level,
                });
            }
        }
    }
    // Deterministic order for the solver, whatever the scan order was.
    crossings.sort_by(|a, b| {
        (a.upper, a.upper_arc.to_bits(), a.lower_level)
            .cmp(&(b.upper, b.upper_arc.to_bits(), b.lower_level))
    });
    Ok(crossings)
}

/// Detects bridge spans crossing water features (rivers, canals, lakes) in
/// the water input: the freeboard constraint of scenario S3. The water
/// surface itself needs no solving — the DEM images water bodies at their
/// level, so the terrain under the crossing *is* the water height.
pub fn detect_water(
    path: &Path,
    bbox: (f64, f64, f64, f64),
    scene: &SceneGraph,
) -> Result<Vec<Crossing>, ReadError> {
    let SpanIndex { edges, grid } = index_spans(scene);
    if grid.is_empty() {
        return Ok(Vec::new());
    }

    let gp = GeoParquet::open(path)?;
    let row_groups = gp.row_groups_intersecting(bbox);
    let mut crossings: Vec<Crossing> = Vec::new();
    let mut seen: HashSet<(u32, u64, i64)> = HashSet::new();
    let mut candidates: Vec<u32> = Vec::new();
    for feature in gp.features(row_groups, &["id", "subtype", "class"])? {
        let f = feature?;
        let source =
            prop_str(&f.properties, "id").map(crate::scene::source_hash).unwrap_or_default();
        for part in water_lines(&f.geometry) {
            for w in part.windows(2) {
                let (c0, c1) = (w[0], w[1]);
                let bb = (c0.x.min(c1.x), c0.y.min(c1.y), c0.x.max(c1.x), c0.y.max(c1.y));
                grid.query(bb, &mut candidates);
                for &ei in candidates.iter() {
                    let e = &edges[ei as usize];
                    let upper = &scene.corridors[e.corridor as usize];
                    let (a, b) = (upper.nodes[e.node], upper.nodes[e.node + 1]);
                    let Some((t_span, _)) = seg_intersect(a, b, c0, c1, upper.cos_lat) else {
                        continue;
                    };
                    if !seen.insert((upper.id, source, e.level)) {
                        continue; // one constraint per (span, water body)
                    }
                    let point =
                        Coord { x: a.x + (b.x - a.x) * t_span, y: a.y + (b.y - a.y) * t_span };
                    crossings.push(Crossing {
                        upper: upper.id,
                        upper_arc: upper.arc[e.node]
                            + t_span * (upper.arc[e.node + 1] - upper.arc[e.node]),
                        point,
                        lower: None,
                        lower_kind: Kind::Water(crate::priors::WaterClass::Still),
                        upper_level: e.level,
                        lower_level: 0,
                    });
                }
            }
        }
    }
    crossings.sort_by(|a, b| (a.upper, a.upper_arc.to_bits()).cmp(&(b.upper, b.upper_arc.to_bits())));
    Ok(crossings)
}

/// The polylines of a water feature: line geometry as-is, polygon rings
/// (banks and shorelines) as closed lines.
fn water_lines(g: &Geometry) -> Vec<Vec<Coord>> {
    fn rings(p: &geo_types::Polygon) -> Vec<Vec<Coord>> {
        std::iter::once(p.exterior())
            .chain(p.interiors().iter())
            .map(|r| r.0.clone())
            .collect()
    }
    match g {
        Geometry::LineString(ls) => vec![ls.0.clone()],
        Geometry::MultiLineString(mls) => mls.0.iter().map(|l| l.0.clone()).collect(),
        Geometry::Polygon(p) => rings(p),
        Geometry::MultiPolygon(mp) => mp.0.iter().flat_map(rings).collect(),
        _ => Vec::new(),
    }
}

/// Proper intersection of segments `ab` and `cd` in cos-lat-scaled space,
/// returning the parameters along each. Parallel or disjoint → `None`.
fn seg_intersect(a: Coord, b: Coord, c: Coord, d: Coord, cos_lat: f64) -> Option<(f64, f64)> {
    let (rx, ry) = ((b.x - a.x) * cos_lat, b.y - a.y);
    let (sx, sy) = ((d.x - c.x) * cos_lat, d.y - c.y);
    let denom = rx * sy - ry * sx;
    if denom.abs() < 1e-18 {
        return None;
    }
    let (qx, qy) = ((c.x - a.x) * cos_lat, c.y - a.y);
    let t = (qx * sy - qy * sx) / denom;
    let u = (qx * ry - qy * rx) / denom;
    ((0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u)).then_some((t, u))
}

/// Cumulative cos-lat-scaled length at each vertex, and the total.
fn cumulative(pts: &[Coord]) -> (Vec<f64>, f64) {
    let cos_lat = run_cos_lat(pts);
    let mut cum = Vec::with_capacity(pts.len());
    let mut acc = 0.0;
    cum.push(0.0);
    for w in pts.windows(2) {
        let dx = (w[1].x - w[0].x) * cos_lat;
        let dy = w[1].y - w[0].y;
        acc += (dx * dx + dy * dy).sqrt();
        cum.push(acc);
    }
    (cum, acc)
}

/// The level at fractional position `t`, or 0 (ground) if no rule covers it.
fn level_at(runs: &[LevelRun], t: f64) -> i64 {
    runs.iter().rev().find(|r| t >= r.start && t <= r.end).map_or(0, |r| r.level)
}

fn prop_str<'a>(props: &'a [(String, Value)], key: &str) -> Option<&'a str> {
    props.iter().find(|(k, _)| k == key).and_then(|(_, v)| match v {
        Value::String(s) => Some(s.as_str()),
        _ => None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn segments_intersect_where_they_cross() {
        let cos = 1.0;
        let got = seg_intersect(
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 1.0, y: 1.0 },
            Coord { x: 0.0, y: 1.0 },
            Coord { x: 1.0, y: 0.0 },
            cos,
        );
        let (t, u) = got.expect("crossing diagonals intersect");
        assert!((t - 0.5).abs() < 1e-12 && (u - 0.5).abs() < 1e-12);
        assert!(seg_intersect(
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 1.0, y: 0.0 },
            Coord { x: 0.0, y: 1.0 },
            Coord { x: 1.0, y: 1.0 },
            cos,
        )
        .is_none(), "parallel segments never intersect");
        assert!(seg_intersect(
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 0.2, y: 0.2 },
            Coord { x: 0.0, y: 1.0 },
            Coord { x: 1.0, y: 0.0 },
            cos,
        )
        .is_none(), "short segment stops before the crossing");
    }

    #[test]
    fn level_at_prefers_the_last_matching_rule() {
        let runs = vec![
            LevelRun { start: 0.0, end: 1.0, level: 1 },
            LevelRun { start: 0.4, end: 0.6, level: 2 },
        ];
        assert_eq!(level_at(&runs, 0.5), 2);
        assert_eq!(level_at(&runs, 0.1), 1);
        assert_eq!(level_at(&[], 0.5), 0);
    }
}
