//! Overture transportation `level_rules` — a linearly-referenced z-order over a
//! road segment — and splitting a segment into runs of constant level.
//!
//! Overture encodes bridges and tunnels not as whole segments but as *spans* of
//! one: a `list<struct<value, between>>` where `value` is the relative vertical
//! level (positive = bridge/elevated deck, negative = tunnel, 0/absent = ground)
//! and `between` is the `[start, end]` fraction of the segment the rule covers.
//! A single motorway segment can run at grade, climb onto a bridge, dive into a
//! tunnel, and surface again — all under one geometry.
//!
//! The tiler treats a level as uniform per feature (it lifts a bridge deck and
//! sinks a tunnel as a whole), so a mixed segment must be cut into maximal
//! constant-level pieces before tiling. [`split_levels`] does that, returning
//! each piece with its level; the ground gaps between rules come back as level-0
//! pieces. The cut points are fractions of the segment's geodesic (metric)
//! length, matching how Overture computes the `between` positions (its splitter
//! resolves them against `ST_LengthSpheroid`).

use geo_types::{Coord, Geometry, LineString};

use arrow::array::{Array, Float64Array, Int32Array, ListArray, StructArray};

/// Smallest fraction treated as a non-empty span; cuts closer than this collapse
/// to a shared vertex rather than spawning a degenerate piece.
const EPS: f64 = 1e-9;

/// A level-0 sliver shorter than this (metres), sandwiched between two structure
/// runs, is treated as a rule-edge mismatch rather than real at-grade road, and
/// dropped so the structures abut. Genuine at-grade stretches between structures
/// are far longer.
const SNAP_RUN_M: f64 = 10.0;

/// Metres per degree of arc, converting [`cumulative`]'s scaled-degree lengths to
/// metres for the [`SNAP_RUN_M`] sliver test.
const DEG_M: f64 = 111_320.0;

/// One rule from `level_rules`: a constant level over the fractional span
/// `[start, end]` (0..1) of a segment. Ground rules (level 0) are dropped at
/// parse time — ground is the implicit default between and around the runs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LevelRun {
    pub start: f64,
    pub end: f64,
    pub level: i64,
}

/// Parses an Overture `level_rules` cell into its non-ground runs, in source
/// order. Returns empty for a null cell, a non-`list<struct<value, between>>`
/// shape, or an all-ground segment — every case the caller treats as "no
/// structure to split on".
pub fn parse(array: &dyn Array, row: usize) -> Vec<LevelRun> {
    parse_inner(array, row).unwrap_or_default()
}

fn parse_inner(array: &dyn Array, row: usize) -> Option<Vec<LevelRun>> {
    if array.is_null(row) {
        return Some(Vec::new());
    }
    let list = array.as_any().downcast_ref::<ListArray>()?;
    let rules = list.value(row);
    let st = rules.as_any().downcast_ref::<StructArray>()?;
    let values = st.column_by_name("value")?.as_any().downcast_ref::<Int32Array>()?;
    let between = st.column_by_name("between")?.as_any().downcast_ref::<ListArray>()?;

    let mut runs = Vec::new();
    for i in 0..st.len() {
        let level = values.value(i) as i64;
        if level == 0 {
            continue; // ground span — implicit, nothing to split on
        }
        // A missing `between` means the rule covers the whole segment.
        let (start, end) = if between.is_null(i) {
            (0.0, 1.0)
        } else {
            let b = between.value(i);
            match b.as_any().downcast_ref::<Float64Array>() {
                Some(a) if a.len() >= 2 => (a.value(0), a.value(1)),
                _ => (0.0, 1.0),
            }
        };
        let (lo, hi) = (start.min(end).clamp(0.0, 1.0), start.max(end).clamp(0.0, 1.0));
        if hi - lo > EPS {
            runs.push(LevelRun { start: lo, end: hi, level });
        }
    }
    Some(runs)
}

/// Splits a segment into maximal runs of constant level, returning each piece
/// with its level (0 for the ground spans between and around the rules). The
/// cuts fall at the rule edges, measured as fractions of the linestring's
/// geodesic length. Non-linestring geometry and degenerate input fall back to a single
/// piece at the [`dominant`] level — there is nothing to linearly reference.
pub fn split_levels(geom: &Geometry, runs: &[LevelRun]) -> Vec<(Geometry, i64)> {
    let Geometry::LineString(line) = geom else {
        return vec![(geom.clone(), dominant(runs))];
    };
    if line.0.len() < 2 || runs.is_empty() {
        return vec![(geom.clone(), dominant(runs))];
    }
    let cum = cumulative(&line.0);
    let total = *cum.last().expect("non-empty");
    if total <= 0.0 {
        return vec![(geom.clone(), dominant(runs))];
    }

    // Breakpoints partitioning [0, 1]: the segment ends plus every rule edge.
    let mut breaks = vec![0.0_f64, 1.0];
    for r in runs {
        breaks.push(r.start);
        breaks.push(r.end);
    }
    breaks.sort_by(|a, b| a.partial_cmp(b).expect("finite breakpoints"));
    breaks.dedup();

    // Coalesce neighbouring intervals that share a level so a road that stays a
    // bridge across two adjacent rules emits one piece, not two.
    let mut intervals: Vec<(f64, f64, i64)> = Vec::new();
    for w in breaks.windows(2) {
        let (b0, b1) = (w[0], w[1]);
        if b1 - b0 <= EPS {
            continue;
        }
        let level = level_at(runs, 0.5 * (b0 + b1));
        match intervals.last_mut() {
            Some(last) if last.2 == level && (b0 - last.1).abs() <= EPS => last.1 = b1,
            _ => intervals.push((b0, b1, level)),
        }
    }

    // Overture's rule edges don't always meet: a tunnel ending at 0.2588 and a
    // bridge starting at 0.2591 leave a ~2 m phantom at-grade sliver that would
    // drape as a round-capped stub poking between the two solids. A real at-grade
    // stretch between structures is far longer; a sub-[`SNAP_RUN_M`] level-0
    // sliver flanked by two structures is a rule-edge mismatch, so drop it and let
    // the structures abut at its midpoint.
    let mut i = 1;
    while i + 1 < intervals.len() {
        let (s0, s1, level) = intervals[i];
        let sliver = level == 0
            && (s1 - s0) * total * DEG_M < SNAP_RUN_M
            && intervals[i - 1].2 != 0
            && intervals[i + 1].2 != 0;
        if sliver {
            let mid = 0.5 * (s0 + s1);
            intervals[i - 1].1 = mid;
            intervals[i + 1].0 = mid;
            intervals.remove(i);
        } else {
            i += 1;
        }
    }

    let mut pieces = Vec::with_capacity(intervals.len());
    for (t0, t1, level) in intervals {
        let pts = substring(&line.0, &cum, t0 * total, t1 * total);
        if pts.len() >= 2 {
            pieces.push((Geometry::LineString(LineString(pts)), level));
        }
    }
    if pieces.is_empty() {
        return vec![(geom.clone(), dominant(runs))];
    }
    pieces
}

/// The single level best representing a whole segment: the non-ground level with
/// the widest single run, unless ground covers more of the segment. The fallback
/// for geometry that can't be linearly referenced (non-linestrings).
pub fn dominant(runs: &[LevelRun]) -> i64 {
    let mut ruled = 0.0_f64;
    let mut best_level = 0_i64;
    let mut best_cov = 0.0_f64;
    for r in runs {
        let span = (r.end - r.start).abs();
        ruled += span;
        if r.level != 0 && span > best_cov {
            best_cov = span;
            best_level = r.level;
        }
    }
    let ground = (1.0 - ruled).max(0.0);
    if best_level == 0 || ground >= best_cov {
        0
    } else {
        best_level
    }
}

/// The level at fractional position `t`, or 0 (ground) if no rule covers it.
/// On overlap the last matching rule wins, mirroring source order.
pub fn level_at(runs: &[LevelRun], t: f64) -> i64 {
    runs.iter()
        .rev()
        .find(|r| t >= r.start && t <= r.end)
        .map_or(0, |r| r.level)
}

/// Cumulative geodesic length at each vertex, approximated locally by scaling
/// longitude by `cos(mean latitude)`; `cum[0] == 0`. Overture measures the
/// `between` fractions along the segment's spheroid length — its splitter uses
/// `ST_LengthSpheroid` — so the fractions must ride metric arc length, not raw
/// degrees: at 46° latitude a longitude degree is only ~0.69 of a latitude
/// degree, and ignoring that shifts every cut toward the east-west legs. Only
/// relative length matters here (cuts are fractions of the total), so the metres
/// constant cancels and just the longitude scaling is applied.
fn cumulative(pts: &[Coord]) -> Vec<f64> {
    let mean_lat = pts.iter().map(|c| c.y).sum::<f64>() / pts.len() as f64;
    let cos_lat = mean_lat.to_radians().cos();
    let mut cum = Vec::with_capacity(pts.len());
    let mut acc = 0.0;
    cum.push(0.0);
    for w in pts.windows(2) {
        let dx = (w[1].x - w[0].x) * cos_lat;
        let dy = w[1].y - w[0].y;
        acc += (dx * dx + dy * dy).sqrt();
        cum.push(acc);
    }
    cum
}

/// The point at arc length `d` along the polyline, interpolated within the
/// containing segment and clamped to the line's extent.
fn point_at(pts: &[Coord], cum: &[f64], d: f64) -> Coord {
    let total = *cum.last().expect("non-empty");
    let d = d.clamp(0.0, total);
    for i in 0..pts.len() - 1 {
        if d <= cum[i + 1] {
            let seg = cum[i + 1] - cum[i];
            let t = if seg > 0.0 { (d - cum[i]) / seg } else { 0.0 };
            return Coord {
                x: pts[i].x + (pts[i + 1].x - pts[i].x) * t,
                y: pts[i].y + (pts[i + 1].y - pts[i].y) * t,
            };
        }
    }
    *pts.last().expect("non-empty")
}

/// The portion of the polyline between arc lengths `d0` and `d1`: the two
/// interpolated cut points plus every original vertex strictly between them.
/// Adjacent duplicates (a cut landing on a vertex) are collapsed.
fn substring(pts: &[Coord], cum: &[f64], d0: f64, d1: f64) -> Vec<Coord> {
    let mut out = vec![point_at(pts, cum, d0)];
    for i in 0..pts.len() {
        if cum[i] > d0 && cum[i] < d1 {
            out.push(pts[i]);
        }
    }
    out.push(point_at(pts, cum, d1));
    out.dedup();
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use arrow::array::{ArrayRef, Float64Array, Int32Array, ListArray, StructArray};
    use arrow::buffer::{OffsetBuffer, ScalarBuffer};
    use arrow::datatypes::{DataType, Field, Fields};
    use std::sync::Arc;

    fn run(start: f64, end: f64, level: i64) -> LevelRun {
        LevelRun { start, end, level }
    }

    /// A horizontal line from (0,0) to (10,0): arc length == x, so fractions map
    /// straight to x coordinates.
    fn unit_line() -> Geometry {
        Geometry::LineString(LineString(vec![
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 10.0, y: 0.0 },
        ]))
    }

    fn xs(geom: &Geometry) -> Vec<f64> {
        match geom {
            Geometry::LineString(ls) => ls.0.iter().map(|c| c.x).collect(),
            _ => panic!("expected linestring"),
        }
    }

    #[test]
    fn no_runs_yields_one_ground_piece() {
        let pieces = split_levels(&unit_line(), &[]);
        assert_eq!(pieces.len(), 1);
        assert_eq!(pieces[0].1, 0);
    }

    #[test]
    fn bridge_in_the_middle_splits_into_three() {
        let pieces = split_levels(&unit_line(), &[run(0.4, 0.6, 1)]);
        let levels: Vec<i64> = pieces.iter().map(|(_, l)| *l).collect();
        assert_eq!(levels, vec![0, 1, 0]);
        // The bridge piece spans x in [4, 6]; its abutments are the cut points.
        assert_eq!(xs(&pieces[1].0), vec![4.0, 6.0]);
        // Ground pieces meet the bridge exactly — no gap.
        assert_eq!(*xs(&pieces[0].0).last().unwrap(), 4.0);
        assert_eq!(xs(&pieces[2].0)[0], 6.0);
    }

    #[test]
    fn bridge_then_tunnel_keeps_levels_distinct() {
        // A braided segment like the A9 example: bridge, then tunnel.
        let pieces = split_levels(&unit_line(), &[run(0.1, 0.4, 1), run(0.4, 0.8, -5)]);
        let levels: Vec<i64> = pieces.iter().map(|(_, l)| *l).collect();
        // ground, bridge, tunnel, ground — the tunnel never inherits the deck.
        assert_eq!(levels, vec![0, 1, -5, 0]);
    }

    #[test]
    fn adjacent_same_level_rules_coalesce() {
        // Two touching bridge rules become one piece, not two.
        let pieces = split_levels(&unit_line(), &[run(0.2, 0.5, 1), run(0.5, 0.7, 1)]);
        let levels: Vec<i64> = pieces.iter().map(|(_, l)| *l).collect();
        assert_eq!(levels, vec![0, 1, 0]);
        assert_eq!(xs(&pieces[1].0), vec![2.0, 7.0]);
    }

    #[test]
    fn phantom_sliver_between_structures_is_dropped() {
        // ~111 m line; a tunnel and a bridge whose rule edges don't meet, leaving
        // a ~5 m at-grade sliver between them (an Overture rule-edge mismatch). The
        // sliver is dropped so the structures abut — no draped stub poking out.
        let line = Geometry::LineString(LineString(vec![
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 0.001, y: 0.0 },
        ]));
        let pieces = split_levels(&line, &[run(0.0, 0.45, -5), run(0.50, 1.0, 1)]);
        let levels: Vec<i64> = pieces.iter().map(|(_, l)| *l).collect();
        assert_eq!(levels, vec![-5, 1], "the phantom at-grade sliver must be dropped");
        // They meet at the sliver midpoint — no gap.
        let tun_end = *xs(&pieces[0].0).last().unwrap();
        let br_start = xs(&pieces[1].0)[0];
        assert!((tun_end - br_start).abs() < 1e-9, "structures must abut");
    }

    #[test]
    fn real_at_grade_stretch_between_structures_survives() {
        // A genuine ~445 m at-grade stretch (far longer than a rule-edge sliver)
        // between two structures stays as its own ground piece.
        let line = Geometry::LineString(LineString(vec![
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 0.01, y: 0.0 }, // ~1.1 km
        ]));
        let pieces = split_levels(&line, &[run(0.0, 0.3, -5), run(0.7, 1.0, 1)]);
        let levels: Vec<i64> = pieces.iter().map(|(_, l)| *l).collect();
        assert_eq!(levels, vec![-5, 0, 1], "a long at-grade stretch must remain");
    }

    #[test]
    fn full_span_rule_yields_single_piece() {
        let pieces = split_levels(&unit_line(), &[run(0.0, 1.0, -1)]);
        assert_eq!(pieces.len(), 1);
        assert_eq!(pieces[0].1, -1);
    }

    #[test]
    fn interior_vertices_are_preserved() {
        let line = Geometry::LineString(LineString(vec![
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 4.0, y: 0.0 },
            Coord { x: 8.0, y: 0.0 },
            Coord { x: 10.0, y: 0.0 },
        ]));
        // Bridge over [0.5, 1.0] → x in [5, 10], keeping the vertex at x=8.
        let pieces = split_levels(&line, &[run(0.5, 1.0, 1)]);
        let bridge = pieces.iter().find(|(_, l)| *l == 1).unwrap();
        assert_eq!(xs(&bridge.0), vec![5.0, 8.0, 10.0]);
    }

    #[test]
    fn cuts_follow_geodesic_not_planar_length() {
        // An L-bend at 60°N, where a longitude degree is ~half a latitude degree.
        // Leg AB runs east-west (lon 0→2), leg BC north-south (lat 60→61): planar
        // leg lengths are 2 and 1, but geodesically AB shrinks by ~cos 60° to ~1,
        // so the two legs are nearly equal on the ground. A rule over the second
        // half must therefore cut at the corner B, not mid-AB. Measuring fractions
        // in raw degrees (the old bug) would cut at planar x=1.5, lopping off part
        // of the east-west leg — exactly the kind of shift that made the Chillon
        // tunnel too short and its bridge start too early.
        let line = Geometry::LineString(LineString(vec![
            Coord { x: 0.0, y: 60.0 },
            Coord { x: 2.0, y: 60.0 },
            Coord { x: 2.0, y: 61.0 },
        ]));
        let pieces = split_levels(&line, &[run(0.5, 1.0, 1)]);
        let bridge = pieces.iter().find(|(_, l)| *l == 1).expect("a bridge piece");
        let first = match &bridge.0 {
            Geometry::LineString(ls) => ls.0[0],
            _ => panic!("expected linestring"),
        };
        // Geodesic cut lands at the corner (x≈2); the planar cut would be x=1.5.
        assert!(
            (first.x - 2.0).abs() < 0.05,
            "bridge starts at x={}, expected the corner ~2.0 (planar bug cuts at 1.5)",
            first.x
        );
    }

    #[test]
    fn dominant_picks_widest_non_ground_or_ground() {
        // A short bridge over mostly-grade collapses to ground.
        assert_eq!(dominant(&[run(0.45, 0.55, 1)]), 0);
        // A segment that is mostly a tunnel collapses to the tunnel.
        assert_eq!(dominant(&[run(0.1, 0.9, -1)]), -1);
        assert_eq!(dominant(&[]), 0);
    }

    /// Builds an Overture-shaped `level_rules` array — one cell holding a list of
    /// `{value, between:[start,end]}` structs — and checks [`parse`].
    #[test]
    fn parse_reads_value_and_between() {
        let value: ArrayRef = Arc::new(Int32Array::from(vec![1, 0, -5]));
        // `between` as list<float64>, two endpoints per struct.
        let between_values = Float64Array::from(vec![0.1, 0.4, 0.4, 0.6, 0.6, 0.9]);
        let between_offsets = OffsetBuffer::new(ScalarBuffer::from(vec![0, 2, 4, 6]));
        let between: ArrayRef = Arc::new(ListArray::new(
            Arc::new(Field::new("item", DataType::Float64, true)),
            between_offsets,
            Arc::new(between_values),
            None,
        ));
        let struct_fields = Fields::from(vec![
            Field::new("value", DataType::Int32, true),
            Field::new("between", between.data_type().clone(), true),
        ]);
        let structs = StructArray::new(struct_fields.clone(), vec![value, between], None);
        // One outer row containing all three rules.
        let outer = ListArray::new(
            Arc::new(Field::new("item", DataType::Struct(struct_fields), true)),
            OffsetBuffer::new(ScalarBuffer::from(vec![0, 3])),
            Arc::new(structs),
            None,
        );
        let runs = parse(&outer, 0);
        // The ground rule (value 0) is dropped; the bridge and tunnel remain.
        assert_eq!(runs, vec![run(0.1, 0.4, 1), run(0.6, 0.9, -5)]);
    }

    #[test]
    fn parse_handles_null_cell() {
        let value: ArrayRef = Arc::new(Int32Array::from(vec![1]));
        let between: ArrayRef = Arc::new(ListArray::new(
            Arc::new(Field::new("item", DataType::Float64, true)),
            OffsetBuffer::new(ScalarBuffer::from(vec![0, 0])),
            Arc::new(Float64Array::from(Vec::<f64>::new())),
            None,
        ));
        let struct_fields = Fields::from(vec![
            Field::new("value", DataType::Int32, true),
            Field::new("between", between.data_type().clone(), true),
        ]);
        let structs = StructArray::new(struct_fields.clone(), vec![value, between], None);
        // A single null outer row.
        let nulls = arrow::buffer::NullBuffer::from(vec![false]);
        let outer = ListArray::new(
            Arc::new(Field::new("item", DataType::Struct(struct_fields), true)),
            OffsetBuffer::new(ScalarBuffer::from(vec![0, 1])),
            Arc::new(structs),
            Some(nulls),
        );
        assert!(parse(&outer, 0).is_empty());
    }
}
