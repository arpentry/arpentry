//! Joining road segments into corridors and resolving level annotations into
//! corridor-wide structure spans (docs/GENERATION.md §5 stage 1).
//!
//! Overture splits a road wherever any attribute changes, so one physical
//! viaduct — one grade line — arrives as many segments, each annotating its own
//! slice of the structure. Nothing in the data links them; the connectivity
//! must come from the graph. Two segment ends meeting at a connector that no
//! other participating segment touches, with the same class, are the same road
//! continuing: they are spliced. Walking the splices chains segments into
//! [`Corridor`]s, and each segment's linearly-referenced level runs are
//! remapped into corridor arc space, where consecutive same-level runs merge
//! into one [`Span`] per physical structure (S1/S8) and annotation noise —
//! sub-[`SNAP_RUN_M`] grade slivers between structures, spans too short to be
//! real structures — is resolved once, globally (S10).

use std::collections::HashMap;

use geo_types::Coord;

use crate::levels::LevelRun;
use crate::priors::{Kind, RoadClass, MAX_CORRIDOR_M, SNAP_RUN_M};
use crate::scene::{
    metric_len, run_cos_lat, Corridor, CorridorId, Junction, JunctionMember, SegmentRef, Span,
    SpanKind,
};
use crate::value::Value;

/// Smallest arc difference treated as a distinct span edge, metres.
const EPS_M: f64 = 1e-6;

/// One participating source segment, as read from the transportation input.
pub struct RawSegment {
    pub source: u64,
    pub line: Vec<Coord>,
    /// The §9 prior key, `(modality, class)`.
    pub kind: Kind,
    /// Whether the subclass marks a ramp (`link`) — narrower structures.
    pub link: bool,
    /// Raw class string — splice compatibility compares the exact class, and
    /// the styling consumers want it finer than [`Kind`] buckets it.
    pub class_key: String,
    pub subtype_key: String,
    pub level_runs: Vec<LevelRun>,
    /// Connector at the segment's first vertex, if any.
    pub start_connector: Option<u64>,
    /// Connector at the segment's last vertex, if any.
    pub end_connector: Option<u64>,
    /// Every connector the segment touches with its linear reference — the
    /// ends *and* the interior ones. Overture attaches a side road to a
    /// through road by a connector partway along the through segment
    /// (`0 < at < 1`), and on the Swiss extract that is most attachments:
    /// 749,524 paved connectors are interior to one segment while an end of
    /// another. Ends alone are not the graph.
    pub connectors: Vec<super::columns::Connector>,
    pub properties: Vec<(String, Value)>,
}

/// Which end of a segment a splice attaches to.
#[derive(Clone, Copy, PartialEq, Eq)]
enum End {
    Start,
    End,
}

impl End {
    fn other(self) -> End {
        match self {
            End::Start => End::End,
            End::End => End::Start,
        }
    }
}

/// Joins the participating segments into corridors and finds the junctions
/// where they meet (invariant 2).
pub fn build(segments: Vec<RawSegment>) -> (Vec<Corridor>, Vec<Junction>) {
    let links = splice_links(&segments);
    let chains = walk_chains(&segments, &links);
    let mut corridors = Vec::with_capacity(chains.len());
    // `ARPT_NO_INTERIOR_PORTS=1` keeps the junctions end-only again, so an A/B
    // re-tile of the interior ports is a flag rather than a patch — the same
    // reason `ARPT_NO_ABUTMENT_CUT` exists.
    let interior_ports = std::env::var_os("ARPT_NO_INTERIOR_PORTS").is_none();
    // Every corridor's connector ports (connector id → the corridor and the arc
    // it sits at), bucketed by connector so shared ones become junctions.
    let mut by_connector: HashMap<u64, Vec<(CorridorId, f64, Coord)>> = HashMap::new();
    for chain in chains {
        if let Some((c, ports)) =
            build_corridor(corridors.len() as u32, &segments, &chain, interior_ports)
        {
            for (conn, arc, point) in ports {
                by_connector.entry(conn).or_default().push((c.id, arc, point));
            }
            corridors.push(c);
        }
    }
    (corridors, junctions(by_connector))
}

/// Turns the connector→ports map into junctions: a connector touched by two or
/// more distinct corridors is a place they meet. Deterministic — the map is
/// drained into a connector-sorted list so the junction order (and thus the
/// weld order) never depends on hashing.
fn junctions(by_connector: HashMap<u64, Vec<(CorridorId, f64, Coord)>>) -> Vec<Junction> {
    let mut conns: Vec<(u64, Vec<(CorridorId, f64, Coord)>)> = by_connector.into_iter().collect();
    conns.sort_by_key(|(id, _)| *id);
    let mut out = Vec::new();
    for (conn, mut ports) in conns {
        // One member per corridor (an interior splice touches a connector from
        // both sides at the same arc); keep the lowest corridor id's port.
        ports.sort_by(|a, b| a.0.cmp(&b.0).then(a.1.partial_cmp(&b.1).expect("finite arc")));
        ports.dedup_by_key(|p| p.0);
        if ports.len() < 2 {
            continue; // a plain through-splice or a dangling end: no junction
        }
        let point = ports[0].2;
        let members =
            ports.iter().map(|&(corridor, arc, _)| JunctionMember { corridor, arc }).collect();
        out.push(Junction { point, connector: conn, members });
    }
    out
}

/// For each (segment, end), the (segment, end) it splices to, if any. Two ends
/// splice when they are the *only* participating ends at their connector and
/// the segments agree on class and subtype — a road continuing, not a
/// junction. Three or more ends (a fork) or a class change never splice; the
/// solver's junction constraints handle those later.
fn splice_links(segments: &[RawSegment]) -> HashMap<(usize, u8), (usize, u8)> {
    let mut at_connector: HashMap<u64, Vec<(usize, End)>> = HashMap::new();
    for (i, seg) in segments.iter().enumerate() {
        if let Some(c) = seg.start_connector {
            at_connector.entry(c).or_default().push((i, End::Start));
        }
        if let Some(c) = seg.end_connector {
            at_connector.entry(c).or_default().push((i, End::End));
        }
    }
    let mut links = HashMap::new();
    for ends in at_connector.values() {
        // Group the ends by (class, subtype): an exact pair within a group is
        // a road continuing through the connector — including through a
        // junction where ramps of another class also attach. Splicing the
        // through pair keeps one corridor (one solved profile) across every
        // interchange; breaking there once landed a corridor end mid-bridge,
        // where the end-of-corridor chord dived a deck 5 m under its own
        // continuation. Three or more same-class ends (a genuine fork) still
        // never splice; the solver's junction constraints handle those.
        let mut groups: HashMap<(&str, &str, bool), Vec<(usize, End)>> = HashMap::new();
        for &(i, e) in ends {
            // The link flag joins the key: an Overture ramp shares the
            // mainline's class (`motorway`), so without it every interchange
            // connector has 3–4 same-class ends and nothing ever splices.
            let key = (
                segments[i].class_key.as_str(),
                segments[i].subtype_key.as_str(),
                segments[i].link,
            );
            groups.entry(key).or_default().push((i, e));
        }
        for group in groups.values() {
            let [(i, ie), (j, je)] = group[..] else { continue };
            if i == j {
                continue; // a loop segment touching itself
            }
            // The pair must continue roughly straight through the connector.
            // Where a dual carriageway's two directions meet end-to-end (an
            // interchange terminus) the tangents oppose — splicing there
            // folds the corridor into a hairpin that runs beside itself for
            // kilometres, and fragment projection then lands on the wrong
            // pass (deck stubs metres off in height).
            if !continues_through(&segments[i], ie, &segments[j], je) {
                continue;
            }
            links.insert((i, ie as u8), (j, je as u8));
            links.insert((j, je as u8), (i, ie as u8));
        }
    }
    links
}

/// Whether travel arriving along `a` (at its `ae` end) leaves along `b` (from
/// its `be` end) without reversing: the turn angle at the shared connector
/// stays under ~120°. A genuine continuation is near-straight; a dual
/// carriageway folding back on itself is near-180° and must not splice.
fn continues_through(a: &RawSegment, ae: End, b: &RawSegment, be: End) -> bool {
    let dir = |seg: &RawSegment, end: End, outward: bool| -> Option<(f64, f64)> {
        let line = &seg.line;
        if line.len() < 2 {
            return None;
        }
        let (at, next) = match end {
            End::Start => (line[0], line[1]),
            End::End => (line[line.len() - 1], line[line.len() - 2]),
        };
        let cos_lat = at.y.to_radians().cos();
        // Vector pointing away from the connector into the segment; flip it
        // for the arriving direction.
        let (dx, dy) = ((next.x - at.x) * cos_lat, next.y - at.y);
        let len = (dx * dx + dy * dy).sqrt();
        if len < 1e-12 {
            return None;
        }
        let sign = if outward { 1.0 } else { -1.0 };
        Some((sign * dx / len, sign * dy / len))
    };
    match (dir(a, ae, false), dir(b, be, true)) {
        // cos 120° = −0.5: reject sharper turns than 120°.
        (Some((ax, ay)), Some((bx, by))) => ax * bx + ay * by > -0.5,
        _ => false,
    }
}

/// One chain entry: a segment index and whether it runs forward (its start
/// facing the chain's head) in the corridor.
type ChainLink = (usize, bool);

/// Walks the splice links into maximal chains. Every segment lands in exactly
/// one chain (a singleton when nothing splices to it); rings are broken at an
/// arbitrary link, and chains are cut at [`MAX_CORRIDOR_M`].
fn walk_chains(
    segments: &[RawSegment],
    links: &HashMap<(usize, u8), (usize, u8)>,
) -> Vec<Vec<ChainLink>> {
    let seg_len =
        |i: usize| -> f64 { polyline_len(&segments[i].line) };
    let mut visited = vec![false; segments.len()];
    let mut chains = Vec::new();
    for start in 0..segments.len() {
        if visited[start] {
            continue;
        }
        // Walk left from `start` to the chain head. `cur` is the (segment,
        // outward end) whose splice we follow next; stopping leaves `cur` as
        // the head with its outward end free.
        let mut cur = (start, End::Start);
        let mut steps = 0usize;
        while let Some(&(next, next_end)) = links.get(&(cur.0, cur.1 as u8)) {
            if next == start || visited[next] {
                break; // a ring closing on itself, or claimed by another chain
            }
            cur = (next, End::from_u8(next_end).other());
            steps += 1;
            if steps > segments.len() {
                break; // defensive: malformed link tables can't loop forever
            }
        }
        // Walk right from the head, collecting the chain in order. A segment
        // whose free end is Start runs forward.
        let mut chain: Vec<ChainLink> = Vec::new();
        let mut arc = 0.0;
        let (mut seg, mut lead) = cur;
        loop {
            visited[seg] = true;
            chain.push((seg, lead == End::Start));
            arc += seg_len(seg);
            let tail = lead.other();
            let Some(&(next, next_end)) = links.get(&(seg, tail as u8)) else {
                break;
            };
            if visited[next] || arc > MAX_CORRIDOR_M {
                break;
            }
            seg = next;
            lead = End::from_u8(next_end);
        }
        chains.push(chain);
    }
    chains
}

impl End {
    fn from_u8(v: u8) -> End {
        if v == End::Start as u8 {
            End::Start
        } else {
            End::End
        }
    }
}

fn polyline_len(line: &[Coord]) -> f64 {
    let cos_lat = run_cos_lat(line);
    line.windows(2).map(|w| metric_len(w[0], w[1], cos_lat)).sum()
}

/// Builds one corridor from a chain: concatenates the oriented centerlines
/// (deduplicating shared junction vertices), remaps each segment's level runs
/// into corridor arc space, and resolves the runs into spans.
fn build_corridor(
    id: u32,
    segments: &[RawSegment],
    chain: &[ChainLink],
    interior_ports: bool,
) -> Option<(Corridor, Vec<(u64, f64, Coord)>)> {
    // Concatenate oriented nodes, remembering each segment's node range.
    let mut nodes: Vec<Coord> = Vec::new();
    let mut ranges: Vec<(usize, usize, bool)> = Vec::with_capacity(chain.len());
    for &(si, forward) in chain {
        let line = &segments[si].line;
        let node0 = if nodes.is_empty() { 0 } else { nodes.len() - 1 };
        let mut push = |c: Coord| {
            if nodes.last() != Some(&c) {
                nodes.push(c);
            }
        };
        if forward {
            line.iter().for_each(|&c| push(c));
        } else {
            line.iter().rev().for_each(|&c| push(c));
        }
        let node1 = nodes.len().saturating_sub(1);
        ranges.push((node0, node1, forward));
    }
    if nodes.len() < 2 {
        return None;
    }

    let cos_lat = run_cos_lat(&nodes);
    let arc = crate::scene::cumulative_arc(&nodes);
    let total = arc[arc.len() - 1];
    if total <= 0.0 {
        return None;
    }

    // Remap each segment's level runs (fractions of the segment) into corridor
    // arc metres, flipping fractions for reversed segments.
    let mut runs: Vec<(f64, f64, i64)> = Vec::new();
    for (&(si, _), &(node0, node1, forward)) in chain.iter().zip(&ranges) {
        let (a0, a1) = (arc[node0], arc[node1]);
        let len = a1 - a0;
        if len <= 0.0 {
            continue;
        }
        for r in &segments[si].level_runs {
            let (s, e) = if forward { (r.start, r.end) } else { (1.0 - r.end, 1.0 - r.start) };
            runs.push((a0 + s * len, a0 + e * len, r.level));
        }
    }

    let spans = resolve_spans(&runs, total);
    let seg_refs = chain
        .iter()
        .zip(&ranges)
        .map(|(&(si, _), &(node0, node1, _))| SegmentRef {
            source: segments[si].source,
            node0,
            node1,
            properties: segments[si].properties.clone(),
        })
        .collect();
    // The corridor's connector ports: every connector of every member segment,
    // at the corridor arc (and coordinate) it sits at. A reversed segment's
    // start connector lands at its high node and vice versa; an *interior*
    // connector — a side road attached partway along the segment — lands at
    // its linear reference. Shared with other corridors, these ports are the
    // junctions, and the interior ones are most of them: keyed on ends alone,
    // a through road passing a connector mid-segment could never be a junction
    // member there, so the fork it carries got no weld, no plate, and a side
    // road solved metres off the surface it joins (the Colondalles fork,
    // 6.9026,46.4455, stood 4.7 m over its tertiary).
    let mut ports: Vec<(u64, f64, Coord)> = Vec::new();
    // The same entries as `(id, arc)` for [`Corridor::connectors`] — one
    // derivation, so the crossing exclusion and the verify checks read the
    // ports the junctions were built from.
    let mut connectors: Vec<(u64, f64)> = Vec::new();
    for (&(si, forward), &(node0, node1, _)) in chain.iter().zip(&ranges) {
        let seg = &segments[si];
        let (sc_node, ec_node) = if forward { (node0, node1) } else { (node1, node0) };
        let (a0, a1) = (arc[node0], arc[node1]);
        for c in &seg.connectors {
            // An end connector sits exactly on its node — read the node's arc
            // rather than re-deriving it, so the two sides of a splice agree
            // bitwise and dedup to one entry.
            let (carc, point, interior) = if c.at <= super::END_AT_EPS {
                (arc[sc_node], nodes[sc_node], false)
            } else if c.at >= 1.0 - super::END_AT_EPS {
                (arc[ec_node], nodes[ec_node], false)
            } else {
                // `at` is a fraction of the segment's length, and the
                // segment's span of corridor arc is its length, so the two
                // scales agree by construction.
                let frac = if forward { c.at } else { 1.0 - c.at };
                let carc = a0 + frac * (a1 - a0);
                (carc, point_at_arc(&nodes, &arc, carc), true)
            };
            connectors.push((c.id, carc));
            if !interior || interior_ports {
                ports.push((c.id, carc, point));
            }
        }
    }
    connectors.sort_unstable_by(|a, b| {
        a.0.cmp(&b.0).then(a.1.partial_cmp(&b.1).expect("finite arc"))
    });
    connectors.dedup();

    Some((
        Corridor {
            id,
            kind: chain
                .first()
                .map(|&(si, _)| segments[si].kind)
                .unwrap_or(Kind::Road(RoadClass::Other)),
            class_key: chain
                .first()
                .map(|&(si, _)| segments[si].class_key.clone())
                .unwrap_or_default(),
            link: chain.iter().all(|&(si, _)| segments[si].link),
            width_m: chain
                .iter()
                .filter_map(|&(si, _)| segment_width_m(&segments[si]))
                .fold(None, |w: Option<f64>, s| Some(w.map_or(s, |w| w.max(s)))),
            nodes,
            arc,
            cos_lat,
            spans,
            segments: seg_refs,
            connectors,
        },
        ports,
    ))
}

/// The point at arc `a` on the corridor's node chain — where an interior
/// connector's port sits. The connector is a vertex of the mapped line, so the
/// interpolation lands on (or within float noise of) that vertex.
fn point_at_arc(nodes: &[Coord], arc: &[f64], a: f64) -> Coord {
    let n = nodes.len();
    debug_assert!(n >= 2 && arc.len() == n);
    let i = match arc.binary_search_by(|v| v.partial_cmp(&a).expect("finite arc")) {
        Ok(i) => i.min(n - 2),
        Err(i) => i.saturating_sub(1).min(n - 2),
    };
    let span = arc[i + 1] - arc[i];
    let t = if span > 0.0 { ((a - arc[i]) / span).clamp(0.0, 1.0) } else { 0.0 };
    let (p, q) = (nodes[i], nodes[i + 1]);
    Coord { x: p.x + (q.x - p.x) * t, y: p.y + (q.y - p.y) * t }
}

/// One segment's carriageway width in metres, from the same derivation the
/// tiled `width_m` property uses: the mapped `width_rules` value where
/// plausible, else the class prior. `None` for a non-drivable class.
fn segment_width_m(seg: &RawSegment) -> Option<f64> {
    let measured = crate::value::width_rules_m(&seg.properties);
    let subclass = crate::value::str_of(&seg.properties, "subclass");
    crate::priors::carriageway_width_m(Some(seg.class_key.as_str()), subclass, measured)
}

/// Resolves arc-referenced level runs into a clean partition of `[0, total]`:
/// maximal constant-level spans, with consecutive same-level runs merged into
/// one span per physical structure and sub-[`SNAP_RUN_M`] grade slivers
/// between structures dropped (annotation-edge mismatches, S10). Structure
/// spans shorter than [`crate::priors::MIN_STRUCTURE_M`] survive here:
/// whether a short span
/// is a real deck (a bridge over a deep gully) or annotation noise (a
/// footbridge better left draped) is a *terrain* question, resolved by the
/// solve stage against the DEM (`solve::reconcile_short_spans`).
fn resolve_spans(runs: &[(f64, f64, i64)], total: f64) -> Vec<Span> {
    // Breakpoints: the corridor ends plus every run edge.
    let mut breaks = vec![0.0, total];
    for &(s, e, _) in runs {
        breaks.push(s.clamp(0.0, total));
        breaks.push(e.clamp(0.0, total));
    }
    breaks.sort_by(|a, b| a.partial_cmp(b).expect("finite arcs"));
    breaks.dedup_by(|a, b| (*a - *b).abs() <= EPS_M);

    // Interval levels; on overlap the last run wins, mirroring source order.
    let level_at = |arc: f64| -> i64 {
        runs.iter().rev().find(|&&(s, e, _)| arc >= s && arc <= e).map_or(0, |&(_, _, l)| l)
    };
    let mut intervals: Vec<(f64, f64, i64)> = Vec::new();
    for w in breaks.windows(2) {
        let (b0, b1) = (w[0], w[1]);
        if b1 - b0 <= EPS_M {
            continue;
        }
        let level = level_at(0.5 * (b0 + b1));
        match intervals.last_mut() {
            Some(last) if last.2 == level => last.1 = b1,
            _ => intervals.push((b0, b1, level)),
        }
    }

    // Annotation-noise passes, each followed by a same-level coalesce:
    // 1. Drop sub-SNAP grade slivers wedged between two structures — the
    //    structures abut at the sliver midpoint.
    drop_where(&mut intervals, |iv, i| {
        iv[i].2 == 0
            && iv[i].1 - iv[i].0 < SNAP_RUN_M
            && i > 0
            && i + 1 < iv.len()
            && iv[i - 1].2 != 0
            && iv[i + 1].2 != 0
    });
    coalesce(&mut intervals);

    intervals
        .into_iter()
        .map(|(arc0, arc1, level)| Span {
            arc0,
            arc1,
            level,
            kind: match level.signum() {
                1 => SpanKind::Bridge,
                -1 => SpanKind::Tunnel,
                _ => SpanKind::Grade,
            },
        })
        .collect()
}

/// Removes intervals matching `pred`, splitting each removed interval between
/// its neighbours at the midpoint.
fn drop_where(intervals: &mut Vec<(f64, f64, i64)>, pred: impl Fn(&[(f64, f64, i64)], usize) -> bool) {
    let mut i = 0;
    while i < intervals.len() {
        if pred(intervals, i) {
            let mid = 0.5 * (intervals[i].0 + intervals[i].1);
            if i > 0 {
                intervals[i - 1].1 = mid;
            }
            if i + 1 < intervals.len() {
                intervals[i + 1].0 = mid;
            }
            intervals.remove(i);
        } else {
            i += 1;
        }
    }
}

/// Merges adjacent same-level intervals in place.
fn coalesce(intervals: &mut Vec<(f64, f64, i64)>) {
    let mut i = 1;
    while i < intervals.len() {
        if intervals[i].2 == intervals[i - 1].2 {
            intervals[i - 1].1 = intervals[i].1;
            intervals.remove(i);
        } else {
            i += 1;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scene::source_hash;

    /// An east-west segment of `len_m` metres starting at `x0_m` metres east of
    /// lon 6, at lat 46.
    fn seg(
        source: &str,
        x0_m: f64,
        len_m: f64,
        runs: Vec<LevelRun>,
        start: Option<u64>,
        end: Option<u64>,
    ) -> RawSegment {
        let cos_lat = 46.0_f64.to_radians().cos();
        let to_deg = |m: f64| m / (crate::scene::DEG_M * cos_lat);
        let n = 8;
        let line = (0..=n)
            .map(|i| Coord { x: 6.0 + to_deg(x0_m + len_m * i as f64 / n as f64), y: 46.0 })
            .collect();
        RawSegment {
            source: source_hash(source),
            line,
            kind: Kind::Road(RoadClass::Motorway),
            link: false,
            class_key: "motorway".into(),
            subtype_key: "road".into(),
            level_runs: runs,
            start_connector: start,
            end_connector: end,
            connectors: start
                .into_iter()
                .map(|id| Connector { id, at: 0.0 })
                .chain(end.into_iter().map(|id| Connector { id, at: 1.0 }))
                .collect(),
            properties: vec![],
        }
    }

    use crate::assemble::columns::Connector;

    /// A straight segment from `(x0_m, y0_m)` metres east/north of (lon 6,
    /// lat 46), running `len_m` metres at `bearing_deg`.
    fn seg_at(
        source: &str,
        x0_m: f64,
        y0_m: f64,
        len_m: f64,
        bearing_deg: f64,
        class: &str,
        start: Option<u64>,
        end: Option<u64>,
    ) -> RawSegment {
        let cos_lat = 46.0_f64.to_radians().cos();
        let (de, dn) = (bearing_deg.to_radians().sin(), bearing_deg.to_radians().cos());
        let n = 8;
        let point = |m: f64| Coord {
            x: 6.0 + (x0_m + de * m) / (crate::scene::DEG_M * cos_lat),
            y: 46.0 + (y0_m + dn * m) / crate::scene::DEG_M,
        };
        let mut s = seg(source, 0.0, len_m, vec![], start, end);
        s.line = (0..=n).map(|i| point(len_m * i as f64 / n as f64)).collect();
        s.class_key = class.into();
        s.kind = Kind::parse(Some("road"), Some(class), None);
        s
    }

    fn run(start: f64, end: f64, level: i64) -> LevelRun {
        LevelRun { start, end, level }
    }

    #[test]
    fn splices_two_segments_into_one_corridor() {
        // a: 0..1000 m, b: 1000..2000 m, sharing connector 7 (a.end = b.start).
        let a = seg("a", 0.0, 1000.0, vec![run(0.5, 1.0, 1)], Some(1), Some(7));
        let b = seg("b", 1000.0, 1000.0, vec![run(0.0, 0.5, 1)], Some(7), Some(2));
        let (cs, _junctions) = build(vec![a, b]);
        assert_eq!(cs.len(), 1);
        let c = &cs[0];
        assert_eq!(c.segments.len(), 2);
        // The two half-viaduct annotations merge into ONE bridge span 500..1500.
        let bridges: Vec<&Span> = c.spans.iter().filter(|s| s.kind == SpanKind::Bridge).collect();
        assert_eq!(bridges.len(), 1, "adjacent same-level spans must merge into one structure");
        assert!((bridges[0].arc0 - 500.0).abs() < 20.0, "bridge starts near 500, got {}", bridges[0].arc0);
        assert!((bridges[0].arc1 - 1500.0).abs() < 20.0, "bridge ends near 1500, got {}", bridges[0].arc1);
    }

    #[test]
    fn reversed_segment_flips_its_level_runs() {
        // b is digitized east→west: its end connector is the shared one, and its
        // bridge run [0.0, 0.5] covers its *eastern* half... after reversal the
        // merged bridge must still be contiguous around the joint.
        let a = seg("a", 0.0, 1000.0, vec![run(0.5, 1.0, 1)], Some(1), Some(7));
        let mut b = seg("b", 1000.0, 1000.0, vec![run(0.5, 1.0, 1)], Some(2), Some(7));
        b.line.reverse(); // digitized east→west; its start is at 2000 m
        let (cs, _junctions) = build(vec![a, b]);
        assert_eq!(cs.len(), 1);
        let bridges: Vec<&Span> =
            cs[0].spans.iter().filter(|s| s.kind == SpanKind::Bridge).collect();
        assert_eq!(bridges.len(), 1, "the reversed run must land adjacent and merge");
        assert!((bridges[0].arc1 - bridges[0].arc0 - 1000.0).abs() < 20.0);
    }

    #[test]
    fn a_junction_of_three_does_not_splice() {
        // Three motorway ends at connector 7 — a fork, not a continuation.
        let a = seg("a", 0.0, 1000.0, vec![run(0.0, 1.0, 1)], Some(1), Some(7));
        let b = seg("b", 1000.0, 1000.0, vec![run(0.0, 1.0, 1)], Some(7), Some(2));
        let c = seg("c", 1000.0, 500.0, vec![run(0.0, 1.0, 1)], Some(7), Some(3));
        let (cs, junctions) = build(vec![a, b, c]);
        assert_eq!(cs.len(), 3, "a degree-3 connector must not splice");
        // The fork is a junction: all three corridors meet at connector 7.
        assert_eq!(junctions.len(), 1, "the shared fork connector is one junction");
        assert_eq!(junctions[0].members.len(), 3, "all three legs are members");
    }

    #[test]
    fn an_interior_connector_is_a_junction_with_the_through_road() {
        // A side road ends on a connector partway along the through road's one
        // segment — Overture's usual T-attachment. The through road must be a
        // junction member there, at the arc the connector sits at; keyed on
        // segment ends it never was, and the side road solved unwelded.
        let mut through = seg("t", 0.0, 1000.0, vec![], Some(1), Some(2));
        through.connectors.push(Connector { id: 9, at: 0.5 });
        let side = seg_at("s", 500.0, 0.0, 200.0, 0.0, "primary", Some(9), Some(3));
        let (cs, junctions) = build(vec![through, side]);
        assert_eq!(cs.len(), 2);
        assert_eq!(junctions.len(), 1, "the interior attachment is a junction");
        let j = &junctions[0];
        assert_eq!(j.connector, 9);
        assert_eq!(j.members.len(), 2, "the through road is a member");
        let through_member = j
            .members
            .iter()
            .find(|m| m.arc > 1.0)
            .expect("the through member sits mid-corridor");
        assert!(
            (through_member.arc - 500.0).abs() < 1.0,
            "through arc {} should be the connector's linear reference",
            through_member.arc
        );
    }

    #[test]
    fn a_fork_off_a_through_road_keeps_its_junction() {
        // The Colondalles shape (6.9026,46.4455): two same-class arms fork off
        // a through road at one interior connector. The arms turn under 120°
        // so they splice into one V-shaped corridor — and with end-only ports
        // that V deduped to a single port and the junction vanished, leaving
        // the fork unwelded 4.7 m over the road it joins.
        let mut through = seg("t", 0.0, 1000.0, vec![], Some(1), Some(2));
        through.connectors.push(Connector { id: 9, at: 0.5 });
        let arm_a = seg_at("a", 500.0, 0.0, 200.0, -60.0, "primary", Some(9), Some(3));
        let arm_b = seg_at("b", 500.0, 0.0, 200.0, 60.0, "primary", Some(9), Some(4));
        let (cs, junctions) = build(vec![through, arm_a, arm_b]);
        assert_eq!(cs.len(), 2, "the arms splice into one V corridor");
        assert_eq!(junctions.len(), 1, "the fork connector is still a junction");
        assert_eq!(junctions[0].members.len(), 2, "the V and the through road");
        assert!(
            junctions[0].members.iter().any(|m| (m.arc - 500.0).abs() < 1.0),
            "the through road meets the fork mid-corridor"
        );
    }

    #[test]
    fn class_change_does_not_splice() {
        let a = seg("a", 0.0, 1000.0, vec![run(0.0, 1.0, 1)], Some(1), Some(7));
        let mut b = seg("b", 1000.0, 1000.0, vec![run(0.0, 1.0, 1)], Some(7), Some(2));
        b.class_key = "primary".into();
        let (cs, _junctions) = build(vec![a, b]);
        assert_eq!(cs.len(), 2, "a class change is a new corridor");
    }

    #[test]
    fn short_structure_spans_survive_assemble() {
        // A 20 m bridge survives assemble: whether it is a real deck over a
        // gully or a footbridge annotation better left draped is a terrain
        // question, answered by the solve stage against the DEM.
        let a = seg("a", 0.0, 1000.0, vec![run(0.49, 0.51, 1)], None, None);
        let (cs, _junctions) = build(vec![a]);
        let kinds: Vec<SpanKind> = cs[0].spans.iter().map(|s| s.kind).collect();
        assert_eq!(kinds, vec![SpanKind::Grade, SpanKind::Bridge, SpanKind::Grade]);
    }

    #[test]
    fn grade_sliver_between_structures_is_dropped() {
        // Tunnel to 0.449, bridge from 0.451: the 2 m grade sliver is annotation
        // noise; the structures must abut.
        let a = seg("a", 0.0, 1000.0, vec![run(0.0, 0.449, -5), run(0.451, 1.0, 1)], None, None);
        let (cs, _junctions) = build(vec![a]);
        let kinds: Vec<SpanKind> = cs[0].spans.iter().map(|s| s.kind).collect();
        assert_eq!(kinds, vec![SpanKind::Tunnel, SpanKind::Bridge]);
        assert!((cs[0].spans[0].arc1 - cs[0].spans[1].arc0).abs() < 1e-9, "structures must abut");
    }

    #[test]
    fn a_ring_terminates() {
        // Two segments forming a closed loop: the walk must not spin forever.
        let a = seg("a", 0.0, 1000.0, vec![], Some(1), Some(2));
        let mut b = seg("b", 1000.0, 1000.0, vec![], Some(2), Some(1));
        // Make b's geometry return to a's start so the ring is geometric too.
        b.line = {
            let mut l = a.line.clone();
            l.reverse();
            l
        };
        let (cs, _junctions) = build(vec![a, b]);
        assert_eq!(cs.iter().map(|c| c.segments.len()).sum::<usize>(), 2);
    }

    #[test]
    fn corridors_are_capped_in_length() {
        // A chain of 40 × 1 km segments: the cap (30 km) splits it.
        let mut segs = Vec::new();
        for i in 0..40 {
            segs.push(seg(
                &format!("s{i}"),
                i as f64 * 1000.0,
                1000.0,
                vec![],
                Some(100 + i as u64),
                Some(101 + i as u64),
            ));
        }
        let (cs, _junctions) = build(segs);
        assert!(cs.len() >= 2, "a 40 km chain must be cut at the corridor cap");
        assert!(cs.iter().all(|c| c.total() <= MAX_CORRIDOR_M + 2000.0));
    }
}
