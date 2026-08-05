//! The assembled scene model — the entities the solver works on
//! (docs/GENERATION.md §5 stage 1).
//!
//! The map data's unit is the *segment* (an Overture feature split wherever any
//! attribute changes); the physical world's unit is the *corridor* — a road
//! that holds one engineered grade for kilometres, across many segments. The
//! assemble stage joins segments into [`Corridor`]s at their shared connectors
//! and resolves the linearly-referenced level annotations into corridor-wide
//! [`Span`]s (one span = one structure entity: a whole viaduct, a whole
//! tunnel), so the solver fits one profile per corridor instead of one per
//! fragment.
//!
//! The scene graph is the boundary between assembly and everything after it:
//! solve reads corridors and spans; the tiling phase looks features up by
//! their source id ([`SceneGraph::lookup`]) and re-emits them as constant-kind
//! pieces cut at span boundaries ([`Corridor::pieces`]).

use std::collections::HashMap;

use geo_types::Coord;

use crate::priors::Kind;
use crate::value::Value;

/// Index of a corridor in [`SceneGraph::corridors`].
pub type CorridorId = u32;

/// What a span of a corridor is: at grade on the ground, lifted on a bridge
/// deck, or sunk in a tunnel bore.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpanKind {
    Grade,
    Bridge,
    Tunnel,
}

/// A maximal constant-kind stretch of a corridor, in corridor arc metres.
/// Consecutive same-level annotation runs — across segment boundaries — are
/// merged into one span, so a span is a whole structure entity (S1/S8).
#[derive(Debug, Clone, Copy)]
pub struct Span {
    pub arc0: f64,
    pub arc1: f64,
    /// The Overture level ordinal (an ordering, never a height). 0 at grade.
    pub level: i64,
    pub kind: SpanKind,
}

/// One source segment's place in a corridor: its node range, and the styling
/// properties the tiler re-emits with each of its pieces.
#[derive(Debug, Clone)]
pub struct SegmentRef {
    /// Hash of the source feature id ([`source_hash`]).
    pub source: u64,
    /// First and last (inclusive) index into [`Corridor::nodes`].
    pub node0: usize,
    pub node1: usize,
    /// Styling properties of the source feature, minus the internals the
    /// assemble stage consumed (`level_rules`, connectors).
    pub properties: Vec<(String, Value)>,
}

/// A run of segments joined end-to-end at shared connectors — the unit the
/// solver fits one elevation profile over.
#[derive(Debug, Clone)]
pub struct Corridor {
    pub id: CorridorId,
    /// Concatenated centerline (shared connector vertices deduplicated).
    pub nodes: Vec<Coord>,
    /// Cumulative metric arc length at each node, metres (`arc[0] == 0`).
    pub arc: Vec<f64>,
    /// `cos(mean latitude)`, scaling longitude into the local metric space.
    pub cos_lat: f64,
    /// The §9 prior key: `(modality, class)`. Every class-dependent number
    /// this corridor uses comes from `kind.prior()`.
    pub kind: Kind,
    /// Raw Overture class string of the member segments (splice compatibility
    /// keeps it constant along the chain).
    ///
    /// **Styling only.** Junction plates and the palette need the exact class
    /// string to match the street colours, which [`Kind`] deliberately buckets
    /// away. It must never decide geometry: every class-dependent *number*
    /// comes from [`kind`](Self::kind)'s prior, and a second classification
    /// path is how a railway came to be a minor street in the first place.
    pub class_key: String,
    /// Whether every member segment is a `link` (ramp) — the swept structures
    /// and earthworks are a single lane wide, whatever the class.
    pub link: bool,
    /// Physical carriageway width in metres — the widest of its member
    /// segments' [`crate::priors::carriageway_width_m`], the same derivation
    /// (mapped `width_rules` where plausible, else the class prior) the tiled
    /// `width_m` property and the surface band read. `None` for a corridor
    /// whose class lays no asphalt. One cross-section per
    /// corridor, read by every consumer (docs/ROADS.md invariant 1) — the
    /// intersection areas size their legs from this, so a mouth and the band
    /// that lands on it can never disagree.
    pub width_m: Option<f64>,
    /// Constant-kind spans partitioning `[0, total]`, in arc order.
    pub spans: Vec<Span>,
    /// Source segments in corridor order.
    pub segments: Vec<SegmentRef>,
    /// Every connector id its member segments touch, sorted. A plan
    /// intersection with a feature sharing one of these is a *junction*
    /// (things meeting), never a crossing (things passing over each other).
    pub connectors: Vec<u64>,
}

/// One geometric crossing: a corridor's bridge span passing over another
/// feature. Level ordinals give the ordering (upper strictly above lower),
/// never heights; the solver turns the ordering into a clearance
/// (docs/GENERATION.md I3, scenario S4).
#[derive(Debug, Clone, Copy)]
pub struct Crossing {
    /// The corridor whose structure passes over.
    pub upper: CorridorId,
    /// Corridor arc of the intersection, metres.
    pub upper_arc: f64,
    /// The plan intersection point.
    pub point: Coord,
    /// The crossed feature's corridor, when it is in the scene graph; `None`
    /// for a plain road/rail with no vertical model of its own (its height is
    /// the ground).
    pub lower: Option<CorridorId>,
    /// What is being crossed, as the §9 key: it is the *crossed* feature's
    /// `clearance_over_m` that the deck above must respect, so this is the
    /// prior key, not a coarse bucket.
    pub lower_kind: Kind,
    pub upper_level: i64,
    pub lower_level: i64,
}

/// A point where corridors meet — two or more sharing a connector, at least
/// one of them ending there. Unlike a [`Crossing`] (features passing over one
/// another) the members physically connect, so their road surfaces must agree
/// in height (docs/GENERATION.md I2). The solver welds the members:
/// the structural weld lifts a leg to the elevated road it merges onto (a
/// ramp meeting a flyover), the street weld then pulls meeting street ends
/// to one height (docs/GROUND.md §1).
#[derive(Debug, Clone)]
pub struct Junction {
    /// The shared connector's plan position.
    pub point: Coord,
    /// The connector id the members share.
    pub connector: u64,
    pub members: Vec<JunctionMember>,
}

/// One corridor at a [`Junction`]: which corridor, and the arc along it where
/// the connector sits (`0` or `total` when the corridor ends there — a leg to
/// be welded; interior otherwise — the through road that sets the height).
#[derive(Debug, Clone, Copy)]
pub struct JunctionMember {
    pub corridor: CorridorId,
    pub arc: f64,
}

/// A still water body (a lake, reservoir, pond) whose surface the ground stage
/// flattens to one level (docs/GENERATION.md I4). The data gives no
/// surface elevation; the DEM images the shoreline at the waterline, so the
/// level is read from the ring and burned flat across the interior — the client
/// then drapes water on a flat surface instead of following terrain noise.
/// Flowing water (rivers, canals) is not held here: it wants a monotone
/// descent, deferred.
#[derive(Debug, Clone)]
pub struct WaterBody {
    /// The exterior shoreline ring, a closed lon/lat loop.
    pub exterior: Vec<Coord>,
    /// Interior rings (islands) excluded from the surface.
    pub holes: Vec<Vec<Coord>>,
    /// Bounding box `(west, south, east, north)` for indexing.
    pub bbox: (f64, f64, f64, f64),
}

/// One constant-kind piece of a source segment, ready to tile: the geometry cut
/// at span boundaries, and the span it belongs to.
#[derive(Debug)]
pub struct Piece {
    pub line: Vec<Coord>,
    /// Index into [`Corridor::spans`].
    pub span: u32,
    pub level: i64,
    pub kind: SpanKind,
}

impl Corridor {
    /// Total corridor arc length in metres.
    pub fn total(&self) -> f64 {
        *self.arc.last().unwrap_or(&0.0)
    }

    /// Cuts one source segment into constant-kind pieces at the span
    /// boundaries. Pieces follow corridor direction; interior vertices are
    /// preserved and cut points interpolated, so abutting pieces share their
    /// boundary vertex exactly.
    pub fn pieces(&self, seg: &SegmentRef) -> Vec<Piece> {
        self.pieces_in(seg, &self.spans)
    }

    /// [`Corridor::pieces`] against a caller-supplied span list — the solved
    /// stage reconciles the annotated spans with the geometry (tunnel spans
    /// clamped to their portal crossings, `solve::portals::reconcile_spans`)
    /// and cuts against the reconciled list.
    pub fn pieces_in(&self, seg: &SegmentRef, spans: &[Span]) -> Vec<Piece> {
        let (a0, a1) = (self.arc[seg.node0], self.arc[seg.node1]);
        let mut out = Vec::new();
        for (i, span) in spans.iter().enumerate() {
            let lo = span.arc0.max(a0);
            let hi = span.arc1.min(a1);
            if hi - lo <= f64::EPSILON {
                continue;
            }
            let line = self.substring(lo, hi);
            if line.len() >= 2 {
                out.push(Piece { line, span: i as u32, level: span.level, kind: span.kind });
            }
        }
        out
    }

    /// The corridor centerline between arc positions `d0` and `d1`: the two
    /// interpolated cut points plus every node strictly between them.
    fn substring(&self, d0: f64, d1: f64) -> Vec<Coord> {
        let mut out = vec![self.point_at(d0)];
        for i in 0..self.nodes.len() {
            if self.arc[i] > d0 && self.arc[i] < d1 {
                out.push(self.nodes[i]);
            }
        }
        out.push(self.point_at(d1));
        out.dedup();
        out
    }

    /// The point at arc position `d`, interpolated within its containing edge.
    fn point_at(&self, d: f64) -> Coord {
        let d = d.clamp(0.0, self.total());
        // Binary search for the containing edge; arc is non-decreasing.
        let i = match self.arc.binary_search_by(|a| a.partial_cmp(&d).expect("finite arc")) {
            Ok(i) => return self.nodes[i.min(self.nodes.len() - 1)],
            Err(i) => i.saturating_sub(1).min(self.nodes.len() - 2),
        };
        let seg = self.arc[i + 1] - self.arc[i];
        let t = if seg > 0.0 { (d - self.arc[i]) / seg } else { 0.0 };
        Coord {
            x: self.nodes[i].x + (self.nodes[i + 1].x - self.nodes[i].x) * t,
            y: self.nodes[i].y + (self.nodes[i + 1].y - self.nodes[i].y) * t,
        }
    }
}

/// The assembled scene: every corridor the solver may need, the crossings
/// between them and the rest of the network, indexed by the source features
/// they were built from.
#[derive(Default)]
pub struct SceneGraph {
    pub corridors: Vec<Corridor>,
    pub crossings: Vec<Crossing>,
    /// Where corridors meet and their heights must agree (invariant 2). With
    /// every paving road a corridor, this covers the street intersections
    /// too — the synth stage plates any junction with three or more legs.
    pub junctions: Vec<Junction>,
    /// Still water bodies whose surface the ground stage flattens (invariant 4).
    pub water: Vec<WaterBody>,
    /// Source feature id hash → (corridor, segment index within it).
    by_source: HashMap<u64, (CorridorId, u32)>,
}

impl SceneGraph {
    pub fn new(corridors: Vec<Corridor>) -> SceneGraph {
        let mut by_source = HashMap::new();
        for c in &corridors {
            for (i, seg) in c.segments.iter().enumerate() {
                by_source.insert(seg.source, (c.id, i as u32));
            }
        }
        SceneGraph {
            corridors,
            crossings: Vec::new(),
            junctions: Vec::new(),
            water: Vec::new(),
            by_source,
        }
    }

    /// Looks a source feature up by its id hash, returning its corridor and
    /// segment. `None` for features the assemble stage did not claim (they
    /// tile as plain draped geometry).
    pub fn lookup(&self, source: u64) -> Option<(&Corridor, &SegmentRef)> {
        let &(cid, seg) = self.by_source.get(&source)?;
        let c = &self.corridors[cid as usize];
        Some((c, &c.segments[seg as usize]))
    }
}

/// Metres per degree of latitude (spherical approximation), for the local
/// metric space every stage measures arcs in.
pub const DEG_M: f64 = 111_320.0;

/// Planar distance between two lon/lat points in metres (lon scaled by
/// `cos_lat`).
pub fn metric_len(a: Coord, b: Coord, cos_lat: f64) -> f64 {
    let dx = (b.x - a.x) * cos_lat * DEG_M;
    let dy = (b.y - a.y) * DEG_M;
    (dx * dx + dy * dy).sqrt()
}

/// `cos(mean latitude)` of a run, for the longitude scaling.
pub fn run_cos_lat(run: &[Coord]) -> f64 {
    let mean = run.iter().map(|c| c.y).sum::<f64>() / run.len().max(1) as f64;
    mean.to_radians().cos()
}

/// Deterministic hash of a source feature id (FNV-1a). Links the assemble
/// stage's claimed segments to the same features streaming through the tiling
/// phase, without carrying the id string around.
pub fn source_hash(id: &str) -> u64 {
    let mut h: u64 = 0xcbf29ce484222325;
    for b in id.as_bytes() {
        h ^= *b as u64;
        h = h.wrapping_mul(0x100000001b3);
    }
    h
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::priors::RoadClass;

    /// A straight east-west corridor of `n` nodes spanning `len_m` metres with
    /// one segment covering it all.
    fn corridor(spans: Vec<Span>, n: usize, len_m: f64) -> Corridor {
        let cos_lat = 46.0_f64.to_radians().cos();
        let deg = len_m / (111_320.0 * cos_lat);
        let nodes: Vec<Coord> =
            (0..n).map(|i| Coord { x: 6.0 + deg * i as f64 / (n - 1) as f64, y: 46.0 }).collect();
        let arc: Vec<f64> = (0..n).map(|i| len_m * i as f64 / (n - 1) as f64).collect();
        let segments = vec![SegmentRef { source: 1, node0: 0, node1: n - 1, properties: vec![] }];
        Corridor { id: 0, nodes, arc, cos_lat, kind: Kind::Road(RoadClass::Residential), class_key: String::new(), link: false, width_m: Some(5.5), spans, segments, connectors: vec![] }
    }

    fn span(arc0: f64, arc1: f64, level: i64) -> Span {
        let kind = match level.signum() {
            1 => SpanKind::Bridge,
            -1 => SpanKind::Tunnel,
            _ => SpanKind::Grade,
        };
        Span { arc0, arc1, level, kind }
    }

    #[test]
    fn pieces_cut_at_span_boundaries_and_abut() {
        let c = corridor(vec![span(0.0, 400.0, 0), span(400.0, 600.0, 1), span(600.0, 1000.0, 0)], 11, 1000.0);
        let pieces = c.pieces(&c.segments[0]);
        assert_eq!(pieces.len(), 3);
        assert_eq!(pieces[1].kind, SpanKind::Bridge);
        // Adjacent pieces share the cut vertex exactly.
        assert_eq!(*pieces[0].line.last().unwrap(), pieces[1].line[0]);
        assert_eq!(*pieces[1].line.last().unwrap(), pieces[2].line[0]);
        // Interior nodes are preserved (100 m spacing → 400..600 keeps the 500 node).
        assert_eq!(pieces[1].line.len(), 3);
    }

    #[test]
    fn piece_extraction_respects_the_segment_range() {
        // Two segments; the bridge span lies wholly in the second.
        let mut c = corridor(vec![span(0.0, 600.0, 0), span(600.0, 1000.0, 1)], 11, 1000.0);
        c.segments = vec![
            SegmentRef { source: 1, node0: 0, node1: 5, properties: vec![] },
            SegmentRef { source: 2, node0: 5, node1: 10, properties: vec![] },
        ];
        let first = c.pieces(&c.segments[0]);
        assert_eq!(first.len(), 1, "first segment is all grade");
        assert_eq!(first[0].kind, SpanKind::Grade);
        let second = c.pieces(&c.segments[1]);
        assert_eq!(second.len(), 2, "second segment splits at the span edge");
        assert_eq!(second[1].kind, SpanKind::Bridge);
    }

    #[test]
    fn lookup_finds_segments_by_source_hash() {
        let c = corridor(vec![span(0.0, 1000.0, 0)], 3, 1000.0);
        let scene = SceneGraph::new(vec![c]);
        assert!(scene.lookup(1).is_some());
        assert!(scene.lookup(99).is_none());
    }

    #[test]
    fn source_hash_is_stable() {
        assert_eq!(source_hash("abc"), source_hash("abc"));
        assert_ne!(source_hash("abc"), source_hash("abd"));
    }
}
