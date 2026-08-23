//! What the paved surface is built out of: the carriageway stretches, the cuts
//! that trim them at a structure, and the extents of the intersections they
//! meet at (docs/GENERATION.md scenario S4, docs/ROADS.md R6/R10).
//!
//! Three products, all baked once from the solved model and shared by every
//! worker, because heights are a pure function of that model and tiles must
//! agree at their seams (invariant 5):
//!
//! - [`SourceSeg`] — every carriageway stretch with the width it paves, read
//!   along the corridor's *smoothed* sweep line so the band rides the same
//!   centerline the deck is swept along and the ground benched beside
//!   (ROADS.md H2). `synth::pavement` buffers and unions these into the road
//!   surface; `synth::height` uses them as the road height field's sources.
//! - [`Handover`] — the line across the band where an at-grade run ends and a
//!   deck takes over. Recorded here because this is the only walk that still
//!   knows it: the union that follows is a boolean over buffered polylines and
//!   dissolves which input each stretch of boundary came from.
//! - [`Intersection`] — the extent of each place roads meet, with the height
//!   and grade-separation layer its members share. Used as an *extent*: the
//!   marking trim, the height-field pin and the curb-return mask.
//!
//! **This module no longer draws anything.** It was written to mesh a filled
//! plate per intersection, and that is what its name and this comment used to
//! say; the unioned surface (`synth::pavement`) replaced the plates, and
//! [`Area`] has had no meshing operation since. What survived is the model the
//! union is built from, which is why the file is named for that instead.
//!
//! The unit for an intersection is the *place*, not the connector. Overture
//! maps a place where roads meet as however many connectors its geometry
//! needs: a plain crossroads is one, a staggered junction two, a roundabout a
//! dozen ringed around an island. Treating each connector separately is what
//! made a roundabout render as a ring of shards, so connectors are clustered
//! into intersections first ([`cluster`]) and one [`Area`] baked per cluster.
//! A roundabout needs no rule of its own: its ring arcs are short corridors, so
//! the clustering swallows them.
//!
//! A leg's width comes from its corridor's [`Corridor::width_m`] — the same
//! cross-section the surface band reads, so a mouth and the band that lands on
//! it are the same width (ROADS.md invariant 1 and 5); a non-drivable member (a
//! footway, a crossing) joins the intersection without contributing paved area.
//!
//! The extent and the band are not in quite the same *place*, to within the
//! smoothing displacement: the area is built at the mapped connector point with
//! mapped leg headings, while the band is buffered around the smoothed sweep
//! line ([`carriageway_sources`]), a median 0.45 m away. That is inside what
//! the extent is for — it only has to say roughly where the intersection is —
//! but it is the one place the "by construction" in invariant 1 is now an
//! approximation. Moving it would mean choosing *which* member's smoothed line
//! a shared connector sits on, and each corridor smooths its own
//! independently.

use std::collections::HashMap;

use geo_types::Coord;

use crate::assemble::facades::{Facades, Section};
use crate::assemble::grid::GridIndex;
use crate::priors;
use crate::scene::{Corridor, CorridorId, SceneGraph, SpanKind, DEG_M};
use crate::solve::SolvedModel;
use crate::synth::area::{Area, Leg};
use crate::synth::sheets;

/// Nearer than this to a corridor end, in metres, and a junction sits *at* the
/// end: the road does not carry on past it, so there is no leg that way.
const END_EPS_M: f64 = 1.5;

/// How much longer than the two intersections' own reaches a corridor between
/// them may be and still count as one place, in metres. A stub this short is
/// intersection geometry — a slip lane's nose, the offset of a staggered
/// crossroads — not a block of street.
const MERGE_SLACK_M: f64 = 6.0;

/// Widest an intersection cluster may grow, in metres. The merge rule is
/// local and could otherwise walk a dense old town into one lake of asphalt;
/// past this the next merge is refused and the junctions stay separate places.
const MAX_CLUSTER_M: f64 = 45.0;

/// One place roads meet: its extent, the height its members solved to share,
/// and the grade-separation layer that puts it on.
///
/// An *extent*, not a surface. Nothing here is drawn — the union paves the real
/// shape — so this only has to say roughly where the intersection is, for the
/// marking trim, the height-field pin and the curb-return mask.
pub struct Intersection {
    area: Area,
    /// The height the intersection's members solved to share, metres — the mean
    /// over its clustered junctions of [`SolvedModel::junction_height`]. `None`
    /// when no member carried a profile, so nothing is known. This is what the
    /// road height field pins the intersection to; unlike `level_mm` it exists
    /// for a street intersection too, and is a height rather than a decision
    /// about whether to drape.
    height: Option<f64>,
    /// The grade-separation layer the intersection stands on
    /// ([`crate::synth::sheets`]) — the sheet its pin belongs to. Filled in
    /// after the sources are layered, since that is what defines it.
    layer: u32,
}

impl Intersection {
    /// The intersection centre.
    pub fn point(&self) -> Coord {
        self.area.centre()
    }

    /// The paved area, for the band and marking trims.
    pub fn area(&self) -> &Area {
        &self.area
    }

    /// The solved height of the intersection in metres, if it has one.
    pub fn height(&self) -> Option<f64> {
        self.height
    }

    /// The grade-separation layer this intersection's asphalt is on.
    pub fn layer(&self) -> u32 {
        self.layer
    }
}

/// The paved network's inputs, baked once from the solved model and shared by
/// every worker through an `Arc`. Coarse spatial indexes answer "what is near
/// this box" without a linear scan, which the per-segment marking trims (phase
/// 1, millions of segments) and the per-tile height field both depend on.
pub struct CarriagewayModel {
    junctions: Vec<Intersection>,
    grid: HashMap<(i32, i32), Vec<u32>>,
    /// Every carriageway segment in the extract, with the width it paves — the
    /// corridor half of the road height field's sources
    /// ([`crate::synth::height`]). Baked here because it is the same walk over
    /// the scene the intersections come from, and because a height field built
    /// per tile would re-derive it for every zoom.
    sources: Vec<SourceSeg>,
    source_grid: GridIndex,
    /// Where the at-grade band stops because a structure takes over
    /// ([`Handover`]). Recorded here because this is the only walk that still
    /// knows it: the union that follows is a boolean over buffered polylines
    /// and dissolves which input each stretch of boundary came from.
    handovers: Vec<Handover>,
}

/// The line across the band where an at-grade run ends at a span boundary — a
/// bridge abutment or a tunnel portal, in plan.
///
/// It exists so the mesher can tell that piece of boundary apart from a kerb.
/// The two are opposite situations wearing the same shape: a kerb is where the
/// paved surface *ends* and the ground beside it begins, which is what the
/// casing rim is drawn to edge; a handover is where the surface continues onto
/// a deck, and edging it draws a line straight across the carriageway a few
/// tenths of a metre before the bridge.
#[derive(Debug, Clone, Copy)]
pub struct Handover {
    /// The cut's two ends, one per side of the carriageway.
    pub a: Coord,
    pub b: Coord,
}

/// One at-grade stretch of one corridor, and the sheet it stands on.
///
/// It exists so `synth::walkway` can put a sidewalk on the same sheet as the
/// asphalt it borders without re-deriving the layering: `synth::sheets`
/// decides once, here, and the band reads the verdict. A band that decided for
/// itself would occasionally put a sidewalk on the flyover its street passes
/// under.
#[derive(Debug, Clone, Copy)]
pub struct GradeRun {
    pub corridor: CorridorId,
    pub arc0: f64,
    pub arc1: f64,
    pub layer: u32,
    /// Index of this run's first [`SourceSeg`], which is where its layer is
    /// read from once the sheets are assigned.
    pub first: usize,
}

/// One stretch of centerline between two nodes, and how far either side of it
/// that corridor's asphalt reaches. Carries the corridor *id* rather than a
/// borrowed profile so the model stays self-contained and shareable.
#[derive(Debug, Clone, Copy)]
pub struct SourceSeg {
    pub a: Coord,
    pub b: Coord,
    pub cos_lat: f64,
    /// The class's own half-width — what this corridor paves where nothing
    /// crowds it. Still the segment's identity for everything that reasons
    /// about the corridor rather than about the drawn edge: the run chaining,
    /// the handover cut's reach, the chunk padding, and the road height field,
    /// whose support must not shrink away from the bench beside it just
    /// because a wall narrowed the asphalt.
    pub half_m: f64,
    /// What is actually drawn at each end of the stretch — [`half_m`] on both
    /// sides, or less where a facade stands closer than
    /// [`priors::FACADE_CLEAR_M`] outside it.
    ///
    /// [`half_m`]: Self::half_m
    pub sect_a: Section,
    pub sect_b: Section,
    pub level: i64,
    /// Grade-separation layer: which *sheet* of asphalt this stretch belongs to
    /// ([`crate::synth::sheets`]). Zero for anything nothing else stacks over,
    /// so ordinary streets all share a layer and still merge.
    ///
    /// Load-bearing for the union, because Overture's `level` ordinal does not
    /// carry this: a flyover's bridge span is excluded from the union already,
    /// but its *approaches* are ordinary at-grade spans at level 0, and so is the
    /// road they pass over. Keyed on level alone they merged into one region, and
    /// the mesh then ramped continuously between two roads that are metres apart
    /// vertically.
    pub layer: u32,
    /// The structure cross-section bounding this segment's own end of the run,
    /// where one does. The band is buffered *through* the boundary and then
    /// cut by this line, so its edge there is the deck's end face rather than
    /// whatever the buffer's cap happened to butt onto — see
    /// [`carriageway_sources`].
    pub cut_a: Option<Handover>,
    pub cut_b: Option<Handover>,
    /// The solved road-surface height at `a` and at `b`, metres — read at the
    /// segment's own *arc*, never by plan lookup, so a corridor that doubles
    /// back on itself gives each arm its own height instead of the nearer one's.
    /// This is what [`crate::synth::sheets`] compares to decide which
    /// overlapping asphalt is one surface.
    ///
    /// Both ends, not a midpoint: a stretch on a grade is not at one height, and
    /// two stretches are only ever compared *where they meet*.
    pub height_a: f64,
    pub height_b: f64,
    pub corridor: CorridorId,
    /// What this stretch's surface is made of ([`priors::Surface`]): asphalt
    /// and ballast are separate regions with separate materials, so the union
    /// must not merge a carriageway with the rail formation it crosses.
    pub surface: priors::Surface,
    /// How far this stretch's surface stands *above* the height its corridor's
    /// profile gives — the kerb a sidewalk rides
    /// ([`priors::KERB_RISE_M`]). Zero for everything that is the road
    /// surface rather than something standing on it.
    ///
    /// It lives on the source rather than on the region because the two ends
    /// of the same walkway are not alike: the stretch attached to a street
    /// carries the kerb, and the path it continues into stands on the ground.
    /// The height field blends the two, so the rise ramps out over a band's
    /// width instead of stepping — which is what a dropped kerb is.
    pub rise_m: f64,
    /// Arc of `a` along whatever line this stretch was stationed on: its
    /// corridor's, or — for a walkway band with no host — the way's own.
    ///
    /// Carried because a band outlives the loop that built it. It is built
    /// before the ground is derived (the ground under a walkway is that
    /// walkway, `ground::walk_earthworks`) and stamped with its grade layer
    /// after the carriageway has settled its sheets, and both of those readers
    /// need to know *where along its run* a segment sits without re-deriving
    /// it from the plan.
    pub arc0: f64,
}

impl SourceSeg {
    /// The road surface at parameter `t` along `a`→`b`, metres.
    pub fn height_at(&self, t: f64) -> f64 {
        self.height_a + (self.height_b - self.height_a) * t
    }
}

/// Grid cell size in degrees (~1 km): plates per cell stay in the tens even
/// in towns, and a tile or segment query touches a handful of cells.
const GRID_DEG: f64 = 0.01;

fn grid_cell(x: f64, y: f64) -> (i32, i32) {
    ((x / GRID_DEG).floor() as i32, (y / GRID_DEG).floor() as i32)
}

impl CarriagewayModel {
    fn build(
        junctions: Vec<Intersection>,
        sources: Vec<SourceSeg>,
        handovers: Vec<Handover>,
    ) -> CarriagewayModel {
        let mut grid: HashMap<(i32, i32), Vec<u32>> = HashMap::new();
        for (i, j) in junctions.iter().enumerate() {
            let p = j.point();
            grid.entry(grid_cell(p.x, p.y)).or_default().push(i as u32);
        }
        let mut source_grid = GridIndex::new();
        for (i, s) in sources.iter().enumerate() {
            let pad = s.half_m / crate::scene::DEG_M;
            source_grid.insert(
                (
                    s.a.x.min(s.b.x) - pad,
                    s.a.y.min(s.b.y) - pad,
                    s.a.x.max(s.b.x) + pad,
                    s.a.y.max(s.b.y) + pad,
                ),
                i as u32,
            );
        }
        CarriagewayModel { junctions, grid, sources, source_grid, handovers }
    }

    /// Every abutment cut in the extract. Few — one per structure span end —
    /// so the consumers filter them by box rather than index them.
    pub fn handovers(&self) -> &[Handover] {
        &self.handovers
    }

    /// The carriageway segments whose paved band reaches into the
    /// `(west, south, east, north)` box, in a sorted, deduplicated order that is
    /// a function of the model rather than of hashing.
    pub fn sources_near(&self, b: (f64, f64, f64, f64), out: &mut Vec<u32>) {
        self.source_grid.query(b, out);
    }

    /// One carriageway segment by index, as returned by [`Self::sources_near`].
    pub fn source(&self, i: u32) -> &SourceSeg {
        &self.sources[i as usize]
    }

    /// Number of carriageway segments the height field can draw on.
    pub fn source_count(&self) -> usize {
        self.sources.len()
    }

    /// The grade-separation layer of the carriageway nearest `at` whose surface
    /// sits at `height_m` — the sheet a thing standing at that height belongs
    /// to. Used to place an intersection's pin on its own sheet rather than on
    /// whatever passes over or under it.
    fn layer_at_height(&self, at: Coord, height_m: f64, scratch: &mut Vec<u32>) -> u32 {
        self.source_grid.query((at.x, at.y, at.x, at.y), scratch);
        let mut best: Option<(f64, u32)> = None;
        for &i in scratch.iter() {
            let s = &self.sources[i as usize];
            // Read the stretch's surface *beside the pin*, not at its midpoint:
            // on a grade those are different heights and only the near one is
            // the asphalt this intersection stands on.
            let (d, t) = sheets::point_to_segment(at, s.a, s.b, s.cos_lat);
            if (s.height_at(t) - height_m).abs() > sheets::SHEET_SEPARATION_M {
                continue; // a different sheet: not what this pin stands on
            }
            if best.is_none_or(|(bd, _)| d < bd) {
                best = Some((d, s.layer));
            }
        }
        best.map_or(0, |(_, l)| l)
    }

    pub fn len(&self) -> usize {
        self.junctions.len()
    }

    pub fn is_empty(&self) -> bool {
        self.junctions.is_empty()
    }

    pub fn iter(&self) -> impl Iterator<Item = &Intersection> {
        self.junctions.iter()
    }

    /// The plates whose centres fall in the `(west, south, east, north)` box.
    /// The caller pads the box by whatever reach (trim radius, plate size)
    /// matters to it.
    pub fn near(&self, b: (f64, f64, f64, f64)) -> Vec<&Intersection> {
        // Tested against each plate's *area*, not its centre. A star-shaped
        // intersection region reaches far past the padding a tile query carries
        // — a big roundabout's legs run tens of metres — so a centre test drops
        // plates that still cover the tile. That made the height field
        // tile-dependent: two neighbours sharing a border point covered by such
        // a plate disagreed about whether it was pinned at all, which is a
        // 0.36 m step in the drawn asphalt across the seam (invariant 2).
        //
        // The grid cells scanned are widened to match, or the lookup would drop
        // the very plates the area test exists to keep.
        let mut reach = (0.0f64, 0.0f64);
        for j in &self.junctions {
            let r = j.area().reach_deg();
            reach = (reach.0.max(r.0), reach.1.max(r.1));
        }
        let (x0, y0) = grid_cell(b.0 - reach.0, b.1 - reach.1);
        let (x1, y1) = grid_cell(b.2 + reach.0, b.3 + reach.1);
        let mut out = Vec::new();
        for cx in x0..=x1 {
            for cy in y0..=y1 {
                if let Some(cell) = self.grid.get(&(cx, cy)) {
                    for &i in cell {
                        let j = &self.junctions[i as usize];
                        let p = j.point();
                        let r = j.area().reach_deg();
                        if p.x + r.0 >= b.0
                            && p.x - r.0 <= b.2
                            && p.y + r.1 >= b.1
                            && p.y - r.1 <= b.3
                        {
                            out.push(j);
                        }
                    }
                }
            }
        }
        out
    }
}

/// Bakes a plate for every intersection with three or more paved legs. An
/// *engineered* intersection sits at a fixed level — the height the welds made
/// its profiled members share (an interchange is flat at its merge). A street
/// intersection drapes per vertex on the engineered ground instead: its
/// members' benches already agree there (the street weld), and a fixed disc
/// would cut into the slope the intersection genuinely sits on. One with no
/// profiled member has no known height and is skipped.
pub fn bake(
    scene: &SceneGraph,
    solved: &SolvedModel,
    facades: &Facades,
    walk_bands: Vec<SourceSeg>,
) -> CarriagewayModel {
    let ports = Ports::build(scene);
    let clusters = cluster(scene, &ports);
    let mut junctions = Vec::new();
    for c in &clusters {
        if let Some(b) = bake_one(scene, solved, &ports, c) {
            junctions.push(b);
        }
    }
    // The grade-separation layer is measured off the solved heights, per
    // carriageway stretch (`synth::sheets`), not read off the mapped bridge
    // spans per corridor. Sources are built first because the layering is a
    // property of how they overlap, then stamped back onto them.
    let (mut sources, handovers, mut grade_runs) = carriageway_sources(scene, solved, facades);
    let layers = sheets::assign(scene, &sources);
    for (s, &l) in sources.iter_mut().zip(layers.iter()) {
        s.layer = l;
    }
    // The walkway bands ride the sheets the carriageway just settled, and are
    // appended *after* the assignment: a sidewalk must not vote on the
    // grade-separation layering of the street it stands beside. They arrive
    // already built — the ground was benched from these same segments before
    // this stage ran (`synth::walkway::bands`) — and all that is left is the
    // sheet each one rides.
    for r in &mut grade_runs {
        r.layer = layers.get(r.first).copied().unwrap_or(0);
    }
    let mut walk_bands = walk_bands;
    super::walkway::stamp_layers(&mut walk_bands, &grade_runs);
    sources.extend(walk_bands);
    let mut model = CarriagewayModel::build(junctions, sources, handovers);
    // An intersection pins the sheet it stands on, which is the sheet of the
    // asphalt at its own solved height. Resolved after the sources are stamped,
    // because that is when there is a layering to read.
    let mut scratch = Vec::new();
    for i in 0..model.junctions.len() {
        let (p, h) = (model.junctions[i].point(), model.junctions[i].height);
        model.junctions[i].layer = h.map_or(0, |h| model.layer_at_height(p, h, &mut scratch));
    }
    model
}

/// Every carriageway segment of every corridor that paves anything, in corridor
/// then node order. The height field's corridor sources; also the input the
/// unioned surface buffers. Layers are stamped afterwards by [`bake`].
fn carriageway_sources(
    scene: &SceneGraph,
    solved: &SolvedModel,
    facades: &Facades,
) -> (Vec<SourceSeg>, Vec<Handover>, Vec<GradeRun>) {
    let mut out = Vec::new();
    let mut handovers = Vec::new();
    let mut grade_runs: Vec<GradeRun> = Vec::new();
    // `ARPT_NO_ABUTMENT_CUT=1` stops the band on the boundary again and
    // withholds the cuts with it, so an A/B re-tile of this is a flag rather
    // than a patch — the same reason `--no-hole` exists. Read once: it is a
    // constant for the run, and this loop is every carriageway in the extract.
    let no_cut = std::env::var_os("ARPT_NO_ABUTMENT_CUT").is_some();
    // `ARPT_NO_FACADE_ROOM=1` gives every street the open-ground cross-section
    // again, for the same reason.
    let no_room = std::env::var_os("ARPT_NO_FACADE_ROOM").is_some();
    let mut scratch: Vec<u32> = Vec::new();
    for c in &scene.corridors {
        let Some(half_m) = corridor_half_width_m(c) else {
            continue; // not a carriageway: paves nothing, so covers nothing
        };
        let profile = solved.profile(c.id);
        // Read at the stretch's own *arc*. `Profile::height_at` projects onto
        // the nearest corridor edge in plan, which at a hairpin is the other
        // arm — and telling the two arms apart is precisely what this height is
        // for.
        let at = |arc: f64| profile.map_or(0.0, |p| p.road_at_arc(arc));
        for (a0, a1, level, kind) in level_runs(c) {
            // Only at-grade asphalt is unioned. A bridge or a bore already
            // carries its road surface as a swept solid (`synth::structure`), so
            // paving its level here would draw the carriageway twice — once as a
            // deck top and once as a region floating at the same height.
            if kind != SpanKind::Grade {
                continue;
            }
            // Each end of this run that a span is on the other side of. A run
            // covers the corridor end to end ([`level_runs`]), so an at-grade
            // run starting past zero or ending short of the total has a
            // structure beside it there and nothing else can.
            //
            // **and the line the band is cut by there.** Two ends cut from one
            // arc still do not meet, because each side turns that arc into
            // geometry its own way: the deck lays down an explicit cross-section
            // (`Profile::deck_nodes`), while the band gets whatever
            // `poly::buffer_line` butts onto the last chord of its polyline, a
            // station's worth of curve back. The two lines cross at the
            // centreline and diverge to either kerb — bare ground at one, an
            // overlap at the other. Montreux z16 measured 43 joints gapping
            // 0.15-0.6 m, 40 of them with an overlapping edge on the same cap.
            //
            // Neither side is patched here. The band is buffered *through* the
            // boundary by [`STRUCTURE_OVERRUN_M`] and then **cut by the deck's
            // own cross-section** ([`handover_cut`], applied in
            // `synth::pavement::bake_chunk`): generate long, trim to the thing
            // it must meet. The shared edge is then one set of coordinates
            // because one shape cut the other, which is a different kind of
            // agreement from two constructions arriving at the same place.
            let total = *c.arc.last().unwrap_or(&0.0);
            let at_span = |arc: f64, at_end: bool| {
                if at_end {
                    arc < total - RUN_EPS_M
                } else {
                    arc > RUN_EPS_M
                }
            };
            let cut_lo = at_span(a0, false)
                .then(|| handover_cut(c, profile, a0, half_m))
                .flatten();
            let cut_hi = at_span(a1, true)
                .then(|| handover_cut(c, profile, a1, half_m))
                .flatten();
            handovers.extend(cut_lo.iter().chain(cut_hi.iter()).copied());
            let (cut_lo, cut_hi) = if no_cut { (None, None) } else { (cut_lo, cut_hi) };
            // Generate long: the band runs on into the span and the cut takes it
            // back to the deck's face. Only where the run has the length to
            // spare — on a stretch shorter than two overruns the two cuts would
            // pass each other and leave nothing.
            let overrun = if no_cut { 0.0 } else { STRUCTURE_OVERRUN_M };
            let room = ((a1 - a0) * 0.5).min(overrun).max(0.0);
            let lo = if cut_lo.is_some() { a0 - room } else { a0 };
            let hi = if cut_hi.is_some() { a1 + room } else { a1 };
            // The stations this run is buffered around. **The corridor's own
            // solved stations, not its mapped vertices** — the band rides the
            // smoothed sweep line (below), and a smoothed curve sampled at the
            // mapped vertices is chorded straight back across every correction
            // between them. The profile's densified nodes are where the
            // smoothing lives, so they are where the band has to be read; a
            // corridor with no profile keeps its own vertices, since there is
            // no other curve to sample.
            let stations: Vec<f64> = match profile {
                Some(p) => p.arc().iter().copied().filter(|&s| s > lo && s < hi).collect(),
                None => c.arc.iter().copied().filter(|&s| s > lo && s < hi).collect(),
            };
            // Both ends are exact: the run boundary is where the deck begins,
            // so the band has to end *there* and not at the nearest station.
            let mut stops: Vec<f64> = Vec::with_capacity(stations.len() + 2);
            stops.push(lo);
            for s in stations.into_iter().chain(std::iter::once(hi)) {
                if s - stops[stops.len() - 1] > RUN_EPS_M {
                    stops.push(s);
                }
            }
            if stops.len() < 2 {
                continue;
            }
            // Where this at-grade run's sources begin, so `synth::sheets`'
            // verdict on them can be read back and handed to the walkway that
            // rides the same stretch — one sheet decision, not two.
            grade_runs.push(GradeRun {
                corridor: c.id,
                arc0: a0,
                arc1: a1,
                layer: 0,
                first: out.len(),
            });
            // **One centerline** (docs/ROADS.md H2, invariant 5): the band is
            // buffered around the same smoothed line the deck is swept along
            // and the ground benched beside, so nothing steps at the abutment
            // where one hands over to the other.
            let point = |arc: f64| match profile {
                Some(p) => p.smooth_at_arc(arc),
                None => raw_point_at_arc(c, arc),
            };
            let pts: Vec<Coord> = stops.iter().map(|&s| point(s)).collect();
            let sections = sections_along(c, &stops, &pts, half_m, facades, no_room, &mut scratch);
            for i in 0..stops.len() - 1 {
                out.push(SourceSeg {
                    a: pts[i],
                    b: pts[i + 1],
                    cos_lat: c.cos_lat,
                    half_m,
                    sect_a: sections[i],
                    sect_b: sections[i + 1],
                    level,
                    layer: 0,
                    cut_a: (i == 0).then_some(cut_lo).flatten(),
                    cut_b: (i + 2 == stops.len()).then_some(cut_hi).flatten(),
                    height_a: at(stops[i].clamp(a0, a1)),
                    height_b: at(stops[i + 1].clamp(a0, a1)),
                    corridor: c.id,
                    surface: c.kind.prior().surface,
                    rise_m: 0.0,
                    arc0: stops[i],
                });
            }
        }
    }
    (out, handovers, grade_runs)
}

/// The cross-section at every station of one run: the class prior on both
/// sides, narrowed where a facade stands closer than [`priors::FACADE_CLEAR_M`]
/// outside the asphalt, and never below [`priors::MIN_CARRIAGEWAY_HALF_M`].
///
/// **The allocation order is the model, and asphalt is last in it.** A
/// footprint carries its own plan error while a carriageway width is a survey
/// prior, so the room a wall leaves is spent on the sidewalk and the verge
/// before any of it is taken off the road (`data/plans` — the street as a room
/// between facades). Phase 2 has only asphalt to spend it on, so the whole cap
/// lands here; when the walk band and the per-side bench exist they take their
/// share first and this becomes the remainder.
///
/// Asphalt only. A railway through a building is a station under its roof, not
/// a formation drawn through a wall, and narrowing the ballast there would
/// shave the platform it stands on — the same exclusion `order.building_overlap`
/// makes on the measuring side.
pub(crate) fn sections_along(
    c: &Corridor,
    stops: &[f64],
    pts: &[Coord],
    half_m: f64,
    facades: &Facades,
    no_room: bool,
    scratch: &mut Vec<u32>,
) -> Vec<Section> {
    let uniform = vec![Section::uniform(half_m); stops.len()];
    if no_room || facades.is_empty() || c.kind.prior().surface != priors::Surface::Asphalt {
        return uniform;
    }
    let m_lon = DEG_M * c.cos_lat;
    let reach = half_m + priors::FACADE_CLEAR_M;
    let mut out = uniform;
    for i in 0..stops.len() {
        // The tangent is a central difference where there is one, so a station
        // reads the direction the road runs *through* it rather than the
        // direction of whichever chord happens to be indexed with it.
        let (j, k) = (i.saturating_sub(1), (i + 1).min(stops.len() - 1));
        if j == k {
            continue;
        }
        let (dx, dy) = ((pts[k].x - pts[j].x) * m_lon, (pts[k].y - pts[j].y) * DEG_M);
        let len = dx.hypot(dy);
        if !(len > 0.0) {
            continue;
        }
        // **A station is responsible for its own stretch of centerline**, so it
        // looks at least as far along the road as the gap to its neighbours.
        // That is what makes consecutive stations see the same wall: a facade
        // is caught by every station whose window it falls in, so the two that
        // bracket its ends are both narrowed and the width interpolated between
        // them never crosses it. A shorter window would let a wall between two
        // stations go unseen; a longer one only tapers the street sooner.
        let window = (stops[k] - stops[j])
            .max(ROOM_WINDOW_MIN_M)
            .min(ROOM_WINDOW_MAX_M);
        let room =
            facades.room(pts[i], c.cos_lat, (dx / len, dy / len), reach, window, scratch);
        out[i] = room.allot(half_m, priors::MIN_CARRIAGEWAY_HALF_M);
    }
    out
}

/// Shortest and longest stretch of centerline, in metres, that one station
/// looks along for the walls beside it ([`sections_along`]). The floor keeps a
/// densified profile's centimetre stations from each reading a slit of wall;
/// the ceiling bounds the query on a corridor whose mapped vertices are
/// hundreds of metres apart, where the cap would otherwise reach far past
/// anything it could be said to measure.
pub(crate) const ROOM_WINDOW_MIN_M: f64 = 4.0;
pub(crate) const ROOM_WINDOW_MAX_M: f64 = 32.0;

/// The cut across the band at arc `arc` — the line the at-grade run ends on and
/// the deck begins on, both being swept from that same station on the same
/// smoothed centerline.
///
/// Extended a little past the paved half-width on each side. The union can
/// widen the band beyond its own buffer where a fillet or a second carriageway
/// reaches the abutment, and a cut that stopped at the half-width would leave
/// the outer corner of a widened cut still wearing a kerb line.
fn handover_cut(
    c: &Corridor,
    profile: Option<&crate::solve::Profile>,
    arc: f64,
    half_m: f64,
) -> Option<Handover> {
    let profile = profile?;
    // **The deck's own cross-section**, not a second derivation of it. The
    // sweep places its end face at `deck_nodes`' position with `deck_nodes`'
    // left vector (`synth::structure::sweep_deck`), so asking the same
    // function the same question is what makes this line *be* that face rather
    // than agree with it to within whatever two constructions have in common.
    // The band is then cut by it, so the two share coordinates instead of
    // sharing an intention.
    // The point is taken in the *profile's* own arc space and handed back to
    // the profile, so `deck_nodes` walks to the station this cut is for. Going
    // through the corridor's arc instead would work only where the two
    // parameterisations coincide, which is a thing that happens to be true
    // rather than a thing that is arranged.
    let node = profile.deck_nodes(&[profile.point_at_arc(arc)]).into_iter().next()?;
    let len = (node.left_e * node.left_e + node.left_n * node.left_n).sqrt();
    if !(len > 0.0) {
        return None;
    }
    // Out to the reach the band can occupy — wider than the deck, since the
    // cut has to remove *all* of the overrun, including whatever a fillet or a
    // second carriageway added to it, and everything past the cut is the
    // deck's to draw.
    let reach = half_m + CUT_OVERREACH_M;
    let deg_m = crate::scene::DEG_M;
    let (ex, ey) = (
        node.left_e / len * reach / (deg_m * c.cos_lat),
        node.left_n / len * reach / deg_m,
    );
    Some(Handover {
        a: Coord { x: node.lon - ex, y: node.lat - ey },
        b: Coord { x: node.lon + ex, y: node.lat + ey },
    })
}

/// How far past its own half-width a handover cut reaches, in metres — the
/// slack that keeps a widened or filleted abutment fully covered. Half a
/// curb-return radius: enough for the fillet material at a junction that
/// happens to sit on an abutment, and short enough that the cut cannot reach a
/// second carriageway that merely passes nearby.
const CUT_OVERREACH_M: f64 = 0.5 * crate::priors::CURB_RETURN_M;

/// A run shorter than this is float slop at a boundary, not a stretch of road.
pub(crate) const RUN_EPS_M: f64 = 1e-6;

/// How far the band is buffered *past* a structure boundary before being cut
/// back to the deck's face, in metres.
///
/// Generate long, trim to the thing it must meet: the overrun exists only to
/// guarantee there is material on the far side of the cut for the cut to
/// remove, so it needs to exceed the two constructions' disagreement and
/// nothing more. That disagreement is `half_w · θ`, θ being the turn the
/// buffer's last chord spans — 0.4 m at the profile's ~4 m stations on a 25 m
/// ramp radius. A metre and a half covers it several times over and is still
/// short enough that a short at-grade stretch between two structures keeps a
/// middle.
///
/// None of it is drawn: everything past the cut is removed before the union.
const STRUCTURE_OVERRUN_M: f64 = 1.5;

/// The point at arc `a` on a corridor's own mapped polyline — the fallback for
/// a corridor the solve returned no profile for, which therefore has no
/// smoothed line to read.
pub(crate) fn raw_point_at_arc(c: &Corridor, a: f64) -> Coord {
    let n = c.nodes.len();
    if n < 2 {
        return c.nodes.first().copied().unwrap_or(Coord { x: 0.0, y: 0.0 });
    }
    let i = match c.arc.binary_search_by(|v| v.partial_cmp(&a).expect("finite arc")) {
        Ok(i) => i.min(n - 2),
        Err(i) => i.saturating_sub(1).min(n - 2),
    };
    let span = c.arc[i + 1] - c.arc[i];
    let t = if span > 0.0 { ((a - c.arc[i]) / span).clamp(0.0, 1.0) } else { 0.0 };
    let (p, q) = (c.nodes[i], c.nodes[i + 1]);
    Coord { x: p.x + (q.x - p.x) * t, y: p.y + (q.y - p.y) * t }
}

/// The corridor's runs of constant `(level, kind)`, in **arc**, as
/// `(arc0, arc1, level, kind)` covering the whole corridor end to end.
///
/// A corridor's level lives on its spans, not on the corridor
/// (`scene.rs:41-47`), so anything that partitions by level has to get it from
/// them.
///
/// **In arc, because a span boundary is an arc.** It falls where the solve put
/// the abutment, which is almost never on a mapped vertex, and the boundary is
/// where the deck begins — so the band must end exactly there or the two are
/// not one surface. Rounding it to a node is a hole or an overlap of up to half
/// a segment, and a mapper's vertex spacing decides which and how big: measured
/// on the Montreux extract, a quarter of all abutments had bare ground drawn
/// between the approach and the deck, out to 19 m of it (`seam.abutment_bare`).
///
/// Three rules have been tried here and only this one is exact. Widening a node
/// range to cover the boundary let the at-grade runs on either side of a bridge
/// meet in the middle and pave the whole flyover. Truncating it dropped a
/// segment of asphalt at *every* boundary, which at an interchange reads as a
/// row of holes punched across the carriageway. Assigning each segment to the
/// span holding its midpoint gave every segment exactly one owner and so could
/// do neither — but it still moved the boundary to the nearest half-segment,
/// which is this metric's whole population. Cutting the segment at the boundary
/// keeps the one-owner property and puts the cut where it belongs.
pub(crate) fn level_runs(c: &Corridor) -> Vec<(f64, f64, i64, SpanKind)> {
    let n = c.nodes.len();
    if n < 2 {
        return Vec::new();
    }
    let total = c.arc[n - 1];
    if c.spans.is_empty() {
        return vec![(0.0, total, 0, SpanKind::Grade)];
    }
    // Spans in arc order, with the gaps between them at grade: a corridor is
    // covered end to end, and what no span claims is road on the ground.
    let mut spans: Vec<&crate::scene::Span> = c.spans.iter().collect();
    spans.sort_by(|a, b| a.arc0.total_cmp(&b.arc0));
    let mut out: Vec<(f64, f64, i64, SpanKind)> = Vec::new();
    let mut cursor = 0.0;
    for s in spans {
        let (a0, a1) = (s.arc0.clamp(0.0, total), s.arc1.clamp(0.0, total));
        if a1 - cursor <= RUN_EPS_M {
            continue; // wholly behind us: overlapping spans, first claim wins
        }
        let a0 = a0.max(cursor);
        if a0 - cursor > RUN_EPS_M {
            out.push((cursor, a0, 0, SpanKind::Grade));
        }
        if a1 - a0 > RUN_EPS_M {
            out.push((a0, a1, s.level, s.kind));
        }
        cursor = a1;
    }
    if total - cursor > RUN_EPS_M {
        out.push((cursor, total, 0, SpanKind::Grade));
    }
    out
}

/// Where every junction sits along every corridor: per corridor, its junctions
/// in arc order. This is the adjacency the whole module runs on — which
/// junctions a corridor joins, and how far apart they are along it.
struct Ports {
    by_corridor: HashMap<CorridorId, Vec<(f64, u32)>>,
}

impl Ports {
    fn build(scene: &SceneGraph) -> Ports {
        let mut by_corridor: HashMap<CorridorId, Vec<(f64, u32)>> = HashMap::new();
        for (i, j) in scene.junctions.iter().enumerate() {
            for m in &j.members {
                by_corridor.entry(m.corridor).or_default().push((m.arc, i as u32));
            }
        }
        for v in by_corridor.values_mut() {
            v.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        }
        Ports { by_corridor }
    }

    /// The junction next along `corridor` from arc `at`, in the direction
    /// `forward` (increasing arc) or back, with the arc gap to it.
    fn neighbour(&self, corridor: CorridorId, at: f64, forward: bool) -> Option<(u32, f64)> {
        let ports = self.by_corridor.get(&corridor)?;
        if forward {
            ports
                .iter()
                .find(|&&(arc, _)| arc > at + END_EPS_M)
                .map(|&(arc, j)| (j, arc - at))
        } else {
            ports
                .iter()
                .rev()
                .find(|&&(arc, _)| arc < at - END_EPS_M)
                .map(|&(arc, j)| (j, at - arc))
        }
    }

    /// Every corridor edge between two junctions: `(j0, j1, arc gap)`.
    fn edges(&self) -> Vec<(u32, u32, f64)> {
        let mut out = Vec::new();
        let mut corridors: Vec<&CorridorId> = self.by_corridor.keys().collect();
        corridors.sort_unstable();
        for c in corridors {
            for w in self.by_corridor[c].windows(2) {
                let gap = w[1].0 - w[0].0;
                if gap > END_EPS_M {
                    out.push((w[0].1, w[1].1, gap));
                }
            }
        }
        out
    }
}

/// One intersection: the junctions that collapsed into it, and their centre.
struct Cluster {
    members: Vec<u32>,
    centre: Coord,
}

/// Disjoint-set over junction indices, the one structure the clustering needs.
struct Union {
    parent: Vec<u32>,
}

impl Union {
    fn new(n: usize) -> Union {
        Union { parent: (0..n as u32).collect() }
    }

    fn find(&mut self, mut i: u32) -> u32 {
        while self.parent[i as usize] != i {
            self.parent[i as usize] = self.parent[self.parent[i as usize] as usize];
            i = self.parent[i as usize];
        }
        i
    }

    fn union(&mut self, a: u32, b: u32) -> bool {
        let (ra, rb) = (self.find(a), self.find(b));
        if ra == rb {
            return false;
        }
        self.parent[rb.max(ra) as usize] = ra.min(rb);
        true
    }
}

/// Groups the scene's junctions into intersections. One rule, strictly local:
/// a corridor too short to be a block joins the intersections at its ends. That
/// covers a staggered crossroads, the two connectors of a slip lane's nose, and
/// the ring of connectors a roundabout is cut into — all the cases where
/// per-connector plates used to pile shards on one another. Merges are tried
/// shortest-first and refused once a cluster reaches [`MAX_CLUSTER_M`], so a
/// dense old town cannot chain into one lake of asphalt. The result is
/// deterministic: every candidate list is walked in a sorted order, never a
/// hashed one.
fn cluster(scene: &SceneGraph, ports: &Ports) -> Vec<Cluster> {
    let n = scene.junctions.len();
    let mut uf = Union::new(n);
    let mut edges = ports.edges();
    edges.sort_by(|a, b| a.2.total_cmp(&b.2).then(a.0.cmp(&b.0)).then(a.1.cmp(&b.1)));
    let reach: Vec<f64> = (0..n).map(|i| junction_reach_m(scene, i)).collect();
    let mut extent: Vec<(f64, f64, f64, f64)> = scene
        .junctions
        .iter()
        .map(|j| (j.point.x, j.point.y, j.point.x, j.point.y))
        .collect();
    for (a, b, gap) in edges {
        if gap > reach[a as usize] + reach[b as usize] + MERGE_SLACK_M {
            continue;
        }
        let (ra, rb) = (uf.find(a), uf.find(b));
        if ra == rb {
            continue;
        }
        let merged = union_box(extent[ra as usize], extent[rb as usize]);
        if box_extent_m(merged, scene.junctions[a as usize].point.y) > MAX_CLUSTER_M {
            continue;
        }
        uf.union(a, b);
        extent[uf.find(a) as usize] = merged;
    }

    // Collect, in junction order so the output never depends on hashing.
    let mut by_root: HashMap<u32, Vec<u32>> = HashMap::new();
    for j in 0..n as u32 {
        by_root.entry(uf.find(j)).or_default().push(j);
    }
    let mut roots: Vec<u32> = by_root.keys().copied().collect();
    roots.sort_unstable();
    roots
        .into_iter()
        .map(|root| {
            let members = by_root.remove(&root).expect("a root has members");
            let centre = centroid(scene, &members);
            Cluster { members, centre }
        })
        .collect()
}

/// How far one junction's own paved area reaches, in metres — its widest
/// member's half-width. The merge rules compare corridor lengths against this:
/// a stub shorter than the areas that would meet over it is not a street.
///
/// Asphalt members only, here and in [`bake_one`]: an intersection plate is a
/// *flat* entity — one height its legs share — and that is the road model. A
/// rail junction is a switch on whatever grade its line climbs, and pinning
/// the height field flat across one tilts the formation around it: measured
/// as an 828 % interior face at the Chamby rail junction. Rail corridors
/// contribute carriageway *sources* (the union and the field need them) and
/// never plates, pins or closing masks.
fn junction_reach_m(scene: &SceneGraph, j: usize) -> f64 {
    scene.junctions[j]
        .members
        .iter()
        .filter(|m| plates(&scene.corridors[m.corridor as usize]))
        .filter_map(|m| corridor_half_width_m(&scene.corridors[m.corridor as usize]))
        .fold(0.0, f64::max)
}

/// Whether a corridor's class takes part in intersection plates — see
/// [`junction_reach_m`] for why this is asphalt-only.
fn plates(c: &Corridor) -> bool {
    c.kind.prior().surface == priors::Surface::Asphalt
}

/// The half-width in metres of a corridor's surface band — its carriageway
/// plus the class's shoulder, or its rail formation outright
/// ([`priors::Prior::shoulder_m`]), exactly what `synth::surface` offsets to.
/// `None` for a corridor with no surface of its own: a footway or a crossing
/// joins an intersection without paving any of it.
pub(crate) fn corridor_half_width_m(c: &Corridor) -> Option<f64> {
    let prior = c.kind.prior();
    (prior.surface != priors::Surface::None).then_some(())?;
    Some(c.width_m? * 0.5 + prior.shoulder_m())
}

/// Bakes one cluster's plate, or `None` when it paves nothing: fewer than
/// three paved legs, no member with a solved profile, or a degenerate area.
fn bake_one(
    scene: &SceneGraph,
    solved: &SolvedModel,
    ports: &Ports,
    cluster: &Cluster,
) -> Option<Intersection> {
    let centre = cluster.centre;
    let m_lon = DEG_M * centre.y.to_radians().cos();

    let mut legs: Vec<Leg> = Vec::new();
    let mut paves = false;
    let mut has_profiled_member = false;
    let mut offset_max = 0.0f64;
    let mut half_max = 0.0f64;
    // The solved height, averaged over the cluster's junctions. A merged cluster
    // has one per member junction and they may differ by centimetres across a
    // roundabout; the mean is order-independent, so the pin is deterministic.
    let mut pin_sum = 0.0f64;
    let mut pin_count = 0u32;

    for &j in &cluster.members {
        if let Some(h) = solved.junction_height(j as usize) {
            pin_sum += h;
            pin_count += 1;
        }
        let jn = &scene.junctions[j as usize];
        let off = ((jn.point.x - centre.x) * m_lon, (jn.point.y - centre.y) * DEG_M);
        offset_max = offset_max.max(off.0.hypot(off.1));
        for m in &jn.members {
            let c = &scene.corridors[m.corridor as usize];
            if !plates(c) {
                continue; // rail joins here but takes no part in the plate
            }
            has_profiled_member |= solved.profile(m.corridor).is_some();
            let Some(half_w) = corridor_half_width_m(c) else {
                continue; // a footway joins here but paves nothing
            };
            half_max = half_max.max(half_w);
            paves = true;
            for (e, n) in outward_headings(scene, ports, &cluster.members, m.corridor, m.arc) {
                // Each leg is its own carriageway's half-width, no more. This
                // used to be widened by the leg's offset from the cluster centre
                // so the centre-anchored rectangle still *covered* the
                // carriageway it stood for — necessary while the area was the
                // paved plate, and pure harm now that it is only the
                // intersection's extent: a merged crossroads got a visibly
                // over-wide blob of invented asphalt. The union paves the real
                // shape; this extent only has to say roughly where the
                // intersection is, for the marking trim, the height-field pin and
                // the curb-return mask.
                legs.push(Leg { e, n, half_w });
            }
        }
    }
    if !has_profiled_member {
        return None; // nothing is known about where this intersection sits
    }
    if !paves {
        return None; // no member paves anything
    }
    if legs.len() < 3 {
        return None;
    }
    // Legs run from the centre out to the widest carriageway's far edge: a
    // narrow street is paved right across the road it meets, and no leg
    // overshoots into asphalt its own band should be laying.
    let area = Area::new(centre, legs, half_max + offset_max)?;

    Some(Intersection {
        area,
        height: (pin_count > 0).then(|| pin_sum / pin_count as f64),
        layer: 0, // stamped by `bake` once the sources are layered
    })
}

/// The unit ENU headings a corridor leaves this intersection on, at arc `at`.
/// A corridor running on past the junction leaves both ways, one ending there
/// leaves one — and a direction whose next junction is in the same cluster
/// leaves *nothing*: that is the intersection's own interior (a roundabout's
/// arc, a slip lane's throat), already paved by the disc, and a leg along it
/// would rake a rectangle across the middle.
fn outward_headings(
    scene: &SceneGraph,
    ports: &Ports,
    cluster: &[u32],
    corridor: CorridorId,
    at: f64,
) -> Vec<(f64, f64)> {
    let c = &scene.corridors[corridor as usize];
    if c.nodes.len() < 2 {
        return Vec::new();
    }
    let i = edge_at(&c.arc, at);
    let (a, b) = (c.nodes[i], c.nodes[i + 1]);
    let (de, dn) = ((b.x - a.x) * c.cos_lat, b.y - a.y);
    let len = (de * de + dn * dn).sqrt();
    if len < 1e-12 {
        return Vec::new();
    }
    let (e, n) = (de / len, dn / len);
    let total = c.total();
    let mut out = Vec::new();
    let interior = |nb: Option<(u32, f64)>| -> bool {
        nb.is_some_and(|(n, _)| cluster.binary_search(&n).is_ok())
    };
    if at < total - END_EPS_M && !interior(ports.neighbour(corridor, at, true)) {
        out.push((e, n));
    }
    if at > END_EPS_M && !interior(ports.neighbour(corridor, at, false)) {
        out.push((-e, -n));
    }
    out
}

/// The edge index whose arc span contains `at` (clamped to a valid edge).
fn edge_at(arc: &[f64], at: f64) -> usize {
    match arc.binary_search_by(|v| v.partial_cmp(&at).expect("finite arc")) {
        Ok(i) => i.min(arc.len() - 2),
        Err(i) => i.saturating_sub(1).min(arc.len() - 2),
    }
}

/// The mean position of a set of junctions.
fn centroid(scene: &SceneGraph, members: &[u32]) -> Coord {
    let n = members.len().max(1) as f64;
    let (mut x, mut y) = (0.0, 0.0);
    for &m in members {
        x += scene.junctions[m as usize].point.x;
        y += scene.junctions[m as usize].point.y;
    }
    Coord { x: x / n, y: y / n }
}

fn union_box(a: (f64, f64, f64, f64), b: (f64, f64, f64, f64)) -> (f64, f64, f64, f64) {
    (a.0.min(b.0), a.1.min(b.1), a.2.max(b.2), a.3.max(b.3))
}

/// The diagonal of a lon/lat box in metres.
fn box_extent_m(b: (f64, f64, f64, f64), lat: f64) -> f64 {
    let m_lon = DEG_M * lat.to_radians().cos();
    ((b.2 - b.0) * m_lon).hypot((b.3 - b.1) * DEG_M)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::priors::{Kind, RoadClass};
    use crate::scene::{Span, SpanKind};

    /// A straight corridor of `n` nodes, 10 m apart, at lat 46.
    fn corridor(width_m: f64, n: usize) -> Corridor {
        Corridor {
            id: 0,
            nodes: (0..n).map(|i| Coord { x: 6.0 + i as f64 * 1e-4, y: 46.0 }).collect(),
            arc: (0..n).map(|i| i as f64 * 10.0).collect(),
            cos_lat: 46f64.to_radians().cos(),
            kind: Kind::Road(RoadClass::Residential),
            class_key: "residential".to_string(),
            link: false,
            width_m: Some(width_m),
            spans: Vec::new(),
            segments: Vec::new(),
            connectors: Vec::new(),
        }
    }

    /// Every point of the corridor belongs to exactly one run: the runs are
    /// contiguous, in order, and reach both ends. Stated as a tiling rather
    /// than per segment, because the runs are now cut in arc and a boundary
    /// falls wherever the solve put the abutment.
    fn assert_tiles(runs: &[(f64, f64, i64, SpanKind)], total: f64) {
        assert!(!runs.is_empty(), "no runs");
        assert!(runs[0].0.abs() < 1e-9, "first run starts at {}, not 0: {runs:?}", runs[0].0);
        for w in runs.windows(2) {
            assert!(
                (w[1].0 - w[0].1).abs() < 1e-9,
                "run {:?} does not begin where {:?} ends",
                w[1],
                w[0]
            );
        }
        let end = runs[runs.len() - 1].1;
        assert!((end - total).abs() < 1e-9, "last run ends at {end}, not {total}: {runs:?}");
    }

    #[test]
    fn level_runs_cover_every_metre_once_per_level() {
        let mut c = corridor(6.0, 11);
        // No spans: one at-grade run over the whole corridor.
        assert_eq!(level_runs(&c), vec![(0.0, 100.0, 0, SpanKind::Grade)]);
        // Grade / bridge / grade: the bridge is its own level.
        c.spans = vec![
            Span { arc0: 0.0, arc1: 40.0, level: 0, kind: SpanKind::Grade },
            Span { arc0: 40.0, arc1: 60.0, level: 1, kind: SpanKind::Bridge },
            Span { arc0: 60.0, arc1: 100.0, level: 0, kind: SpanKind::Grade },
        ];
        let runs = level_runs(&c);
        assert_eq!(runs.len(), 3, "three runs: {runs:?}");
        assert_eq!(runs[1].2, 1, "the middle run is the bridge level");
        assert_eq!(runs[1].3, SpanKind::Bridge, "and it is a bridge, so the union skips it");
        assert_tiles(&runs, 100.0);

        // The boundary case that matters: spans that end *between* nodes. The
        // cut lands on the boundary itself, so the band ends where the deck
        // begins instead of at the nearest mapped vertex — neither a hole nor
        // an overlap of half a segment.
        c.spans = vec![
            Span { arc0: 0.0, arc1: 35.0, level: 0, kind: SpanKind::Grade },
            Span { arc0: 35.0, arc1: 65.0, level: 1, kind: SpanKind::Bridge },
            Span { arc0: 65.0, arc1: 100.0, level: 0, kind: SpanKind::Grade },
        ];
        let runs = level_runs(&c);
        assert_tiles(&runs, 100.0);
        assert_eq!(runs[1].0, 35.0, "the bridge run starts at the boundary: {runs:?}");
        assert_eq!(runs[1].1, 65.0, "and ends at it: {runs:?}");

        // A span that does not reach an end leaves at-grade road either side.
        c.spans = vec![Span { arc0: 20.0, arc1: 30.0, level: 1, kind: SpanKind::Bridge }];
        let runs = level_runs(&c);
        assert_tiles(&runs, 100.0);
        assert_eq!(runs.len(), 3, "grade, bridge, grade: {runs:?}");
        // A degenerate corridor yields nothing.
        c.nodes.truncate(1);
        assert!(level_runs(&c).is_empty());
    }

    /// Plan distance in metres from `q` to a polyline.
    fn to_polyline(pts: &[Coord], cos_lat: f64, q: Coord) -> f64 {
        let m = |c: Coord| (c.x * cos_lat * DEG_M, c.y * DEG_M);
        let (qx, qy) = m(q);
        let mut best = f64::INFINITY;
        for w in pts.windows(2) {
            let ((ax, ay), (bx, by)) = (m(w[0]), m(w[1]));
            let (ex, ey) = (bx - ax, by - ay);
            let len2 = ex * ex + ey * ey;
            let t = if len2 > 0.0 { (((qx - ax) * ex + (qy - ay) * ey) / len2).clamp(0.0, 1.0) } else { 0.0 };
            best = best.min((qx - (ax + ex * t)).hypot(qy - (ay + ey * t)));
        }
        best
    }

    #[test]
    fn the_band_rides_the_same_smoothed_line_the_deck_is_swept_along() {
        // A wiggly corridor, so the smoother has something to remove and the
        // two curves are actually distinct: without that this passes for the
        // wrong reason.
        let mut c = corridor(6.0, 21);
        for (i, n) in c.nodes.iter_mut().enumerate() {
            n.y += if i % 2 == 0 { 1.2e-5 } else { -1.2e-5 };
        }
        // The corridor's own metric arc, built by the one function that builds
        // every arc, so its run bounds and the profile's stations are the same
        // parameterisation of the same line. Rolling a third copy here is what
        // hid this test's real residual: it used a different latitude scale, so
        // the fixture's stations and the profile's disagreed by 0.7 % and the
        // error cancelled against the offset being measured.
        let cos_lat = crate::scene::run_cos_lat(&c.nodes);
        let arc = crate::scene::cumulative_arc(&c.nodes);
        let total = arc[arc.len() - 1];
        c.arc = arc;
        let (b0, b1) = (total * 0.4, total * 0.6);
        c.spans = vec![
            Span { arc0: 0.0, arc1: b0, level: 0, kind: SpanKind::Grade },
            Span { arc0: b0, arc1: b1, level: 1, kind: SpanKind::Bridge },
            Span { arc0: b1, arc1: total, level: 0, kind: SpanKind::Grade },
        ];
        let p = crate::solve::Profile::from_heights(
            &c.nodes,
            vec![400.0; c.nodes.len()],
            vec![400.0; c.nodes.len()],
        );
        let smooth: Vec<Coord> = p.smooth().to_vec();
        let raw = c.nodes.clone();
        // The smoother must have moved the line, or this proves nothing.
        let moved = raw
            .iter()
            .map(|r| to_polyline(&smooth, cos_lat, *r))
            .fold(0.0f64, f64::max);
        assert!(moved > 0.2, "fixture not wiggly enough to tell the curves apart ({moved:.3} m)");

        let mut scene = SceneGraph::default();
        scene.corridors.push(c);
        let solved = SolvedModel::from_profiles(vec![Some(p)], 16);
        let (sources, cuts, _) = carriageway_sources(&scene, &solved, &Facades::empty());
        assert!(!sources.is_empty(), "the corridor paved nothing");

        // Every source endpoint lies on the smoothed line. Sampled densely
        // rather than taken as the chord polyline through the smooth nodes: the
        // band reads `smooth_at_arc`, which is the Catmull-Rom *through* those
        // nodes, so a point taken between two of them stands off their chord by
        // the spline's own sagitta and would read as a defect it is not. The
        // abutment overrun deliberately takes such a point.
        let prof = solved.profile(0).expect("profiled");
        let total = *prof.arc().last().expect("an arc");
        // Sampled at a centimetre, not a decimetre. `smooth_at_arc` takes an
        // arc but spends it as a *spline* parameter (`edge_at_arc` turns it
        // into a fraction of the raw edge, `smooth_point` feeds that to the
        // Catmull-Rom), and a Catmull-Rom's parameter is not arc length. So
        // stepping the arc uniformly does not step along the curve uniformly:
        // where the parameterisation compresses, two samples 0.1 m apart in arc
        // are far enough apart in space that a point genuinely *on* the curve
        // stands 28 mm off the chord between them. That is this proxy
        // polyline's error, not the band's, and it read as the band's until the
        // fixture stopped hiding it.
        const STEP_M: f64 = 0.01;
        let curve: Vec<Coord> = (0..=(total / STEP_M) as usize)
            .map(|i| prof.smooth_at_arc((i as f64 * STEP_M).min(total)))
            .collect();
        let off_smooth = sources
            .iter()
            .flat_map(|s| [s.a, s.b])
            .map(|q| to_polyline(&curve, cos_lat, q))
            .fold(0.0f64, f64::max);
        assert!(off_smooth < 0.01, "a band source stands {off_smooth:.3} m off the smoothed line");
        // ...and not on the raw one, which is the whole change.
        let off_raw = sources
            .iter()
            .flat_map(|s| [s.a, s.b])
            .map(|q| to_polyline(&raw, cos_lat, q))
            .fold(0.0f64, f64::max);
        assert!(off_raw > 0.2, "the band is still on the raw line ({off_raw:.3} m from it)");

        // And it is generated *past* where the deck's sweep starts, so the trim
        // in `synth::pavement` has material to take back to the deck's face.
        // The band no longer tries to *end* on the boundary: two constructions
        // cannot be made to agree there, so one cuts the other instead.
        let beyond = sources
            .iter()
            .flat_map(|s| [s.a, s.b])
            .map(|q| to_polyline(&[q, q], cos_lat, prof.smooth_at_arc(b0)))
            .filter(|d| *d < STRUCTURE_OVERRUN_M * 2.0)
            .fold(0.0f64, f64::max);
        assert!(
            beyond > STRUCTURE_OVERRUN_M * 0.9,
            "the band should run past the abutment for the cut to trim, got {beyond:.3} m"
        );
        // And the cut it will be trimmed by is the deck's own cross-section.
        assert!(
            cuts.iter().any(|h| {
                let mid = Coord { x: 0.5 * (h.a.x + h.b.x), y: 0.5 * (h.a.y + h.b.y) };
                to_polyline(&[mid, mid], cos_lat, prof.smooth_at_arc(b0)) < 0.01
            }),
            "no handover cut lands on the abutment"
        );
    }

    #[test]
    fn a_band_is_cut_at_the_abutment_not_at_the_nearest_vertex() {
        // Nodes every 10 m; the bridge runs 35..65, straddling two of them.
        let mut c = corridor(6.0, 11);
        c.spans = vec![
            Span { arc0: 0.0, arc1: 35.0, level: 0, kind: SpanKind::Grade },
            Span { arc0: 35.0, arc1: 65.0, level: 1, kind: SpanKind::Bridge },
            Span { arc0: 65.0, arc1: 100.0, level: 0, kind: SpanKind::Grade },
        ];
        let mut scene = SceneGraph::default();
        scene.corridors.push(c);
        let solved = SolvedModel::from_profiles(vec![None], 16);
        let (sources, _, _) = carriageway_sources(&scene, &solved, &Facades::empty());
        // The paved share of the corridor: 35 m of its 100 before the bridge
        // and 35 after, so 70 %. Measured as a
        // fraction of the corridor's own length because the fixture's `arc` is
        // synthetic and does not match the metric length of its nodes. A
        // vertex-rounded cut gives 60 % or 80 %, which is exactly the
        // half-segment this is here to keep out.
        let span = |a: Coord, b: Coord| (b.x - a.x).hypot(b.y - a.y);
        let paved: f64 = sources.iter().map(|s| span(s.a, s.b)).sum();
        let total = span(scene.corridors[0].nodes[0], scene.corridors[0].nodes[10]);
        let share = paved / total;
        assert!(
            (share - 0.70).abs() < 0.005,
            "the band should cover 70 % of the corridor, not {:.1} %",
            share * 100.0
        );
    }

    /// A straight west→east corridor whose arc really is its metric length —
    /// the room measurement is in metres and reads both, so the fixture above
    /// (10 m of arc over 7.7 m of ground) would make the window a lie.
    fn ew_corridor(width_m: f64, n: usize, step_m: f64) -> Corridor {
        let cos_lat = 46f64.to_radians().cos();
        let d = step_m / (DEG_M * cos_lat);
        Corridor {
            id: 0,
            nodes: (0..n).map(|i| Coord { x: 6.0 + i as f64 * d, y: 46.0 }).collect(),
            arc: (0..n).map(|i| i as f64 * step_m).collect(),
            cos_lat,
            kind: Kind::Road(RoadClass::Residential),
            class_key: "residential".to_string(),
            link: false,
            width_m: Some(width_m),
            spans: Vec::new(),
            segments: Vec::new(),
            connectors: Vec::new(),
        }
    }

    /// A wall parallel to [`ew_corridor`], `north_m` north of it (negative for
    /// south), running the whole fixture.
    fn parallel_wall(north_m: f64) -> Facades {
        let y = 46.0 + north_m / DEG_M;
        Facades::from_edges([[Coord { x: 5.99, y }, Coord { x: 6.01, y }]])
    }

    /// The cross-sections of every source of a one-corridor scene.
    fn sections_of(c: Corridor, facades: &Facades) -> Vec<Section> {
        let scene = SceneGraph::new(vec![c]);
        let solved = SolvedModel::empty(15);
        let (sources, _, _) = carriageway_sources(&scene, &solved, facades);
        sources.iter().flat_map(|s| [s.sect_a, s.sect_b]).collect()
    }

    #[test]
    fn open_ground_keeps_the_class_prior_on_both_sides() {
        let half = 3.0 + priors::STRUCTURE_SHOULDER_M;
        for s in sections_of(ew_corridor(6.0, 11, 10.0), &Facades::empty()) {
            assert_eq!(s, Section::uniform(half));
        }
    }

    #[test]
    fn a_facade_narrows_the_side_it_stands_on_and_only_that_side() {
        // 3.5 m of room minus the half-metre clearance leaves 3.0 m of asphalt
        // on the north side; the south side never saw a wall.
        let half = 3.0 + priors::STRUCTURE_SHOULDER_M;
        let sections = sections_of(ew_corridor(6.0, 11, 10.0), &parallel_wall(3.5));
        for s in &sections {
            assert!((s.left_m - 3.0).abs() < 1e-6, "left {} is not 3.0", s.left_m);
            assert_eq!(s.right_m, half, "the open side keeps its prior");
        }
    }

    #[test]
    fn a_facade_past_the_prior_narrows_nothing() {
        // The prior is 4 m and the clearance half a metre, so a wall 4.5 m out
        // is exactly the reach: it costs the street nothing.
        let half = 3.0 + priors::STRUCTURE_SHOULDER_M;
        for s in sections_of(ew_corridor(6.0, 11, 10.0), &parallel_wall(4.5)) {
            assert_eq!(s, Section::uniform(half), "a wall at the reach is not a wall in the way");
        }
    }

    #[test]
    fn a_wall_on_the_centerline_leaves_a_lane_rather_than_no_road() {
        // The floor is what separates a street a wall crowds from a way whose
        // centerline runs inside a footprint: the second keeps a carriageway.
        for s in sections_of(ew_corridor(6.0, 11, 10.0), &parallel_wall(0.2)) {
            assert_eq!(s.left_m, priors::MIN_CARRIAGEWAY_HALF_M);
            assert!(s.right_m > s.left_m, "the far side is untouched: {s:?}");
        }
    }

    #[test]
    fn a_rail_formation_is_not_narrowed_by_the_roof_over_it() {
        let mut c = ew_corridor(6.0, 11, 10.0);
        c.kind = Kind::Rail(crate::priors::RailClass::StandardGauge);
        c.class_key = "rail".to_string();
        let half = corridor_half_width_m(&c).expect("a formation");
        for s in sections_of(c, &parallel_wall(1.0)) {
            assert_eq!(s, Section::uniform(half), "a platform is not a defect");
        }
    }

    #[test]
    fn a_wall_shorter_than_the_station_spacing_is_still_seen() {
        // A 3 m stub of wall halfway between two 10 m-spaced stations. The
        // window is what catches it; without one it would fall through the
        // sampling entirely and the band would be drawn straight over it.
        let cos_lat = 46f64.to_radians().cos();
        let at = |m: f64| 6.0 + m / (DEG_M * cos_lat);
        let y = 46.0 + 3.0 / DEG_M;
        let f = Facades::from_edges([[Coord { x: at(43.5), y }, Coord { x: at(46.5), y }]]);
        let sections = sections_of(ew_corridor(6.0, 11, 10.0), &f);
        let narrowest = sections.iter().fold(f64::INFINITY, |m, s| m.min(s.left_m));
        assert!((narrowest - 2.5).abs() < 1e-6, "narrowest left is {narrowest}, not 2.5");
    }

    #[test]
    fn only_carriageways_become_sources() {
        // A drivable corridor contributes one source per segment; a footway
        // contributes none, because it paves nothing.
        let c = corridor(6.0, 11);
        let scene = crate::scene::SceneGraph::new(vec![c]);
        let solved = SolvedModel::empty(15);
        assert_eq!(carriageway_sources(&scene, &solved, &Facades::empty()).0.len(), 10, "one per segment");
        let half = corridor_half_width_m(&scene.corridors[0]).expect("a carriageway");
        assert!((half - (3.0 + priors::STRUCTURE_SHOULDER_M)).abs() < 1e-12);

        let mut path = corridor(6.0, 11);
        path.kind = crate::priors::Kind::Road(crate::priors::RoadClass::Footway);
        path.width_m = None;
        let scene = crate::scene::SceneGraph::new(vec![path]);
        assert!(carriageway_sources(&scene, &solved, &Facades::empty()).0.is_empty(), "a footway paves nothing");
        assert!(corridor_half_width_m(&scene.corridors[0]).is_none());
    }
}
