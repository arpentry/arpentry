//! Junction plates — a filled road surface meshed across an intersection so
//! the legs meet on one paved area instead of overlapping strokes
//! (docs/GENERATION.md scenario S4, docs/ROADS.md R6/R10).
//!
//! The unit here is the *intersection*, not the connector. Overture maps a
//! place where roads meet as however many connectors its geometry needs: a
//! plain crossroads is one, a staggered junction two, a roundabout a dozen
//! ringed around an island. Plating each connector separately is what made a
//! roundabout render as a ring of shards, so this module clusters connectors
//! into intersections first ([`cluster`]) and bakes one [`Area`] per cluster.
//! A roundabout needs no rule of its own: its ring arcs are short corridors,
//! so the clustering swallows them and the areas of its exits meet as one
//! paved band around the island.
//!
//! The area itself is a star-shaped region (`synth::area`), which is what lets
//! one primitive serve the plate mesh, the point test, and the band and
//! marking trims. A leg's width comes from its corridor's
//! [`Corridor::width_m`] — the same cross-section the surface band reads, so a
//! mouth and the band that lands on it agree by construction (ROADS.md
//! invariant 1 and 5); a non-drivable member (a footway, a crossing) joins the
//! intersection without contributing paved area.
//!
//! Plates are baked once from the solved model (heights are a pure function of
//! it) and emitted by the single tile that owns the intersection centre, so
//! tiles agree at their seams (invariant 5). Coordinates are tile-local
//! quantized uint16 / int32-mm with an up ENU normal, matching `MeshGeometry`.

use std::collections::HashMap;

use geo_types::Coord;

use crate::assemble::grid::GridIndex;
use crate::building_mesh::{M_PER_DEG_LAT, M_PER_DEG_LON_EQUATOR};
use crate::priors;
use crate::scene::{Corridor, CorridorId, SceneGraph, SpanKind};
use crate::solve::SolvedModel;
use crate::synth::area::{Area, Leg};

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
/// past this the next merge is refused and the junctions plate separately.
const MAX_CLUSTER_M: f64 = 45.0;

/// A baked intersection plate: its paved area, the styling class, and its
/// surface level — a fixed int32-mm height (an engineered junction, at its
/// welded level) or `None` for an at-grade one, which drapes on the ground.
pub struct BakedJunction {
    area: Area,
    /// The height the intersection's members solved to share, metres — the mean
    /// over its clustered junctions of [`SolvedModel::junction_height`]. `None`
    /// when no member carried a profile, so nothing is known. This is what the
    /// road height field pins the intersection to; unlike `level_mm` it exists
    /// for a street intersection too, and is a height rather than a decision
    /// about whether to drape.
    height: Option<f64>,
}

impl BakedJunction {
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
}

/// Every intersection plate, baked from the solved model — shared by the emit
/// workers through an `Arc`. A coarse geographic grid answers "which plates
/// are near this box" without a linear scan, which both the per-tile plate
/// emission and the per-segment marking trims (phase 1, millions of
/// segments) depend on.
pub struct JunctionModel {
    junctions: Vec<BakedJunction>,
    grid: HashMap<(i32, i32), Vec<u32>>,
    /// Every carriageway segment in the extract, with the width it paves — the
    /// corridor half of the road height field's sources
    /// ([`crate::synth::height`]). Baked here because it is the same walk over
    /// the scene the intersections come from, and because a height field built
    /// per tile would re-derive it for every zoom.
    sources: Vec<SourceSeg>,
    source_grid: GridIndex,
    /// Grade-separation layer per corridor, indexed by [`CorridorId`].
    layers: Vec<u32>,
}

/// One stretch of centerline between two nodes, and how far either side of it
/// that corridor's asphalt reaches. Carries the corridor *id* rather than a
/// borrowed profile so the model stays self-contained and shareable.
#[derive(Debug, Clone, Copy)]
pub struct SourceSeg {
    pub a: Coord,
    pub b: Coord,
    pub cos_lat: f64,
    pub half_m: f64,
    pub level: i64,
    /// Grade-separation layer: how many crossings this corridor passes *over*
    /// (`solve::crossings::corridor_ranks`). Zero for anything that crosses
    /// nothing, so ordinary streets all share a layer and still merge.
    ///
    /// Load-bearing for the union, because Overture's `level` ordinal does not
    /// carry this: a flyover's bridge span is excluded from the union already,
    /// but its *approaches* are ordinary at-grade spans at level 0, and so is the
    /// road they pass over. Keyed on level alone they merged into one region, and
    /// the mesh then ramped continuously between two roads that are metres apart
    /// vertically.
    pub layer: u32,
    pub corridor: CorridorId,
}

/// Grid cell size in degrees (~1 km): plates per cell stay in the tens even
/// in towns, and a tile or segment query touches a handful of cells.
const GRID_DEG: f64 = 0.01;

fn grid_cell(x: f64, y: f64) -> (i32, i32) {
    ((x / GRID_DEG).floor() as i32, (y / GRID_DEG).floor() as i32)
}

impl JunctionModel {
    fn build(
        junctions: Vec<BakedJunction>,
        sources: Vec<SourceSeg>,
        layers: Vec<u32>,
    ) -> JunctionModel {
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
        JunctionModel { junctions, grid, sources, source_grid, layers }
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

    /// A corridor's grade-separation layer; `0` for anything unranked.
    pub fn layer_of(&self, corridor: CorridorId) -> u32 {
        self.layers.get(corridor as usize).copied().unwrap_or(0)
    }

    pub fn len(&self) -> usize {
        self.junctions.len()
    }

    pub fn is_empty(&self) -> bool {
        self.junctions.is_empty()
    }

    pub fn iter(&self) -> impl Iterator<Item = &BakedJunction> {
        self.junctions.iter()
    }

    /// The plates whose centres fall in the `(west, south, east, north)` box.
    /// The caller pads the box by whatever reach (trim radius, plate size)
    /// matters to it.
    pub fn near(&self, b: (f64, f64, f64, f64)) -> Vec<&BakedJunction> {
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
pub fn bake(scene: &SceneGraph, solved: &SolvedModel) -> JunctionModel {
    let ports = Ports::build(scene);
    let clusters = cluster(scene, &ports);
    let mut junctions = Vec::new();
    for c in &clusters {
        if let Some(b) = bake_one(scene, solved, &ports, c) {
            junctions.push(b);
        }
    }
    let layers = crate::solve::crossings::corridor_ranks(scene);
    JunctionModel::build(junctions, carriageway_sources(scene, &layers), layers)
}

/// Every carriageway segment of every corridor that paves anything, in corridor
/// then node order. The height field's corridor sources; also the input the
/// unioned surface buffers.
fn carriageway_sources(scene: &SceneGraph, layers: &[u32]) -> Vec<SourceSeg> {
    let mut out = Vec::new();
    for c in &scene.corridors {
        let Some(half_m) = corridor_half_width_m(c) else {
            continue; // not a carriageway: paves nothing, so covers nothing
        };
        for ((lo, hi), level, kind) in level_runs(c) {
            // Only at-grade asphalt is unioned. A bridge or a bore already
            // carries its road surface as a swept solid (`synth::structure`), so
            // paving its level here would draw the carriageway twice — once as a
            // deck top and once as a region floating at the same height.
            if kind != SpanKind::Grade {
                continue;
            }
            for k in lo..hi {
                out.push(SourceSeg {
                    a: c.nodes[k],
                    b: c.nodes[k + 1],
                    cos_lat: c.cos_lat,
                    half_m,
                    level,
                    layer: layers.get(c.id as usize).copied().unwrap_or(0),
                    corridor: c.id,
                });
            }
        }
    }
    out
}

/// The corridor's node index runs of constant level, as `((first, last), level)`.
///
/// A corridor's level lives on its spans, not on the corridor
/// (`scene.rs:41-47`), so anything that partitions by level has to get it from
/// them.
///
/// Each *segment* is assigned to the span containing its **midpoint**, and
/// consecutive segments agreeing on level and kind are then grouped into runs.
/// Midpoint assignment is what makes the partition exact: a span boundary falls
/// at an arbitrary arc, rarely on a node, so the segment straddling it belongs to
/// neither span under a node-range rule and to both under a widened one.
///
/// Both mistakes have been made here. Widening let the at-grade runs on either
/// side of a bridge meet in the middle and pave the whole flyover; exact node
/// ranges then dropped one segment of asphalt at *every* boundary, which at an
/// interchange reads as a row of holes punched across the carriageway. Assigning
/// by midpoint gives each segment exactly one owner, so neither can happen.
fn level_runs(c: &Corridor) -> Vec<((usize, usize), i64, SpanKind)> {
    let n = c.nodes.len();
    if n < 2 {
        return Vec::new();
    }
    if c.spans.is_empty() {
        return vec![((0, n - 1), 0, SpanKind::Grade)];
    }
    let mut out: Vec<((usize, usize), i64, SpanKind)> = Vec::new();
    for k in 0..n - 1 {
        let mid = 0.5 * (c.arc[k] + c.arc[k + 1]);
        // The span covering the midpoint; a segment past the last span's end
        // (float slop at the very tail) falls to that span rather than vanishing.
        let Some(s) = c
            .spans
            .iter()
            .find(|s| mid >= s.arc0 && mid < s.arc1)
            .or_else(|| c.spans.last())
        else {
            continue;
        };
        match out.last_mut() {
            Some(((_, hi), lv, kd)) if *lv == s.level && *kd == s.kind && *hi == k => *hi = k + 1,
            _ => out.push(((k, k + 1), s.level, s.kind)),
        }
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
fn junction_reach_m(scene: &SceneGraph, j: usize) -> f64 {
    scene.junctions[j]
        .members
        .iter()
        .filter_map(|m| corridor_half_width_m(&scene.corridors[m.corridor as usize]))
        .fold(0.0, f64::max)
}

/// The half-width in metres of a corridor's paved band — its carriageway plus
/// the structure shoulder, exactly what `synth::surface` offsets to. `None`
/// for a non-drivable corridor: a footway or a crossing joins an intersection
/// without paving any of it.
pub(crate) fn corridor_half_width_m(c: &Corridor) -> Option<f64> {
    c.drivable.then_some(())?;
    Some(c.width_m? * 0.5 + priors::STRUCTURE_SHOULDER_M)
}

/// Bakes one cluster's plate, or `None` when it paves nothing: fewer than
/// three paved legs, no member with a solved profile, or a degenerate area.
fn bake_one(
    scene: &SceneGraph,
    solved: &SolvedModel,
    ports: &Ports,
    cluster: &Cluster,
) -> Option<BakedJunction> {
    let centre = cluster.centre;
    let m_lon = M_PER_DEG_LON_EQUATOR * centre.y.to_radians().cos();

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
        let off = ((jn.point.x - centre.x) * m_lon, (jn.point.y - centre.y) * M_PER_DEG_LAT);
        offset_max = offset_max.max(off.0.hypot(off.1));
        for m in &jn.members {
            let c = &scene.corridors[m.corridor as usize];
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

    Some(BakedJunction {
        area,
        height: (pin_count > 0).then(|| pin_sum / pin_count as f64),
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
    let m_lon = M_PER_DEG_LON_EQUATOR * lat.to_radians().cos();
    ((b.2 - b.0) * m_lon).hypot((b.3 - b.1) * M_PER_DEG_LAT)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::priors::RoadClass;
    use crate::scene::{Span, SpanKind};

    /// A straight corridor of `n` nodes, 10 m apart, at lat 46.
    fn corridor(width_m: f64, n: usize) -> Corridor {
        Corridor {
            id: 0,
            nodes: (0..n).map(|i| Coord { x: 6.0 + i as f64 * 1e-4, y: 46.0 }).collect(),
            arc: (0..n).map(|i| i as f64 * 10.0).collect(),
            cos_lat: 46f64.to_radians().cos(),
            class: RoadClass::Minor,
            class_key: "residential".to_string(),
            link: false,
            drivable: true,
            width_m: Some(width_m),
            spans: Vec::new(),
            segments: Vec::new(),
            connectors: Vec::new(),
        }
    }

    #[test]
    fn level_runs_cover_every_segment_once_per_level() {
                let mut c = corridor(6.0, 11);
        // No spans: one at-grade run over the whole corridor.
        assert_eq!(level_runs(&c), vec![((0, 10), 0, SpanKind::Grade)]);
        // Grade / bridge / grade: the bridge is its own level, and the runs
        // overlap by a node so no segment falls between two runs.
        c.spans = vec![
            Span { arc0: 0.0, arc1: 40.0, level: 0, kind: SpanKind::Grade },
            Span { arc0: 40.0, arc1: 60.0, level: 1, kind: SpanKind::Bridge },
            Span { arc0: 60.0, arc1: 100.0, level: 0, kind: SpanKind::Grade },
        ];
        let runs = level_runs(&c);
        assert_eq!(runs.len(), 3, "three runs: {runs:?}");
        assert_eq!(runs[1].1, 1, "the middle run is the bridge level");
        assert_eq!(runs[1].2, SpanKind::Bridge, "and it is a bridge, so the union skips it");
        // Every segment is covered exactly once — no gap, no double-paving.
        for k in 0..10 {
            let owners = runs.iter().filter(|&&((lo, hi), _, _)| k >= lo && k + 1 <= hi).count();
            assert_eq!(owners, 1, "segment {k} has {owners} owners: {runs:?}");
        }

        // The boundary case that matters: spans that end *between* nodes. A
        // node-range rule drops the straddling segment (a hole in the asphalt at
        // every bridge end); a widened one gives it to both (paving the flyover).
        c.spans = vec![
            Span { arc0: 0.0, arc1: 35.0, level: 0, kind: SpanKind::Grade },
            Span { arc0: 35.0, arc1: 65.0, level: 1, kind: SpanKind::Bridge },
            Span { arc0: 65.0, arc1: 100.0, level: 0, kind: SpanKind::Grade },
        ];
        let runs = level_runs(&c);
        for k in 0..10 {
            let owners = runs.iter().filter(|&&((lo, hi), _, _)| k >= lo && k + 1 <= hi).count();
            assert_eq!(owners, 1, "off-node boundary: segment {k} has {owners} owners: {runs:?}");
        }
        // Segment 3 spans arc 30..40, straddling the 35 m boundary; its midpoint
        // is 35, so it belongs to the bridge and to nothing else.
        let owner = runs.iter().find(|&&((lo, hi), _, _)| 3 >= lo && 4 <= hi).expect("an owner");
        assert_eq!(owner.2, SpanKind::Bridge, "the straddling segment went to the wrong span");
        // A degenerate corridor yields nothing.
        c.nodes.truncate(1);
        assert!(level_runs(&c).is_empty());
    }

    #[test]
    fn only_carriageways_become_sources() {
        // A drivable corridor contributes one source per segment; a footway
        // contributes none, because it paves nothing.
        let c = corridor(6.0, 11);
        let scene = crate::scene::SceneGraph::new(vec![c]);
        assert_eq!(carriageway_sources(&scene, &[0]).len(), 10, "one per segment");
        let half = corridor_half_width_m(&scene.corridors[0]).expect("a carriageway");
        assert!((half - (3.0 + priors::STRUCTURE_SHOULDER_M)).abs() < 1e-12);

        let mut path = corridor(6.0, 11);
        path.drivable = false;
        path.width_m = None;
        let scene = crate::scene::SceneGraph::new(vec![path]);
        assert!(carriageway_sources(&scene, &[0]).is_empty(), "a footway paves nothing");
        assert!(corridor_half_width_m(&scene.corridors[0]).is_none());
    }
}
