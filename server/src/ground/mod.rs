//! Stage 3 — the engineered ground (docs/GENERATION.md §5, invariant 1).
//!
//! One authoritative ground function that every later consumer reads: terrain
//! meshing, road draping, building founding, structure contact. The function
//! is the natural DEM plus the earthworks the solved model implies, applied
//! as local modifiers ([`modifiers::Earthworks`]).
//!
//! [`derive`] translates the solved model into earthworks: every at-grade
//! stretch of every profiled road benches the ground to its solved height —
//! a grade-limited cut through a bump, the embankment ramp the clearance
//! solver demanded for an overpass approach, or simply the flat band under a
//! road the DEM already carries (D3). Consumers are untouched: they already
//! read through [`sampler::GroundSampler`].

pub mod breaklines;
pub mod modifiers;
pub mod sampler;

use std::path::Path;
use std::sync::Mutex;

use geo_types::Coord;

use crate::assemble::facades::{Facades, Room, Section};
use crate::assemble::grid::GridIndex;
use crate::dem::Dem;
use crate::priors::{
    BENCH_GAP_SPAN_M, DECK_THICKNESS_M, EARTHWORK_BATTER,
    BATTER_DIVERGENCE_SLOP, EARTHWORK_MARGIN_M, EARTHWORK_MAX_BATTER_M, MAX_BENCH_FACE_M,
    EARTHWORK_MIN_BATTER_M, EARTHWORK_SHOULDER_M, WALK_MAX_FACE_M, WALL_BATTER,
    MAX_CLEARANCE_LIFT_M,
    PORTAL_CLEARANCE_M, PORTAL_CUT_LEN_M, WATER_LEVEL_PCTL,
};
use crate::priors::Stratum;
use crate::scene::{CorridorId, SceneGraph, SpanKind, DEG_M};
use crate::solve::{portals, reference_surface, SolvedModel};

use modifiers::{EarthworkEdge, Earthworks, WaterFill, Waters, LEFT, RIGHT};

/// Most shoreline vertices sampled when reading a water body's level — enough
/// to be robust on a big lake, bounded so a many-thousand-vertex ring is cheap.
const SHORELINE_SAMPLES: usize = 128;

/// One stratum's contribution to the ground: the earthworks it cut and filled,
/// and the water it flattened.
///
/// A layer is what stratum *n* imprints, and nothing else. It never sees the
/// layers above or below it — the fold in [`GroundStack::height`] composes
/// them, in authority order, exactly once each.
pub struct GroundLayer {
    /// Which stratum imprinted this. Layers are held in ascending authority
    /// order, so this is also the layer's position in the fold.
    pub stratum: Stratum,
    earthworks: Earthworks,
    waters: Waters,
}

impl GroundLayer {
    /// A layer that only reshapes the ground — no water. The shape every
    /// stratum but H has.
    pub fn of_earthworks(stratum: Stratum, earthworks: Earthworks) -> GroundLayer {
        GroundLayer { stratum, earthworks, waters: Waters::new(Vec::new()) }
    }

    /// This layer applied to the ground beneath it: water flattens first, then
    /// the earthworks bench and batter against the result. A road earthwork
    /// (a bridge abutment's approach berm at the shore) overrides the water
    /// where the two overlap, so the roadbed wins over what it climbs away
    /// from.
    #[cfg(test)]
    pub(super) fn apply_for_test(
        &self,
        lon: f64,
        lat: f64,
        base: f64,
        cell_m: f64,
        scratch: &mut Vec<u32>,
    ) -> f64 {
        self.apply(lon, lat, base, cell_m, scratch)
    }

    fn apply(&self, lon: f64, lat: f64, base: f64, cell_m: f64, scratch: &mut Vec<u32>) -> f64 {
        let base = if self.waters.is_empty() {
            base
        } else {
            self.waters.level_at(lon, lat, scratch).unwrap_or(base)
        };
        if self.earthworks.is_empty() {
            return base;
        }
        self.earthworks.height(lon, lat, base, cell_m, scratch)
    }

    /// Whether this layer's declared footprint covers `(lon, lat)` — the
    /// predicate I8 is stated over: *"`groundₙ₊₁` differs from `groundₙ` only
    /// inside stratum n's declared footprints"*.
    ///
    /// Declared, not observed: it asks the same grid and the same
    /// [`EarthworkEdge::reach_m`] the height function asks, so a change outside
    /// it means a reach that does not bound its own influence. That is exactly
    /// what makes the check non-vacuous — `batter_reach` is separately clamped,
    /// and either side could stop bounding the other silently.
    pub fn covers(&self, lon: f64, lat: f64, scratch: &mut Vec<u32>) -> bool {
        self.earthworks.covers(lon, lat, scratch)
            || (!self.waters.is_empty() && self.waters.level_at(lon, lat, scratch).is_some())
    }

    /// The bench target here, if this layer holds one.
    fn target_at(&self, lon: f64, lat: f64, scratch: &mut Vec<u32>) -> Option<f64> {
        self.earthworks.target_at(lon, lat, scratch)
    }

    /// The same, over the benches that draw contact lines — the crest
    /// derivation's question (docs/GROUND.md §3, and see
    /// [`modifiers::Earthworks::crest_target_at`] for why the two differ).
    pub(super) fn crest_target_at(
        &self,
        lon: f64,
        lat: f64,
        scratch: &mut Vec<u32>,
    ) -> Option<f64> {
        self.earthworks.crest_target_at(lon, lat, scratch)
    }

    pub fn earthworks(&self) -> &Earthworks {
        &self.earthworks
    }
}

/// The engineered ground (I1): one function every consumer reads, built as the
/// **accumulating stack** of §4.3.
///
/// ```text
/// ground₀   = conditioned DEM
/// groundₙ₊₁ = groundₙ ⊕ stratum n's earthworks
/// ```
///
/// Each stratum imprints on the ground its senior published, so a road cutting
/// is carved into a ground that already holds the rail embankment. There is
/// never a moment when two stages hold different opinions about the ground,
/// because there is only ever one ground and it only moves forward.
///
/// A flat vector rather than a chain of wrappers, for two reasons. I8 has to
/// evaluate `groundₙ` and `groundₙ₊₁` at one point and attribute the difference
/// to layer *n*, which is [`height_through`](Self::height_through) either side
/// of an index; and "applied exactly once" becomes a construction invariant
/// over distinct ascending strata rather than a property of a linked structure.
pub struct GroundStack {
    /// In ascending authority order: seniors first, so the fold runs H → R → S.
    layers: Vec<GroundLayer>,
    /// The bench contact lines the detail mesh preserves (docs/GROUND.md §3),
    /// derived from the assembled field of every layer.
    breaklines: breaklines::Breaklines,
}

impl GroundStack {
    /// A ground with no layers: the conditioned DEM passes straight through.
    pub fn empty() -> GroundStack {
        GroundStack { layers: Vec::new(), breaklines: breaklines::Breaklines::derive(&[]) }
    }

    /// Builds the stack from its layers, which must be distinct and in
    /// ascending authority order — the "exactly once" half of I8, asserted at
    /// construction rather than measured.
    pub fn new(layers: Vec<GroundLayer>) -> GroundStack {
        debug_assert!(
            layers.windows(2).all(|w| w[0].stratum < w[1].stratum),
            "ground layers must be distinct and in ascending authority order"
        );
        let breaklines = breaklines::Breaklines::derive(&layers);
        GroundStack { layers, breaklines }
    }

    pub fn breaklines(&self) -> &breaklines::Breaklines {
        &self.breaklines
    }

    pub fn layers(&self) -> &[GroundLayer] {
        &self.layers
    }

    /// The layer stratum *s* imprinted, if it imprinted one.
    pub fn layer(&self, stratum: Stratum) -> Option<&GroundLayer> {
        self.layers.iter().find(|l| l.stratum == stratum)
    }

    /// Number of earthwork edges across every layer, for run stats.
    pub fn earthwork_count(&self) -> usize {
        self.layers.iter().map(|l| l.earthworks.len()).sum()
    }

    /// Number of flattened water bodies across every layer, for run stats.
    pub fn water_count(&self) -> usize {
        self.layers.iter().map(|l| l.waters.len()).sum()
    }

    /// The bench target the road rides here, or `None` outside every bench —
    /// the most junior layer holding one, since that is what the fold leaves.
    pub fn bed_target(&self, lon: f64, lat: f64, scratch: &mut Vec<u32>) -> Option<f64> {
        self.layers.iter().rev().find_map(|l| l.target_at(lon, lat, scratch))
    }

    /// THE ground function: the engineered height at `(lon, lat)`, given the
    /// raw DEM sample `raw` for that point. `scratch` is the caller's reusable
    /// query buffer (see [`sampler::GroundSampler`]).
    ///
    /// `cell_m` is the sample spacing of whatever is asking — the lattice cell
    /// of the terrain mesh being built, or 0 for an exact point query (a road
    /// reading its own bed). An earthwork narrower than that spacing is left
    /// out: it cannot be drawn at that resolution, and sampling it *does* the
    /// damage, because a corner that happens to land inside a 10 m bench takes
    /// the road's height while its neighbours a cell away take the hillside,
    /// and the mesh spikes. Whole slopes of terraced vineyard tracks turned
    /// into sawtooth noise one zoom out from the reference. The road
    /// compensates with its per-zoom datum lift (docs/GROUND.md §4), so
    /// dropping the bench from the *drawn ground* does not float it.
    pub fn height(
        &self,
        lon: f64,
        lat: f64,
        raw: f64,
        cell_m: f64,
        scratch: &mut Vec<u32>,
    ) -> f64 {
        self.height_through(self.layers.len(), lon, lat, raw, cell_m, scratch)
    }

    /// `groundₙ`: the ground as it stands after the first `n` layers — what
    /// stratum *n* reads while it solves, and what I8 diffs against
    /// `groundₙ₊₁` to attribute a change to one layer's footprint.
    pub fn height_through(
        &self,
        n: usize,
        lon: f64,
        lat: f64,
        raw: f64,
        cell_m: f64,
        scratch: &mut Vec<u32>,
    ) -> f64 {
        let mut h = raw;
        for layer in self.layers.iter().take(n) {
            h = layer.apply(lon, lat, h, cell_m, scratch);
        }
        h
    }
}

/// Derives the engineered ground from the solved model: a bench under every
/// at-grade stretch of every profiled road (holding the carriageway flat at
/// its solved height, with a batter face reaching out as far as the cut or
/// fill it makes needs), and a daylighting cut in front of every solved
/// tunnel portal (S5 — the mouth face must not hide below grade).
///
/// **The ground accumulates** (§4.3). Each stratum imprints on the ground its
/// senior published, in authority order:
///
/// ```text
/// groundₙ₊₁ = groundₙ ⊕ stratum n's earthworks
/// ```
///
/// so a road cutting is carved into a ground that already holds the rail
/// embankment. The layer being built reads the stack beneath it — which is the
/// difference between "every earthwork against the natural hillside" and "each
/// earthwork against the world as it stands when it is built".
pub fn derive(
    scene: &SceneGraph,
    solved: &SolvedModel,
    facades: &Facades,
    walk_bands: &[crate::synth::carriageway::SourceSeg],
    terrain_path: Option<&Path>,
    threads: usize,
) -> GroundStack {
    let seniors = derive_seniors(scene, solved, facades, terrain_path, threads);
    derive_draped(seniors, walk_bands, terrain_path, solved.z_ref)
}

/// The strata that hold authority — H, R and S — imprinted in that order.
///
/// **Split from [`derive_draped`] so a draped feature can see the ground it
/// stands on.** GENERATION.md §4.2 defines stratum D as the layer that *samples
/// the finished ground*, and until this split the one draped feature that
/// imprints — the walkway band — could not: `synth::walkway::bands` ran before
/// `derive`, so the band's own width was chosen against a world in which no
/// railway embankment and no street terrace existed yet. The ground then had to
/// bench whatever plan the band had already committed to, and where that bench
/// was implausible it was refused outright and the band was left standing on
/// the raw hillside — 16.8 % of drawn path length, which was nearly all of
/// `slope.walk_crossfall`.
///
/// With the seniors in hand the band is fitted to them first
/// (`synth::walkway::fit_to_ground`) and D benches the fitted band, so the two
/// are one cross-section rather than two constructions of one.
pub fn derive_seniors(
    scene: &SceneGraph,
    solved: &SolvedModel,
    facades: &Facades,
    terrain_path: Option<&Path>,
    threads: usize,
) -> Vec<GroundLayer> {
    // H first: water is gravity-defined and no earthwork changes it, so it is
    // the ground everything else is cut into.
    let waters = derive_waters(scene, solved, terrain_path, threads);
    let mut layers: Vec<GroundLayer> = Vec::new();
    if !waters.is_empty() {
        layers.push(GroundLayer { stratum: Stratum::H, earthworks: Earthworks::new(Vec::new()), waters });
    }
    for stratum in [Stratum::R, Stratum::S] {
        let members: Vec<usize> = scene
            .corridors
            .iter()
            .enumerate()
            .filter(|(_, c)| c.kind.stratum() == stratum && solved.profile(c.id).is_some())
            .map(|(i, _)| i)
            .collect();
        if members.is_empty() {
            continue;
        }
        let edges =
            derive_earthworks(scene, solved, facades, &members, &layers, terrain_path, threads);
        if edges.is_empty() {
            continue;
        }
        layers.push(GroundLayer {
            stratum,
            earthworks: Earthworks::new(edges),
            waters: Waters::new(Vec::new()),
        });
    }
    layers
}

/// Stratum D over the seniors, and the finished ground.
///
/// **The walkway is a draped feature, and D is where draped features imprint.**
/// A pedestrian way holds no authority and solves nothing (§4.2): it samples
/// the finished ground and lays its own narrow bench on top of it, which is the
/// last earthwork built and the first any other stratum would overwrite. So it
/// belongs in this layer, and only here — put junior to the street it borders,
/// its bench would be cut away again by the street's own batter, which is the
/// defect it exists to remove.
///
/// No corridor is draped: `assemble::run` admits a feature to the scene graph
/// only if its stratum solves, so the whole of D is the walkway bands.
pub fn derive_draped(
    mut layers: Vec<GroundLayer>,
    walk_bands: &[crate::synth::carriageway::SourceSeg],
    terrain_path: Option<&Path>,
    z_ref: u8,
) -> GroundStack {
    let edges = walk_earthworks(walk_bands, &layers, terrain_path, z_ref);
    if !edges.is_empty() {
        layers.push(GroundLayer {
            stratum: Stratum::D,
            earthworks: Earthworks::new(edges),
            waters: Waters::new(Vec::new()),
        });
    }
    GroundStack::new(span_bench_gaps(layers))
}

/// Arc separation past which two edges of one chain count as different legs —
/// a switchback returning above itself — rather than the same run continuing
/// around a bend. A hairpin tight enough to matter turns through at least this
/// much arc between its arms; contiguous neighbours along a curve never do.
const HAIRPIN_ARC_M: f64 = 50.0;

/// Where walkway chain ids start, clear of every corridor id.
///
/// A chain says "these edges are one run" and is compared across the whole
/// assembled stack (`span_bench_gaps`), so a walkway run that happened to share
/// a number with a corridor would be read as that corridor continuing.
const WALK_CHAIN_BASE: u32 = u32::MAX / 2;

/// The bench under one drawn pedestrian band, per side and per segment
/// (docs/GROUND.md §2).
///
/// **The ground under a walkway is the walkway.** A carriageway has said this
/// since the imprint existed; a pedestrian band, which is a drawn surface with
/// its own hole and its own apron, had no bench at all. What that costs is two
/// measured things. A sidewalk is seated on its host's cross-section a kerb
/// above the carriageway while the ground under it is still the street's bench
/// — which stops a verge past the asphalt — or the batter face beyond it, so
/// the band meets the ground at a step (`contact.walk_rim`, p50 0.19 m over the
/// sidewalk population, reaching 15 m). A path *is* the drawn ground, so it
/// carries the full cross-slope of whatever it crosses: half of all drawn path
/// length tilted more than 30 % across its own two metres
/// (`slope.walk_crossfall`), which is a ribbon glued to a hillside rather than
/// a footpath cut into one.
///
/// Three things make this bench different from a corridor's, and each is the
/// band being narrow:
///
/// - **It is derived from the band, not from a class prior.** The half-width is
///   the drawn band's own, plus the same verge every bench carries, so the bench
///   fits the surface it holds by construction rather than by two derivations
///   agreeing.
/// - **Its target is its seat.** A sidewalk's is the host's road surface plus
///   the kerb, which is exactly the height the band is drawn at; a path's is
///   the ground *beneath this stratum* along its own centerline, so the bench
///   flattens the cross-section and changes nothing along the way — a footpath
///   still climbs whatever hill it climbs, it is simply no longer tilted
///   sideways at the angle of the slope.
/// - **It emits no contact lines** ([`EarthworkEdge::crest`]).
fn walk_earthworks(
    bands: &[crate::synth::carriageway::SourceSeg],
    beneath: &[GroundLayer],
    terrain_path: Option<&Path>,
    z_ref: u8,
) -> Vec<EarthworkEdge> {
    if std::env::var_os("ARPT_NO_WALK_BENCH").is_some() {
        return Vec::new(); // the A/B control: bands drawn, ground unbenched
    }
    // Only what is drawn at grade. A band over a bridge is carried by the
    // structure (`synth::carried`) and a band under one is not what the ground
    // there is; neither has a bench to lay.
    let seats: Vec<usize> = (0..bands.len()).filter(|&i| bands[i].level == 0).collect();
    if seats.is_empty() {
        return Vec::new();
    }
    // One chain per contiguous run of band segments, in the order they were
    // built, so a run that bends is one run and two bands that merely touch are
    // not (see [`WALK_CHAIN_BASE`]).
    let mut chain_of: Vec<u32> = Vec::with_capacity(seats.len());
    let mut chain = WALK_CHAIN_BASE;
    for (k, &i) in seats.iter().enumerate() {
        if k > 0 {
            let prev = &bands[seats[k - 1]];
            if prev.b != bands[i].a || prev.corridor != bands[i].corridor {
                chain += 1;
            }
        }
        chain_of.push(chain);
    }

    let census = std::env::var_os("ARPT_DEBUG_WALK").is_some();
    let rules = WalkBenchRules::from_env();
    let seated: Vec<(Option<EarthworkEdge>, Option<WalkCensusRow>)> =
        over_senior_ground(seats.len(), beneath, terrain_path, z_ref, |k, sample| {
            let s = &bands[seats[k]];
            let e = walk_edge(s, chain_of[k], &rules, sample);
            let row = census.then(|| walk_census_row(s, e.is_some(), sample));
            (e, row)
        });
    let (edges, rows): (Vec<_>, Vec<_>) = seated.into_iter().unzip();
    if census {
        report_walk_census(&rows.into_iter().flatten().collect::<Vec<_>>());
    }
    let mut edges: Vec<EarthworkEdge> = edges.into_iter().flatten().collect();
    // Back into band order whatever the threads did with them (invariant 5).
    edges.sort_by(|a, b| (a.chain, a.arc0).partial_cmp(&(b.chain, b.arc0)).expect("finite arcs"));
    edges
}

/// Runs `f` over `0..n`, handing each call a sampler for **the ground a draped
/// feature stands on**: the reference surface the solve read
/// ([`reference_surface`], not a raw DEM point — a point sample charges the
/// in-cell interpolation to the earthwork), with every senior stratum's
/// earthworks already applied (§4.3). A sidewalk beside a street reads the
/// terrace the street cut, not the hillside it cut it from.
///
/// Shared by the two readers of that ground — `synth::walkway::fit_to_ground`,
/// which decides how wide a band the allowance can carry, and
/// [`walk_earthworks`], which benches the band it decided on — so the width and
/// the bench can never be fitted against two different worlds.
///
/// Results come back indexed by item, so the answer is a function of the model
/// and not of how the threads happened to interleave (invariant 5).
pub fn over_senior_ground<T: Send>(
    n: usize,
    beneath: &[GroundLayer],
    terrain_path: Option<&Path>,
    z_ref: u8,
    f: impl Fn(usize, &mut dyn FnMut(Coord) -> f64) -> T + Sync,
) -> Vec<T> {
    let dem = terrain_path.and_then(|p| Dem::open(p).ok());
    let threads = dem.as_ref().map_or(1, |_| n.min(8).max(1));
    let out: Mutex<Vec<Vec<(usize, T)>>> = Mutex::new((0..threads).map(|_| Vec::new()).collect());
    let next = Mutex::new(0usize);
    // Work is handed out in blocks: a band segment is eight metres of a
    // footpath, far too little to pay for a lock per item, and neighbouring
    // segments share the DEM tiles they sample.
    const BLOCK: usize = 256;
    std::thread::scope(|scope| {
        for t in 0..threads {
            let (out, next, f) = (&out, &next, &f);
            let dem = dem.as_ref();
            scope.spawn(move || {
                let mut fork = dem.and_then(|d| d.fork().ok());
                let mut scratch: Vec<u32> = Vec::new();
                let mut mine: Vec<(usize, T)> = Vec::new();
                loop {
                    let lo = {
                        let mut cur = next.lock().expect("senior ground queue poisoned");
                        if *cur >= n {
                            break;
                        }
                        let lo = *cur;
                        *cur += BLOCK;
                        lo
                    };
                    for k in lo..(lo + BLOCK).min(n) {
                        let mut sample = |q: Coord| {
                            let raw = match fork.as_mut() {
                                Some(d) => reference_surface(d, z_ref, q.x, q.y),
                                None => 0.0,
                            };
                            let mut h = raw;
                            for layer in beneath {
                                h = layer.apply(q.x, q.y, h, 0.0, &mut scratch);
                            }
                            h
                        };
                        mine.push((k, f(k, &mut sample)));
                    }
                }
                out.lock().expect("senior ground results poisoned")[t] = mine;
            });
        }
    });
    let mut all: Vec<(usize, T)> =
        out.into_inner().expect("senior ground results poisoned").into_iter().flatten().collect();
    all.sort_by_key(|(k, _)| *k);
    all.into_iter().map(|(_, v)| v).collect()
}

/// One band segment, as `ARPT_DEBUG_WALK` sees it: whether its bench was laid,
/// how much cut or fill it asked for, and the cross-slope of the ground it was
/// asked to lay it on.
struct WalkCensusRow {
    path: bool,
    benched: bool,
    len_m: f64,
    /// The deeper of the two verge faces the bench would cut or fill — the
    /// quantity [`walk_edge`]'s cap is read against.
    face_m: f64,
    /// The natural cross-slope across the band, rise per metre: the tilt the
    /// drawn band inherits wherever no bench is laid, and what
    /// `slope.walk_crossfall` reads.
    fall: f64,
    /// The half-width the bench asked its face at: the band's own plus the
    /// verge.
    bench_w_m: f64,
}

/// Reads one band segment for the census, sampling the same three points
/// [`walk_edge`] does so the two agree by construction.
fn walk_census_row(
    s: &crate::synth::carriageway::SourceSeg,
    benched: bool,
    sample: &mut dyn FnMut(Coord) -> f64,
) -> WalkCensusRow {
    let cos_lat = s.cos_lat;
    let (dx, dy) = ((s.b.x - s.a.x) * cos_lat, s.b.y - s.a.y);
    let len = (dx * dx + dy * dy).sqrt();
    let mid = Coord { x: (s.a.x + s.b.x) * 0.5, y: (s.a.y + s.b.y) * 0.5 };
    let path = s.corridor == CorridorId::MAX;
    let mut row = WalkCensusRow {
        path,
        benched,
        len_m: len * DEG_M,
        face_m: 0.0,
        fall: 0.0,
        bench_w_m: s.drawn_half() + EARTHWORK_MARGIN_M,
    };
    if !(len > 0.0) {
        return row;
    }
    let (px, py) = (-dy / len, dx / len);
    // The width the band is *drawn* at. `half_m` is the run's chaining key
    // (`SourceSeg::drawn_half_at`) and is the class nominal however much the
    // room took off, so benching off it would carve a terrace wider than the
    // pavement standing on it.
    let w = s.drawn_half() + EARTHWORK_MARGIN_M;
    let target =
        if path { (sample(s.a) + sample(s.b)) * 0.5 } else { (s.height_a + s.height_b) * 0.5 };
    let at = |side: f64| Coord {
        x: mid.x + side * px * w / (DEG_M * cos_lat),
        y: mid.y + side * py * w / DEG_M,
    };
    let (l, r) = (sample(at(1.0)), sample(at(-1.0)));
    row.face_m = (target - l).abs().max((target - r).abs());
    row.fall = (l - r).abs() / (2.0 * w);
    row
}

fn report_walk_census(rows: &[WalkCensusRow]) {
    let q = |v: &mut Vec<f64>, f: f64| -> f64 {
        if v.is_empty() {
            return 0.0;
        }
        v.sort_by(f64::total_cmp);
        v[((v.len() - 1) as f64 * f) as usize]
    };
    for (name, path) in [("sidewalk", false), ("path", true)] {
        let mine: Vec<&WalkCensusRow> = rows.iter().filter(|r| r.path == path).collect();
        if mine.is_empty() {
            continue;
        }
        let total_m: f64 = mine.iter().map(|r| r.len_m).sum();
        let refused: Vec<&&WalkCensusRow> = mine.iter().filter(|r| !r.benched).collect();
        let refused_m: f64 = refused.iter().map(|r| r.len_m).sum();
        eprintln!(
            "[walk] {name:<9} {:>7} segments  {:>8.2} km   refused {:>6} ({:>5.2} %)  \
             {:>7.2} km ({:>5.2} % of length)",
            mine.len(),
            total_m / 1000.0,
            refused.len(),
            100.0 * refused.len() as f64 / mine.len() as f64,
            refused_m / 1000.0,
            100.0 * refused_m / total_m.max(1e-9),
        );
        let mut face: Vec<f64> = mine.iter().map(|r| r.face_m).collect();
        eprintln!(
            "[walk]   face asked (m)   p50 {:.2}  p75 {:.2}  p90 {:.2}  p95 {:.2}  p99 {:.2}  \
             max {:.2}",
            q(&mut face, 0.50),
            q(&mut face, 0.75),
            q(&mut face, 0.90),
            q(&mut face, 0.95),
            q(&mut face, 0.99),
            q(&mut face, 1.0),
        );
        let mut all_fall: Vec<f64> = mine.iter().map(|r| r.fall).collect();
        let mut ref_fall: Vec<f64> = refused.iter().map(|r| r.fall).collect();
        eprintln!(
            "[walk]   ground fall      p50 {:.2}  p90 {:.2}  max {:.2}   \
             (refused only: p50 {:.2}  p90 {:.2}  max {:.2})",
            q(&mut all_fall, 0.50),
            q(&mut all_fall, 0.90),
            q(&mut all_fall, 1.0),
            q(&mut ref_fall, 0.50),
            q(&mut ref_fall, 0.90),
            q(&mut ref_fall, 1.0),
        );
        // What a *lower* cap would refuse, and what a higher one would accept:
        // the fraction of length on each side of a candidate face allowance.
        let mut line = String::from("[walk]   length by face:");
        for t in [0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0] {
            let over: f64 = mine.iter().filter(|r| r.face_m > t).map(|r| r.len_m).sum();
            line.push_str(&format!("  >{t:.2}m {:.1} %", 100.0 * over / total_m.max(1e-9)));
        }
        eprintln!("{line}");
        // **The band narrowed to what the allowance can bench**, estimated: the
        // face grows with the bench's width, so the width whose face is the cap
        // is the cap's share of the width this one asked at, and the band
        // inside it is that less the verge. The question the estimate answers
        // is whether a rule that narrows the surface instead of refusing its
        // bench leaves a plausible world or deletes the mountain paths.
        for (cap_name, cap) in [("1.0 m", WALK_MAX_FACE_M), ("1.5 m", 1.5)] {
            let (mut full, mut narrowed, mut gone) = (0.0, 0.0, 0.0);
            for r in &mine {
                let w = r.bench_w_m;
                let allow = if r.face_m > cap { w * cap / r.face_m } else { w };
                let width = 2.0 * (allow - EARTHWORK_MARGIN_M);
                if width >= 2.0 * (w - EARTHWORK_MARGIN_M) - 1e-9 {
                    full += r.len_m;
                } else if width >= crate::priors::WALK_MIN_WIDTH_M {
                    narrowed += r.len_m;
                } else {
                    gone += r.len_m;
                }
            }
            let t = total_m.max(1e-9);
            eprintln!(
                "[walk]   band at cap {cap_name}:  full width {:.1} %   narrowed {:.1} %   \
                 under the minimum {:.1} %",
                100.0 * full / t,
                100.0 * narrowed / t,
                100.0 * gone / t,
            );
        }
    }
}

/// How much of the cap a narrowed bench aims at, so it lands inside it rather
/// than on it.
const FIT_MARGIN: f64 = 0.85;

/// The two knobs the walk bench is being measured on, read once per derivation
/// rather than per band segment.
///
/// Both are **experiments**, off by default, and both exist to put a number on
/// the same question: the bench refuses itself where its verge face is deeper
/// than the material may plausibly retain, and a refused bench leaves the drawn
/// band standing on the raw hillside — 16.8 % of drawn path length in the
/// Montreux window, carrying a p50 72 % cross-slope, which is nearly all of
/// what `slope.walk_crossfall` reports.
struct WalkBenchRules {
    /// `ARPT_WALK_CAP` — override the per-material plausibility cap. At
    /// effectively no cap the tilt collapses (`slope.walk_crossfall` 22.5 →
    /// 2.4 %, `contact.walk_rim` 2.8 → 0.5 %), which is what says the refusals
    /// *are* the defect; it also builds the fictional earthworks the cap exists
    /// to refuse (`clearance.bore_cover` 5.87 → 6.14 %,
    /// `lod.structure_drift` 1.65 → 1.80 %), which is what says the answer is
    /// not simply a bigger allowance.
    cap_m: Option<f64>,
    /// `ARPT_WALK_FIT` — narrow the bench until its face fits the cap instead
    /// of dropping it, floored at the drawn band's own half-width. Recovers
    /// three quarters of the refused length at the *existing* allowance
    /// (16.8 → 4.6 %) and takes `slope.walk_crossfall` 22.5 → 8.5 %, but pays
    /// `contact.walk_rim` 2.8 → 3.8 %: a bench narrowed to the band has no
    /// verge left, so the terrain's hole rim lands where the batter starts.
    /// That regression is the argument for narrowing the *band* with the bench
    /// rather than the bench alone.
    fit: bool,
}

impl WalkBenchRules {
    /// The shipped rule: the per-material cap, refusing rather than narrowing.
    fn shipped() -> Self {
        WalkBenchRules { cap_m: None, fit: false }
    }

    fn from_env() -> Self {
        WalkBenchRules {
            cap_m: std::env::var_os("ARPT_WALK_CAP").and_then(|v| v.to_str()?.parse().ok()),
            fit: std::env::var_os("ARPT_WALK_FIT").is_some(),
        }
    }
}

/// One band segment's bench, or `None` where holding one is not plausible.
fn walk_edge(
    s: &crate::synth::carriageway::SourceSeg,
    chain: u32,
    rules: &WalkBenchRules,
    sample: &mut dyn FnMut(Coord) -> f64,
) -> Option<EarthworkEdge> {
    let cos_lat = s.cos_lat;
    let (dx, dy) = ((s.b.x - s.a.x) * cos_lat, s.b.y - s.a.y);
    let len = (dx * dx + dy * dy).sqrt();
    if !(len > 0.0) {
        return None;
    }
    let (px, py) = (-dy / len, dx / len); // lateral unit, metric (left)
    let mid = Coord { x: (s.a.x + s.b.x) * 0.5, y: (s.a.y + s.b.y) * 0.5 };
    // The width the band is *drawn* at. `half_m` is the run's chaining key
    // (`SourceSeg::drawn_half_at`) and is the class nominal however much the
    // room took off, so benching off it would carve a terrace wider than the
    // pavement standing on it.
    let w = s.drawn_half() + EARTHWORK_MARGIN_M;
    let ground = sample(mid);
    // A sidewalk's seat is the height it is drawn at; a path's is the ground it
    // stands on, read at its own centerline so the bench flattens the section
    // without moving the way along its length.
    //
    // **At both ends, never at the middle.** A bench interpolates its target
    // along the edge, so reading one height for the whole segment holds eight
    // metres of footpath flat at its midpoint's ground — which on a path
    // climbing at 50 % is two metres of cut at one end, two of fill at the
    // other, and a four-metre step where the next segment's terrace starts. It
    // measured exactly that: a staircase of terraces up the flank above
    // Territet, drawn as a **324:1 terrain face**, with the lateral face cap
    // blind to it because at the midpoint there is nothing to see. Sharing the
    // endpoint samples makes consecutive benches agree there by construction.
    let (target_a, target_b) = if s.corridor == CorridorId::MAX {
        (sample(s.a), sample(s.b))
    } else {
        (s.height_a, s.height_b)
    };
    let target = (target_a + target_b) * 0.5;
    let mut face = |side: f64, w: f64| -> (f64, (f64, f64)) {
        let q = Coord {
            x: mid.x + side * px * w / (DEG_M * cos_lat),
            y: mid.y + side * py * w / DEG_M,
        };
        let edge = sample(q);
        (target - edge, batter_reach(target - edge, (edge - ground) / w.max(1e-6)))
    };
    let (rise_l, reach_l) = face(1.0, w);
    let (rise_r, reach_r) = face(-1.0, w);
    // The plausibility cap (docs/GROUND.md §2), and **the two materials do not
    // hold the same one**, because they are not the same claim about the world.
    //
    // A *sidewalk* is part of a street's cross-section, which is flat from kerb
    // to facade. Where that street stands on a terrace with a wall down to the
    // hillside, the sidewalk stands on the same terrace and the wall is the
    // street's — so it may hold what the street's own bench holds, and its
    // apron draws that wall exactly as the street's does. It rarely needs the
    // allowance: the street beside it has usually benched the ground already,
    // and the face the cap then sees is a kerb rise.
    //
    // A *path* across open ground claims an earthwork of its own, and a
    // two-metre ribbon cutting three metres into a flank is not a footpath, it
    // is a retaining structure nobody built ([`WALK_MAX_FACE_M`]).
    let cap = rules.cap_m.unwrap_or(crate::priors::bench_face_cap_m(s.surface));
    // **Narrowing before refusing** (`ARPT_WALK_FIT`, the experiment). A bench
    // whose verge face is too deep is not the same claim as a bench that cannot
    // be built: the face grows with the width, so pulling the verge in until it
    // fits keeps the band flat and shrinks the earthwork instead of dropping
    // both. The floor is the drawn band's own half-width — a bench narrower
    // than the surface it carries hangs that surface over unbenched ground,
    // which is what `carriageway_m` exists to prevent, and it is the same floor
    // the road bench takes against the room a facade leaves it.
    let (mut w_l, mut w_r) = (w, w);
    let (mut rise_l, mut reach_l) = (rise_l, reach_l);
    let (mut rise_r, mut reach_r) = (rise_r, reach_r);
    if rules.fit {
        let mut fit = |side: f64, rise: f64, w_side: &mut f64| -> (f64, (f64, f64)) {
            if rise.abs() <= cap {
                return (rise, face(side, *w_side).1);
            }
            // The ground is near-linear across a two-metre band, so the width
            // whose face is exactly the cap is the cap's share of this one —
            // taken with a margin, because landing *on* the cap means any
            // roughness in the ground between the two samples puts it back
            // over, and the first cut of this recovered a sixth of what the
            // linear estimate said it should.
            *w_side = (w * cap * FIT_MARGIN / rise.abs()).max(s.drawn_half());
            face(side, *w_side)
        };
        let l = fit(1.0, rise_l, &mut w_l);
        let r = fit(-1.0, rise_r, &mut w_r);
        (rise_l, reach_l) = l;
        (rise_r, reach_r) = r;
    }
    if rise_l.abs().max(rise_r.abs()) > cap {
        return None;
    }
    Some(EarthworkEdge {
        a: s.a,
        b: s.b,
        target_a,
        target_b,
        half_width_m: [w_l, w_r],
        // The drawn band itself, held outright against any neighbouring bench —
        // the same rule that keeps a road's own carriageway out of a
        // neighbour's reach (`EarthworkEdge::carriageway_m`).
        carriageway_m: s.drawn_half(),
        batter_m: [reach_l.0, reach_r.0],
        batter_run: [reach_l.1, reach_r.1],
        chain,
        arc0: s.arc0,
        cos_lat,
        carve: false,
        headwall: false,
        crest: false,
    })
}

/// Smallest bench-to-bench height difference worth a connecting face; below it
/// the natural ground carries the step on its own.
const PARTNER_STEP_MIN_M: f64 = 0.1;

/// Lateral march step when probing for a partner bench, metres.
const PARTNER_PROBE_STEP_M: f64 = 0.5;

/// The crowded-bench formulation of docs/GROUND.md §2: where a face could not
/// daylight — the reach collapsed to a wall at the bench edge, or to zero —
/// but another bench stands within [`BENCH_GAP_SPAN_M`] on that side, the face
/// is rebuilt as the plane reaching **to the other bench's edge**, so the two
/// benches' faces meet instead of leaving the natural ground standing between
/// them.
///
/// The defect this retires was measured on the Territet rail trench: the
/// cutting's east face collapsed against the climbing flank, so the DEM stood
/// proud in the two-to-eight-metre strip between the rail formation and the
/// road bench above it — a near-vertical sliver of hillside hugging the
/// outermost rail (`slope.terrain_face` 23:1 over 0.64 m at the wall retry,
/// 112:1 where the reach collapsed to zero), with the step landing in open
/// ground where no contact line runs (`slope.terrain_tearing`). The connecting
/// plane spans the whole gap instead: it leaves this bench's verge at its
/// target and lands *inside* the partner's bench, so its far end falls where
/// the partner's own crest line already constrains the mesh, and there is no
/// step in open ground left to tear. It stays a `min`/`max` of planes, and
/// planes cannot tear.
///
/// **Both faces are rewritten to the one plane** — "meet each other" is
/// literal. The cut plane alone measured as no change at all at the site that
/// motivated it: the partner's own face toward the gap was derived against the
/// pre-pass proud ground, so in the fold the junior road's gentle stale fill
/// re-raised the strip right over the senior rail's new ramp
/// (`max(ramp, fill)`), and the drawn section did not move. So the partner's
/// facing face is *steepened* to the plane's run — never lengthened, and only
/// where the plane is steeper than the face it replaces. A steeper fill face
/// is strictly less fill, so the rewrite cannot dam what the face was not
/// already damming; within the partner's existing reach its face now lies on
/// the plane, the collapsed side's cut lies on the plane, and the two compose
/// seamlessly while ground genuinely below the plane (a drainage notch) keeps
/// its dips.
///
/// The pass runs over the assembled stack — every stratum's edges at once —
/// because the pair this exists for is usually cross-stratum (a rail trench
/// beside a road bench) and the senior side, which is the one whose face
/// collapsed, derives before the junior bench exists. It reads only bench
/// geometry (extents, targets, chains), never the batters it rewrites, so the
/// result is independent of iteration order (invariant 5).
fn span_bench_gaps(layers: Vec<GroundLayer>) -> Vec<GroundLayer> {
    // Snapshot every bench in the stack, with a grid over bench extents so a
    // probe point resolves to the benches that hold the ground there.
    let mut edges: Vec<Vec<EarthworkEdge>> =
        layers.iter().map(|l| l.earthworks.edges().to_vec()).collect();
    let benches: Vec<(usize, usize)> = edges
        .iter()
        .enumerate()
        .flat_map(|(li, es)| {
            es.iter().enumerate().filter(|(_, e)| !e.carve).map(move |(ei, _)| (li, ei))
        })
        .collect();
    if benches.is_empty() {
        return layers;
    }
    let mut grid = GridIndex::new();
    for (bi, &(li, ei)) in benches.iter().enumerate() {
        let e = &edges[li][ei];
        let r = e.bench_m() / (DEG_M * e.cos_lat.min(1.0).max(0.1));
        grid.insert(
            (
                e.a.x.min(e.b.x) - r,
                e.a.y.min(e.b.y) - r,
                e.a.x.max(e.b.x) + r,
                e.a.y.max(e.b.y) + r,
            ),
            bi as u32,
        );
    }

    // (layer, edge, side) → the connecting face, computed wholly from the
    // snapshot before anything is rewritten. `clamps` steepens the partner's
    // facing face to the same plane; it never lengthens a reach.
    let mut faces: Vec<(usize, usize, usize, f64, f64)> = Vec::new();
    let mut clamps: Vec<(usize, usize, usize, f64)> = Vec::new();
    let mut scratch: Vec<u32> = Vec::new();
    for &(li, ei) in &benches {
        let e = &edges[li][ei];
        let dx = (e.b.x - e.a.x) * e.cos_lat;
        let dy = e.b.y - e.a.y;
        let len = (dx * dx + dy * dy).sqrt();
        if len < 1e-15 {
            continue;
        }
        let (ux, uy) = (dx / len, dy / len);
        let mid = Coord { x: (e.a.x + e.b.x) * 0.5, y: (e.a.y + e.b.y) * 0.5 };
        let target = (e.target_a + e.target_b) * 0.5;
        for side in [LEFT, RIGHT] {
            let collapsed = e.batter_run[side] == WALL_BATTER || e.batter_m[side] == 0.0;
            if !collapsed {
                continue;
            }
            let (lx, ly) = if side == LEFT { (-uy, ux) } else { (uy, -ux) };
            let mut d = e.half_width_m[side] + PARTNER_PROBE_STEP_M;
            let hit = loop {
                if d > e.half_width_m[side] + BENCH_GAP_SPAN_M {
                    break None;
                }
                let q = Coord {
                    x: mid.x + d * lx / (DEG_M * e.cos_lat.min(1.0).max(0.1)),
                    y: mid.y + d * ly / DEG_M,
                };
                grid.query((q.x, q.y, q.x, q.y), &mut scratch);
                scratch.sort_unstable();
                let found = scratch.iter().find_map(|&bi| {
                    let (pl, pe) = benches[bi as usize];
                    if (pl, pe) == (li, ei) {
                        return None;
                    }
                    let p = &edges[pl][pe];
                    if p.chain == e.chain && (p.arc0 - e.arc0).abs() < HAIRPIN_ARC_M {
                        return None; // the same run continuing, not a partner
                    }
                    let (dp, tp, ps) = modifiers::lateral_distance(p, q.x, q.y);
                    if dp > p.half_width_m[ps] {
                        return None;
                    }
                    Some((pl, pe, dp, ps, p.target_a + (p.target_b - p.target_a) * tp))
                });
                if let Some(hit) = found {
                    break Some((d, hit));
                }
                d += PARTNER_PROBE_STEP_M;
            };
            // Cut toward a higher partner only. The mirrored fill extension —
            // a collapsed face reaching *down* to a lower partner — was tried
            // and measured: it holds diving ground up across the gap, and
            // where that gap drains a stream the plane is a dam
            // (`water.descends` 2.498 % → 2.511 %). A cut cannot dam
            // anything, and the fill-side deficit already has its answer: the
            // rim wall the model implies is drawn as an apron and measured at
            // the kerb (docs/GROUND.md §3).
            if let Some((d, (pl, pe, dp, ps, partner))) = hit {
                let dh = partner - target;
                if dh > PARTNER_STEP_MIN_M {
                    // One plane through both crests: the run comes from the
                    // true edge-to-edge gap, so this side's cut and the
                    // partner's fill are the same function. Only the *reach*
                    // is biased long (the probe point lies inside the partner
                    // bench), so the plane's far end always crosses the
                    // partner's crest line rather than stopping short of it.
                    let reach = d - e.half_width_m[side];
                    let gap = (reach - (edges[pl][pe].half_width_m[ps] - dp))
                        .max(PARTNER_PROBE_STEP_M * 0.5);
                    let run = gap / dh;
                    faces.push((li, ei, side, reach, run));
                    // The partner's face toward this bench joins the same
                    // plane — steepened only, within its existing reach, so
                    // its stale fill (derived against the pre-pass proud
                    // ground) stops standing over the ramp.
                    let (_, _, facing) = modifiers::lateral_distance(
                        &edges[pl][pe],
                        mid.x,
                        mid.y,
                    );
                    clamps.push((pl, pe, facing, run));
                }
            }
        }
    }

    if faces.is_empty() {
        return layers;
    }
    for &(li, ei, side, reach, run) in &faces {
        edges[li][ei].batter_m[side] = reach;
        edges[li][ei].batter_run[side] = run;
    }
    // Steepen-only, so the cumulative result is the min over every suggesting
    // plane whatever the order, and a face already steeper is left alone.
    for &(li, ei, side, run) in &clamps {
        if run < edges[li][ei].batter_run[side] {
            edges[li][ei].batter_run[side] = run;
        }
    }
    layers
        .into_iter()
        .zip(edges)
        .map(|(l, es)| GroundLayer {
            stratum: l.stratum,
            earthworks: Earthworks::new(es),
            waters: l.waters,
        })
        .collect()
}

/// Every profiled corridor's earthworks, derived in parallel (the bench-edge
/// side sampling is DEM-decode bound, like the shoreline reads) and
/// flattened in corridor order, so the edge indices — and the modifier
/// tie-breaking they feed — are deterministic run to run (invariant 5).
fn derive_earthworks(
    scene: &SceneGraph,
    solved: &SolvedModel,
    facades: &Facades,
    members: &[usize],
    beneath: &[GroundLayer],
    terrain_path: Option<&Path>,
    threads: usize,
) -> Vec<EarthworkEdge> {
    let n = members.len();
    let dem = terrain_path.and_then(|p| Dem::open(p).ok());
    let Some(primary) = dem else {
        // No DEM: no side sampling — the centerline trigger alone (the flat
        // world solves no profiles anyway; this is the test path).
        return members
            .iter()
            .map(|&i| &scene.corridors[i])
            .filter_map(|c| {
                solved.profile(c.id).map(|p| corridor_earthworks(c, p, Some(facades), None))
            })
            .flatten()
            .collect();
    };
    let z_ref = solved.z_ref;
    let threads = threads.max(1).min(n.max(1));
    let next = Mutex::new(0usize);
    let out: Mutex<Vec<Vec<EarthworkEdge>>> = Mutex::new((0..n).map(|_| Vec::new()).collect());
    std::thread::scope(|scope| {
        for _ in 0..threads {
            scope.spawn(|| {
                let Ok(mut dem) = primary.fork() else { return };
                loop {
                    let i = {
                        let mut cur = next.lock().expect("earthwork queue poisoned");
                        if *cur >= n {
                            break;
                        }
                        let i = *cur;
                        *cur += 1;
                        i
                    };
                    let c = &scene.corridors[members[i]];
                    let Some(p) = solved.profile(c.id) else { continue };
                    // **The ground beneath this stratum, not the raw DEM.**
                    // This sampler decides how far a batter reaches and whether
                    // a bench is plausible at all, from the cut or fill at the
                    // bench edge and the cross-slope there. Reading the natural
                    // hillside made every earthwork answer as if it were the
                    // only one — a road bench beside a rail embankment computed
                    // its face against ground the embankment had already
                    // replaced. I1 says no generator samples the raw DEM
                    // outside terrain conditioning; this was the one place in
                    // the shipped code that did.
                    let mut scratch: Vec<u32> = Vec::new();
                    let mut sample = |q: Coord| {
                        let raw = reference_surface(&mut dem, z_ref, q.x, q.y);
                        let mut h = raw;
                        for layer in beneath {
                            h = layer.apply(q.x, q.y, h, 0.0, &mut scratch);
                        }
                        h
                    };
                    let edges = corridor_earthworks(c, p, Some(facades), Some(&mut sample));
                    out.lock().expect("earthwork results poisoned")[i] = edges;
                }
            });
        }
    });
    out.into_inner().expect("earthwork results poisoned").into_iter().flatten().collect()
}

/// The earthwork edges one profiled corridor implies (docs/GROUND.md §2):
/// the bench the ground holds under the carriageway, the under-deck
/// daylighting carves, and the portal cuts.
///
/// Every at-grade stretch gets its bench, whether or not the road departs the
/// natural ground: the bench is not only how an embankment or a cutting is
/// expressed, it is how a road's own band *holds* its height against the
/// earthworks around it. A road the DEM already images correctly used to get
/// no bench at all, and a neighbouring motorway's approach fill would then
/// bury it under 12 m of ground. `side` samples the natural ground at the
/// bench edges so the batter's reach scales with the deepest face, which on a
/// cross-slope sits at the bench edge rather than under the centerline;
/// `None` (no DEM) falls back to the centerline depth alone.
fn corridor_earthworks(
    c: &crate::scene::Corridor,
    p: &crate::solve::Profile,
    facades: Option<&Facades>,
    side: Option<&mut dyn FnMut(Coord) -> f64>,
) -> Vec<EarthworkEdge> {
    let mut edges: Vec<EarthworkEdge> = Vec::new();
    {
        // Earthworks run along the *smoothed* sweep line — the same curve the
        // decks are swept along and the paint snaps to — so the roadbed crest
        // stays parallel to a deck edge instead of wiggling ±1–2 m beside it
        // (at a grazing view the crest occludes the deck's lower edge, and a
        // wiggling crest reads as a jagged deck).
        let nodes = p.smooth();
        // ...but the smoothed line is not the raw one *along* its own
        // direction: low-passing a curved centerline shortens it, so
        // `smooth[k]` lags `nodes[k]` by a lag that accumulates through every
        // bend — 11.6 m by the middle of the Territet funicular. Pairing it
        // with `road[k]` puts the bench at the height of a point it is no
        // longer standing at, and that error is the lag times the grade:
        // 0.7 m at a motorway's 6 %, where it hides inside the batter, and
        // 6.9 m at a funicular's 59 %, where it buries the track under its own
        // bench. So read the profile at the arc the smoothed point occupies.
        let at: Vec<f64> = nodes.iter().map(|c| p.arc_of(c.x, c.y)).collect();
        let road: Vec<f64> = at.iter().map(|&a| p.road_at_arc(a)).collect();
        let terrain: Vec<f64> = at.iter().map(|&a| p.surface_at_arc(a)).collect();
        let (road, terrain) = (&road[..], &terrain[..]);
        let at_grade = p.at_grade();
        let arcs = p.arc();
        let cos_lat = c.cos_lat;
        // Carves keep the engineering width; the road bench adds a narrow
        // verge beyond the asphalt (see EARTHWORK_MARGIN_M).
        let half_width = c.kind.prior().half_width_m(c.link).unwrap_or(0.0) + EARTHWORK_SHOULDER_M;
        let bench_half_width = half_width + EARTHWORK_MARGIN_M;
        // The asphalt this bench carries, which it holds against any neighbour
        // (see `EarthworkEdge::carriageway_m`): the half-width the pavement
        // paints (`synth::carriageway::corridor_half_width_m`) *plus the verge*,
        // bounded by the bench itself so the claim never exceeds what is held.
        //
        // The verge is the point. Where this bench loses to a neighbour the
        // field steps, and against a road several metres higher that step is a
        // near-vertical retaining wall. A wall's triangles occupy almost no
        // plan area but span its whole height, so wherever one stands over the
        // asphalt — even by a centimetre — the drawn ground reads metres above
        // the road across that sliver, and at a grazing view the sliver
        // projects to the wall's full height. Claiming the paint alone put the
        // step exactly on the kerb and did just that. Claiming the verge too
        // moves every wall a verge clear of the drawn surface.
        let carriageway = c
            .width_m
            .map(|w| w * 0.5 + crate::priors::STRUCTURE_SHOULDER_M + EARTHWORK_MARGIN_M)
            .unwrap_or(bench_half_width)
            .min(bench_half_width);

        // **The room the facades leave, per node and per side** — the same
        // question `synth::carriageway::sections_along` asks of the asphalt,
        // asked here of the ground under and beside it. Resolved once, now,
        // into numbers baked onto each edge: `GroundStack::height` runs per
        // lattice vertex and must never learn what a facade is (the rejected
        // lateral-trench rule cost 38 s of tiling time for exactly that shape
        // of index in the hot path).
        //
        // Rail is out, for the reason `order.building_overlap` leaves it out:
        // a station roof over its platforms is a level relation the model
        // cannot state, and narrowing the formation there shaves the platform.
        // **Off by default, and the measurement is why.** Built, tiled and
        // scored: clipping the bench at the wall takes `authority.facade_ground`
        // 1.934 -> 1.096 % (2,225 fewer wall samples past a metre) and pays
        // 3,850 *more* kerbs with a drop beside them (`contact.kerb_lip` 6.80
        // -> 7.70 %). Clipping the batter with it doubles both sides of that
        // trade (0.656 % and 7,828 kerbs) and it is still a loss.
        //
        // The mechanism is not subtle once seen: `contact.kerb_lip` probes one
        // metre outside the kerb, and narrowing the bench puts that probe on
        // the batter face instead of on the verge. Any narrowing costs it,
        // whatever the room says — *until something occupies the strip between
        // the kerb and the facade*. That something is the walk band (the
        // plan's phase 5), riding the host's cross-section at `KERB_RISE_M`.
        // So the machinery lands and the switch waits for it.
        let clips = std::env::var_os("ARPT_FACADE_BENCH")
            .and(facades)
            .filter(|_| c.kind.prior().surface == crate::priors::Surface::Asphalt)
            .filter(|f| !f.is_empty());
        let clip_batter = std::env::var_os("ARPT_FACADE_BATTER").is_some();
        // The drawn band this bench must hold whatever a wall says — phase 2's
        // own allocation, so bench and band come off one cross-section
        // (ROADS.md invariant 1). A bench narrower than its asphalt hangs the
        // drawn surface over unbenched ground, which is what `carriageway_m`
        // exists to prevent.
        let band_nominal = c.width_m.map(|w| w * 0.5 + c.kind.prior().shoulder_m());
        let mut room_at: Vec<Room> = vec![Room::open(f64::INFINITY); nodes.len()];
        let mut bench: Vec<Section> = vec![Section::uniform(bench_half_width); nodes.len()];
        if let Some(facades) = clips {
            let mut scratch: Vec<u32> = Vec::new();
            // Far enough out to see the wall a *face* would run under, not
            // only the one the bench would: half the population this exists
            // for is on the batter beyond the bench.
            let reach = bench_half_width + EARTHWORK_MAX_BATTER_M;
            for k in 0..nodes.len() {
                if !at_grade[k] {
                    continue;
                }
                let (ux, uy) = heading(nodes, k, cos_lat);
                let window = node_window_m(arcs, k);
                room_at[k] = facades.room(nodes[k], cos_lat, (ux, uy), reach, window, &mut scratch);
                let band = match band_nominal {
                    Some(b) => room_at[k].allot(b, crate::priors::MIN_CARRIAGEWAY_HALF_M),
                    None => Section::uniform(0.0),
                };
                // **The ground reaches the wall; nothing crosses it.**
                // `FACADE_CLEAR_M` is a *drawn surface* clearance — asphalt
                // keeps off a footprint — and the ground is not a drawn
                // surface in that sense: a wall stands *on* ground, and a
                // street between buildings is flat from facade to facade.
                // Clipping the bench a clearance short of the wall was built
                // and measured first, and it moves the defect to the kerb
                // rather than removing it: the flat ground stops half a metre
                // outside the asphalt, the hill takes over there, and
                // `contact.kerb_lip` went 6.80 → 9.08 % for a drop that a real
                // street does not have. What must not happen is the road's
                // terrace continuing *past* the building line into the hill,
                // which is the batter's clip below.
                bench[k] = Section {
                    left_m: bench_half_width.min(room_at[k].left).max(band.left_m),
                    right_m: bench_half_width.min(room_at[k].right).max(band.right_m),
                };
            }
        }

        // Per node and per side, how far the batter runs before it daylights.
        // The bench-edge sample gives both the face depth there and the
        // cross-slope the natural ground carries outward, which is what
        // decides whether a face of [`EARTHWORK_BATTER`] ever meets it.
        // Without a sampler (no DEM) the centerline depth stands in on both
        // sides.
        let centre_reach = |k: usize| batter_reach(road[k] - terrain[k], 0.0);
        let mut batter: Vec<[(f64, f64); 2]> =
            (0..nodes.len()).map(|k| [centre_reach(k), centre_reach(k)]).collect();
        // Whether a bench is plausible at all here — see [`MAX_BENCH_FACE_M`]
        // for the trade that cap balances.
        //
        // A railway is not on either side of that trade, so it benches
        // unconditionally. Refusing buys a smaller retaining wall and pays for
        // it with a float, and for a road the float is invisible: the terrain
        // is cut back to the kerb, so the drawn ground never competes with the
        // drawn asphalt. Rail paves nothing — no carriageway mesh, no kerb, no
        // hole, no apron — so it buys nothing and pays the whole price in
        // daylight. The face is bounded anyway by the profile's own deviation
        // budget; what the cap adds is a *discontinuity*, benching a node and
        // declining its neighbour, and the step it leaves is the defect.
        let always_bench = c.kind.modality() == crate::priors::Modality::Rail;
        let mut benched: Vec<bool> = vec![true; nodes.len()];
        if let Some(sample) = side {
            for k in 0..nodes.len() {
                if !at_grade[k] {
                    continue;
                }
                let (ux, uy) = heading(nodes, k, cos_lat);
                let (px, py) = (-uy, ux); // lateral unit, metric (left)
                let hw = |s: f64| if s > 0.0 { bench[k].left_m } else { bench[k].right_m };
                let mut face = |s: f64| -> (f64, (f64, f64)) {
                    // Sampled at the bench's *own* edge on this side, which is
                    // where the face stands: with the room clipping one side
                    // the two edges are no longer the same distance out, and
                    // reading both at the nominal half-width would size the
                    // narrowed side's face from ground it never reaches.
                    let w = hw(s);
                    let q = Coord {
                        x: nodes[k].x + s * px * w / (DEG_M * cos_lat),
                        y: nodes[k].y + s * py * w / DEG_M,
                    };
                    let edge_raw = sample(q);
                    // The face the bench cuts or fills at its edge, and the
                    // outward slope of the natural ground from the centerline
                    // (positive uphill).
                    let rise = road[k] - edge_raw;
                    (rise, batter_reach(rise, (edge_raw - terrain[k]) / w.max(1e-6)))
                };
                let (rise_l, reach_l) = face(1.0);
                let (rise_r, reach_r) = face(-1.0);
                // **The face is clipped to the same room the bench is.** Half
                // of `authority.facade_ground`'s population is a batter face
                // running under a wall, and narrowing the bench alone moves
                // the face inward while it still runs outward until it
                // daylights — so that half would have got worse. Beyond the
                // room the ground is the hill's, and whatever step is left at
                // the building line is a retaining wall, which is what a
                // street between buildings has.
                let clip = |reach: (f64, f64), s: f64| -> (f64, f64) {
                    if !clip_batter {
                        return reach;
                    }
                    let room = if s > 0.0 { room_at[k].left } else { room_at[k].right };
                    (reach.0.min((room - hw(s)).max(0.0)), reach.1)
                };
                batter[k] = [clip(reach_l, 1.0), clip(reach_r, -1.0)];
                // Unchanged, and deliberately so. The step the clip leaves is
                // never deeper than the face's own rise, so this test already
                // refuses every node the clip could have made implausible —
                // the 108 node-sides the census counted past
                // `MAX_BENCH_FACE_M` are ones today already declines. The clip
                // adds no refusals, which is what keeps its effect attributable
                // to the ground it stops moving rather than to roads it stops
                // benching.
                benched[k] = always_bench || rise_l.abs().max(rise_r.abs()) <= MAX_BENCH_FACE_M;
            }
        }

        // Every at-grade edge carries a bench. Where the road already lies on
        // the natural ground the bench is nearly a no-op — it flattens the
        // carriageway across the cross-slope and reserves the band against
        // neighbouring earthworks — and its batter reach collapses to the
        // floor, so it costs the ground nothing beyond its own width.
        for k in 0..nodes.len().saturating_sub(1) {
            if !at_grade[k] || !at_grade[k + 1] || !benched[k] || !benched[k + 1] {
                continue;
            }
            edges.push(EarthworkEdge {
                a: nodes[k],
                b: nodes[k + 1],
                target_a: road[k],
                target_b: road[k + 1],
                // The narrower of the edge's two ends on each side: an edge is
                // one bench, and it may not reach where either of its ends was
                // told it could not.
                half_width_m: [
                    bench[k].left_m.min(bench[k + 1].left_m),
                    bench[k].right_m.min(bench[k + 1].right_m),
                ],
                carriageway_m: carriageway,
                batter_m: [
                    batter[k][LEFT].0.max(batter[k + 1][LEFT].0),
                    batter[k][RIGHT].0.max(batter[k + 1][RIGHT].0),
                ],
                // The steeper of the two nodes' faces: a run that changes along
                // an edge is one face, and the edge must not draw it shallower
                // than either end asked for.
                batter_run: [
                    batter[k][LEFT].1.min(batter[k + 1][LEFT].1),
                    batter[k][RIGHT].1.min(batter[k + 1][RIGHT].1),
                ],
                chain: c.id,
                arc0: arcs[k],
                cos_lat: crate::scene::run_cos_lat(&[nodes[k], nodes[k + 1]]),
                carve: false,
                headwall: false,
                crest: true,
            });
        }

        // Deck daylighting (the S10 mirror of the portal cut): inside a
        // mapped bridge span the deck is trusted to fly, but a DEM bump can
        // poke above it — a wooded ridge a surface DEM reads as ground — and
        // the terrain, drawn first, swallows the deck there. The bump is
        // carved to just below the deck underside. Interior bumps only: a
        // run reaching a span end is the deck legitimately meeting the
        // ground (an abutment, a portal in the hillside, S7) and stays for
        // the occlusion to work. A bump deeper than [`MAX_CLEARANCE_LIFT_M`]
        // is a data contradiction (a "bridge" through a real hill): the
        // terrain is trusted and the deck stays buried.
        for span in c.spans.iter().filter(|s| s.kind == SpanKind::Bridge) {
            let s0 = arcs.partition_point(|&a| a < span.arc0);
            let s1 = arcs.partition_point(|&a| a <= span.arc1);
            let intrudes = |i: usize| terrain[i] > road[i] - DECK_THICKNESS_M;
            let mut i = s0;
            while i < s1 {
                if !intrudes(i) {
                    i += 1;
                    continue;
                }
                let first = i;
                while i < s1 && intrudes(i) {
                    i += 1;
                }
                let last = i - 1;
                if first == s0 || last + 1 == s1 {
                    continue; // touches a span end: the deck meets the ground
                }
                let depth = (first..=last)
                    .map(|k| terrain[k] - (road[k] - DECK_THICKNESS_M))
                    .fold(0.0, f64::max);
                if depth > MAX_CLEARANCE_LIFT_M {
                    continue;
                }
                // Pad one node each side so the notch eases in below grade.
                let (lo, hi) = (first - 1, (last + 1).min(nodes.len() - 1));
                for k in lo..hi {
                    edges.push(EarthworkEdge {
                        a: nodes[k],
                        b: nodes[k + 1],
                        target_a: road[k] - DECK_THICKNESS_M - PORTAL_CLEARANCE_M,
                        target_b: road[k + 1] - DECK_THICKNESS_M - PORTAL_CLEARANCE_M,
                        half_width_m: [half_width; 2],
                        carriageway_m: 0.0,
                        batter_m: [(EARTHWORK_BATTER * depth).max(EARTHWORK_MIN_BATTER_M); 2],
                        batter_run: [EARTHWORK_BATTER; 2],
                        chain: c.id,
                        arc0: arcs[k],
                        cos_lat: crate::scene::run_cos_lat(&[nodes[k], nodes[k + 1]]),
                        carve: true,
                        headwall: false,
                        crest: true,
                    });
                }
            }
        }

        // Portal daylighting: carve the ground down to the bore floor in a
        // short cut outward from each solved portal, so the mouth's lower
        // metres stand clear instead of hiding below grade. Cut-only — where
        // the ground has already fallen away there is nothing to remove — and
        // closed at the mouth by its own face (`headwall`), because the cut is
        // in *front* of the portal and the hill behind it is the tunnel's
        // cover.
        for portal in portals::portals(p, &c.spans) {
            let a = p.point_at_arc(portal.arc);
            let b = p.point_at_arc(portal.arc + portal.outward * PORTAL_CUT_LEN_M);
            if a == b {
                continue; // portal at the corridor end: no outward run
            }
            edges.push(EarthworkEdge {
                a,
                b,
                target_a: portal.floor_m,
                target_b: portal.floor_m,
                half_width_m: [c.kind.prior().half_width_m(c.link).unwrap_or(0.0)
                    + EARTHWORK_SHOULDER_M; 2],
                carriageway_m: 0.0,
                batter_m: [EARTHWORK_MIN_BATTER_M; 2],
                batter_run: [EARTHWORK_BATTER; 2],
                chain: c.id,
                arc0: portal.arc,
                cos_lat: crate::scene::run_cos_lat(&[a, b]),
                carve: true,
                headwall: true,
                crest: true,
            });
        }
    }
    edges
}

/// How far along the centerline node `k` looks for the walls beside it, in
/// metres — at least the gap to its neighbours, so consecutive nodes see the
/// same wall and the width they interpolate between never crosses it. The same
/// rule `synth::carriageway::sections_along` uses, and for the same reason: a
/// shorter window lets a wall between two nodes go unseen, a longer one only
/// clips the ground sooner.
fn node_window_m(arcs: &[f64], k: usize) -> f64 {
    let (j, l) = (k.saturating_sub(1), (k + 1).min(arcs.len().saturating_sub(1)));
    (arcs[l] - arcs[j]).clamp(ROOM_WINDOW_MIN_M, ROOM_WINDOW_MAX_M)
}

/// Bounds on [`node_window_m`], matching `synth::carriageway`'s.
const ROOM_WINDOW_MIN_M: f64 = 4.0;
const ROOM_WINDOW_MAX_M: f64 = 32.0;

/// How far a batter face runs outward before it daylights, in metres, given
/// the face's height at the bench edge (`rise`, positive where the road stands
/// above the ground — a fill — negative in a cutting) and the outward slope
/// the natural ground carries from there (`cross`, positive uphill).
///
/// The face leaves the bench at 1 in [`EARTHWORK_BATTER`] and the ground runs
/// away at `cross`; they meet where the two close the gap. Where they never do
/// — ground falling faster than a fill's batter, or climbing faster than a
/// cutting's — the reach collapses to **zero**: the bench is retained by a wall
/// at its own edge, which is what a road cut into a steep flank has, rather
/// than a terrace that runs out into the hillside and ends in a cliff. Clamped
/// above by [`EARTHWORK_MAX_BATTER_M`] so a deep fill against near-flat ground
/// still has a bounded footprint.
///
/// **A wall where a batter cannot close, and only then zero.** The face is
/// tried twice: once as an earth slope at 1 in [`EARTHWORK_BATTER`], and where
/// that cannot daylight, again as a wall at 1 in [`WALL_BATTER`]. Both are the
/// same self-limiting geometry and the same daylight test, so the second is not
/// a special case, it is the same rule asked of a steeper face. Only when
/// neither closes does the reach collapse to zero.
///
/// This is what a road cut into a steep flank physically has. Leaving it at zero
/// — which is what this did at first — means the earthwork builds *nothing*
/// there, and the height field is left carrying a step it does not have: over
/// the Montreux extract the DEM carries a median 84 % of the separation two
/// crowded platforms' profiles carry, and under half in 29 % of them
/// (docs/GROUND.md §2). The wall is how the model supplies the difference
/// without inventing a shape: it is bounded by the same daylight test as the
/// batter, so where the ground *does* carry the step it builds nothing.
///
/// **Zero, where it used to be a floor.** The collapsed case used to keep a
/// two-metre bevel, the same [`EARTHWORK_MIN_BATTER_M`] that eases a converging
/// face in. On a diverging face that bevel is not an easing, it is a trench: the
/// ground is clamped to a plane rising at 1 in 2.5 while the hillside beside it
/// climbs at 1 in 1, so the clamp bites harder the further out it runs, and at
/// two metres it stops and the field drops back to the hillside in one step.
/// Beside a Territet switchback that step was 1.7 m, and it stands at a fixed
/// offset from a centerline the lattice does not follow, out in open ground
/// where no contact line runs — so the detail mesh sampled it in and out and
/// drew the flank as a row of teeth (`slope.terrain_tearing`). Collapsing to
/// zero puts the wall on the bench edge instead, which is the one place out
/// there a crest line already holds ([`super::breaklines`]).
///
/// The floor stays on the *converging* branch, where it costs nothing: past the
/// point a converging face daylights, the natural ground is already inside the
/// face, so `min`/`max` returns it unchanged however much further the reach is
/// allowed to run. Removing it there measured worse, not better — 0.20 % of
/// terrain vertices tearing became 0.24 % — because it also deleted the easing
/// under every ordinary road.
///
/// A face that does not close *quickly* must not be built at all. The face is
/// a plane and the hillside is not: where the ground runs away at anything
/// near the batter's own rate, the plane keeps departing from it, and since
/// the ground is clamped to the face everywhere inside the reach, the
/// earthwork goes on cutting (or filling) the whole way out — a footpath on a
/// steep flank whose estimated reach came to 40 m carved sixty metres off the
/// hillside. So the reach is honoured only while the face closes at least
/// about as fast as it would on flat ground; [`BATTER_DIVERGENCE_SLOP`] allows
/// for a gentle fall-away, and beyond it the answer is the wall, whose height
/// is only the bench half-width times the cross-slope.
fn batter_reach(rise: f64, cross: f64) -> (f64, f64) {
    // How far a face of this shape runs before it meets the ground, or `None`
    // where it never does.
    let daylight = |run: f64, floor: f64| -> Option<f64> {
        let slope = 1.0 / run;
        // A fill descends against ground that must rise to meet it; a cutting
        // climbs against ground that must fall. One expression: the closing
        // rate is the face minus however fast the ground runs away with it.
        let closing = slope - cross * if rise >= 0.0 { -1.0 } else { 1.0 };
        if closing <= 1e-9 {
            return None;
        }
        let reach = rise.abs() / closing;
        // Where the face would daylight on flat ground — the yardstick for
        // "closes quickly". Past it the ground is running away with the face.
        if reach > run * rise.abs() * BATTER_DIVERGENCE_SLOP {
            return None;
        }
        Some(reach.clamp(floor, EARTHWORK_MAX_BATTER_M))
    };
    // The earth slope keeps its floor, which costs nothing where it closes.
    // The wall gets none: it is already short, and a floor on a steep face is
    // how the batter's own floor became a trench.
    if let Some(r) = daylight(EARTHWORK_BATTER, EARTHWORK_MIN_BATTER_M) {
        return (r, EARTHWORK_BATTER);
    }
    if let Some(r) = daylight(WALL_BATTER, 0.0) {
        return (r, WALL_BATTER);
    }
    (0.0, EARTHWORK_BATTER)
}

/// Unit heading of the sweep line at node `i` in the metric (east, north)
/// frame: the direction of the segment the node starts (the last node
/// borrows the segment it ends).
fn heading(nodes: &[Coord], i: usize, cos_lat: f64) -> (f64, f64) {
    let (a, b) =
        if i + 1 < nodes.len() { (nodes[i], nodes[i + 1]) } else { (nodes[i - 1], nodes[i]) };
    let dx = (b.x - a.x) * cos_lat;
    let dy = b.y - a.y;
    let len = (dx * dx + dy * dy).sqrt();
    if len < 1e-15 {
        (1.0, 0.0)
    } else {
        (dx / len, dy / len)
    }
}

/// Reads a flat surface level for every still water body from the DEM along its
/// shoreline, so the ground stage can burn it flat (invariant 4). Parallelized
/// over `threads` workers (each forks the DEM to share the decoded-tile cache),
/// since sampling scattered shorelines is DEM-decode bound. Without a DEM there
/// is nothing to read a level from; the water stays draped on the terrain.
fn derive_waters(
    scene: &SceneGraph,
    solved: &SolvedModel,
    terrain_path: Option<&Path>,
    threads: usize,
) -> Waters {
    let bodies = &scene.water;
    if bodies.is_empty() {
        return Waters::new(Vec::new());
    }
    let Some(path) = terrain_path else {
        return Waters::new(Vec::new());
    };
    let Ok(primary) = Dem::open(path) else {
        return Waters::new(Vec::new()); // no DEM: leave the water draped
    };
    let z_ref = solved.z_ref;
    let n = bodies.len();
    let threads = threads.max(1).min(n);
    let next = Mutex::new(0usize);
    let fills: Mutex<Vec<Option<WaterFill>>> = Mutex::new(vec![None; n]);
    std::thread::scope(|scope| {
        for _ in 0..threads {
            scope.spawn(|| {
                let Ok(mut dem) = primary.fork() else { return };
                loop {
                    let i = {
                        let mut cur = next.lock().expect("water queue poisoned");
                        if *cur >= n {
                            break;
                        }
                        let i = *cur;
                        *cur += 1;
                        i
                    };
                    let w = &bodies[i];
                    if let Some(level) =
                        water_level(&w.exterior, |c| reference_surface(&mut dem, z_ref, c.x, c.y))
                    {
                        fills.lock().expect("water fills poisoned")[i] = Some(WaterFill {
                            exterior: w.exterior.clone(),
                            holes: w.holes.clone(),
                            bbox: w.bbox,
                            level,
                        });
                    }
                }
            });
        }
    });
    Waters::new(fills.into_inner().expect("water fills poisoned").into_iter().flatten().collect())
}

/// A still water body's surface level: a low percentile of the DEM sampled
/// along its shoreline. The exterior ring traces the waterline, so the ground
/// there images the water level; a low percentile ([`WATER_LEVEL_PCTL`]) leans
/// toward the water rather than a vertex that climbed the bank, so the flat
/// surface sits in its basin instead of spilling over the shore. The ring is
/// subsampled to at most [`SHORELINE_SAMPLES`] points so a huge lake stays
/// cheap. The sampler is injected so the level is testable without a DEM.
fn water_level(ring: &[Coord], mut sample: impl FnMut(Coord) -> f64) -> Option<f64> {
    if ring.len() < 3 {
        return None;
    }
    let step = (ring.len() / SHORELINE_SAMPLES).max(1);
    let mut hs: Vec<f64> = ring.iter().step_by(step).map(|&c| sample(c)).collect();
    if hs.is_empty() {
        return None;
    }
    hs.sort_by(|a, b| a.partial_cmp(b).expect("finite elevations"));
    let idx = ((hs.len() as f64 * WATER_LEVEL_PCTL) as usize).min(hs.len() - 1);
    Some(hs[idx])
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The fold is the ground: `groundₙ₊₁ = groundₙ ⊕ layer n`, and
    /// `height_through` is what stratum n reads while it solves.
    #[test]
    fn the_stack_folds_its_layers_in_authority_order() {
        let cos_lat = 46.0_f64.to_radians().cos();
        // A senior bench at 400 m and a junior one at 410 m over the same spot.
        let bench = |target: f64, chain: u32| EarthworkEdge {
            a: Coord { x: 6.0, y: 46.0 },
            b: Coord { x: 6.001, y: 46.0 },
            target_a: target,
            target_b: target,
            half_width_m: [8.0; 2],
            carriageway_m: 6.0,
            batter_m: [4.0; 2],
            batter_run: [EARTHWORK_BATTER; 2],
            chain,
            arc0: 0.0,
            cos_lat,
            carve: false,
            headwall: false,
            crest: true,
        };
        let stack = GroundStack::new(vec![
            GroundLayer::of_earthworks(Stratum::R, Earthworks::new(vec![bench(400.0, 0)])),
            GroundLayer::of_earthworks(Stratum::S, Earthworks::new(vec![bench(410.0, 1)])),
        ]);
        let mut sc = Vec::new();
        let (lon, lat) = (6.0005, 46.0);
        // ground₀ is the DEM; ground₁ holds the rail bench; ground₂ the road's,
        // cut into a ground that already contains it.
        assert_eq!(stack.height_through(0, lon, lat, 380.0, 0.0, &mut sc), 380.0);
        assert_eq!(stack.height_through(1, lon, lat, 380.0, 0.0, &mut sc), 400.0);
        assert_eq!(stack.height_through(2, lon, lat, 380.0, 0.0, &mut sc), 410.0);
        // `height` is the whole fold — the junior layer has the last word,
        // which is what "a road cutting is carved into a ground that already
        // holds the rail embankment" means.
        assert_eq!(stack.height(lon, lat, 380.0, 0.0, &mut sc), 410.0);
    }

    /// The accumulation, at its narrowest: a junior layer's bench sees the
    /// senior layer's ground, not the natural hillside.
    ///
    /// This is what `groundₙ₊₁ = groundₙ ⊕ stratum n's earthworks` buys. The
    /// bench-edge sampler decides how far a batter reaches from the cut or fill
    /// it makes there; against the raw DEM every earthwork answers as if it
    /// were the only one in the world.
    #[test]
    fn a_junior_layer_reads_the_ground_its_senior_left() {
        let cos_lat = 46.0_f64.to_radians().cos();
        let bench = |target: f64, chain: u32| EarthworkEdge {
            a: Coord { x: 6.0, y: 46.0 },
            b: Coord { x: 6.002, y: 46.0 },
            target_a: target,
            target_b: target,
            half_width_m: [8.0; 2],
            carriageway_m: 6.0,
            batter_m: [4.0; 2],
            batter_run: [EARTHWORK_BATTER; 2],
            chain,
            arc0: 0.0,
            cos_lat,
            carve: false,
            headwall: false,
            crest: true,
        };
        // A senior embankment at 420 m over ground the DEM puts at 400 m.
        let senior = GroundLayer::of_earthworks(Stratum::R, Earthworks::new(vec![bench(420.0, 0)]));
        let mut sc = Vec::new();
        let (lon, lat) = (6.001, 46.0);
        // Beside the embankment, a point the batter still reaches: the ground
        // there is no longer the DEM's 400 m.
        let off = 46.0 + 10.0 / DEG_M;
        let raw = 400.0;
        let senior_ground = senior.apply_for_test(lon, off, raw, 0.0, &mut sc);
        assert!(
            senior_ground > raw,
            "the senior embankment should have raised the ground beside it, got {senior_ground}"
        );
        // On the embankment itself the ground *is* the senior's target, which
        // is what a junior road built across it has to bench against.
        assert_eq!(senior.apply_for_test(lon, lat, raw, 0.0, &mut sc), 420.0);
    }

    /// I8's predicate: a layer moves the ground only inside its own declared
    /// footprint. Zero by construction *unless* a reach stops bounding its
    /// influence, which is the thing worth measuring.
    #[test]
    fn a_layer_moves_the_ground_only_inside_its_declared_footprint() {
        let cos_lat = 46.0_f64.to_radians().cos();
        let layer = GroundLayer::of_earthworks(
            Stratum::S,
            Earthworks::new(vec![EarthworkEdge {
                a: Coord { x: 6.0, y: 46.0 },
                b: Coord { x: 6.001, y: 46.0 },
                target_a: 400.0,
                target_b: 400.0,
                half_width_m: [8.0; 2],
                carriageway_m: 6.0,
                batter_m: [4.0; 2],
                batter_run: [EARTHWORK_BATTER; 2],
                chain: 0,
                arc0: 0.0,
                cos_lat,
                carve: false,
                headwall: false,
                crest: true,
            }]),
        );
        let stack = GroundStack::new(vec![layer]);
        let mut sc = Vec::new();
        // Walk out across the bench, its batter, and well past the reach.
        for i in 0..400 {
            let lat = 46.0 + (i as f64 * 0.25) / DEG_M;
            let lon = 6.0005;
            let raw = 390.0;
            let moved = stack.height(lon, lat, raw, 0.0, &mut sc) != raw;
            let covered = stack.layers()[0].covers(lon, lat, &mut sc);
            assert!(
                !moved || covered,
                "the ground moved {:.2} m outside the declared footprint",
                (i as f64 * 0.25)
            );
        }
    }
    use crate::priors::{Kind, RoadClass};
    use crate::scene::{Corridor, Crossing, SegmentRef, Span, SpanKind, DEG_M};
    use geo_types::Coord;

    /// An E-W bench edge ~160 m long at lat 46 with a collapsed face where the
    /// caller says so — the [`span_bench_gaps`] fixtures.
    fn gap_edge(target: f64, north_m: f64, chain: u32, arc0: f64) -> EarthworkEdge {
        let cos_lat = 46.0_f64.to_radians().cos();
        EarthworkEdge {
            a: Coord { x: 6.0, y: 46.0 + north_m / DEG_M },
            b: Coord { x: 6.0 + 160.0 / (DEG_M * cos_lat), y: 46.0 + north_m / DEG_M },
            target_a: target,
            target_b: target,
            half_width_m: [4.0; 2],
            carriageway_m: 3.0,
            batter_m: [2.0; 2],
            batter_run: [EARTHWORK_BATTER; 2],
            chain,
            arc0,
            cos_lat,
            carve: false,
            headwall: false,
            crest: true,
        }
    }

    /// The §2 crowded-bench formulation, cross-stratum: a rail bench whose
    /// north face collapsed to zero, a road bench 12 m north and 5 m up. The
    /// pass rebuilds the face as the plane landing inside the road bench, so
    /// natural ground standing proud between them is cut to the connecting
    /// plane instead of hugging the rail as a wall.
    #[test]
    fn a_collapsed_face_spans_to_the_partner_bench() {
        let mut rail = gap_edge(390.0, 0.0, 0, 0.0);
        rail.batter_m[LEFT] = 0.0; // collapsed: no face at all
        let road = gap_edge(395.0, 12.0, 1, 0.0);
        let layers = span_bench_gaps(vec![
            GroundLayer::of_earthworks(Stratum::R, Earthworks::new(vec![rail])),
            GroundLayer::of_earthworks(Stratum::S, Earthworks::new(vec![road])),
        ]);
        let stack = GroundStack::new(layers);
        let mut sc = Vec::new();
        let mid_x = 6.0 + 80.0 / (DEG_M * 46.0_f64.to_radians().cos());
        // Mid-gap, natural ground standing a metre proud of the road bench:
        // cut down to the connecting plane (390 at the rail verge, 395 inside
        // the road bench, ~392.5 six metres out).
        let h = stack.height(mid_x, 46.0 + 6.0 / DEG_M, 396.0, 0.0, &mut sc);
        assert!(
            (h - 392.5).abs() < 0.5,
            "mid-gap ground must lie on the connecting plane, got {h}"
        );
        // Just outside the rail verge the plane has barely left the bench.
        let toe = stack.height(mid_x, 46.0 + 4.5 / DEG_M, 396.0, 0.0, &mut sc);
        assert!(toe < 391.0, "the face must leave the bench at its target, got {toe}");
        // Ground already below the plane is not raised by the cut face.
        let dip = stack.height(mid_x, 46.0 + 6.0 / DEG_M, 391.0, 0.0, &mut sc);
        assert!(dip <= 391.0 + 1e-9, "a cut face must not fill, got {dip}");
        // The partner's facing face is steepened onto the same plane: inside
        // its own reach it now fills to the plane (393.75 here), not to the
        // gentle stale face (394.6) that used to stand over the ramp.
        let joined = stack.height(mid_x, 46.0 + 7.0 / DEG_M, 391.0, 0.0, &mut sc);
        assert!(
            (joined - 393.75).abs() < 0.1,
            "the partner face must lie on the connecting plane, got {joined}"
        );
    }

    /// No partner within [`BENCH_GAP_SPAN_M`]: the collapsed face stays
    /// collapsed — a road cut into an open mountain flank keeps its wall at
    /// the bench edge, and the hillside beyond stays the hillside.
    #[test]
    fn a_collapsed_face_with_no_partner_stays_collapsed() {
        let mut rail = gap_edge(390.0, 0.0, 0, 0.0);
        rail.batter_m[LEFT] = 0.0;
        let layers = span_bench_gaps(vec![GroundLayer::of_earthworks(
            Stratum::R,
            Earthworks::new(vec![rail]),
        )]);
        let stack = GroundStack::new(layers);
        let mut sc = Vec::new();
        let mid_x = 6.0 + 80.0 / (DEG_M * 46.0_f64.to_radians().cos());
        let h = stack.height(mid_x, 46.0 + 6.0 / DEG_M, 396.0, 0.0, &mut sc);
        assert_eq!(h, 396.0, "with no partner bench the ground stays natural");
    }

    /// A lower partner does not get a connecting plane: the mirrored fill —
    /// the upper bench reaching down — holds diving ground up across the gap,
    /// and where the gap drains a stream that plane is a dam
    /// (`water.descends` measured it, 2.498 % → 2.511 %). The upper bench
    /// keeps its collapsed edge, and the wall the model implies there is the
    /// apron's to draw.
    #[test]
    fn a_lower_partner_does_not_hoist_the_gap() {
        let mut upper = gap_edge(395.0, 0.0, 0, 0.0);
        upper.batter_m[LEFT] = 1.0;
        upper.batter_run[LEFT] = WALL_BATTER; // collapsed to the wall retry
        let lower = gap_edge(390.0, 12.0, 1, 0.0);
        let layers = span_bench_gaps(vec![
            GroundLayer::of_earthworks(Stratum::R, Earthworks::new(vec![upper])),
            GroundLayer::of_earthworks(Stratum::S, Earthworks::new(vec![lower])),
        ]);
        let stack = GroundStack::new(layers);
        let mut sc = Vec::new();
        let mid_x = 6.0 + 80.0 / (DEG_M * 46.0_f64.to_radians().cos());
        // Mid-gap over a dive to 385: the stream notch between the benches
        // stays a notch (the collapsed wall reaches 1 m; past it, natural).
        let h = stack.height(mid_x, 46.0 + 6.0 / DEG_M, 385.0, 0.0, &mut sc);
        assert_eq!(h, 385.0, "diving ground between benches must stay natural");
    }

    /// One chain, two arms: a probe must not take the same run continuing
    /// around a bend for a partner, but a switchback returning far along its
    /// own arc is one.
    #[test]
    fn a_partner_on_the_same_chain_needs_arc_separation() {
        let cos_lat = 46.0_f64.to_radians().cos();
        let mid_x = 6.0 + 80.0 / (DEG_M * cos_lat);
        let mut sc = Vec::new();
        // Near in arc: not a partner, the face stays collapsed.
        let mut a = gap_edge(390.0, 0.0, 7, 0.0);
        a.batter_m[LEFT] = 0.0;
        let near = gap_edge(395.0, 12.0, 7, 20.0);
        let stack = GroundStack::new(span_bench_gaps(vec![GroundLayer::of_earthworks(
            Stratum::S,
            Earthworks::new(vec![a, near]),
        )]));
        let h = stack.height(mid_x, 46.0 + 6.0 / DEG_M, 396.0, 0.0, &mut sc);
        assert_eq!(h, 396.0, "a contiguous neighbour along the run is not a partner");
        // Far in arc — a returning switchback leg — is.
        let mut a = gap_edge(390.0, 0.0, 7, 0.0);
        a.batter_m[LEFT] = 0.0;
        let far = gap_edge(395.0, 12.0, 7, 200.0);
        let stack = GroundStack::new(span_bench_gaps(vec![GroundLayer::of_earthworks(
            Stratum::S,
            Earthworks::new(vec![a, far]),
        )]));
        let h = stack.height(mid_x, 46.0 + 6.0 / DEG_M, 396.0, 0.0, &mut sc);
        assert!((h - 392.5).abs() < 0.5, "a returning leg spans the gap, got {h}");
    }

    /// A viaduct over a valley with one sharp DEM bump poking above the deck
    /// mid-span (a wooded ridge a surface DEM reads as ground): the bump is
    /// carved to below the deck underside so the deck stays visible, while
    /// the valley floor and the span ends are untouched.
    #[test]
    fn a_bump_over_a_deck_is_daylighted() {
        let cos_lat = 46.0_f64.to_radians().cos();
        let len_m = 1000.0;
        let deg = len_m / (DEG_M * cos_lat);
        let n = 201;
        let nodes: Vec<Coord> =
            (0..n).map(|i| Coord { x: 6.0 + deg * i as f64 / (n - 1) as f64, y: 46.0 }).collect();
        let arc: Vec<f64> = (0..n).map(|i| len_m * i as f64 / (n - 1) as f64).collect();
        let spans = vec![
            Span { arc0: 0.0, arc1: 200.0, level: 0, kind: SpanKind::Grade },
            Span { arc0: 200.0, arc1: 800.0, level: 1, kind: SpanKind::Bridge },
            Span { arc0: 800.0, arc1: 1000.0, level: 0, kind: SpanKind::Grade },
        ];
        // Plateaus at 100 m, a valley 40 m deep under the span, and a bump at
        // mid-span rising to 105 m — 5 m above the ~100 m deck line.
        let terrain = |c: Coord| {
            let x = (c.x - 6.0) / deg * len_m; // corridor metres
            let bump = (1.0 - ((x - 500.0) / 40.0).abs()).max(0.0) * 45.0;
            if !(200.0..=800.0).contains(&x) { 100.0 } else { (60.0 + bump).min(105.0) }
        };
        let scene = SceneGraph::new(vec![Corridor {
            id: 0,
            nodes: nodes.clone(),
            arc,
            cos_lat,
            kind: Kind::Road(RoadClass::Motorway),
            class_key: "motorway".into(),
            link: false,
            width_m: Some(5.5),
            spans: spans.clone(),
            segments: vec![SegmentRef { source: 1, node0: 0, node1: n - 1, properties: vec![] }],
            connectors: vec![],
        }]);
        let profiles = vec![crate::solve::profile::solve(
            &nodes,
            &spans,
            crate::solve::Mode::Engineered { grade: 0.06 },
            &mut |c| terrain(c),
        )];
        let solved = crate::solve::SolvedModel::from_profiles(profiles, 14);
        let ground = derive(&scene, &solved, &Facades::empty(), &[], None, 1);

        let mut scratch = Vec::new();
        let at = |x_m: f64| Coord { x: 6.0 + deg * x_m / len_m, y: 46.0 };
        // On the bump crest the ground is cut below the ~100 m deck.
        let crest = at(500.0);
        let cut = ground.height(crest.x, crest.y, terrain(crest), 0.0, &mut scratch);
        assert!(cut < 99.0, "the bump must be carved below the deck, got {cut}");
        assert!(cut > 90.0, "the notch is a daylight cut, not a canyon, got {cut}");
        // The valley floor under the deck is untouched.
        let floor = at(350.0);
        assert_eq!(ground.height(floor.x, floor.y, terrain(floor), 0.0, &mut scratch), terrain(floor));
        // The at-grade approach is untouched by the carve.
        let approach = at(100.0);
        let h = ground.height(approach.x, approach.y, terrain(approach), 0.0, &mut scratch);
        assert!((h - 100.0).abs() < 0.5, "approach ground stays natural, got {h}");
    }

    /// The S4 scene end-to-end through solve + derive: a flat-ground overpass
    /// leaves embankment approaches in the engineered ground.
    #[test]
    fn overpass_approaches_become_embankments() {
        let cos_lat = 46.0_f64.to_radians().cos();
        let len_m = 1000.0;
        let deg = len_m / (DEG_M * cos_lat);
        let n = 41;
        let nodes: Vec<Coord> =
            (0..n).map(|i| Coord { x: 6.0 + deg * i as f64 / (n - 1) as f64, y: 46.0 }).collect();
        let arc: Vec<f64> = (0..n).map(|i| len_m * i as f64 / (n - 1) as f64).collect();
        let spans = vec![
            Span { arc0: 0.0, arc1: 450.0, level: 0, kind: SpanKind::Grade },
            Span { arc0: 450.0, arc1: 550.0, level: 1, kind: SpanKind::Bridge },
            Span { arc0: 550.0, arc1: 1000.0, level: 0, kind: SpanKind::Grade },
        ];
        let scene = SceneGraph::new(vec![Corridor {
            id: 0,
            nodes: nodes.clone(),
            arc,
            cos_lat,
            kind: Kind::Road(RoadClass::Secondary),
            class_key: "secondary".into(),
            link: false,
            width_m: Some(5.5),
            spans: spans.clone(),
            segments: vec![SegmentRef { source: 1, node0: 0, node1: n - 1, properties: vec![] }],
            connectors: vec![],
        }]);
        let mid = Coord { x: 6.0 + deg * 0.5, y: 46.0 };
        let crossings = vec![Crossing {
            upper: 0,
            upper_arc: 500.0,
            point: mid,
            lower: None,
            lower_arc: 0.0,
            lower_kind: Kind::Road(RoadClass::Residential),
            upper_level: 1,
            lower_level: 0,
        }];

        let mut profiles = vec![crate::solve::profile::solve(
            &nodes,
            &spans,
            crate::solve::Mode::for_kind(Kind::Road(RoadClass::Secondary)),
            &mut |_| 372.0,
        )];
        // The clearance lift comes from the fused graph — the same path the
        // pipeline runs, so what this test asserts about the ground is what the
        // shipped solve actually produces.
        {
            let mut g = crate::solve::graph::build(&scene, &profiles, &crossings, Stratum::S, &[], &[]);
            crate::solve::relax::solve(&mut g);
            crate::solve::relax::reconstruct(&g, &mut profiles);
        }
        let solved = crate::solve::SolvedModel::from_profiles(profiles, 14);
        let ground = derive(&scene, &solved, &Facades::empty(), &[], None, 1);
        assert!(ground.earthwork_count() > 0, "the lifted approaches must become earthworks");

        // On the approach centerline (~80 m before the crossing, 30 m before
        // the span edge) the engineered ground rises to the solved road; far
        // away it is natural.
        let mut scratch = Vec::new();
        let approach = Coord { x: mid.x - 80.0 / (DEG_M * cos_lat), y: 46.0 };
        let road_there = solved.profile(0).unwrap().height_at(approach.x, approach.y);
        let h = ground.height(approach.x, approach.y, 372.0, 0.0, &mut scratch);
        assert!(
            (h - road_there).abs() < 1e-6,
            "engineered ground {h} must meet the road {road_there}"
        );
        assert!(h > 372.5, "the approach is a real embankment, got {h}");
        let far = Coord { x: 6.0 + deg * 0.02, y: 46.0 };
        assert_eq!(ground.height(far.x, far.y, 372.0, 0.0, &mut scratch), 372.0);
        // Under the bridge span itself the natural ground is untouched — the
        // deck stands on air, not on a berm.
        assert_eq!(ground.height(mid.x, mid.y, 372.0, 0.0, &mut scratch), 372.0);
    }

    /// A drivable Minor street corridor over `pts` with its Street-mode
    /// profile solved against `sample` — the universal-profile replacement
    /// for the old bed fixtures.
    fn street(
        pts: &[Coord],
        sample: &mut dyn FnMut(Coord) -> f64,
    ) -> (Corridor, crate::solve::Profile) {
        let cos_lat = crate::scene::run_cos_lat(pts);
        let mut arc = vec![0.0];
        for w in pts.windows(2) {
            arc.push(arc.last().unwrap() + crate::scene::metric_len(w[0], w[1], cos_lat));
        }
        let c = Corridor {
            id: 0,
            nodes: pts.to_vec(),
            arc,
            cos_lat,
            kind: Kind::Road(RoadClass::Residential),
            class_key: "residential".into(),
            link: false,
            width_m: Some(5.5),
            spans: vec![],
            segments: vec![],
            connectors: vec![],
        };
        let mode = crate::solve::Mode::for_kind(Kind::Road(RoadClass::Residential));
        let p = crate::solve::profile::solve(pts, &[], mode, sample).expect("a street profile");
        (c, p)
    }

    /// A straight west→east street at lat 46, 10 m nodes, on ground that
    /// climbs `slope` to the north — so the bench cuts on one side and fills
    /// on the other and both faces have somewhere to run.
    fn cross_slope_street(
        n: usize,
        slope: f64,
    ) -> (Corridor, crate::solve::Profile, impl Fn(Coord) -> f64) {
        let cos_lat = 46.0_f64.to_radians().cos();
        let pts: Vec<Coord> = (0..n)
            .map(|i| Coord { x: 6.0 + (i as f64 * 10.0) / (DEG_M * cos_lat), y: 46.0 })
            .collect();
        let ground = move |c: Coord| 400.0 + (c.y - 46.0) * DEG_M * slope;
        let (c, p) = street(&pts, &mut |c| ground(c));
        (c, p, ground)
    }

    /// The clip is off by default (see `corridor_earthworks`), so a test that
    /// wants it must say so. Serialized, because the environment is global.
    fn with_clip<R>(batter: bool, f: impl FnOnce() -> R) -> R {
        static LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());
        let _g = LOCK.lock().unwrap_or_else(|e| e.into_inner());
        std::env::set_var("ARPT_FACADE_BENCH", "1");
        if batter {
            std::env::set_var("ARPT_FACADE_BATTER", "1");
        }
        let out = f();
        std::env::remove_var("ARPT_FACADE_BENCH");
        std::env::remove_var("ARPT_FACADE_BATTER");
        out
    }

    /// A wall parallel to that street, `north_m` north of it.
    fn wall_north(north_m: f64) -> Facades {
        let y = 46.0 + north_m / DEG_M;
        Facades::from_edges([[Coord { x: 5.99, y }, Coord { x: 6.01, y }]])
    }

    /// A road's ground stops *at* the wall and does not pass it. With a wall
    /// 3 m north of the centerline the bench reaches exactly 3 m that way and
    /// its full nominal width to the south, where nothing stands. Not a
    /// clearance short of the wall: `FACADE_CLEAR_M` keeps a drawn *surface*
    /// off a footprint, and a wall stands on ground.
    #[test]
    fn a_facade_clips_the_bench_on_its_own_side_only() {
        let (c, p, ground) = cross_slope_street(12, 0.2);
        let nominal = Kind::Road(RoadClass::Residential).prior().half_width_m(false).unwrap()
            + EARTHWORK_SHOULDER_M
            + EARTHWORK_MARGIN_M;
        let mut side = |c: Coord| ground(c);
        let edges =
            with_clip(false, || corridor_earthworks(&c, &p, Some(&wall_north(3.0)), Some(&mut side)));
        assert!(!edges.is_empty(), "the street must bench");
        for e in edges.iter().filter(|e| !e.carve) {
            // The corridor runs west→east, so north is its left.
            assert!(
                (e.half_width_m[LEFT] - 3.0).abs() < 1e-6,
                "left bench {} does not stop at the 3 m wall",
                e.half_width_m[LEFT]
            );
            assert_eq!(e.half_width_m[RIGHT], nominal, "the open side keeps its bench");
        }
    }

    /// Half of `authority.facade_ground`'s population is the *face* beyond the
    /// bench, so the batter is clipped to the same room: nothing this edge
    /// draws may reach the wall.
    #[test]
    fn a_facade_stops_the_batter_face_as_well_as_the_bench() {
        let (c, p, ground) = cross_slope_street(12, 0.5);
        let nominal = Kind::Road(RoadClass::Residential).prior().half_width_m(false).unwrap()
            + EARTHWORK_SHOULDER_M
            + EARTHWORK_MARGIN_M;
        let mut side = |c: Coord| ground(c);
        let open = corridor_earthworks(&c, &p, None, Some(&mut side));
        // The wall stands *outside* the bench (4.25 m) but inside the face's
        // reach, so the bench itself fits and only the batter is cut short —
        // the half of the population a bench-only fix would have made worse.
        let clipped =
            with_clip(true, || corridor_earthworks(&c, &p, Some(&wall_north(4.6)), Some(&mut side)));
        let reach = |es: &[EarthworkEdge], s: usize| {
            es.iter()
                .filter(|e| !e.carve)
                .map(|e| e.half_width_m[s] + e.batter_m[s])
                .fold(0.0, f64::max)
        };
        assert!(
            reach(&open, LEFT) > 4.6,
            "the fixture must have a face worth clipping, got {}",
            reach(&open, LEFT)
        );
        for e in clipped.iter().filter(|e| !e.carve) {
            assert_eq!(e.half_width_m[LEFT], nominal, "the bench itself still fits");
        }
        assert!(
            reach(&clipped, LEFT) <= 4.6 + 1e-6,
            "nothing this edge draws may pass the 4.6 m wall, got {}",
            reach(&clipped, LEFT)
        );
        assert_eq!(
            reach(&open, RIGHT),
            reach(&clipped, RIGHT),
            "the side with no wall is untouched"
        );
    }

    /// The bench may give up its verge and no more: below the band it carries
    /// the drawn asphalt would hang over unbenched ground, which is what
    /// `carriageway_m` exists to prevent.
    #[test]
    fn a_clipped_bench_never_goes_below_the_band_it_carries() {
        let (c, p, ground) = cross_slope_street(12, 0.2);
        let mut side = |c: Coord| ground(c);
        // A wall almost on the centerline: the room is gone entirely.
        let edges =
            with_clip(false, || corridor_earthworks(&c, &p, Some(&wall_north(0.2)), Some(&mut side)));
        let band = 5.5 * 0.5 + crate::priors::STRUCTURE_SHOULDER_M;
        let floor = crate::priors::MIN_CARRIAGEWAY_HALF_M.min(band);
        for e in edges.iter().filter(|e| !e.carve) {
            assert!(
                e.half_width_m[LEFT] >= floor - 1e-9,
                "bench {} fell below the band's own floor {floor}",
                e.half_width_m[LEFT]
            );
        }
    }

    /// A railway is out of this, for the reason `order.building_overlap`
    /// leaves it out: a station roof over its platforms is a level relation,
    /// and narrowing the formation there shaves the platform.
    #[test]
    fn a_facade_does_not_clip_a_rail_formation() {
        let (mut c, p, ground) = cross_slope_street(12, 0.2);
        c.kind = Kind::Rail(crate::priors::RailClass::StandardGauge);
        let mut side = |c: Coord| ground(c);
        let open = corridor_earthworks(&c, &p, None, Some(&mut side));
        let clipped =
            with_clip(true, || corridor_earthworks(&c, &p, Some(&wall_north(2.0)), Some(&mut side)));
        assert_eq!(open.len(), clipped.len());
        for (a, b) in open.iter().zip(clipped.iter()) {
            assert_eq!(a.half_width_m, b.half_width_m, "a platform is not a defect");
            assert_eq!(a.batter_m, b.batter_m);
        }
    }

    /// A bench edge must carry the height of the place it actually stands, not
    /// of the raw node it was indexed by. Smoothing shortens a curved
    /// centerline, so `smooth[k]` lags `nodes[k]` along the alignment — and on
    /// a steep climb that lag is a height error big enough to bury the very
    /// alignment the bench is for (the Territet funicular, 11.6 m of lag at
    /// 59 %, 6.9 m of ground over its own track).
    #[test]
    fn a_bench_carries_the_height_of_the_point_it_stands_at() {
        let cos_lat = 46.0_f64.to_radians().cos();
        // A curved, steeply climbing alignment: bends give smoothing something
        // to shorten, and the grade turns the lag into metres.
        let n = 60;
        let pts: Vec<Coord> = (0..n)
            .map(|i| {
                let t = i as f64 / (n - 1) as f64;
                Coord {
                    x: 6.0 + (200.0 * t + 25.0 * (t * 6.0).sin()) / (DEG_M * cos_lat),
                    y: 46.0 + (60.0 * t) / DEG_M,
                }
            })
            .collect();
        // Ground climbing 50 % along the alignment, so the profile rides it.
        let ground = |c: Coord| {
            400.0 + ((c.x - 6.0) * DEG_M * cos_lat + (c.y - 46.0) * DEG_M) * 0.5
        };
        let (mut c, p) = street(&pts, &mut |c| ground(c));
        c.kind = Kind::Rail(crate::priors::RailClass::Funicular);
        let mut side = |c: Coord| ground(c);
        let edges = corridor_earthworks(&c, &p, None, Some(&mut side));
        assert!(!edges.is_empty(), "a climbing alignment must bench");
        // Every bench target must match the profile at the arc its own
        // endpoint occupies. Paired with the raw node instead, these differ by
        // the lag times the grade.
        for e in &edges {
            let want = p.road_at_arc(p.arc_of(e.a.x, e.a.y));
            assert!(
                (e.target_a - want).abs() < 0.5,
                "bench at ({:.6},{:.6}) targets {:.2} where the profile is {:.2}",
                e.a.x, e.a.y, e.target_a, want,
            );
        }
    }

    /// A street across a side-slope: its bench holds the solved profile flat
    /// across the carriageway (D3) — the cross-slope side sampling finds the
    /// earthwork even though the centerline sits exactly on grade. A flank too
    /// steep for a terrace to be plausible gets no bench at all.
    #[test]
    fn a_street_bench_is_flat_across_a_side_slope() {
        let cos_lat = 46.0_f64.to_radians().cos();
        // A 100 m west-east street on ground rising 0.5 m per metre northward.
        let pts = vec![
            Coord { x: 6.0, y: 46.0 },
            Coord { x: 6.0 + 100.0 / (DEG_M * cos_lat), y: 46.0 },
        ];
        let slope = |c: Coord| 400.0 + (c.y - 46.0) * DEG_M * 0.5;
        let (c, p) = street(&pts, &mut |c| slope(c));
        let mut side = |c: Coord| slope(c);
        let edges = corridor_earthworks(&c, &p, None, Some(&mut side));
        assert!(!edges.is_empty(), "the cross-slope must trigger a bench");
        assert!(edges.iter().all(|e| (e.target_a - 400.0).abs() < 1e-9));

        let ew = Earthworks::new(edges);
        let mut scratch = Vec::new();
        let mid_x = 6.0 + 50.0 / (DEG_M * cos_lat);
        // 3 m uphill of the centerline the natural ground is higher, but the
        // bench holds the centerline height: flat across.
        let uphill = 46.0 + 3.0 / DEG_M;
        let h = ew.height(mid_x, uphill, slope(Coord { x: mid_x, y: uphill }), 0.0, &mut scratch);
        assert!((h - 400.0).abs() < 1e-9, "bench must hold flat across, got {h}");
        // The drape reads the same answer through target_at…
        assert_eq!(ew.target_at(mid_x, uphill, &mut scratch), Some(400.0));
        // …but only inside the held width; the batter is not the bench.
        let past = 46.0 + 10.0 / DEG_M;
        assert_eq!(ew.target_at(mid_x, past, &mut scratch), None);
        // Far off the street the slope is untouched.
        let far = 46.0 + 40.0 / DEG_M;
        let raw = slope(Coord { x: mid_x, y: far });
        assert_eq!(ew.height(mid_x, far, raw, 0.0, &mut scratch), raw);

        // A flank of 2 m per metre would need a face taller than
        // MAX_BENCH_FACE_M to hold the same band flat: no bench is emitted and
        // the street is left on the hillside.
        let cliff = |c: Coord| 400.0 + (c.y - 46.0) * DEG_M * 2.0;
        let (c, p) = street(&pts, &mut |c| cliff(c));
        let mut side = |c: Coord| cliff(c);
        assert!(
            corridor_earthworks(&c, &p, None, Some(&mut side)).is_empty(),
            "a terrace on a cliff is a fiction: no bench there"
        );
    }

    /// A street along rough steep ground: the profile irons the DEM's
    /// terraces to its class grade while never leaving the deviation budget —
    /// a road that climbs plausibly instead of staircasing (S9 both ways).
    #[test]
    fn a_steep_rough_street_is_grade_limited() {
        let cos_lat = 46.0_f64.to_radians().cos();
        let len_m = 600.0;
        let deg = len_m / (DEG_M * cos_lat);
        let pts = vec![Coord { x: 6.0, y: 46.0 }, Coord { x: 6.0 + deg, y: 46.0 }];
        // A 10 % mean climb with ±2 m terrace noise every ~60 m — steeper than
        // the minor grade cap in the noisy stretches.
        let rough = |c: Coord| {
            let x = (c.x - 6.0) / deg * len_m;
            400.0 + 0.10 * x + 2.0 * (x / 60.0 * std::f64::consts::PI).sin()
        };
        let (_, p) = street(&pts, &mut |c| rough(c));
        let (arc, road) = (p.arc(), p.road_m());
        let max_grade = Kind::Road(RoadClass::Residential).prior().grade().unwrap();
        for i in 1..road.len() {
            let run = arc[i] - arc[i - 1];
            let pitch = (road[i] - road[i - 1]).abs() / run;
            assert!(pitch <= max_grade + 1e-9, "street pitch {pitch} exceeds the grade cap");
        }
        // Deviation is budgeted against the conditioned reference (the road
        // may ride over the DEM's dips and through its false crests), and it
        // never digs below the natural ground by more than the budget.
        let natural = p.terrain_m();
        let reference = crate::solve::profile::condition_reference(arc, natural);
        let budget = Kind::Road(RoadClass::Residential).prior().deviation_m;
        for i in 0..road.len() {
            let dev = (road[i] - reference[i]).abs();
            assert!(dev <= budget + 1e-9, "street leaves the reference by {dev} m");
            assert!(
                road[i] >= natural[i] - budget - 1e-9,
                "the street must never dive below the ground budget"
            );
        }
        // The street still climbs the hill: the ends stay ~60 m apart.
        let climb = road.last().unwrap() - road[0];
        assert!(climb > 50.0, "the street must climb with its hill, got {climb}");
    }

    /// A street across a narrow gully — the DEM images the stream cut, the
    /// road crosses it on fill and a culvert: the profile holds rim height
    /// across instead of diving in and out ("the road falls into holes").
    #[test]
    fn a_street_spans_a_narrow_gully() {
        let cos_lat = 46.0_f64.to_radians().cos();
        let len_m = 300.0;
        let deg = len_m / (DEG_M * cos_lat);
        let pts = vec![Coord { x: 6.0, y: 46.0 }, Coord { x: 6.0 + deg, y: 46.0 }];
        // Flat ground at 500 m with a 60 m-wide, 8 m-deep V-notch mid-street.
        let gully = |c: Coord| {
            let x = (c.x - 6.0) / deg * len_m;
            500.0 - (1.0 - ((x - 150.0) / 30.0).abs()).max(0.0) * 8.0
        };
        let (_, p) = street(&pts, &mut |c| gully(c));
        // The DEM genuinely dips at the profile's nodes…
        assert!(
            p.terrain_m().iter().any(|&t| t < 496.0),
            "the fixture must sample inside the gully"
        );
        // …but the road carries across at rim height.
        for (i, &h) in p.road_m().iter().enumerate() {
            assert!(
                (h - 500.0).abs() < 0.75,
                "the street must span the gully at rim height, got {h} at arc {}",
                p.arc()[i]
            );
        }
        // A gorge deeper than the fill cap is a genuine descent: the street
        // keeps the terrain (no 40 m embankment wall from thin air).
        let gorge = |c: Coord| {
            let x = (c.x - 6.0) / deg * len_m;
            500.0 - (1.0 - ((x - 150.0) / 40.0).abs()).max(0.0) * 40.0
        };
        let (_, p) = street(&pts, &mut |c| gorge(c));
        let mid = p.arc().iter().position(|&a| (a - 150.0).abs() < 16.0).expect("a mid node");
        assert!(
            p.road_m()[mid] < 495.0,
            "a gorge past the fill cap keeps the terrain, got {}",
            p.road_m()[mid]
        );
    }

    /// The batter reaches only as far as it needs to daylight: on a gentle
    /// flank it meets the natural ground at the predicted distance, on a flank
    /// steeper than the batter it collapses to its floor — the bench is
    /// retained by a wall at its edge instead of terracing out into the
    /// hillside — and on flat ground the bench moves nothing at all.
    #[test]
    fn the_batter_reaches_only_as_far_as_it_daylights() {
        let cos_lat = 46.0_f64.to_radians().cos();
        let pts = vec![
            Coord { x: 6.0, y: 46.0 },
            Coord { x: 6.0 + 100.0 / (DEG_M * cos_lat), y: 46.0 },
        ];
        let bench_half =
            Kind::Road(RoadClass::Residential).prior().half_width_m(false).unwrap() + EARTHWORK_SHOULDER_M + EARTHWORK_MARGIN_M;

        // A 0.15 m/m side-slope: gentle enough that both faces still close on
        // the ground, at |face| / (1/batter − 0.15).
        let gentle = |c: Coord| 400.0 + (c.y - 46.0) * DEG_M * 0.15;
        let (c, p) = street(&pts, &mut |c| gentle(c));
        let mut side = |c: Coord| gentle(c);
        let edges = corridor_earthworks(&c, &p, None, Some(&mut side));
        assert!(!edges.is_empty());
        let want = (0.15 * bench_half) / (1.0 / EARTHWORK_BATTER - 0.15);
        for e in &edges {
            for reach in e.batter_m {
                assert!(
                    (reach - want).abs() < 1e-6,
                    "batter reach {reach} must daylight at {want}"
                );
            }
        }

        // A 0.3 m/m side-slope. The uphill face would still meet the ground
        // eventually, but only by running far out across the flank — an earth
        // batter is diverging there, so it is rebuilt as a wall: steep, short,
        // and closing by the same daylight test rather than by a cap.
        let leaning = |c: Coord| 400.0 + (c.y - 46.0) * DEG_M * 0.3;
        let (c, p) = street(&pts, &mut |c| leaning(c));
        let mut side = |c: Coord| leaning(c);
        let edges = corridor_earthworks(&c, &p, None, Some(&mut side));
        assert!(!edges.is_empty());
        for e in &edges {
            assert_eq!(e.batter_run[LEFT], WALL_BATTER, "the uphill face must be a wall");
            assert!(
                e.batter_m[LEFT] < 1.0,
                "a wall closes inside a metre, got {}",
                e.batter_m[LEFT]
            );
        }

        let ew = Earthworks::new(edges);
        let mut scratch = Vec::new();
        let mid_x = 6.0 + 50.0 / (DEG_M * cos_lat);
        // Past the wall the hillside is untouched — the face is self-limiting,
        // so it reshapes what it must and stops, with no bevel holding the
        // ground down beyond it and no step where such a bevel would have
        // ended.
        let out = 46.0 + (bench_half + 1.0) / DEG_M;
        let raw = leaning(Coord { x: mid_x, y: out });
        assert_eq!(ew.height(mid_x, out, raw, 0.0, &mut scratch), raw);
        // …while the bench itself still holds the road flat across.
        let on_bench = 46.0 + (bench_half - 0.5) / DEG_M;
        let raw = leaning(Coord { x: mid_x, y: on_bench });
        assert!((ew.height(mid_x, on_bench, raw, 0.0, &mut scratch) - 400.0).abs() < 1e-9);

        // Flat ground: the bench is still there (the band is flat and defends
        // itself) but it moves nothing — its batter is the floor and its
        // target is the ground.
        let (c, p) = street(&pts, &mut |_| 400.0);
        let mut side = |_: Coord| 400.0;
        let edges = corridor_earthworks(&c, &p, None, Some(&mut side));
        assert!(!edges.is_empty(), "every at-grade road benches its own band");
        assert!(edges.iter().all(|e| e.batter_m == [EARTHWORK_MIN_BATTER_M; 2]));
        let ew = Earthworks::new(edges);
        assert!((ew.height(mid_x, 46.0, 400.0, 0.0, &mut scratch) - 400.0).abs() < 1e-9);
        let far = 46.0 + 40.0 / DEG_M;
        assert_eq!(ew.height(mid_x, far, 400.0, 0.0, &mut scratch), 400.0);
    }

    #[test]
    fn water_level_takes_a_low_shoreline_percentile() {
        // A shoreline that mostly images 372 m with a few bank vertices at
        // 378 m: the level must land on the waterline, not the bank.
        let ring: Vec<Coord> =
            (0..20).map(|i| Coord { x: 6.0 + i as f64 * 0.001, y: 46.0 }).collect();
        let level = water_level(&ring, |c| {
            if ((c.x * 1000.0).round() as i64) % 5 == 0 { 378.0 } else { 372.0 }
        })
        .expect("a level from a 20-vertex ring");
        assert!((level - 372.0).abs() < 1e-9, "level {level} must be the waterline, not the bank");
    }

    #[test]
    fn ground_flattens_water_and_a_shore_earthwork_overrides_it() {
        let mut scratch = Vec::new();
        let cos_lat = 46.0_f64.to_radians().cos();
        // A square lake [6.00, 6.01] × [46.00, 46.01], flattened to 372 m.
        let exterior = vec![
            Coord { x: 6.00, y: 46.00 },
            Coord { x: 6.01, y: 46.00 },
            Coord { x: 6.01, y: 46.01 },
            Coord { x: 6.00, y: 46.01 },
            Coord { x: 6.00, y: 46.00 },
        ];
        let waters = Waters::new(vec![WaterFill {
            exterior,
            holes: vec![],
            bbox: (6.00, 46.00, 6.01, 46.01),
            level: 372.0,
        }]);
        // A shore berm (a bridge approach earthwork) crossing the lake middle,
        // target 375 m — it must win over the water where they overlap.
        let earthworks = Earthworks::new(vec![EarthworkEdge {
            a: Coord { x: 6.00, y: 46.005 },
            b: Coord { x: 6.01, y: 46.005 },
            target_a: 375.0,
            target_b: 375.0,
            half_width_m: [8.0; 2],
            carriageway_m: 6.0,
            batter_m: [4.0; 2],
            batter_run: [EARTHWORK_BATTER; 2],
            chain: 0,
            arc0: 0.0,
            cos_lat,
            carve: false,
            headwall: false,
            crest: true,
        }]);
        let g = GroundStack::new(vec![GroundLayer { stratum: Stratum::S, earthworks, waters }]);
        // Open water away from the berm: flattened to the level (over raw 360).
        assert_eq!(g.height(6.002, 46.002, 360.0, 0.0, &mut scratch), 372.0);
        // On the berm centerline inside the lake: the road overrides the water.
        assert!((g.height(6.005, 46.005, 360.0, 0.0, &mut scratch) - 375.0).abs() < 1e-9);
        // Outside the lake: the raw DEM passes through.
        assert_eq!(g.height(6.02, 46.02, 360.0, 0.0, &mut scratch), 360.0);
    }

    /// A band segment fixture: `a`→`b` running east, `half_m` wide, with a
    /// host (a sidewalk seated at `height`) or without one (a path).
    fn band(
        from_m: f64,
        to_m: f64,
        half_m: f64,
        corridor: crate::scene::CorridorId,
        height: (f64, f64),
    ) -> crate::synth::carriageway::SourceSeg {
        let cos_lat = 46.0_f64.to_radians().cos();
        let east = |m: f64| 6.0 + m / (DEG_M * cos_lat);
        // Material and kerb follow the host, exactly as `synth::walkway` emits
        // them: a band with no corridor is a path standing on the ground, and
        // one with a host is a sidewalk a kerb above its street. The fixture
        // used to call every band a `Walkway` whatever its corridor, which was
        // invisible while the face cap was keyed on the corridor and wrong the
        // moment it was keyed on the material it is a statement about.
        let path = corridor == crate::scene::CorridorId::MAX;
        crate::synth::carriageway::SourceSeg {
            a: Coord { x: east(from_m), y: 46.0 },
            b: Coord { x: east(to_m), y: 46.0 },
            cos_lat,
            half_m,
            sect_a: Section::uniform(half_m),
            sect_b: Section::uniform(half_m),
            level: 0,
            layer: 0,
            cut_a: None,
            cut_b: None,
            height_a: height.0,
            height_b: height.1,
            corridor,
            surface: if path { crate::priors::Surface::Path } else { crate::priors::Surface::Walkway },
            rise_m: if path { 0.0 } else { crate::priors::KERB_RISE_M },
            arc0: from_m,
        }
    }

    /// A path stands on the ground *along its own length*: the bench ramps
    /// between the heights sampled at its two ends.
    ///
    /// Holding one height for the whole segment — the midpoint's — is what
    /// turned a footpath climbing a flank into a staircase of terraces, each
    /// flat at its own middle and stepping to the next, and the lateral face
    /// cap could not see it because at the midpoint there is nothing to see.
    #[test]
    fn a_path_bench_ramps_between_its_own_ends() {
        let seg = band(0.0, 8.0, 1.0, CorridorId::MAX, (0.0, 0.0));
        // A hillside climbing east at 25 %: gentle enough across the band that
        // the bench is plausible, steep enough along it to matter.
        let cos_lat = seg.cos_lat;
        let mut ground = |q: Coord| (q.x - 6.0) * DEG_M * cos_lat * 0.25;
        let e = walk_edge(&seg, 0, &WalkBenchRules::shipped(), &mut ground).expect("a plausible bench");
        assert!((e.target_a - 0.0).abs() < 1e-6, "the west end: {}", e.target_a);
        assert!((e.target_b - 2.0).abs() < 1e-6, "the east end: {}", e.target_b);
        // The next segment starts where this one ends, so the two agree there
        // by construction and no step is left between them.
        let next = band(8.0, 16.0, 1.0, CorridorId::MAX, (0.0, 0.0));
        let n = walk_edge(&next, 0, &WalkBenchRules::shipped(), &mut ground).expect("a plausible bench");
        assert!((n.target_a - e.target_b).abs() < 1e-6, "{} vs {}", n.target_a, e.target_b);
    }

    /// The bench is the band plus the verge every bench carries, and it holds
    /// the band itself outright against any neighbour.
    #[test]
    fn a_walkway_bench_is_the_band_plus_a_verge_and_draws_no_crest() {
        let seg = band(0.0, 8.0, 1.0, CorridorId::MAX, (0.0, 0.0));
        let mut flat = |_: Coord| 0.0;
        let e = walk_edge(&seg, 7, &WalkBenchRules::shipped(), &mut flat).expect("flat ground always benches");
        assert_eq!(e.half_width_m, [1.0 + EARTHWORK_MARGIN_M; 2]);
        assert_eq!(e.carriageway_m, 1.0);
        assert_eq!(e.chain, 7);
        assert!(!e.crest, "the band's own ring is the constraint");
        assert!(!e.carve, "a walkway bench fills as well as cuts");
    }

    /// Past the cap the terrace is a fiction and the path is left on the
    /// hillside — the same ladder a corridor bench holds, at a footpath's size.
    #[test]
    fn a_path_across_too_steep_a_flank_gets_no_bench() {
        let seg = band(0.0, 8.0, 1.0, CorridorId::MAX, (0.0, 0.0));
        let cos_lat = seg.cos_lat;
        // A flank falling north at 100 %: the bench edge is 1.5 m out, so
        // holding the band flat would cut and fill 1.5 m — past WALK_MAX_FACE_M.
        let mut flank = |q: Coord| (q.y - 46.0) * DEG_M;
        assert!(walk_edge(&seg, 0, &WalkBenchRules::shipped(), &mut flank).is_none());
        // At a third of that slope the same band benches: half a metre of face.
        let mut gentle = |q: Coord| (q.y - 46.0) * DEG_M * 0.33;
        assert!(walk_edge(&seg, 0, &WalkBenchRules::shipped(), &mut gentle).is_some());
    }

    /// A sidewalk is seated where its street's cross-section puts it, and may
    /// stand as far above the hillside as the street's own bench does: the wall
    /// under it is the street's, and the band's apron draws it.
    #[test]
    fn a_sidewalk_bench_takes_its_seat_from_its_host_not_from_the_ground() {
        let seg = band(0.0, 8.0, 1.0, 3, (410.0, 411.0));
        let mut ground = |_: Coord| 408.0;
        let e = walk_edge(&seg, 0, &WalkBenchRules::shipped(), &mut ground).expect("the street's own allowance");
        assert_eq!((e.target_a, e.target_b), (410.0, 411.0));
        // Two metres above the hillside beside it — past a path's cap, inside
        // the street's.
        let mut deeper = |_: Coord| 405.0;
        assert!(walk_edge(&seg, 0, &WalkBenchRules::shipped(), &mut deeper).is_none(), "past the street's cap too");
    }
}
