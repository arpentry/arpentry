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

use crate::dem::Dem;
use crate::priors::{
    DECK_THICKNESS_M, EARTHWORK_BATTER,
    BATTER_DIVERGENCE_SLOP, EARTHWORK_MARGIN_M, EARTHWORK_MAX_BATTER_M, MAX_BENCH_FACE_M,
    EARTHWORK_MIN_BATTER_M, EARTHWORK_SHOULDER_M, WALL_BATTER,
    MAX_CLEARANCE_LIFT_M,
    PORTAL_CLEARANCE_M, PORTAL_CUT_LEN_M, WATER_LEVEL_PCTL,
};
use crate::priors::Stratum;
use crate::scene::{SceneGraph, SpanKind, DEG_M};
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

    /// The bench target here, if this layer holds one — the crest derivation's
    /// question (docs/GROUND.md §3).
    fn target_at(&self, lon: f64, lat: f64, scratch: &mut Vec<u32>) -> Option<f64> {
        self.earthworks.target_at(lon, lat, scratch)
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
    terrain_path: Option<&Path>,
    threads: usize,
) -> GroundStack {
    // H first: water is gravity-defined and no earthwork changes it, so it is
    // the ground everything else is cut into.
    let waters = derive_waters(scene, solved, terrain_path, threads);
    let mut layers: Vec<GroundLayer> = Vec::new();
    if !waters.is_empty() {
        layers.push(GroundLayer { stratum: Stratum::H, earthworks: Earthworks::new(Vec::new()), waters });
    }
    for stratum in [Stratum::R, Stratum::S, Stratum::D] {
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
        let edges = derive_earthworks(scene, solved, &members, &layers, terrain_path, threads);
        layers.push(GroundLayer {
            stratum,
            earthworks: Earthworks::new(edges),
            waters: Waters::new(Vec::new()),
        });
    }
    GroundStack::new(layers)
}

/// Every profiled corridor's earthworks, derived in parallel (the bench-edge
/// side sampling is DEM-decode bound, like the shoreline reads) and
/// flattened in corridor order, so the edge indices — and the modifier
/// tie-breaking they feed — are deterministic run to run (invariant 5).
fn derive_earthworks(
    scene: &SceneGraph,
    solved: &SolvedModel,
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
            .filter_map(|c| solved.profile(c.id).map(|p| corridor_earthworks(c, p, None)))
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
                    let edges = corridor_earthworks(c, p, Some(&mut sample));
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
        // paints (`synth::junction::corridor_half_width_m`) *plus the verge*,
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

        // Per node and per side, how far the batter runs before it daylights.
        // The bench-edge sample gives both the face depth there and the
        // cross-slope the natural ground carries outward, which is what
        // decides whether a face of [`EARTHWORK_BATTER`] ever meets it.
        // Without a sampler (no DEM) the centerline depth stands in on both
        // sides.
        let centre_reach = |k: usize| batter_reach(road[k] - terrain[k], 0.0);
        let mut batter: Vec<[(f64, f64); 2]> =
            (0..nodes.len()).map(|k| [centre_reach(k), centre_reach(k)]).collect();
        // Whether a bench is plausible at all here — see [`MAX_BENCH_FACE_M`].
        let mut benched: Vec<bool> = vec![true; nodes.len()];
        if let Some(sample) = side {
            for k in 0..nodes.len() {
                if !at_grade[k] {
                    continue;
                }
                let (ux, uy) = heading(nodes, k, cos_lat);
                let (px, py) = (-uy, ux); // lateral unit, metric (left)
                let mut face = |s: f64| -> (f64, (f64, f64)) {
                    let q = Coord {
                        x: nodes[k].x + s * px * bench_half_width / (DEG_M * cos_lat),
                        y: nodes[k].y + s * py * bench_half_width / DEG_M,
                    };
                    let edge_raw = sample(q);
                    // The face the bench cuts or fills at its edge, and the
                    // outward slope of the natural ground from the centerline
                    // (positive uphill).
                    let rise = road[k] - edge_raw;
                    (rise, batter_reach(rise, (edge_raw - terrain[k]) / bench_half_width))
                };
                let (rise_l, reach_l) = face(1.0);
                let (rise_r, reach_r) = face(-1.0);
                batter[k] = [reach_l, reach_r];
                benched[k] = rise_l.abs().max(rise_r.abs()) <= MAX_BENCH_FACE_M;
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
                half_width_m: bench_half_width,
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
                        half_width_m: half_width,
                        carriageway_m: 0.0,
                        batter_m: [(EARTHWORK_BATTER * depth).max(EARTHWORK_MIN_BATTER_M); 2],
                        batter_run: [EARTHWORK_BATTER; 2],
                        chain: c.id,
                        arc0: arcs[k],
                        cos_lat: crate::scene::run_cos_lat(&[nodes[k], nodes[k + 1]]),
                        carve: true,
                    });
                }
            }
        }

        // Portal daylighting: carve the ground down to the bore floor in a
        // short cut outward from each solved portal, so the mouth's lower
        // metres stand clear instead of hiding below grade. Cut-only — where
        // the ground has already fallen away there is nothing to remove.
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
                half_width_m: c.kind.prior().half_width_m(c.link).unwrap_or(0.0)
                    + EARTHWORK_SHOULDER_M,
                carriageway_m: 0.0,
                batter_m: [EARTHWORK_MIN_BATTER_M; 2],
                batter_run: [EARTHWORK_BATTER; 2],
                chain: c.id,
                arc0: portal.arc,
                cos_lat: crate::scene::run_cos_lat(&[a, b]),
                carve: true,
            });
        }
    }
    edges
}

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
            half_width_m: 8.0,
            carriageway_m: 6.0,
            batter_m: [4.0; 2],
            batter_run: [EARTHWORK_BATTER; 2],
            chain,
            arc0: 0.0,
            cos_lat,
            carve: false,
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
            half_width_m: 8.0,
            carriageway_m: 6.0,
            batter_m: [4.0; 2],
            batter_run: [EARTHWORK_BATTER; 2],
            chain,
            arc0: 0.0,
            cos_lat,
            carve: false,
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
                half_width_m: 8.0,
                carriageway_m: 6.0,
                batter_m: [4.0; 2],
                batter_run: [EARTHWORK_BATTER; 2],
                chain: 0,
                arc0: 0.0,
                cos_lat,
                carve: false,
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
        let ground = derive(&scene, &solved, None, 1);

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
            let mut g = crate::solve::graph::build(&scene, &profiles, &crossings, Stratum::S);
            crate::solve::relax::solve(&mut g);
            crate::solve::relax::reconstruct(&g, &mut profiles);
        }
        let solved = crate::solve::SolvedModel::from_profiles(profiles, 14);
        let ground = derive(&scene, &solved, None, 1);
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
        let edges = corridor_earthworks(&c, &p, Some(&mut side));
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
        let edges = corridor_earthworks(&c, &p, Some(&mut side));
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
            corridor_earthworks(&c, &p, Some(&mut side)).is_empty(),
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
        let edges = corridor_earthworks(&c, &p, Some(&mut side));
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
        let edges = corridor_earthworks(&c, &p, Some(&mut side));
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
        let edges = corridor_earthworks(&c, &p, Some(&mut side));
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
            half_width_m: 8.0,
            carriageway_m: 6.0,
            batter_m: [4.0; 2],
            batter_run: [EARTHWORK_BATTER; 2],
            chain: 0,
            arc0: 0.0,
            cos_lat,
            carve: false,
        }]);
        let g = GroundStack::new(vec![GroundLayer { stratum: Stratum::S, earthworks, waters }]);
        // Open water away from the berm: flattened to the level (over raw 360).
        assert_eq!(g.height(6.002, 46.002, 360.0, 0.0, &mut scratch), 372.0);
        // On the berm centerline inside the lake: the road overrides the water.
        assert!((g.height(6.005, 46.005, 360.0, 0.0, &mut scratch) - 375.0).abs() < 1e-9);
        // Outside the lake: the raw DEM passes through.
        assert_eq!(g.height(6.02, 46.02, 360.0, 0.0, &mut scratch), 360.0);
    }
}

