//! Stage 3 — solve the vertical model (docs/GENERATION.md §5).
//!
//! One pass over the assembled scene graph turns topology into geometry:
//! every corridor that needs a vertical model — every drivable road, plus
//! anything carrying a structure span — gets a [`Profile`] (docs/GROUND.md
//! §1): road-surface heights everywhere along it, anchored to the reference
//! terrain at its at-grade spans and interpolated at a gentle grade across
//! its structures. The [`Mode`] the corridor's class implies parameterises
//! the solve; there is no second vertical model.
//!
//! The reference terrain is the *rendered* ground at the reference zoom (the
//! run's maximum): the same global [`terrain::surface_height`] lattice the
//! emit workers mesh, so a solved at-grade anchor sits exactly on the drawn
//! ground at that zoom. The solved heights are a function of the scene graph
//! and the DEM only — never of a tile window — so every tile fragment reads
//! identical heights (invariant 5), and heights do not change between zoom
//! levels (no popping).

pub mod consistency;
pub mod graph;
pub mod portals;
pub mod profile;
pub mod relax;

use std::path::Path;
use std::sync::Mutex;

use geo_types::Coord;

use crate::dem::Dem;
use crate::priors::{MIN_STRUCTURE_M, SHORT_STRUCTURE_DIP_M};
use crate::project::Bounds;
use crate::scene::{CorridorId, SceneGraph, Span, SpanKind};
use crate::terrain;

pub use profile::{Mode, Profile};

type Error = Box<dyn std::error::Error + Send + Sync>;

/// The solved vertical model: one profile per corridor that needs one, indexed
/// by [`CorridorId`]. Immutable after the solve; shared by every emit worker.
pub struct SolvedModel {
    profiles: Vec<Option<Profile>>,
    /// The height every junction's members share, by index into
    /// `SceneGraph::junctions`; `None` where no member carries a profile. Dense
    /// because that index is already the currency of every junction consumer,
    /// and because a hashed order is a determinism hazard.
    junction_h: Vec<Option<f64>>,
    /// The zoom whose rendered terrain lattice anchored the solve.
    pub z_ref: u8,
}

impl SolvedModel {
    /// A model with no profiles — the DEM-less run, where nothing is elevated.
    pub fn empty(z_ref: u8) -> SolvedModel {
        SolvedModel { profiles: Vec::new(), junction_h: Vec::new(), z_ref }
    }

    /// Wraps already-solved profiles — for tests and stage-isolated tooling.
    /// Junction heights are unknown on this path; the surface then falls back to
    /// the corridors' own profiles, which is what it does at an unprofiled
    /// intersection anyway.
    pub fn from_profiles(profiles: Vec<Option<Profile>>, z_ref: u8) -> SolvedModel {
        SolvedModel { profiles, junction_h: Vec::new(), z_ref }
    }

    /// Attaches solved junction heights to a model assembled in stages — the
    /// counterpart of [`SolvedModel::from_profiles`] for a caller that also ran
    /// the fuse.
    pub fn with_junction_heights(mut self, junction_h: Vec<Option<f64>>) -> SolvedModel {
        self.junction_h = junction_h;
        self
    }

    pub fn profile(&self, corridor: CorridorId) -> Option<&Profile> {
        self.profiles.get(corridor as usize)?.as_ref()
    }

    /// The solved height of a junction, by its index in `SceneGraph::junctions`.
    /// `None` when the intersection has no profiled member, so nothing is known
    /// about where its surface sits.
    pub fn junction_height(&self, junction: usize) -> Option<f64> {
        self.junction_h.get(junction).copied().flatten()
    }

    /// Number of corridors carrying a solved profile.
    pub fn solved_count(&self) -> usize {
        self.profiles.iter().filter(|p| p.is_some()).count()
    }
}

/// Solves the scene graph against the DEM at reference zoom `z_ref`,
/// parallelized over `threads` workers (each owning its own DEM reader).
/// Without a DEM there is nothing to anchor to: the model is empty and roads
/// stay flat, exactly like the terrain they would drape on.
///
/// The scene is mutable for one reconciliation the assemble stage could not
/// make: the terrain fate of sub-[`MIN_STRUCTURE_M`] structure spans
/// ([`reconcile_short_spans`]), which rewrites the corridor spans every later
/// consumer (profiles, earthworks, emit) reads.
pub fn run(
    scene: &mut SceneGraph,
    terrain_path: Option<&Path>,
    z_ref: u8,
    threads: usize,
) -> Result<SolvedModel, Error> {
    let Some(path) = terrain_path else {
        // No DEM: no terrain test — every short span demotes, so a flat run
        // never bakes tiny decks floating over its flat ground.
        for c in &mut scene.corridors {
            demote_short_spans(&mut c.spans, &mut |_| true);
        }
        return Ok(SolvedModel::empty(z_ref));
    };
    // One primary DEM handle; the reconcile pass and every solve worker fork
    // it to share the decoded-tile cache.
    let primary_dem = Dem::open(path)?;
    {
        let mut dem = primary_dem.fork()?;
        reconcile_short_spans(scene, &mut |c: Coord| {
            reference_surface(&mut dem, z_ref, c.x, c.y)
        });
    }

    // Spans are settled: the workers below only read.
    let scene: &SceneGraph = scene;
    let todo: Vec<usize> = scene
        .corridors
        .iter()
        .enumerate()
        .filter(|(_, c)| c.needs_profile())
        .map(|(i, _)| i)
        .collect();
    let mut profiles: Vec<Option<Profile>> = Vec::new();
    profiles.resize_with(scene.corridors.len(), || None);

    let threads = threads.max(1).min(todo.len().max(1));
    let next = Mutex::new(0usize);
    let results: Mutex<&mut Vec<Option<Profile>>> = Mutex::new(&mut profiles);
    std::thread::scope(|scope| -> Result<(), Error> {
        let mut handles = Vec::with_capacity(threads);
        for _ in 0..threads {
            handles.push(scope.spawn(|| -> Result<(), Error> {
                let mut dem = primary_dem.fork()?;
                loop {
                    let i = {
                        let mut n = next.lock().expect("solve queue poisoned");
                        if *n >= todo.len() {
                            break;
                        }
                        let i = *n;
                        *n += 1;
                        i
                    };
                    let c = &scene.corridors[todo[i]];
                    let mode = Mode::for_kind(c.kind);
                    let solved = profile::solve(&c.nodes, &c.spans, mode, &mut |p| {
                        reference_surface(&mut dem, z_ref, p.x, p.y)
                    });
                    results.lock().expect("solve results poisoned")[todo[i]] = solved;
                }
                Ok(())
            }));
        }
        for handle in handles {
            handle.join().map_err(|_| "solve worker panicked")??;
        }
        Ok(())
    })?;

    // Global vertical consistency (docs/GENERATION.md §4.4): fuse the per-corridor
    // profiles into one constraint graph whose junction connectors are *shared*
    // height variables, and relax it. Continuity (I2) then holds by construction
    // — two corridors at a connector read one number, so no step is
    // representable — and clearance (I3) is enforced as a raise-only projection
    // in the same loop.
    // The junction heights are read out of the graph before it is dropped: they
    // are what the surface mesh pins each intersection to, and nothing else can
    // reproduce them exactly once the values have been scattered into `road_m`.
    let junction_h = {
        let mut g = graph::build(scene, &profiles);
        relax::solve(&mut g);
        relax::reconstruct(&g, &mut profiles);
        relax::junction_heights(&g)
    };

    Ok(SolvedModel { profiles, junction_h, z_ref })
}

/// The terrain fate of the short structure spans assemble keeps
/// (sub-[`MIN_STRUCTURE_M`], `assemble::corridors::resolve_spans`): a short
/// bridge stays a deck only where the ground genuinely falls away beneath it
/// — its mid-span terrain more than [`SHORT_STRUCTURE_DIP_M`] below the
/// span's end-to-end chord — and a short tunnel only where the ground rises
/// over it. Everything else demotes to grade: on near-flat ground the drape
/// (with the notch closing) carries the road, and a tiny baked deck would
/// float over the hill. The deep-gully case is why the test exists at all: a
/// 25 m annotated bridge over a 30 m stream cut, demoted blindly, dived
/// through the gorge and dragged its earthworks with it.
fn reconcile_short_spans(scene: &mut SceneGraph, sample: &mut impl FnMut(Coord) -> f64) {
    for c in &mut scene.corridors {
        let (nodes, arc) = (std::mem::take(&mut c.nodes), std::mem::take(&mut c.arc));
        demote_short_spans(&mut c.spans, &mut |span: &Span| {
            !spans_a_gap(&nodes, &arc, span, sample)
        });
        (c.nodes, c.arc) = (nodes, arc);
    }
}

/// Demotes every sub-[`MIN_STRUCTURE_M`] structure span for which `demote`
/// says so, then coalesces the adjacent same-kind spans the demotions leave.
fn demote_short_spans(spans: &mut Vec<Span>, demote: &mut impl FnMut(&Span) -> bool) {
    let mut changed = false;
    for s in spans.iter_mut() {
        if s.kind != SpanKind::Grade && s.arc1 - s.arc0 < MIN_STRUCTURE_M && demote(s) {
            s.kind = SpanKind::Grade;
            s.level = 0;
            changed = true;
        }
    }
    if changed {
        spans.dedup_by(|cur, prev| {
            if prev.kind == cur.kind && prev.level == cur.level {
                prev.arc1 = cur.arc1;
                true
            } else {
                false
            }
        });
    }
}

/// Whether the terrain departs from the span's end-to-end chord — dips below
/// it for a bridge, rises above it for a tunnel — by more than
/// [`SHORT_STRUCTURE_DIP_M`] anywhere across the span's interior quarters.
fn spans_a_gap(
    nodes: &[Coord],
    arc: &[f64],
    span: &Span,
    sample: &mut impl FnMut(Coord) -> f64,
) -> bool {
    let h0 = sample(point_at_arc(nodes, arc, span.arc0));
    let h1 = sample(point_at_arc(nodes, arc, span.arc1));
    (1..=3).any(|k| {
        let t = k as f64 / 4.0;
        let chord = h0 + (h1 - h0) * t;
        let ground = sample(point_at_arc(nodes, arc, span.arc0 + (span.arc1 - span.arc0) * t));
        let depart = match span.kind {
            SpanKind::Bridge => chord - ground,
            SpanKind::Tunnel => ground - chord,
            SpanKind::Grade => 0.0,
        };
        depart > SHORT_STRUCTURE_DIP_M
    })
}

/// The corridor centerline point at arc `s`, linearly interpolated between
/// the bracketing nodes (clamped to the ends).
fn point_at_arc(nodes: &[Coord], arc: &[f64], s: f64) -> Coord {
    let i = arc.partition_point(|&a| a < s).clamp(1, arc.len() - 1);
    let (a0, a1) = (arc[i - 1], arc[i]);
    let t = if a1 > a0 { ((s - a0) / (a1 - a0)).clamp(0.0, 1.0) } else { 0.0 };
    Coord {
        x: nodes[i - 1].x + (nodes[i].x - nodes[i - 1].x) * t,
        y: nodes[i - 1].y + (nodes[i].y - nodes[i - 1].y) * t,
    }
}

/// The rendered-ground height at `(lon, lat)` on the global zoom-`z` lattice —
/// the same surface [`terrain::surface_height`] gives an emit worker meshing
/// the containing tile, so solved anchors sit exactly on the drawn ground.
/// Only ever called at the reference zoom, so the lattice is the detail grid
/// (`grid_for(z, z)`) — the resolution the z_ref mesh actually renders at;
/// anchors, bed targets, and water levels must all read that same surface.
pub fn reference_surface(dem: &mut Dem, z: u8, lon: f64, lat: f64) -> f64 {
    let b = tile_containing(z, lon, lat);
    let grid = terrain::grid_for(z, z);
    terrain::surface_height(&b, grid, lon, lat, &mut |a, o| dem.elevation(a, o, z))
}

/// Bounds of the zoom-`z` tile containing `(lon, lat)` (the lattice anchor;
/// any covering tile yields the same surface since the lattice is global).
pub fn tile_containing(z: u8, lon: f64, lat: f64) -> Bounds {
    let n = (1u64 << z as u32) as f64;
    let x = (((lon + 180.0) / 360.0) * n).floor().clamp(0.0, n - 1.0) as u32;
    let y = (((lat + 90.0) / 180.0) * n).floor().clamp(0.0, n - 1.0) as u32;
    Bounds::of_tile(z, x, y)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tile_containing_agrees_with_of_tile() {
        let b = tile_containing(14, 6.9185, 46.4355);
        assert!(b.contains(6.9185, 46.4355));
        // Consistent with the tiling scheme: the tile's own bounds contain it.
        let n = (1u64 << 14) as f64;
        let x = (((6.9185 + 180.0) / 360.0) * n).floor() as u32;
        let y = (((46.4355 + 90.0) / 180.0) * n).floor() as u32;
        let direct = Bounds::of_tile(14, x, y);
        assert_eq!(b.west, direct.west);
        assert_eq!(b.south, direct.south);
    }

    #[test]
    fn empty_model_has_no_profiles() {
        let m = SolvedModel::empty(14);
        assert!(m.profile(0).is_none());
        assert_eq!(m.solved_count(), 0);
    }

    /// A short annotated bridge keeps its deck over a real gully and demotes
    /// on near-flat ground — the terrain test the assemble stage defers here.
    #[test]
    fn short_spans_resolve_against_the_terrain() {
        use crate::priors::{Kind, RoadClass};
        use crate::scene::{Corridor, SegmentRef, DEG_M};
        let cos_lat = 46.0_f64.to_radians().cos();
        let len_m = 200.0;
        let deg = len_m / (DEG_M * cos_lat);
        let n = 41;
        let nodes: Vec<Coord> =
            (0..n).map(|i| Coord { x: 6.0 + deg * i as f64 / (n - 1) as f64, y: 46.0 }).collect();
        let arc: Vec<f64> = (0..n).map(|i| len_m * i as f64 / (n - 1) as f64).collect();
        let corridor = |spans: Vec<Span>| Corridor {
            id: 0,
            nodes: nodes.clone(),
            arc: arc.clone(),
            cos_lat,
            kind: Kind::Road(RoadClass::Residential),
            class_key: String::new(),
            link: false,
            width_m: Some(5.5),
            spans,
            segments: vec![SegmentRef { source: 1, node0: 0, node1: n - 1, properties: vec![] }],
            connectors: vec![],
        };
        let short_bridge = || {
            vec![
                Span { arc0: 0.0, arc1: 90.0, level: 0, kind: SpanKind::Grade },
                Span { arc0: 90.0, arc1: 115.0, level: 1, kind: SpanKind::Bridge },
                Span { arc0: 115.0, arc1: 200.0, level: 0, kind: SpanKind::Grade },
            ]
        };
        // A 30 m-deep gully under the 25 m span: the deck survives.
        let gully = |c: Coord| {
            let x = (c.x - 6.0) / deg * len_m;
            500.0 - (1.0 - ((x - 102.5) / 15.0).abs()).max(0.0) * 30.0
        };
        let mut scene = SceneGraph::new(vec![corridor(short_bridge())]);
        reconcile_short_spans(&mut scene, &mut |c| gully(c));
        let kinds: Vec<SpanKind> = scene.corridors[0].spans.iter().map(|s| s.kind).collect();
        assert_eq!(kinds, vec![SpanKind::Grade, SpanKind::Bridge, SpanKind::Grade]);

        // Flat ground: the footbridge annotation demotes, spans coalesce to one.
        let mut scene = SceneGraph::new(vec![corridor(short_bridge())]);
        reconcile_short_spans(&mut scene, &mut |_| 500.0);
        let spans = &scene.corridors[0].spans;
        assert_eq!(spans.len(), 1, "demoted spans must coalesce, got {spans:?}");
        assert_eq!(spans[0].kind, SpanKind::Grade);
        assert_eq!((spans[0].arc0, spans[0].arc1), (0.0, 200.0));

        // A long bridge faces no test: it stays whatever the ground does.
        let long = vec![
            Span { arc0: 0.0, arc1: 80.0, level: 0, kind: SpanKind::Grade },
            Span { arc0: 80.0, arc1: 160.0, level: 1, kind: SpanKind::Bridge },
            Span { arc0: 160.0, arc1: 200.0, level: 0, kind: SpanKind::Grade },
        ];
        let mut scene = SceneGraph::new(vec![corridor(long)]);
        reconcile_short_spans(&mut scene, &mut |_| 500.0);
        assert!(scene.corridors[0].spans.iter().any(|s| s.kind == SpanKind::Bridge));
    }
}
