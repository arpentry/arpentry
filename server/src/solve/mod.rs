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
pub mod crossings;
pub mod graph;
pub mod portals;
pub mod profile;
pub mod relax;
pub mod structures;

use std::path::Path;
use std::sync::Mutex;

use geo_types::Coord;

use crate::dem::Dem;
use crate::priors::{Stratum, MIN_STRUCTURE_M, SHORT_STRUCTURE_DIP_M};
use crate::project::Bounds;
use crate::scene::{CorridorId, SceneGraph, Span, SpanKind};
use crate::terrain;

pub use profile::{Mode, Profile};

type Error = Box<dyn std::error::Error + Send + Sync>;

/// One measured site of the crossing premise: a mapped bore, crossed by an
/// at-grade band, scored by how far its roof-plus-cover stands above this
/// corridor's own ground there. Positive means the "tunnel" the crossing
/// machinery declined to buy clearance from does not actually pass beneath
/// the ground the crossing feature rides on — the two drawn surfaces then
/// stack with neither a bore nor a deck between them.
#[derive(Debug, Clone, Copy)]
pub struct Daylight {
    pub corridor: CorridorId,
    pub arc: f64,
    pub lon: f64,
    pub lat: f64,
    /// `road + TUNNEL_HEIGHT_M + TUNNEL_COVER_M − terrain`, signed: negative
    /// is honest burial margin, positive is roof daylighting through the
    /// crossing feature's roadbed.
    pub deficit_m: f64,
}

/// One stratum's reconciliation write-back — where the annotation hands over
/// to the solved truth (§4.5).
///
/// For each of the stratum's corridors: grow its tunnel spans through the
/// crossings their buried tails pass beneath ([`portals::annex_spans`]),
/// measure the crossing premise at the covered-bore sites
/// (`structure.bore_daylight` — before reconciliation rewrites the spans it
/// is stated against), then clamp every tunnel span to its buried run and
/// re-cover the freed slack as grade ([`portals::reconcile_spans`]), flipping
/// the profile's at-grade flags over each degraded stretch so the benches and
/// the bands read the same partition as the paint. The result is written into
/// `scene.corridors[..].spans`: after this, there is exactly one span truth
/// and every consumer — junior solves included — reads it.
fn reconcile_stratum(
    scene: &mut SceneGraph,
    profiles: &mut [Option<Profile>],
    stratum: Stratum,
    reaches: &[Vec<(f64, f64)>],
    sites: &[Vec<crossings::PlanCrossing>],
    daylight: &mut Vec<Daylight>,
) {
    // ARPT_DEBUG_ANNEX: one line per tunnel-bearing corridor with crossings —
    // the tail bounds against the crossing arcs, and whether the annex took.
    let debug_annex = std::env::var_os("ARPT_DEBUG_ANNEX").is_some();
    for c in scene.corridors.iter_mut() {
        if c.kind.stratum() != stratum {
            continue;
        }
        let Some(p) = profiles.get_mut(c.id as usize).and_then(|p| p.as_mut()) else {
            continue;
        };
        let reaches = reaches.get(c.id as usize).cloned().unwrap_or_default();
        if debug_annex && c.spans.iter().any(|s| s.kind == SpanKind::Tunnel) {
            for s in c.spans.iter().filter(|s| s.kind == SpanKind::Tunnel) {
                let bounds = portals::span_bounds(p, s);
                let near: Vec<&(f64, f64)> = reaches
                    .iter()
                    .filter(|(x, _)| *x > s.arc0 - 250.0 && *x < s.arc1 + 250.0)
                    .collect();
                eprintln!(
                    "[annex] corridor {} {:?} tunnel [{:.1}, {:.1}] bounds {:?} crossings {:?}",
                    c.id, c.kind, s.arc0, s.arc1, bounds, near
                );
            }
        }
        // The deck contract mirrors `relax::reconstruct`: a monotone class's
        // deck is its line; everyone else refits per-run ramps.
        let deck_follows_road = c.kind.prior().monotone
            && profile::monotone_direction(p.terrain_m()).is_some();
        let mut spans = std::mem::take(&mut c.spans);
        if let Some(annexed) = portals::annex_spans(p, &spans, &reaches) {
            if debug_annex {
                eprintln!("[annex] corridor {} {:?} annexed: {:?}", c.id, c.kind, annexed);
            }
            for s in annexed.iter().filter(|s| s.kind == SpanKind::Tunnel) {
                p.annex_structure(s.arc0, s.arc1, deck_follows_road);
            }
            spans = annexed;
        }
        // The crossing premise, measured against the spans the solve used.
        for x in &sites[c.id as usize] {
            let roof = p.road_at_arc(x.arc)
                + crate::priors::TUNNEL_HEIGHT_M
                + crate::priors::TUNNEL_COVER_M;
            let pt = p.point_at_arc(x.arc);
            if let Some(dbg) = std::env::var_os("ARPT_DEBUG_BURY") {
                if dbg.to_string_lossy().parse::<u32>() == Ok(c.id) {
                    eprintln!(
                        "[bury] daylight corridor {} arc={:.1} road={:.2} surface={:.2} deficit={:+.2}",
                        c.id,
                        x.arc,
                        p.road_at_arc(x.arc),
                        p.surface_at_arc(x.arc),
                        roof - p.surface_at_arc(x.arc)
                    );
                }
            }
            daylight.push(Daylight {
                corridor: c.id,
                arc: x.arc,
                lon: pt.x,
                lat: pt.y,
                deficit_m: roof - p.surface_at_arc(x.arc),
            });
        }
        // Shrink to the geometry: each tunnel clamped to its buried run, the
        // freed annotation slack re-covered as painted grade, a tunnel with
        // no buried run at all degraded end to end.
        let reconciled = portals::reconcile_spans(p, &spans);
        for g in reconciled.iter().filter(|s| s.kind == SpanKind::Grade) {
            for t in spans.iter().filter(|s| s.kind == SpanKind::Tunnel) {
                let (lo, hi) = (g.arc0.max(t.arc0), g.arc1.min(t.arc1));
                if hi - lo > f64::EPSILON {
                    p.degrade_structure(lo, hi, deck_follows_road);
                }
            }
        }
        c.spans = reconciled;
    }
}

/// The solved vertical model: one profile per corridor that needs one, indexed
/// by [`CorridorId`]. Immutable after the solve; shared by every emit worker.
pub struct SolvedModel {
    /// The structures the solved heights imply, by [`CorridorId`] (§4.5).
    ///
    /// An *output*. A deck exists where the solved surface departs the ground,
    /// not where a mapper wrote `bridge` — so "a crossing whose bridge was
    /// deleted" is unrepresentable rather than merely rare.
    pub structures: Vec<Vec<structures::StructureRun>>,
    /// The crossings this solve derived (§4.5). They live on the *output*
    /// because they are a consequence of the solved heights, not an input to
    /// them: stored on the scene they went stale the moment anything changed a
    /// span, and nothing re-derived them.
    pub crossings: Vec<crate::scene::Crossing>,
    /// What the relaxation could not honour — the clearance demands its
    /// plausibility cap rejected. Carried on the model so the run can report
    /// them: a silently dropped constraint is indistinguishable from one that
    /// was satisfied.
    pub relaxed: relax::Relaxed,
    /// The crossing premise, measured (`structure.bore_daylight`): one entry
    /// per place a mapped tunnel span is crossed by another alignment's
    /// at-grade band. Carried on the model because the premise is about how
    /// the scene was *solved* — the crossing machinery waives clearance
    /// wherever the lower side's annotation says "bore"
    /// (`graph::in_immovable_bore`), and these entries are that waiver's
    /// collateral, measured against the solved heights before any
    /// reconciliation rewrites the spans.
    pub daylight: Vec<Daylight>,
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
        SolvedModel {
            structures: Vec::new(),
            crossings: Vec::new(),
            relaxed: relax::Relaxed::default(),
            daylight: Vec::new(),
            profiles: Vec::new(),
            junction_h: Vec::new(),
            z_ref,
        }
    }

    /// Wraps already-solved profiles — for tests and stage-isolated tooling.
    /// Junction heights are unknown on this path; the surface then falls back to
    /// the corridors' own profiles, which is what it does at an unprofiled
    /// intersection anyway.
    pub fn from_profiles(profiles: Vec<Option<Profile>>, z_ref: u8) -> SolvedModel {
        SolvedModel {
            structures: Vec::new(),
            crossings: Vec::new(),
            relaxed: relax::Relaxed::default(),
            daylight: Vec::new(),
            profiles,
            junction_h: Vec::new(),
            z_ref,
        }
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
    run_licensed(scene, terrain_path, z_ref, threads, None)
}

/// The plan skeleton, as the solve consumes it: per corridor, the burial
/// licenses (`crossings::covered_bores`), the crossing reaches the annex
/// walks (`crossings::reaches`), and the spans-over-a-corridor exemptions the
/// short-span demotion honours (`crossings::spans_over_a_corridor` — indexed
/// per span, parallel to the corridor's annotated spans). Heights-free by
/// construction — arcs, band reaches, level ordinals and crossing existence
/// only.
pub struct PlanPin {
    pub covered: Vec<Vec<(f64, f64)>>,
    pub reaches: Vec<Vec<(f64, f64)>>,
    pub over: Vec<Vec<bool>>,
}

/// [`run`], with the plan skeleton supplied rather than derived from this
/// scene.
///
/// The skeleton is **input data** — where mapped alignments cross, how far
/// their bands reach, and their level ordinals — which I7 counts alongside
/// the annotation, never as a junior's solved output (no junior *height* is
/// in it). The perturbation experiment (`authority.inversion_*`) is what
/// needs the override: it deletes the junior corridors and re-solves, and
/// holding the skeleton at the full scene's values is exactly the statement
/// "senior heights are a function of the strata and the plan skeleton, and of
/// nothing the deleted juniors *solved*". All three limbs must be pinned —
/// the burial ceilings read `covered`, the reconciliation write-back walks
/// `reaches`, and the short-span demotion honours `over` — because each is a
/// place a junior's plan existence classifies a senior's spans, and a span
/// classification feeds the senior's own solve. Measured on the Lausanne m2
/// twins: unpinned `over` demoted their 26 m station tunnel once the junior
/// streets crossing it were deleted, the burial ceiling then had no tunnel
/// node to cap, and the pair read 11.46 m shallower — an "authority
/// violation" that was really the experiment deleting part of the input.
pub fn run_licensed(
    scene: &mut SceneGraph,
    terrain_path: Option<&Path>,
    z_ref: u8,
    threads: usize,
    licenses: Option<PlanPin>,
) -> Result<SolvedModel, Error> {
    let Some(path) = terrain_path else {
        // No DEM: no terrain test — every short span demotes, so a flat run
        // never bakes tiny decks floating over its flat ground.
        for c in &mut scene.corridors {
            demote_short_spans(&mut c.spans, &mut |_, _| true);
        }
        return Ok(SolvedModel::empty(z_ref));
    };
    let (pin_over, pin_covered, pin_reaches) = match licenses {
        Some(p) => (Some(p.over), Some(p.covered), Some(p.reaches)),
        None => (None, None, None),
    };
    // One primary DEM handle; the reconcile pass and every solve worker fork
    // it to share the decoded-tile cache.
    let primary_dem = Dem::open(path)?;
    {
        let mut dem = primary_dem.fork()?;
        reconcile_short_spans(
            scene,
            &mut |c: Coord| reference_surface(&mut dem, z_ref, c.x, c.y),
            pin_over.as_deref(),
        );
    }

    // Spans are settled until each stratum's own write-back
    // (`reconcile_stratum`): the workers and every read inside the loop see
    // either the annotation (their own stratum, not yet solved) or a senior's
    // reconciled truth (already written back).
    let scene_mut = scene;
    let scene: &SceneGraph = scene_mut;
    // Every corridor in the scene is solved. The gate upstream admits only
    // strata that solve (`assemble::run`), so "does this need a profile" is no
    // longer a question asked here — a draped feature never reaches this point.
    let todo: Vec<usize> = (0..scene.corridors.len()).collect();
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

    // **The partition** (§4.4): one solver, run over the strata in authority
    // order. Each stratum fuses its own corridors into one graph — junction
    // connectors are shared height variables, so continuity (I2) holds by
    // construction — and reads every senior stratum as a *constant*: a
    // published height with no variable of its own, which is the mechanical
    // statement of authority and the whole of I7.
    //
    // Crossings are derived per stratum, from the solved profiles, and handed
    // straight to the graph (§4.5). Nothing can mutate the model between
    // deriving them and consuming them, because there is nowhere for them to
    // wait.
    let mut crossings: Vec<crate::scene::Crossing> = Vec::new();
    let mut junction_h: Vec<Option<f64>> = vec![None; scene.junctions.len()];
    let mut relaxed = relax::Relaxed::default();
    // The plan facts, once: arcs and identities only, no heights (§4.1 — a
    // junior's warm start is not a fact, so nothing height-bearing may cross a
    // stratum boundary here). `covered` is the burial license the bore
    // ceilings need: where an at-grade band crosses a mapped tunnel span, the
    // bore must actually pass beneath the ground that band rides on.
    let plan = crossings::plan_index(scene);
    let covered = pin_covered.unwrap_or_else(|| crossings::covered_bores(scene, &plan));
    let reaches =
        pin_reaches.unwrap_or_else(|| plan.iter().map(|l| crossings::reaches(l)).collect());
    // Every stratum with members, in authority order — including D. A draped
    // feature has no business in the scene at all (§4.2), and after M2 none
    // is, except the railway `paves_today` still admits as a street. Skipping
    // D would leave those corridors with a per-corridor profile and no graph:
    // no shared junction variable, no clearance, no relax. Measured, that put
    // a 2.40 m step where two railways meet, one classed `unknown` and one
    // `standard_gauge`. Solving D last — junior to everything, reading R and S
    // as constants — is both correct and what M6 will inherit.
    // The covered-bore sites, kept alongside the windows: the write-back
    // measures the crossing premise at exactly the sites the ceilings were
    // seeded from (`structure.bore_daylight`).
    let sites = crossings::covered_sites(scene, &plan);
    let mut daylight: Vec<Daylight> = Vec::new();
    for stratum in [Stratum::H, Stratum::R, Stratum::S, Stratum::D, Stratum::B] {
        // Fresh immutable view per stratum: the write-back below needs the
        // scene mutable, and each iteration's reads must see the seniors'
        // reconciled truth, not the annotation they were assembled with.
        let scene: &SceneGraph = scene_mut;
        if !scene.corridors.iter().any(|c| c.kind.stratum() == stratum) {
            continue;
        }
        let derived = crossings::derive(scene, &profiles, stratum);
        let mut g = graph::build(scene, &profiles, &derived, stratum, &covered);
        let r = relax::solve(&mut g);
        relax::reconstruct(&g, &mut profiles);
        // Each stratum publishes the junction heights it owns; a junction
        // belongs to exactly one, so the slots never contend.
        for (ji, h) in relax::junction_heights(&g).into_iter().enumerate() {
            if h.is_some() {
                junction_h[ji] = h;
            }
        }
        relaxed.sweeps = relaxed.sweeps.max(r.sweeps);
        relaxed.demands_dropped += r.demands_dropped;
        relaxed.worst_dropped_m = relaxed.worst_dropped_m.max(r.worst_dropped_m);
        crossings.extend(derived);
        // **One truth per stratum** (§4.5): the annotation served as the
        // solve's prior; what survives it is the *reconciled* partition —
        // tunnels grown through the crossings their buried tails pass beneath
        // (annex), then clamped to their buried runs, the freed slack
        // re-covered as grade — written back before any junior stratum or any
        // consumer reads the spans. A junior deciding "the senior is in a
        // bore here" (`graph::in_immovable_bore`) then reads a bore that
        // exists, and the bands, benches, sheets, paint and solids all cut
        // one partition. The split this closes: paint reconciled privately at
        // emit while the surfaces read the annotation, so a dismissed tunnel
        // was stroked as a road over ground that never benched it.
        reconcile_stratum(scene_mut, &mut profiles, stratum, &reaches, &sites, &mut daylight);
    }

    // The structures the result implies, derived once the heights are final.
    let structures = scene_mut
        .corridors
        .iter()
        .map(|c| match profiles.get(c.id as usize).and_then(|p| p.as_ref()) {
            Some(p) => structures::derive(p, c.kind.prior()),
            None => Vec::new(),
        })
        .collect();

    Ok(SolvedModel { structures, relaxed, crossings, daylight, profiles, junction_h, z_ref })
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
///
/// A span that passes over or under another mapped alignment is exempt, whether
/// or not the ground moves under it ([`crossings::spans_over_a_corridor`]): what
/// makes it a structure is the carriageway beneath, and the annotation is the
/// only thing in the data that says which of the two is on top. Demoting it
/// hands that ordering to the derivation, which reads it off metre-scale
/// differences between solved surfaces — so one alignment ends up crossing over
/// some roads and under others.
fn reconcile_short_spans(
    scene: &mut SceneGraph,
    sample: &mut impl FnMut(Coord) -> f64,
    // The crossing exemptions, pinned by the perturbation experiment: they
    // are a limb of the plan skeleton (I7), and recomputing them with the
    // juniors deleted demotes exactly the short senior structures the juniors
    // justify. `None` derives them from this scene.
    over_pin: Option<&[Vec<bool>]>,
) {
    // Against the whole scene, before any corridor is mutated.
    let over_own;
    let over: &[Vec<bool>] = match over_pin {
        Some(o) => o,
        None => {
            over_own = crossings::spans_over_a_corridor(scene);
            &over_own
        }
    };
    for (ci, c) in scene.corridors.iter_mut().enumerate() {
        let (nodes, arc) = (std::mem::take(&mut c.nodes), std::mem::take(&mut c.arc));
        let over = &over[ci];
        demote_short_spans(&mut c.spans, &mut |i: usize, span: &Span| {
            !over.get(i).copied().unwrap_or(false) && !spans_a_gap(&nodes, &arc, span, sample)
        });
        (c.nodes, c.arc) = (nodes, arc);
    }
}

/// Demotes every sub-[`MIN_STRUCTURE_M`] structure span for which `demote`
/// says so, then coalesces the adjacent same-kind spans the demotions leave.
fn demote_short_spans(spans: &mut Vec<Span>, demote: &mut impl FnMut(usize, &Span) -> bool) {
    let mut changed = false;
    for (i, s) in spans.iter_mut().enumerate() {
        if s.kind != SpanKind::Grade && s.arc1 - s.arc0 < MIN_STRUCTURE_M && demote(i, s) {
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
        reconcile_short_spans(&mut scene, &mut |c| gully(c), None);
        let kinds: Vec<SpanKind> = scene.corridors[0].spans.iter().map(|s| s.kind).collect();
        assert_eq!(kinds, vec![SpanKind::Grade, SpanKind::Bridge, SpanKind::Grade]);

        // Flat ground: the footbridge annotation demotes, spans coalesce to one.
        let mut scene = SceneGraph::new(vec![corridor(short_bridge())]);
        reconcile_short_spans(&mut scene, &mut |_| 500.0, None);
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
        reconcile_short_spans(&mut scene, &mut |_| 500.0, None);
        assert!(scene.corridors[0].spans.iter().any(|s| s.kind == SpanKind::Bridge));
    }
}
