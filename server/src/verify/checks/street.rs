//! The street's cross-section, measured at its two outer edges.
//!
//! Every other check in this harness scores the carriageway against the ground,
//! the deck or its own neighbour. None of them can see what happens *beside* a
//! street, and beside a street is where two defects live that no height check
//! reports:
//!
//! - [`order.building_overlap`] — drawn asphalt inside a building footprint at
//!   the same level. The carriageway's width is a class prior, the footprint is
//!   surveyed, and nothing reconciles the two: the band is laid at the prior's
//!   half-width whatever is standing there, so it runs through walls.
//! - [`contact.sidewalk_grade`] — how far the drawn surface a pedestrian way
//!   stands on departs from the carriageway it runs alongside. The bench a
//!   street imprints reaches `EARTHWORK_SHOULDER_M + EARTHWORK_MARGIN_M` past
//!   its own half-width and no further, so a sidewalk outside that band drapes
//!   on whatever the hillside does.
//!
//! They are one fact seen twice: **nothing owns a street's cross-section**. The
//! carriageway owns a band, the bench owns a slightly wider band, and
//! everything outside that — sidewalks, verges, facades — is unclaimed. The two
//! metrics are what says so in numbers, and what will say whether allocating
//! that cross-section out of the room the buildings leave has worked.
//!
//! Both read the *drawn* archive rather than the plan. A duckdb buffer of the
//! centerlines answers a related but different question — it knows the class
//! prior's half-width and nothing about junction plates, the rim or the
//! links the union dissolves into one region — and the two numbers differ by
//! more than a factor of two for exactly that reason (see the population note
//! on `order.building_overlap`).

use std::collections::HashMap;

use crate::priors::{self, WALK_ALONG, WALK_ATTACH_M, WALK_COVER};
use crate::verify::dist::Dist;
use crate::verify::mesh::{Scale, SurfaceMesh};
use crate::verify::scene::{RoadMesh, TileScene};
use crate::verify::{Invariant, Metric, Offender, Sense, Worst};

use super::{Check, Options};

/// How far drawn at-grade asphalt may reach inside a building footprint before
/// it reads as a defect rather than as the footprint's own plan error.
///
/// **This is a reasoned threshold, not one read off a cliff, and the population
/// says why.** Sorted, the penetration depths climb smoothly through the
/// shallow end — p25 0.39 m, p50 0.88 m, p75 1.61 m — with no gap anywhere a
/// plan error could be cut off. The population *does* separate further out
/// (17.5 % of inside samples past 2 m, 9.6 % past 3 m, then only 2.8 % in the
/// whole band from 3 m to 5 m, and 6.8 % beyond it), but that gap is in the
/// wrong place to gate on: the deep mass is the roads whose *centerline* runs
/// inside a footprint — a parking structure's internal service ways, a way
/// mapped through a courtyard — which no width cap can move, while the shallow
/// mass is precisely what capping the band against the room would remove.
///
/// So the threshold comes from the design instead. The cross-section is meant
/// to keep `FACADE_CLEAR_M` (0.5 m) of drawn surface off a footprint, so half a
/// metre *inside* one is a full metre from where the model puts it — past what
/// a surveyed footprint's own error explains, and squarely inside what the fix
/// is for.
const FACADE_M: f64 = 0.5;

// The attachment rule — `WALK_ATTACH_M`, `WALK_COVER`, `WALK_ALONG` — is read
// from `crate::priors`, where the model that states it keeps it
// (`assemble::walks`). **One definition, deliberately.** The three numbers
// were measured here first, in archive space, because this check had to decide
// what a sidewalk was before anything else did; now the model attaches the
// same population, and a check scoring a different one would be reporting a
// metric about nothing. The reasoning for each number lives with the constant.
//
// What stays archive-side is which *evidence* is available: the archive
// carries a way's class and its geometry, never the `subclass='sidewalk'` tag,
// so this population is the geometric half of the rule alone.

/// How far the surface under an attached pedestrian way may stand from the
/// carriageway beside it before the two are not one cross-section.
///
/// Read off the measured population, which is two-moded: the departure
/// magnitude falls steeply through the near end — 27.2 % past 0.25 m, 12.9 %
/// past 0.5 m, 8.2 % past 0.75 m — and then flattens into a long tail (6.3 %
/// past 1 m, 5.4 % past 1.25 m, 4.7 % past 1.5 m). The knee is the whole
/// finding: one mode is the sidewalk that happens to lie inside its street's
/// bench — the plan-space census put that at a fifth of tagged sidewalk length
/// — and the other is the one that drapes on natural ground outside it.
///
/// A metre sits past the knee, and past what the measurement itself can
/// manufacture: the reference is the *nearest* kerb point in plan, up to
/// [`WALK_ATTACH_M`] away, so at a corner or a band end some of that distance
/// is along the road rather than across it and the street's own longitudinal
/// grade contributes. It is also well clear of the kerb rise plus a verge's
/// cross-fall, which is what a correct cross-section would read here.
const WALK_GRADE_M: f64 = 1.0;

/// Sample spacing along a pedestrian centerline, and along a kerb. A metre
/// matches the surface spacing the other checks use and is finer than any
/// cross-section feature being measured.
const WALK_STEP_M: f64 = 1.0;

/// How far the drawn ground at a pedestrian band's own rim may stand from the
/// band before it is a wall rather than a joint.
///
/// The two surfaces meet along that rim by construction — the band's region is
/// what cut the hole the rim bounds — so the honest zero here is a joint, not a
/// contact band: unlike a building's seat or a deck's, no thickness, no
/// foundation and no clearance stands between them. What is left is
/// quantization (a centimetre on each surface) and the terrain's own rounding
/// of the ring it triangulated. A tenth of a metre is comfortably past that and
/// is also [`priors::KERB_RISE_M`], which is the smallest step in the model
/// worth drawing at all.
const WALK_RIM_M: f64 = 0.1;

/// How far in from a band's edge the cross-fall is read, in metres, and how far
/// in it must stop being the band for the sample to count as a *side* edge.
///
/// A band is [`priors::WALK_WIDTH_M`] wide, so a metre in from one side is its
/// centreline and three metres in is off it. An edge that still has band under
/// it three metres in is an *end* — where "inward" is along the way rather than
/// across it, and the reading would be the way's own longitudinal grade — or a
/// plaza, where there is no cross-section to be wrong about. Both are dropped.
const WALK_FALL_IN_M: f64 = 1.0;
const WALK_FALL_OUT_M: f64 = 3.0;

/// The shorter reaches the probe falls back to where a metre inward is already
/// off the band, and the shortest baseline a reading is trusted over.
///
/// **A fixed metre made the metric blind to exactly the bands most likely to be
/// wrong.** `synth::pave_mesh` insets a band's surface by
/// [`priors::PAVE_RIM_M`] on each side for the rim, so the interior of a
/// band is its width less 0.70 m and a probe a metre inward needs a band
/// **1.70 m wide** to land on anything. Narrower bands exist — the facade room
/// already narrows a sidewalk to [`priors::WALK_MIN_WIDTH_M`] = 0.8 m — and
/// every one of them left the population rather than entering the offender set,
/// silently. That is a metric that rewards narrowing a band over flattening it,
/// which is the wrong incentive to hand a fix.
///
/// So the probe takes the longest baseline the band actually offers. The floor
/// is a quarter metre because the band is triangulated over the terrain lattice
/// and a shorter run divides a centimetre of vertex quantization by too little
/// to mean anything.
const WALK_FALL_LADDER_M: [f64; 3] = [WALK_FALL_IN_M, 0.5, 0.25];

/// Cross-fall a pedestrian band may carry before it is a hillside rather than a
/// walkway, as rise per metre across.
///
/// Set from the drainage cross-fall a real footway is built to (2 %, and 4 % at
/// the accessible limit) with room for the drawn world's own noise on top: the
/// band is triangulated over the terrain lattice, so a metre across it spans a
/// vertex pair whose heights each carry a centimetre of quantization. A tenth
/// is past all of that and far below anything a hillside contributes.
const WALK_FALL: f64 = 0.10;

/// How far a pedestrian band may stand above or below the carriageway sharing
/// its plan position and still be *on* it rather than over or under it.
///
/// [`priors::WALK_ON_ASPHALT_M`], where the model that trims the coincident
/// band keeps it (`synth::pavement`) — one definition, deliberately, like the
/// attachment rule above: within the band a correct pavement rides
/// [`priors::KERB_RISE_M`] above the asphalt and a coincident one must have
/// yielded, while past it a footbridge or a rim path above a sunken road is a
/// stack the model draws on purpose. A check scoring a different coincidence
/// band than the model trims would report a metric about nothing.
const ON_ASPHALT_DZ_M: f64 = priors::WALK_ON_ASPHALT_M;

/// How deep onto the carriageway a coincident band sample must reach before it
/// is a band on the plate rather than the shared kerb edge.
///
/// A sidewalk legitimately *meets* the asphalt along the kerb, so the seam
/// itself must score zero. The depth is read against the road silhouette
/// resampled at [`WALK_STEP_M`], whose nearest-point error is up to half a
/// step; and `road_surface` is inset [`priors::PAVE_RIM_M`] = 0.35 m from the
/// true kerb, so its own silhouette duplicates the boundary a third of a metre
/// in. Three quarters of a metre is past both, and anything deeper is plan
/// space the carriageway owns.
const ON_ASPHALT_EDGE_M: f64 = 0.75;

/// The pedestrian classes this check scores, which is the model's class table
/// ([`priors::is_pedestrian`]) less `steps`.
///
/// A staircase's purpose is to change height relative to what is beside it, so
/// counting one measures the class table rather than the defect — the same
/// reason `slope.road_grade` excludes non-drivable classes. It is still
/// *attached* by the model; it is just not evidence about a cross-section.
/// Every other pedestrian class is in, tagged as a sidewalk or not.
fn is_pedestrian(class: &str) -> bool {
    class != "steps" && priors::is_pedestrian(priors::Kind::parse(None, Some(class), None))
}

/// The at-grade *road* surface: the union's interior band and its rim.
///
/// Rail is deliberately out of the footprint population. A station roof stands
/// over its own platforms, which is a level relation the archive cannot state
/// and not asphalt through a wall — 2,772 m² of the Montreux extract, all of it
/// under station roofs.
fn is_road_band(r: &RoadMesh) -> bool {
    r.level == 0 && matches!(r.class.as_str(), "road_surface" | "road_rim")
}


/// The drawn surface a pedestrian way stands on: a walkway band where one
/// exists, and the drawn ground where it does not.
///
/// Today it is always the ground — nothing lays a pedestrian surface. The
/// lookup is here because the day one is laid, the region it occupies takes a
/// hole out of the terrain with it (the mesher cuts the regions
/// `add_road_surface` returns), so a check reading only the terrain would find
/// nothing there and report its population quietly emptying rather than the
/// cross-section it was built to measure.
fn walk_ground(tile: &TileScene, px: f64, py: f64) -> Option<f64> {
    for r in tile.roads.iter().filter(|r| {
        r.level == 0 && matches!(r.class.as_str(), "walk_surface" | "walk_rim")
    }) {
        if let Some(h) = r.mesh.height_at(px, py) {
            return Some(h);
        }
    }
    tile.terrain.as_ref().and_then(|t| t.height_at(px, py))
}

/// One building's plan footprint: its bounding box, the mesh that answers
/// point-in-footprint, and the wall segments that are its own boundary.
struct Footprint<'a> {
    x0: f64,
    x1: f64,
    y0: f64,
    y1: f64,
    mesh: &'a SurfaceMesh,
    /// Plan projections of the mesh's vertical faces. A wall quad has no plan
    /// area, so it is invisible to every point-in-triangle query and is exactly
    /// the footprint outline — including a courtyard's inner ring, whose walls
    /// bound the void the same way.
    walls: Vec<((f64, f64), (f64, f64))>,
}

/// The tile's footprints, bucketed in plan so a surface sample tests a handful
/// rather than all of them. A dense tile carries fifty buildings and twenty-two
/// million surface samples cross the extract, which is the difference between
/// this check costing a second and costing minutes.
pub(crate) struct Footprints<'a> {
    items: Vec<Footprint<'a>>,
    cells: HashMap<(i32, i32), Vec<usize>>,
    /// Cell size in unit plan space, per axis — about 20 m.
    cw: f64,
    ch: f64,
}

impl<'a> Footprints<'a> {
    pub(crate) fn build(tile: &'a TileScene) -> Footprints<'a> {
        let (cw, ch) = (20.0 / tile.scale.mx, 20.0 / tile.scale.my);
        let mut items = Vec::with_capacity(tile.buildings.len());
        for (_relief, m) in &tile.buildings {
            let (mut x0, mut x1, mut y0, mut y1) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
            for i in 0..m.vertex_count() {
                let (x, y, _) = m.vertex(i);
                x0 = x0.min(x);
                x1 = x1.max(x);
                y0 = y0.min(y);
                y1 = y1.max(y);
            }
            if x0 > x1 {
                continue;
            }
            items.push(Footprint { x0, x1, y0, y1, mesh: m, walls: wall_segments(m, &tile.scale) });
        }
        let mut cells: HashMap<(i32, i32), Vec<usize>> = HashMap::new();
        for (i, f) in items.iter().enumerate() {
            for cx in (f.x0 / cw).floor() as i32..=(f.x1 / cw).floor() as i32 {
                for cy in (f.y0 / ch).floor() as i32..=(f.y1 / ch).floor() as i32 {
                    cells.entry((cx, cy)).or_default().push(i);
                }
            }
        }
        Footprints { items, cells, cw, ch }
    }

    fn is_empty(&self) -> bool {
        self.items.is_empty()
    }

    /// How far inside a footprint a plan point stands, or `None` outside every
    /// one. Where footprints overlap the deepest answers, which is the one a
    /// viewer would see the asphalt disappear into.
    pub(crate) fn depth(&self, px: f64, py: f64, scale: &Scale) -> Option<f64> {
        if self.items.is_empty() {
            return None;
        }
        let key = ((px / self.cw).floor() as i32, (py / self.ch).floor() as i32);
        let bucket = self.cells.get(&key)?;
        let mut best: Option<f64> = None;
        for &i in bucket {
            let f = &self.items[i];
            if px < f.x0 || px > f.x1 || py < f.y0 || py > f.y1 {
                continue;
            }
            if f.mesh.height_at(px, py).is_none() {
                continue;
            }
            let mut d = f64::INFINITY;
            for seg in &f.walls {
                d = d.min(seg_dist(px, py, seg, scale));
            }
            if d.is_finite() && best.is_none_or(|b| d > b) {
                best = Some(d);
            }
        }
        best
    }
}

/// Plan area of a triangle in square metres.
fn plan_area(tri: &[(f64, f64, f64); 3], s: &Scale) -> f64 {
    let (ax, ay) = ((tri[1].0 - tri[0].0) * s.mx, (tri[1].1 - tri[0].1) * s.my);
    let (bx, by) = ((tri[2].0 - tri[0].0) * s.mx, (tri[2].1 - tri[0].1) * s.my);
    (ax * by - ay * bx).abs() * 0.5
}

/// The plan projections of a solid's vertical faces.
///
/// A building ships as a closed solid — walls plus a roof cap — so its
/// footprint outline is not stored anywhere, but it is exactly recoverable: a
/// wall triangle stands on the outline and therefore has no plan area, while
/// every cap triangle, flat or pitched, has some.
fn wall_segments(m: &SurfaceMesh, s: &Scale) -> Vec<((f64, f64), (f64, f64))> {
    let mut out = Vec::new();
    for t in 0..m.triangle_count() {
        let tri = m.triangle(t);
        if plan_area(&tri, s) > 1e-6 {
            continue;
        }
        // The two furthest-apart corners span the segment; the third lies on it.
        let mut best = (0.0f64, 0usize, 1usize);
        for i in 0..3 {
            for j in (i + 1)..3 {
                let d = s.dist(tri[i].0, tri[i].1, tri[j].0, tri[j].1);
                if d > best.0 {
                    best = (d, i, j);
                }
            }
        }
        if best.0 < 1e-6 {
            continue; // a degenerate face, standing on a point
        }
        out.push(((tri[best.1].0, tri[best.1].1), (tri[best.2].0, tri[best.2].1)));
    }
    out
}

/// Plan distance from a point to a segment, in metres.
fn seg_dist(px: f64, py: f64, seg: &((f64, f64), (f64, f64)), s: &Scale) -> f64 {
    let (a, b) = *seg;
    let (dx, dy) = ((b.0 - a.0) * s.mx, (b.1 - a.1) * s.my);
    let (qx, qy) = ((px - a.0) * s.mx, (py - a.1) * s.my);
    let len2 = dx * dx + dy * dy;
    let u = if len2 > 0.0 { ((qx * dx + qy * dy) / len2).clamp(0.0, 1.0) } else { 0.0 };
    let (ex, ey) = (qx - dx * u, qy - dy * u);
    (ex * ex + ey * ey).sqrt()
}

/// One resampled point on a drawn kerb: where it is, how high the carriageway
/// is there, and which way the kerb runs.
#[derive(Clone, Copy)]
struct KerbPoint {
    x: f64,
    y: f64,
    z: f64,
    dir: (f64, f64),
}

/// The tile's kerb line, bucketed in plan at the attachment reach.
struct Kerbs {
    pts: Vec<KerbPoint>,
    cells: HashMap<(i32, i32), Vec<usize>>,
    cw: f64,
    ch: f64,
}

impl Kerbs {
    /// Resamples the silhouette of every at-grade road band.
    ///
    /// Both the interior band and its rim contribute. `road_surface` is
    /// an *inset* of the paved region by `PAVE_RIM_M`, so its silhouette is a
    /// third of a metre inside the true kerb; the rim carries the real one.
    /// Taking the union and asking for the nearest means a way outside the
    /// street finds the outer rim, which is the kerb it stands beside.
    fn build(tile: &TileScene) -> Kerbs {
        let (cw, ch) =
            (WALK_ATTACH_M / tile.scale.mx, WALK_ATTACH_M / tile.scale.my);
        let mut pts = Vec::new();
        for r in tile.roads.iter().filter(|r| is_road_band(r)) {
            for (a, b, _) in r.mesh.boundary_edges() {
                let (ax, ay, az) = r.mesh.vertex(a);
                let (bx, by, bz) = r.mesh.vertex(b);
                let len = tile.scale.dist(ax, ay, bx, by);
                if len < 1e-9 {
                    continue;
                }
                let dir =
                    ((bx - ax) * tile.scale.mx / len, (by - ay) * tile.scale.my / len);
                let steps = (len / WALK_STEP_M).ceil().max(1.0) as usize;
                for k in 0..=steps {
                    let t = k as f64 / steps as f64;
                    pts.push(KerbPoint {
                        x: ax + (bx - ax) * t,
                        y: ay + (by - ay) * t,
                        z: az + (bz - az) * t,
                        dir,
                    });
                }
            }
        }
        let mut cells: HashMap<(i32, i32), Vec<usize>> = HashMap::new();
        for (i, p) in pts.iter().enumerate() {
            cells
                .entry(((p.x / cw).floor() as i32, (p.y / ch).floor() as i32))
                .or_default()
                .push(i);
        }
        Kerbs { pts, cells, cw, ch }
    }

    fn is_empty(&self) -> bool {
        self.pts.is_empty()
    }

    /// The nearest kerb point within [`WALK_ATTACH_M`], with its plan distance.
    fn nearest(&self, px: f64, py: f64, scale: &Scale) -> Option<(f64, KerbPoint)> {
        let (cx, cy) = ((px / self.cw).floor() as i32, (py / self.ch).floor() as i32);
        let mut best: Option<(f64, KerbPoint)> = None;
        for dx in -1..=1 {
            for dy in -1..=1 {
                let Some(bucket) = self.cells.get(&(cx + dx, cy + dy)) else { continue };
                for &i in bucket {
                    let p = self.pts[i];
                    let d = scale.dist(px, py, p.x, p.y);
                    if d > WALK_ATTACH_M {
                        continue;
                    }
                    if best.is_none_or(|(bd, _)| d < bd) {
                        best = Some((d, p));
                    }
                }
            }
        }
        best
    }
}

pub struct Street {
    overlap: Dist,
    overlap_worst: Worst,
    /// The same, over the at-grade pedestrian surface ([`is_walk_band`]).
    walk_indoors: Dist,
    walk_indoors_worst: Worst,
    walk_inside: u64,
    /// How deep onto the at-grade carriageway a pedestrian band stands, where
    /// it stands *on* it (within [`ON_ASPHALT_DZ_M`] vertically), and zero
    /// where it does not. The signed height off the asphalt over the
    /// coincident samples alone, for `ARPT_DEBUG_STREET`.
    on_asphalt: Dist,
    on_asphalt_worst: Worst,
    on_asphalt_dz: Dist,
    grade: Dist,
    grade_worst: Worst,
    /// The step at a pedestrian band's rim, unsigned, and the same step with
    /// its sign for the histogram — the ground stands above the band as often
    /// as it falls away from it, and which it does says which earthwork is
    /// missing.
    rim: Dist,
    rim_signed: Dist,
    rim_worst: Worst,
    /// Rim samples whose step a drawn apron wall spans — closed joints,
    /// scored 0 (the drape guard chooses a wall over an over-tall bench on
    /// purpose, and a walled rim is that choice drawn, not a gap).
    rim_walled: usize,
    /// The band's cross-fall, as rise per metre across it.
    fall: Dist,
    fall_worst: Worst,
    /// The same two populations split by material, for `ARPT_DEBUG_STREET`. A
    /// sidewalk and a path stand on the ground for different reasons — one is
    /// seated on its host's cross-section, the other *is* the drawn ground — so
    /// a single histogram of either metric is two modes stacked, and reading it
    /// as one would attribute a path's tilt to a sidewalk's seat.
    rim_by: [Dist; 2],
    fall_by: [Dist; 2],
    /// The same departures with their sign kept. The scored metric is unsigned
    /// — a way on a bank above its street is the same defect as one in a ditch
    /// — but which side the population sits on is the first thing anyone
    /// reading it wants, and an unsigned histogram cannot answer it.
    grade_signed: Dist,
    /// Samples that landed inside a footprint at all, counted exactly. The
    /// histogram cannot answer this: its first bin is 1.3 cm wide and holds
    /// every zero along with every shallow clip.
    inside: u64,
    /// How many tiles carried a footprint at all. A zoom with no building layer
    /// would otherwise score every sample zero and print a perfect column,
    /// which is a check that stopped running looking exactly like one that
    /// passed.
    footprint_tiles: u64,
}

impl Street {
    pub fn new(opt: &Options) -> Street {
        Street {
            overlap: Dist::new(0.0, 64.0),
            overlap_worst: Worst::new(Sense::HigherIsWorse, opt.worst_k),
            walk_indoors: Dist::new(0.0, 64.0),
            walk_indoors_worst: Worst::new(Sense::HigherIsWorse, opt.worst_k),
            walk_inside: 0,
            on_asphalt: Dist::new(0.0, 64.0),
            on_asphalt_worst: Worst::new(Sense::HigherIsWorse, opt.worst_k),
            on_asphalt_dz: Dist::metres(),
            grade: Dist::new(0.0, 64.0),
            grade_worst: Worst::new(Sense::HigherIsWorse, opt.worst_k),
            grade_signed: Dist::metres(),
            rim: Dist::new(0.0, 64.0),
            rim_signed: Dist::metres(),
            rim_worst: Worst::new(Sense::HigherIsWorse, opt.worst_k),
            rim_walled: 0,
            fall: Dist::new(0.0, 8.0),
            fall_worst: Worst::new(Sense::HigherIsWorse, opt.worst_k),
            rim_by: [Dist::new(0.0, 64.0), Dist::new(0.0, 64.0)],
            fall_by: [Dist::new(0.0, 8.0), Dist::new(0.0, 8.0)],
            inside: 0,
            footprint_tiles: 0,
        }
    }

    /// Invariant 3 at the facade: drawn asphalt standing inside a building.
    ///
    /// The whole drawn at-grade road surface is walked, not only the part
    /// inside a footprint, and every sample outside one contributes a zero.
    /// That is deliberate and it is what makes the rate mean something: scored
    /// over the inside-samples alone the population would be nothing but the
    /// defect, and closing a site would remove samples rather than move the
    /// number — the trap `order.grade_stack`'s note names.
    fn visit_overlap(&mut self, tile: &TileScene, opt: &Options) {
        let prints = Footprints::build(tile);
        if !prints.is_empty() {
            self.footprint_tiles += 1;
        }
        // A tile with asphalt and no buildings still answers the question, with
        // a zero. Skipping it would quietly measure the rate over *streets near
        // buildings* instead of over the drawn street surface, which is 3 % of
        // the population here and would drift with the extract.
        for r in tile.roads.iter().filter(|r| is_road_band(r)) {
            let (mut dist, mut worst) = (Dist::new(0.0, 64.0), Worst::new(Sense::HigherIsWorse, opt.worst_k));
            let mut inside = 0u64;
            r.mesh.sample(&tile.scale, opt.spacing_m, |px, py, _| {
                if !tile.owns(px, py) {
                    return;
                }
                let hit = prints.depth(px, py, &tile.scale);
                let d = hit.unwrap_or(0.0);
                if hit.is_some() {
                    inside += 1;
                }
                dist.push(d);
                if d > FACADE_M {
                    let (lon, lat) = tile.lonlat(px, py);
                    worst.offer(Offender {
                        lon,
                        lat,
                        zoom: tile.z,
                        value: d,
                        note: format!(
                            "drawn {} stands {d:.1} m inside a building footprint",
                            r.class
                        ),
                    });
                }
            });
            self.overlap.merge(&dist);
            self.overlap_worst.merge(worst);
            self.inside += inside;
        }
        // The pedestrian population, over the same footprints and by the same
        // rule: every at-grade band sampled, zeros included.
        for r in tile.roads.iter().filter(|r| is_pedestrian_band(r)) {
            let (mut dist, mut worst) =
                (Dist::new(0.0, 64.0), Worst::new(Sense::HigherIsWorse, opt.worst_k));
            let mut inside = 0u64;
            r.mesh.sample(&tile.scale, opt.spacing_m, |px, py, _| {
                if !tile.owns(px, py) {
                    return;
                }
                let hit = prints.depth(px, py, &tile.scale);
                let d = hit.unwrap_or(0.0);
                if hit.is_some() {
                    inside += 1;
                }
                dist.push(d);
                if d > FACADE_M {
                    let (lon, lat) = tile.lonlat(px, py);
                    worst.offer(Offender {
                        lon,
                        lat,
                        zoom: tile.z,
                        value: d,
                        note: format!(
                            "drawn {} stands {d:.1} m inside a building footprint",
                            r.class
                        ),
                    });
                }
            });
            self.walk_indoors.merge(&dist);
            self.walk_indoors_worst.merge(worst);
            self.walk_inside += inside;
        }
    }

    /// Invariant 4 at the verge: how far the surface an attached pedestrian way
    /// stands on departs from the carriageway beside it.
    ///
    /// **Two populations, because a sidewalk stopped being a line.** From
    /// [`priors::WALK_SURFACE_MIN_ZOOM`] a pedestrian way is drawn as a
    /// `walk_surface` band and its cartographic stroke is deleted, so scoring
    /// the stroke there scores nothing — the metric read 0.000 % on a
    /// population that had emptied by 99.9 %, which is the failure mode
    /// VERIFICATION.md §10 warns about, arriving as a spectacular improvement.
    /// So the band is scored where one exists and the line where it does not,
    /// and both answer the same question: is the thing a pedestrian stands on
    /// part of the street's cross-section, or a metre off it?
    ///
    /// The band's population is the stricter of the two. A stroke had to pass
    /// [`WALK_COVER`] and the along-versus-across test to count, both of them
    /// archive-side re-derivations of the model's attachment rule; a band is
    /// there *because* the model attached it, so every sample counts.
    fn visit_grade(&mut self, tile: &TileScene, opt: &Options) {
        // Resampling every kerb in the tile is the expensive half, and most
        // tiles hold no pedestrian way at all — so ask that first.
        let has_band = tile.roads.iter().any(is_walk_band);
        if !has_band && !tile.lines.iter().any(|l| l.level == 0 && is_pedestrian(&l.class)) {
            return;
        }
        let kerbs = Kerbs::build(tile);
        if kerbs.is_empty() {
            return;
        }
        if has_band {
            self.visit_walk_band(tile, &kerbs, opt);
        }
        let bands: Vec<&RoadMesh> = tile.roads.iter().filter(|r| is_road_band(r)).collect();
        for line in tile.lines.iter().filter(|l| l.level == 0 && is_pedestrian(&l.class)) {
            for part in &line.parts {
                let pts = resample(part, &tile.scale);
                if pts.len() < 2 {
                    continue;
                }
                // Two ways of being with the street: beside its kerb, or on its
                // asphalt (a crossing, or a way the union has paved over).
                let near: Vec<Option<(f64, KerbPoint)>> =
                    pts.iter().map(|p| kerbs.nearest(p.0, p.1, &tile.scale)).collect();
                let on: Vec<bool> = pts
                    .iter()
                    .map(|p| bands.iter().any(|r| r.mesh.height_at(p.0, p.1).is_some()))
                    .collect();
                let with = near.iter().filter(|n| n.is_some()).count()
                    + on.iter().zip(near.iter()).filter(|(o, n)| **o && n.is_none()).count();
                if (with as f64) < WALK_COVER * pts.len() as f64 {
                    continue;
                }
                for i in 0..pts.len() {
                    let (px, py, _) = pts[i];
                    // A way over the asphalt is standing on the carriageway
                    // itself; there is no cross-section relation to measure.
                    if !tile.owns(px, py) || on[i] {
                        continue;
                    }
                    let Some((_, k)) = near[i] else { continue };
                    // Along, not across. The way's own direction against the
                    // kerb it stands beside.
                    let j = if i + 1 < pts.len() { i + 1 } else { i - 1 };
                    let (dx, dy) = (
                        (pts[j].0 - px) * tile.scale.mx,
                        (pts[j].1 - py) * tile.scale.my,
                    );
                    let len = (dx * dx + dy * dy).sqrt();
                    if len < 1e-9 || ((dx / len) * k.dir.0 + (dy / len) * k.dir.1).abs() <= WALK_ALONG
                    {
                        continue;
                    }
                    let Some(g) = walk_ground(tile, px, py) else { continue };
                    let departure = (g - k.z).abs();
                    // **Height disposes what plan proposed** (F3). A way a
                    // storey from a kerb behind a *closed* step is on another
                    // terrace — the wall between them is drawn, the world is
                    // whole (I9), and the pavement relation this check scores
                    // does not exist. Only visible air past the check's own
                    // bar keeps the pair in the population.
                    if departure > WALK_GRADE_M
                        && super::fabric::open_step(tile, px, py, g.min(k.z), g.max(k.z))
                            <= WALK_GRADE_M
                    {
                        continue;
                    }
                    self.grade.push(departure);
                    self.grade_signed.push(g - k.z);
                    if departure > WALK_GRADE_M {
                        let (lon, lat) = tile.lonlat(px, py);
                        let side = if g < k.z { "below" } else { "above" };
                        self.grade_worst.offer(Offender {
                            lon,
                            lat,
                            zoom: tile.z,
                            value: departure,
                            note: format!(
                                "{} stands {departure:.1} m {side} the carriageway it runs \
                                 alongside",
                                line.class
                            ),
                        });
                    }
                }
            }
        }
    }

    /// The band half of [`Self::visit_grade`]: every `walk_surface` sample
    /// against the kerb it stands beside.
    ///
    /// The band's *own* height is read from the mesh rather than through
    /// [`walk_ground`], because that is the number in question — a band drawn
    /// at the right plan position and the wrong height is precisely the defect,
    /// and a lookup that fell back to the terrain would hide it by measuring
    /// the ground the band was supposed to replace.
    ///
    /// A sample with no kerb within [`WALK_ATTACH_M`] is a path across open
    /// ground, which has no cross-section relation to be wrong about; it is
    /// out of the population rather than scored against nothing.
    fn visit_walk_band(&mut self, tile: &TileScene, kerbs: &Kerbs, opt: &Options) {
        let mut dist = Dist::new(0.0, 64.0);
        let mut signed = Dist::metres();
        let mut worst = Worst::new(Sense::HigherIsWorse, opt.worst_k);
        for r in tile.roads.iter().filter(|r| is_walk_band(r)) {
            r.mesh.sample(&tile.scale, opt.spacing_m, |px, py, z| {
                if !tile.owns(px, py) {
                    return;
                }
                let Some((d, k)) = kerbs.nearest(px, py, &tile.scale) else { return };
                // **The population is a kerb-anchored strip, not a class.** A
                // sidewalk starts at the carriageway edge and is at most
                // `WALK_WIDTH_M` wide (`street.kerb_join` reads 0.00 %), so
                // every sample of one is within a band-width of a kerb. A path
                // that merely passes a road is not, and scoring it against that
                // road measures a relation that does not exist — the worst site
                // in the extract was a footpath 17.7 m up a slope.
                //
                // Reading that from the *geometry* rather than from the drawn
                // material is what lets the material be merged: `drawn_material`
                // makes a path and the sidewalk it continues one region so they
                // share one rim, and a class test would then have quietly
                // started scoring hillsides. Measured with the merge withheld,
                // the tightening alone moves this metric by a hair.
                if d > STRIP_REACH_M {
                    return;
                }
                let departure = (z - k.z).abs();
                // **Height disposes what plan proposed** (F3). A walkway a
                // storey from a kerb behind a *closed* step is on another
                // terrace — the wall between them is drawn, the world is
                // whole (I9), and the pavement relation this check scores
                // does not exist: the wall-foot paths of Montreux, and the
                // lower walk sheets the sheet assignment now draws honestly
                // instead of blending upward, are real configurations, not
                // strips off their kerb. Only visible air past the check's
                // own bar keeps the pair in the population; a strip genuinely
                // hanging beside its street is exactly that air.
                if departure > WALK_GRADE_M
                    && super::fabric::open_step(tile, px, py, z.min(k.z), z.max(k.z))
                        <= WALK_GRADE_M
                {
                    return;
                }
                dist.push(departure);
                signed.push(z - k.z);
                if departure > WALK_GRADE_M {
                    let (lon, lat) = tile.lonlat(px, py);
                    let side = if z < k.z { "below" } else { "above" };
                    worst.offer(Offender {
                        lon,
                        lat,
                        zoom: tile.z,
                        value: departure,
                        note: format!(
                            "the walkway stands {departure:.1} m {side} the carriageway it \
                             runs alongside"
                        ),
                    });
                }
            });
        }
        self.grade.merge(&dist);
        self.grade_signed.merge(&signed);
        self.grade_worst.merge(worst);
    }

    /// Invariant 3 on the street's own plan space: a drawn pedestrian band
    /// standing **on** the at-grade carriageway rather than beside it.
    ///
    /// `contact.sidewalk_grade` deliberately scores a way only *beside* the
    /// asphalt, and every height check tolerates the kerb rise — so a band
    /// lying on the plate, 0.12 m above it, is invisible to all of them while
    /// being the most visible defect on screen: the junction diced into blobs
    /// by pavements crossing it. The band and the asphalt genuinely share plan
    /// space; only the plan can say so.
    ///
    /// The value is the plan depth onto the carriageway — the distance to the
    /// nearest at-grade road silhouette point — wherever the band stands
    /// within [`ON_ASPHALT_DZ_M`] of the asphalt under it, zero everywhere
    /// else. Zeros are scored over the whole pedestrian surface for the same
    /// reason `order.building_overlap` scores them: over the on-samples alone
    /// the population would be nothing but the defect. A footbridge over the
    /// street and a rim path above a sunken road fail the height test and
    /// score zero; they are stacks the model draws on purpose.
    fn visit_on_asphalt(&mut self, tile: &TileScene, opt: &Options) {
        if !tile.roads.iter().any(is_pedestrian_band) {
            return;
        }
        let has_asphalt = tile.roads.iter().any(|r| is_road_band(r));
        // The silhouette is only resampled when there is asphalt to stand on;
        // a band tile without any still contributes its zeros below.
        let kerbs = if has_asphalt { Some(Kerbs::build(tile)) } else { None };
        let mut dist = Dist::new(0.0, 64.0);
        let mut dz_dist = Dist::metres();
        let mut worst = Worst::new(Sense::HigherIsWorse, opt.worst_k);
        for r in tile.roads.iter().filter(|r| is_pedestrian_band(r)) {
            r.mesh.sample(&tile.scale, opt.spacing_m, |px, py, z| {
                if !tile.owns(px, py) {
                    return;
                }
                // The asphalt nearest the band in *height*, where surface and
                // rim overlap the same plan point.
                let mut dz: Option<f64> = None;
                if has_asphalt {
                    for a in tile.roads.iter().filter(|r| is_road_band(r)) {
                        if let Some(h) = a.mesh.height_at(px, py) {
                            let d = z - h;
                            if dz.is_none_or(|b: f64| d.abs() < b.abs()) {
                                dz = Some(d);
                            }
                        }
                    }
                }
                let dz = dz.filter(|d| d.abs() <= ON_ASPHALT_DZ_M);
                let d = match (dz, &kerbs) {
                    // On the plate, no silhouette within reach: deeper than
                    // the index can answer, so the reach itself is the floor.
                    (Some(_), Some(k)) => k
                        .nearest(px, py, &tile.scale)
                        .map_or(priors::WALK_ATTACH_M, |(d, _)| d),
                    _ => 0.0,
                };
                dist.push(d);
                if let Some(dz) = dz {
                    dz_dist.push(dz);
                }
                if d > ON_ASPHALT_EDGE_M {
                    let (lon, lat) = tile.lonlat(px, py);
                    worst.offer(Offender {
                        lon,
                        lat,
                        zoom: tile.z,
                        value: d,
                        note: format!(
                            "the {} stands {d:.1} m onto the carriageway \
                             ({:+.2} m off its surface)",
                            r.class,
                            dz.unwrap_or(0.0)
                        ),
                    });
                }
            });
        }
        self.on_asphalt.merge(&dist);
        self.on_asphalt_dz.merge(&dz_dist);
        self.on_asphalt_worst.merge(worst);
    }
}

/// The drawn **sidewalk**: the band attached to a street, and its rim,
/// at grade.
///
/// `path_surface` is deliberately out. A path across a hillside stands on the
/// ground and belongs to no street, so measuring it against the nearest kerb
/// within [`WALK_ATTACH_M`] scores a relation that does not exist — the worst
/// site in the extract was a footpath 17.7 m up a slope from a road it passed
/// near. The two materials are separate for this reason (`priors::Surface`).
fn is_walk_band(r: &RoadMesh) -> bool {
    r.level == 0 && matches!(r.class.as_str(), "walk_surface" | "walk_rim")
}

/// Every drawn pedestrian surface at grade — sidewalk *and* path, interior band
/// and rim.
///
/// Wider than [`is_walk_band`] on purpose, and the difference is the whole
/// reason the two materials exist. `contact.sidewalk_grade` asks how a band
/// relates to the street beside it, which a path across a hillside has no
/// answer to. The ground under a band is a question every pedestrian surface
/// answers, and the path is where it is most often answered badly.
fn is_pedestrian_band(r: &RoadMesh) -> bool {
    r.level == 0
        && matches!(
            r.class.as_str(),
            "walk_surface" | "walk_rim" | "path_surface" | "path_rim"
        )
}

/// Sorted quantiles of one population, for `ARPT_DEBUG_STREET`. VERIFICATION.md
/// §10: before believing a new check's first number, histogram what it scores
/// and look for a second mode.
fn report_population(label: &str, d: &Dist) {
    let q = |p: f64| d.quantile(p).unwrap_or(f64::NAN);
    eprintln!(
        "[street] {label}: n={} p01 {:.2} p05 {:.2} p25 {:.2} p50 {:.2} p75 {:.2} \
         p90 {:.2} p95 {:.2} p99 {:.2} p999 {:.2} min {:.2} max {:.2}",
        d.count(),
        q(0.01),
        q(0.05),
        q(0.25),
        q(0.50),
        q(0.75),
        q(0.90),
        q(0.95),
        q(0.99),
        q(0.999),
        d.min().unwrap_or(f64::NAN),
        d.max().unwrap_or(f64::NAN),
    );
}

/// How far from a kerb a drawn pedestrian sample can be and still be part of
/// *that street's* cross-section, in metres.
///
/// A strip is seated with its inner edge on the carriageway edge and allotted
/// at most [`priors::WALK_WIDTH_M`], so its far edge is one band-width out; the
/// slack covers the profile-smoothing displacement between the drawn kerb and
/// the centerline the strip was offset from. This replaced a test on the drawn
/// *class*, which stopped being able to tell a sidewalk from a path once the
/// two became one drawn region.
const STRIP_REACH_M: f64 = priors::WALK_WIDTH_M + 1.0;

/// A line part resampled at [`WALK_STEP_M`], so a long straight run is not one
/// sample and a densely digitised curve is not a thousand.
fn resample(part: &[(f64, f64, f64)], scale: &Scale) -> Vec<(f64, f64, f64)> {
    let mut out = Vec::new();
    for w in part.windows(2) {
        let (a, b) = (w[0], w[1]);
        let len = scale.dist(a.0, a.1, b.0, b.1);
        let steps = (len / WALK_STEP_M).ceil().max(1.0) as usize;
        for k in 0..steps {
            let t = k as f64 / steps as f64;
            out.push((a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t, a.2 + (b.2 - a.2) * t));
        }
    }
    if let Some(&last) = part.last() {
        out.push(last);
    }
    out
}

impl Street {
    /// Invariant 1 under a pedestrian band: whether the ground beneath it is
    /// *its* ground.
    ///
    /// A carriageway benches the ground it stands on and the terrain mesh then
    /// stops at its kerb, so the two surfaces meet along one rim at one height.
    /// A pedestrian band takes the same hole out of the same terrain and has no
    /// bench under it at all: it is seated on the host's cross-section (a
    /// sidewalk) or draped on whatever the DEM does (a path). Two consequences,
    /// and this walk measures both.
    ///
    /// - [`contact.walk_rim`] — the step where the band meets the ground. Read
    ///   at the terrain's own hole rim rather than a metre outside it, as
    ///   `contact.kerb_lip` reads a carriageway's: a metre out lands on the
    ///   batter face, which is a legitimate slope, and the question here is
    ///   whether the *joint* holds. Anchoring on the rim also survives the trap
    ///   phase 5 fell into — a probe that reads the terrain where the band's own
    ///   hole removed it does not measure a better world, it measures a smaller
    ///   population.
    /// - [`slope.walk_crossfall`] — how far the band tilts across its own
    ///   width. A sidewalk is flat by construction (its height is the host's
    ///   road surface plus a kerb); a path is the hillside, because the drawn
    ///   ground *is* what it stands on. The metric is what separates the two
    ///   claims, and what a bench under the band would move.
    fn visit_walk_ground(&mut self, tile: &TileScene) {
        let bands: Vec<&RoadMesh> =
            tile.roads.iter().filter(|r| is_pedestrian_band(r)).collect();
        if bands.is_empty() {
            return;
        }
        // The band over a rim point, taken as the one **closest in height** to
        // the rim rather than the first found. Two bands can cover one plan
        // point at grade — a sidewalk on a terrace with a path along the foot
        // of its wall, ten metres below — and reading the far one would score
        // the terrace's whole height as a joint that failed. Closest is also
        // the conservative choice: where a stack is a genuine defect it is
        // `order.at_grade_overlap`'s to report, not this one's.
        let band_at = |px: f64, py: f64, near: f64| -> Option<(f64, usize)> {
            bands
                .iter()
                .filter_map(|r| {
                    r.mesh
                        .height_at(px, py)
                        .map(|h| (h, usize::from(r.class.starts_with("path_"))))
                })
                .min_by(|a, b| (a.0 - near).abs().total_cmp(&(b.0 - near).abs()))
        };

        // The rim: every hole-rim edge of the drawn terrain that a pedestrian
        // band covers. The tile's own edge is not a rim — the neighbour's
        // terrain continues across it — and neither is an edge with no band
        // over it, which is a carriageway's rim and `contact.kerb_*`'s to score.
        if let Some(terrain) = &tile.terrain {
            for (a, b, _) in terrain.boundary_edges() {
                let (ax, ay, az) = terrain.vertex(a);
                let (bx, by, bz) = terrain.vertex(b);
                let (mx, my) = ((ax + bx) * 0.5, (ay + by) * 0.5);
                if !tile.owns(mx, my) {
                    continue;
                }
                let on_edge = |v: f64| v.abs() < 1e-6 || (v - 1.0).abs() < 1e-6;
                if on_edge(mx) || on_edge(my) {
                    continue;
                }
                let rim_z = (az + bz) * 0.5;
                let Some((band_z, material)) = band_at(mx, my, rim_z) else { continue };
                let step = band_z - rim_z;
                // A rim a drawn wall closes is a joint that holds. The drape
                // guard refuses an over-tall bench and draws an apron down the
                // face instead (`synth::walkway`), so band-at-the-wall-foot,
                // terrain-at-its-top is that refusal drawn correctly — the
                // same exemption `contact.kerb_unwalled` grants a carriageway,
                // and any apron modality counts, because the wall a walkway
                // runs under is as often the street's as its own.
                let (lo_z, hi_z) = (band_z.min(rim_z), band_z.max(rim_z));
                let walled =
                    step.abs() > WALK_RIM_M && super::apron_spans(tile, mx, my, lo_z, hi_z);
                if walled {
                    self.rim_walled += 1;
                    self.rim.push(0.0);
                    self.rim_signed.push(0.0);
                    self.rim_by[material].push(0.0);
                    continue;
                }
                self.rim.push(step.abs());
                self.rim_signed.push(step);
                self.rim_by[material].push(step.abs());
                if step.abs() > WALK_RIM_M {
                    let (lon, lat) = tile.lonlat(mx, my);
                    // ARPT_DEBUG_RIM: every violation, for the anatomy.
                    if std::env::var_os("ARPT_DEBUG_RIM").is_some() {
                        eprintln!(
                            "[rim] {lon:.6},{lat:.6} {} step {step:+.3} band {band_z:.2} ground {rim_z:.2}",
                            if material == 0 { "walk" } else { "path" }
                        );
                    }
                    let side = if step > 0.0 { "above" } else { "below" };
                    self.rim_worst.offer(Offender {
                        lon,
                        lat,
                        zoom: tile.z,
                        value: step.abs(),
                        note: format!(
                            "the band stands {:.2} m {side} the ground at its own rim \
                             (band {band_z:.2} m, ground {rim_z:.2} m)",
                            step.abs()
                        ),
                    });
                }
            }
        }

        // The cross-fall: at every side edge of a band, the band's own height
        // against its height a metre in.
        //
        // Read on **one mesh**, the interior band's — never across meshes. A
        // rim ring is a third of a metre wide, so a metre inward from its
        // edge is already on a different mesh, and where two bands stack in
        // plan it can be a different band on a different terrace: the first cut
        // of this walk scored a sidewalk at 623 m against a path 9.6 m below it
        // and called the pair a 963 % cross-fall. The interior mesh's own
        // silhouette is inset from the true edge by the rim's width, which
        // costs nothing here — the question is the band's tilt, not where
        // exactly it stops.
        for r in bands.iter().filter(|r| r.class.ends_with("_surface")) {
            for (a, b, opp) in r.mesh.boundary_edges() {
                let (ax, ay, az) = r.mesh.vertex(a);
                let (bx, by, bz) = r.mesh.vertex(b);
                let (ox, oy, _) = r.mesh.vertex(opp);
                let (mx, my) = ((ax + bx) * 0.5, (ay + by) * 0.5);
                if !tile.owns(mx, my) {
                    continue;
                }
                // **Across is the edge's own perpendicular**, turned toward the
                // triangle that holds it — not the direction of that triangle's
                // far vertex, which is only perpendicular when the triangle
                // happens to be small. The band is meshed over the terrain
                // lattice, so it usually is; a band drawn as two big triangles
                // is not, and the reading would then run along the way and
                // report its grade as a cross-fall.
                let (ex, ey) = ((bx - ax) * tile.scale.mx, (by - ay) * tile.scale.my);
                let elen = ex.hypot(ey);
                if elen <= 0.0 {
                    continue;
                }
                let (nx, ny) = (-ey / elen, ex / elen);
                let inward = ((ox - mx) * tile.scale.mx * nx + (oy - my) * tile.scale.my * ny)
                    .signum();
                // Back into plan space, where the probe points live.
                let (dx, dy) =
                    (inward * nx / tile.scale.mx, inward * ny / tile.scale.my);
                let at = |m: f64| (mx + dx * m, my + dy * m);
                let (fx, fy) = at(WALK_FALL_OUT_M);
                // An end edge, or a plaza: inward is not across (see
                // [`WALK_FALL_OUT_M`]). Asked before the ladder, so a wide band
                // is rejected for the same reason at any baseline.
                if r.mesh.height_at(fx, fy).is_some() {
                    continue;
                }
                // The longest baseline this band offers (see
                // [`WALK_FALL_LADDER_M`]) — a fixed metre reads nothing at all
                // on a band under 1.70 m.
                let Some((run, in_z)) = WALK_FALL_LADDER_M.iter().find_map(|&m| {
                    let (ix, iy) = at(m);
                    r.mesh.height_at(ix, iy).map(|z| (m, z))
                }) else {
                    continue;
                };
                let edge_z = (az + bz) * 0.5;
                let fall = (in_z - edge_z).abs() / run;
                self.fall.push(fall);
                self.fall_by[usize::from(r.class.starts_with("path_"))].push(fall);
                if fall > WALK_FALL {
                    let (lon, lat) = tile.lonlat(mx, my);
                    // ARPT_DEBUG_FALL: every violation, for the anatomy.
                    if std::env::var_os("ARPT_DEBUG_FALL").is_some() {
                        eprintln!(
                            "[fall] {lon:.6},{lat:.6} {} fall {fall:.3} edge {edge_z:.2} in {in_z:.2} run {run:.2} elen {elen:.2}",
                            r.class
                        );
                    }
                    self.fall_worst.offer(Offender {
                        lon,
                        lat,
                        zoom: tile.z,
                        value: fall,
                        note: format!(
                            "the {} falls {:.0} % across its own width ({edge_z:.2} m at the \
                             edge, {in_z:.2} m {run:.2} m in)",
                            r.class,
                            fall * 100.0
                        ),
                    });
                }
            }
        }
    }
}

impl Check for Street {
    fn visit(&mut self, tile: &TileScene, opt: &Options) {
        self.visit_overlap(tile, opt);
        self.visit_grade(tile, opt);
        self.visit_on_asphalt(tile, opt);
        self.visit_walk_ground(tile);
    }

    fn finish(self: Box<Self>) -> Vec<Metric> {
        if std::env::var("ARPT_DEBUG_STREET").is_ok() {
            report_population("order.building_overlap depth", &self.overlap);
            eprintln!(
                "  footprint tiles {}, samples inside a footprint {} ({:.3} %)",
                self.footprint_tiles,
                self.inside,
                100.0 * self.inside as f64 / self.overlap.count().max(1) as f64
            );
            for t in [0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 5.0] {
                eprintln!("    depth > {t:.2} m: {:.4} %", 100.0 - self.overlap.pct_below(t));
            }
            report_population("contact.sidewalk_grade |departure|", &self.grade);
            report_population("contact.sidewalk_grade signed", &self.grade_signed);
            eprintln!(
                "  below the kerb {:.2} %, above it {:.2} %",
                self.grade_signed.pct_below(0.0),
                100.0 - self.grade_signed.pct_below(0.0)
            );
            for t in [0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 2.0, 3.0, 5.0] {
                eprintln!("    |departure| > {t:.2} m: {:.4} %", 100.0 - self.grade.pct_below(t));
            }
            report_population("order.walk_on_asphalt depth", &self.on_asphalt);
            report_population("  on-asphalt dz (coincident only)", &self.on_asphalt_dz);
            for t in [0.25, 0.5, 0.75, 1.0, 2.0, 4.0, 6.0] {
                eprintln!(
                    "    depth > {t:.2} m: {:.4} %",
                    100.0 - self.on_asphalt.pct_below(t)
                );
            }
            eprintln!("[street] contact.walk_rim walled rims scored 0: {}", self.rim_walled);
            report_population("contact.walk_rim |step|", &self.rim);
            report_population("contact.walk_rim signed", &self.rim_signed);
            eprintln!(
                "  band below the ground {:.2} %, above it {:.2} %",
                self.rim_signed.pct_below(0.0),
                100.0 - self.rim_signed.pct_below(0.0)
            );
            for t in [0.05, 0.1, 0.25, 0.5, 1.0, 2.0, 4.0] {
                eprintln!("    |step| > {t:.2} m: {:.4} %", 100.0 - self.rim.pct_below(t));
            }
            report_population("  walk_rim sidewalk", &self.rim_by[0]);
            report_population("  walk_rim path", &self.rim_by[1]);
            report_population("slope.walk_crossfall", &self.fall);
            for t in [0.02, 0.05, 0.1, 0.2, 0.4, 0.8] {
                eprintln!("    fall > {:.0} %: {:.4} %", t * 100.0, 100.0 - self.fall.pct_below(t));
            }
            report_population("  crossfall sidewalk", &self.fall_by[0]);
            report_population("  crossfall path", &self.fall_by[1]);
        }
        vec![
            Metric {
                id: "order.building_overlap".into(),
                invariant: Invariant::I3,
                title: "Drawn at-grade asphalt standing inside a building footprint".into(),
                population: "Every drawn at-grade road surface sample (road_surface and its \
                             road_rim strip) the tile owns, scored by how far inside a \
                             building footprint it stands, and zero everywhere it is outside \
                             one. Scoring the zeros is what makes the rate mean something: \
                             over the inside-samples alone the population would be nothing but \
                             the defect, and closing a site would remove samples rather than \
                             move the number. Over the Montreux extract at z16, 22.3 M samples \
                             from 621 tiles of which 338 carry a footprint: 320,828 samples \
                             (1.44 %) lie inside one, and by uniform raster that is 28,719 m² \
                             over 1,662 of 8,615 buildings — 19.3 % of them, 1.0 % of the \
                             drawn at-grade road surface. Levels order themselves out: a \
                             bridge or a bore is not level 0. Three limits are not. Rail bands \
                             are excluded, because a station roof over its platforms is a \
                             level relation the archive cannot state (2,772 m² here). An \
                             `is_covered` arcade at grade cannot be told from a defect — no \
                             level rule fires on the flag and the archive carries no property \
                             for it — which in plan space is 3 pairs and 174 m², about 1.5 % \
                             of the at-grade overlap. And a footprint carries its own survey \
                             error, which is what the threshold is set against."
                    .into(),
                detail: "A road's width is a class prior and a footprint is surveyed, and \
                         nothing reconciles them: the band is laid at the prior's half-width \
                         whatever is standing there. On screen it is asphalt flush into a wall \
                         with no verge and no sidewalk. Two families, and the depth is what \
                         separates them — the shallow one is a band overrunning a facade, \
                         which a width cap removes (at Rue du Marché, 6.9130,46.4336, the \
                         whole tile reads 8.3 % inside with a worst of 3.9 m and nothing past \
                         5 m); the deep one is a way whose *centerline* is inside a footprint, \
                         which no cap can move (the worst here is the Casino Barrière's \
                         7,533 m² footprint with an unknown-class way through it). The \
                         plan-space estimate of the same defect — centerlines buffered to the \
                         tiler's own half-widths — reads 11,937 m², less than half of this, \
                         because it knows nothing of the junction plates, the rim and \
                         the links the union dissolves into one region. This measures what is \
                         drawn."
                    .into(),
                sense: Sense::HigherIsWorse,
                threshold: FACADE_M,
                skipped: (self.footprint_tiles == 0)
                    .then(|| "no building footprint over drawn at-grade road surface".to_string()),
                dist: self.overlap,
                worst: self.overlap_worst.into_vec(),
            },
            Metric {
                id: "order.walk_indoors".into(),
                invariant: Invariant::I3,
                title: "Drawn at-grade pedestrian surface standing inside a building".into(),
                population: "Every drawn at-grade pedestrian surface sample (walk_surface, \
                             path_surface and their rims) the tile owns, scored by how far \
                             inside a building footprint it stands, and zero everywhere it is \
                             outside one — the same footprints and the same zero-scoring rule \
                             as `order.building_overlap`, over the other half of the drawn \
                             surface. Split from it because the two are different defects: a \
                             carriageway inside a footprint is a class width prior meeting a \
                             survey error at the facade, while a walkway inside one is a way \
                             that is *indoors*."
                    .into(),
                detail: "Overture flags a way `is_indoor` or `is_covered` in `road_flags`, and \
                         `levels::parse_flags` reads neither — only `is_bridge` and \
                         `is_tunnel` — so an indoor way is an ordinary level-0 footway to \
                         every stage after it. It earns a walk band, benches stratum D and is \
                         drawn as pavement, which inside a terminal or a shopping centre is a \
                         walkway cut through the building's floor. The Swiss extract holds \
                         3,332 covered and 1,631 indoor footways plus 871 covered and 771 \
                         indoor steps; they cluster hard, 545 indoor ways and 19 km of them in \
                         one 0.02° cell over Zurich Airport, against 0.07 km in the whole \
                         Montreux zone — which is why this reads near zero there and is worth \
                         measuring anyway. 96.7 % of the airport's indoor ways carry no level \
                         rule, so nothing else orders them out. An arcade is the honest \
                         ambiguity in the population, the same one `order.building_overlap` \
                         names: `is_covered` at grade under its own building is real pavement, \
                         and no property in the archive tells it from a way through a wall."
                    .into(),
                sense: Sense::HigherIsWorse,
                threshold: FACADE_M,
                skipped: (self.footprint_tiles == 0)
                    .then(|| "no building footprint over drawn at-grade pedestrian surface".to_string()),
                dist: self.walk_indoors,
                worst: self.walk_indoors_worst.into_vec(),
            },
            Metric {
                id: "order.walk_on_asphalt".into(),
                invariant: Invariant::I3,
                title: "Drawn pedestrian surface standing on the carriageway".into(),
                population: "Every drawn at-grade pedestrian surface sample (walk_surface, \
                             path_surface and their rims) the tile owns, scored by how deep \
                             onto the at-grade carriageway it stands — the plan distance to \
                             the nearest road silhouette point, capped at the attachment \
                             reach — wherever the band lies within [`priors::WALK_ON_ASPHALT_M`] \
                             of the asphalt sharing its plan position, and zero everywhere \
                             else. The zeros are scored for the same reason \
                             `order.building_overlap` scores them: over the on-samples alone \
                             the population would be nothing but the defect. Over the \
                             Montreux extract at z16, 15.0 M samples, of which 1.05 M were \
                             coincident before the model-side yield and their dz median was \
                             +0.11 m — the kerb rise itself, which is the signature this \
                             check exists to catch. The height gate keeps the legitimate \
                             stacks out: a footbridge over the street and a rim path above a \
                             sunken road both fail it and score zero."
                    .into(),
                detail: "By construction no pedestrian band belongs on the plate: crossings \
                         are drawn as zebra paint on the asphalt plus kerb stubs, never as a \
                         band across it, so everything this measures is a band whose plan \
                         space the carriageway owns. The mechanism it was built against: \
                         `synth::pavement::bake_chunk` trimmed Walkway under Asphalt only \
                         within one (level, layer) key, and the walk-sheet assignment gave \
                         walk bands their own layer namespace — where the road sheet number \
                         differed the trim went silently dead, and synthesized pavements \
                         crisscrossed the junction 0.12 m above it, dicing it into blobs \
                         (Montreux, Rue du Marché / Grand-Rue, 6.9113,46.4324; 3.80 % of \
                         the drawn pedestrian surface zone-wide). No height check can see \
                         this: the band rides at exactly the kerb rise every contact \
                         tolerance forgives. The kerb-coincident yield in \
                         `synth::pavement::trench_yields` — segment cuts plus the \
                         intersection's own extent for the fillet space the closing adds — \
                         is the model-side answer, and 0.43 % remains: mostly one family, \
                         bands standing slightly below the asphalt drawn over them."
                    .into(),
                sense: Sense::HigherIsWorse,
                threshold: ON_ASPHALT_EDGE_M,
                skipped: (self.on_asphalt.count() == 0)
                    .then(|| "no drawn at-grade pedestrian surface".to_string()),
                dist: self.on_asphalt,
                worst: self.on_asphalt_worst.into_vec(),
            },
            Metric {
                id: "contact.sidewalk_grade".into(),
                invariant: Invariant::I4,
                title: "A pedestrian way's surface departing from the street beside it".into(),
                population: "Every metre along every level-0 pedestrian centerline (footway, \
                             path, cycleway, pedestrian, bridleway — not steps, whose purpose \
                             is to change height) whose part runs with a street for at least \
                             80 % of its length and which is locally parallel to the kerb \
                             within 30°, taken where it is beside the asphalt rather than over \
                             it. The value is |drawn surface under the way − drawn carriageway \
                             at the nearest kerb point|, unsigned because a way stranded on a \
                             bank above its street is the same defect as one in a ditch below \
                             it; the offender note carries the side. Over the Montreux extract \
                             at z16 that is 30,366 samples out of 421 k pedestrian samples — \
                             12 % of the rest are drawn over the asphalt and the remainder do \
                             not run with a street. The population is very nearly symmetric, \
                             44.9 % below the kerb and 55.1 % above it, which is the whole \
                             point of the unsigned value: the cutting side and the fill side \
                             of one missing cross-section. Signed it reads p05 −0.55 m, p50 \
                             +0.01 m, p95 +0.66 m, from 10.19 m below to 7.84 m above. Two \
                             coverage limits. Attachment is geometric only, because the \
                             archive carries a way's class but never the `subclass='sidewalk'` \
                             tag, so the third of tagged sidewalks that fail a geometric test \
                             are outside this population altogether. And a pair whose step the \
                             drawn world *closes* is out of the population (F3: height \
                             disposes what plan proposed): a way a storey from a kerb behind a \
                             drawn wall — the wall-foot paths of Montreux, a lower walk sheet \
                             beside a terrace street — is another terrace, measured by the \
                             closure question `fabric.closure` asks (open air past the \
                             check's own 1 m bar keeps the pair in; a closed face excludes \
                             it). What remains charged is a pedestrian surface with visible \
                             air between it and the kerb it runs beside, which is the strip \
                             genuinely hanging off its street."
                    .into(),
                detail: "A street's bench reaches its half-width plus a shoulder and a margin \
                         and stops, so a pedestrian way outside that band drapes on whatever \
                         the hillside does — at the measured median offset that is a sidewalk \
                         standing over a metre below the street it borders, with the drawn \
                         ground falling away from the kerb within about three metres. What it \
                         should read once the cross-section is allocated out of the room the \
                         buildings leave is a kerb rise."
                    .into(),
                sense: Sense::HigherIsWorse,
                threshold: WALK_GRADE_M,
                skipped: self
                    .grade
                    .is_empty()
                    .then(|| "no pedestrian way running alongside a drawn carriageway".to_string()),
                dist: self.grade,
                worst: self.grade_worst.into_vec(),
            },
            Metric {
                id: "contact.walk_rim".into(),
                invariant: Invariant::I1,
                title: "Step where a pedestrian band meets the ground at its own rim".into(),
                population: "Every terrain-mesh boundary edge midpoint the tile owns that is \
                             not on the tile's own edge and has a level-0 pedestrian surface \
                             over it (`walk_surface`/`walk_rim` and \
                             `path_surface`/`path_rim` alike). The value is |band − rim|, \
                             unsigned: the ground standing above a band is the same missing \
                             earthwork as the ground falling away from it, and the offender \
                             note carries the side. Read at the rim rather than a metre \
                             outside it, which is where `contact.kerb_lip` reads a \
                             carriageway's: a metre out lands on the batter face, whose slope \
                             is legitimate, so it cannot separate a joint that holds from one \
                             that does not. A rim a drawn apron wall spans — top at the \
                             ground, foot at the band, any modality — scores 0: the drape \
                             guard refuses an over-tall bench and draws the wall instead \
                             (`synth::walkway`), so band-at-the-wall-foot is that refusal \
                             drawn correctly, the same exemption `contact.kerb_unwalled` \
                             grants a carriageway; before it, the whole of this metric's tail \
                             above a metre was 932 walled joints, worst 3.52 m at a terrace \
                             wall its walkway runs under. A rim with no band over it is a \
                             carriageway's and \
                             is `contact.kerb_unwalled`'s to score; the two populations do not \
                             overlap."
                    .into(),
                detail: "The mesher cuts one hole for the whole unioned surface and the band \
                         is part of it, so the band and the drawn ground meet along this rim \
                         by construction — nothing but quantization stands between them. What \
                         puts a step there is that nothing benched the ground for the band: a \
                         sidewalk is seated on its host's cross-section a kerb above the \
                         carriageway while the ground under it is still the street's bench \
                         (which stops a verge past the asphalt) or the batter face beyond it, \
                         and a path is drawn on the drawn ground with no earthwork of its own \
                         at all. Where the step is a wall the `walk_apron` draws it, which is \
                         honest and is not the same as the wall not being there."
                    .into(),
                sense: Sense::HigherIsWorse,
                threshold: WALK_RIM_M,
                skipped: self
                    .rim
                    .is_empty()
                    .then(|| "no drawn pedestrian band at this zoom (below \
                              WALK_SURFACE_MIN_ZOOM, or the archive carries no DEM)".to_string()),
                dist: self.rim,
                worst: self.rim_worst.into_vec(),
            },
            Metric {
                id: "slope.walk_crossfall".into(),
                invariant: Invariant::I1,
                title: "Cross-fall of a drawn pedestrian band".into(),
                population: "Every side-edge midpoint of every level-0 pedestrian band the \
                             tile owns: the band's height there against its height a metre \
                             inward, as rise per metre. An edge that still has band under it \
                             three metres inward is an end (where inward is along the way, so \
                             the reading would be its longitudinal grade) or a plaza, and is \
                             dropped — a band is two metres wide, so a side edge always \
                             leaves it inside three."
                    .into(),
                detail: "A sidewalk's height is its host's road surface plus a kerb, so it is \
                         flat across by construction whatever the hillside does. A path's is \
                         the drawn ground, so it carries the full cross-slope of whatever it \
                         crosses — a two-metre ribbon tilted at the angle of the hill, which \
                         is neither what a footpath is nor what one looks like. The fix is the \
                         same one a road has: bench the ground under the band and let the band \
                         read it."
                    .into(),
                sense: Sense::HigherIsWorse,
                threshold: WALK_FALL,
                skipped: self
                    .fall
                    .is_empty()
                    .then(|| "no drawn pedestrian band at this zoom (below \
                              WALK_SURFACE_MIN_ZOOM)".to_string()),
                dist: self.fall,
                worst: self.fall_worst.into_vec(),
            },
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::project::Bounds;
    use crate::verify::scene::RoadLine;

    const OVERLAP: &str = "order.building_overlap";
    const GRADE: &str = "contact.sidewalk_grade";
    const ON_ASPHALT: &str = "order.walk_on_asphalt";

    /// A tile, with everything laid out in metres from its south-west corner.
    ///
    /// Unit plan space is 2:1 in degrees and shortens with latitude, so writing
    /// test geometry in it would make every distance in these tests a different
    /// number of metres on each axis — and these checks are entirely about
    /// distances in metres.
    struct Site {
        bounds: Bounds,
        scale: Scale,
    }

    impl Site {
        fn new() -> Site {
            let bounds = Bounds::of_tile(16, 34000, 23000);
            Site { scale: Scale::of(&bounds), bounds }
        }

        fn ux(&self, m: f64) -> f32 {
            (m / self.scale.mx) as f32
        }

        fn uy(&self, m: f64) -> f32 {
            (m / self.scale.my) as f32
        }

        /// A flat rectangle in plan at a constant height.
        fn slab(&self, x0: f64, x1: f64, y0: f64, y1: f64, z: f64) -> SurfaceMesh {
            SurfaceMesh::from_parts(
                vec![self.ux(x0), self.ux(x1), self.ux(x1), self.ux(x0)],
                vec![self.uy(y0), self.uy(y0), self.uy(y1), self.uy(y1)],
                vec![z as f32; 4],
                vec![0, 1, 2, 0, 2, 3],
            )
            .unwrap()
        }

        /// A flat band over the plan rectangle `(x0, x1, y0, y1)`, in metres.
        fn band(&self, class: &str, level: i64, r: (f64, f64, f64, f64), z: f64) -> RoadMesh {
            RoadMesh {
                class: class.into(),
                level,
                band: String::new(), fades: false, sheet: None,
                mesh: self.slab(r.0, r.1, r.2, r.3, z),
            }
        }

        /// A box building: four vertical wall quads standing on the footprint
        /// outline, plus a flat roof cap. The walls are what `wall_segments`
        /// recovers the outline from, so a test that built only a cap would be
        /// testing nothing.
        fn building(&self, x0: f64, x1: f64, y0: f64, y1: f64, foot: f64, top: f64) -> SurfaceMesh {
            let (xs, ys) = ([x0, x1, x1, x0], [y0, y0, y1, y1]);
            let (mut vx, mut vy, mut vz, mut idx) = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
            for k in 0..4 {
                let n = (k + 1) % 4;
                let base = vx.len() as u32;
                for &(cx, cy, cz) in &[
                    (xs[k], ys[k], foot),
                    (xs[n], ys[n], foot),
                    (xs[n], ys[n], top),
                    (xs[k], ys[k], top),
                ] {
                    vx.push(self.ux(cx));
                    vy.push(self.uy(cy));
                    vz.push(cz as f32);
                }
                idx.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
            }
            let cap = vx.len() as u32;
            for k in 0..4 {
                vx.push(self.ux(xs[k]));
                vy.push(self.uy(ys[k]));
                vz.push(top as f32);
            }
            idx.extend_from_slice(&[cap, cap + 1, cap + 2, cap, cap + 2, cap + 3]);
            SurfaceMesh::from_parts(vx, vy, vz, idx).unwrap()
        }

        /// A two-vertex centerline in metres, at a constant height.
        fn line(&self, class: &str, a: (f64, f64), b: (f64, f64), z: f64) -> RoadLine {
            RoadLine {
                class: class.into(),
                level: 0,
                width_m: 0.0,
                parts: vec![vec![
                    (self.ux(a.0) as f64, self.uy(a.1) as f64, z),
                    (self.ux(b.0) as f64, self.uy(b.1) as f64, z),
                ]],
            }
        }

        fn scene(
            &self,
            roads: Vec<RoadMesh>,
            buildings: Vec<SurfaceMesh>,
            lines: Vec<RoadLine>,
            terrain: Option<SurfaceMesh>,
        ) -> TileScene {
            TileScene {
                z: 16,
                x: 34000,
                y: 23000,
                scale: self.scale,
                bounds: self.bounds,
                terrain,
                roads,
                lines,
                waters: Vec::new(),
                buildings: buildings.into_iter().map(|m| (0.0, m)).collect(),
            }
        }
    }

    fn run(tile: &TileScene) -> std::collections::HashMap<String, Metric> {
        let opt = Options { spacing_m: 1.0, ..Default::default() };
        let mut c = Box::new(Street::new(&opt));
        c.visit(tile, &opt);
        c.finish().into_iter().map(|m| (m.id.clone(), m)).collect()
    }

    /// A street beside a building, with the whole tile's ground at `ground_m`
    /// and the carriageway at 100 m. The band runs north–south over x 0..8 m,
    /// so its east kerb is the line x = 8.
    fn street(ground_m: f64) -> (Site, Vec<RoadMesh>, SurfaceMesh) {
        let s = Site::new();
        let roads = vec![s.band("road_surface", 0, (0.0, 8.0, 0.0, 80.0), 100.0)];
        let ground = s.slab(-20.0, 200.0, -20.0, 200.0, ground_m);
        (s, roads, ground)
    }

    #[test]
    fn asphalt_clear_of_a_footprint_scores_zero_rather_than_nothing() {
        // The population is the whole drawn band, not the part inside a
        // footprint: a street with a building beside it must contribute its
        // zeros, or closing the defect would empty the population instead of
        // moving the number.
        let s = Site::new();
        let tile = s.scene(
            vec![s.band("road_surface", 0, (0.0, 8.0, 0.0, 80.0), 100.0)],
            vec![s.building(12.0, 30.0, 20.0, 60.0, 98.0, 110.0)],
            Vec::new(),
            None,
        );
        let m = run(&tile);
        assert!(m[OVERLAP].dist.count() > 100, "the band must be sampled");
        assert_eq!(m[OVERLAP].violations(), 0, "asphalt four metres clear of a wall");
        assert_eq!(m[OVERLAP].worst_value(), Some(0.0));
    }

    #[test]
    fn a_sidewalk_beside_its_street_is_not_on_it() {
        // The correct cross-section: band beside the kerb, one kerb rise up.
        // The shared edge must score zero — the seam is where the two are
        // *supposed* to meet.
        let s = Site::new();
        let tile = s.scene(
            vec![
                s.band("road_surface", 0, (0.0, 8.0, 0.0, 80.0), 100.0),
                s.band("walk_surface", 0, (8.0, 11.0, 0.0, 80.0), 100.12),
            ],
            Vec::new(),
            Vec::new(),
            None,
        );
        let m = run(&tile);
        assert!(m[ON_ASPHALT].dist.count() > 100, "the band must be sampled");
        assert_eq!(m[ON_ASPHALT].violations(), 0, "beside the street is not on it");
    }

    #[test]
    fn a_band_across_the_carriageway_is_on_it() {
        // The dead-trim defect: a pavement band lying across the plate at
        // kerb rise. The depth is read against the road silhouette, so the
        // band's centre — four metres from either kerb — is the worst sample.
        let s = Site::new();
        let tile = s.scene(
            vec![
                s.band("road_surface", 0, (0.0, 8.0, 0.0, 80.0), 100.0),
                s.band("walk_surface", 0, (0.0, 8.0, 30.0, 33.0), 100.12),
            ],
            Vec::new(),
            Vec::new(),
            None,
        );
        let m = run(&tile);
        assert!(m[ON_ASPHALT].violations() > 0, "a band on the plate must score");
        let worst = m[ON_ASPHALT].worst_value().unwrap();
        assert!((worst - 4.0).abs() < 0.8, "expected ~4 m onto the plate, got {worst}");
    }

    #[test]
    fn a_walkway_a_storey_up_is_a_stack_and_not_on_the_asphalt() {
        // A rim path above a sunken road — or a footbridge — shares the plan
        // and not the plane. The height gate is what keeps both legitimate
        // stacks out of the population.
        let s = Site::new();
        let tile = s.scene(
            vec![
                s.band("road_surface", 0, (0.0, 8.0, 0.0, 80.0), 100.0),
                s.band("path_surface", 0, (0.0, 8.0, 30.0, 33.0), 103.5),
            ],
            Vec::new(),
            Vec::new(),
            None,
        );
        let m = run(&tile);
        assert!(m[ON_ASPHALT].dist.count() > 100, "the band is still scored");
        assert_eq!(m[ON_ASPHALT].violations(), 0, "a storey of air is a stack");
    }

    #[test]
    fn asphalt_through_a_wall_is_measured_from_the_facade() {
        // The band reaches x = 12 and the facade stands at x = 8, so the
        // deepest asphalt is four metres inside — measured from the nearest
        // wall, not from the footprint's centre.
        let s = Site::new();
        let tile = s.scene(
            vec![s.band("road_surface", 0, (0.0, 12.0, 0.0, 80.0), 100.0)],
            vec![s.building(8.0, 30.0, 20.0, 60.0, 98.0, 110.0)],
            Vec::new(),
            None,
        );
        let m = run(&tile);
        let worst = m[OVERLAP].worst_value().unwrap();
        assert!((worst - 4.0).abs() < 0.3, "expected ~4 m of penetration, got {worst}");
        assert!(m[OVERLAP].violations() > 0);
    }

    #[test]
    fn a_deck_over_a_building_is_a_level_relation_and_not_an_overlap() {
        // Invariant 3 is about things at the *same* level. A bridge over a
        // building is ordered, and counting it would make the metric
        // unclosable.
        let s = Site::new();
        let tile = s.scene(
            vec![
                s.band("road_surface", 0, (0.0, 8.0, 0.0, 80.0), 100.0),
                s.band("motorway", 1, (10.0, 28.0, 20.0, 60.0), 120.0),
            ],
            vec![s.building(8.0, 30.0, 20.0, 60.0, 98.0, 110.0)],
            Vec::new(),
            None,
        );
        let m = run(&tile);
        assert!(m[OVERLAP].dist.count() > 100, "the at-grade band is still scored");
        assert_eq!(m[OVERLAP].violations(), 0, "a deck over a footprint is ordered");
    }

    #[test]
    fn a_station_roof_over_its_platforms_is_not_an_overlap() {
        // A rail formation inside a footprint is a station, which the archive
        // cannot state as a level relation — so the population excludes rail
        // rather than reporting a building nobody can move.
        let s = Site::new();
        let tile = s.scene(
            vec![
                s.band("road_surface", 0, (0.0, 6.0, 0.0, 80.0), 100.0),
                s.band("rail_surface", 0, (10.0, 28.0, 20.0, 60.0), 100.0),
            ],
            vec![s.building(8.0, 30.0, 20.0, 60.0, 98.0, 110.0)],
            Vec::new(),
            None,
        );
        let m = run(&tile);
        assert_eq!(m[OVERLAP].violations(), 0, "a station roof is not asphalt through a wall");
    }

    #[test]
    fn a_footway_level_with_its_street_is_clean() {
        let (s, roads, ground) = street(100.0);
        let tile = s.scene(
            roads,
            Vec::new(),
            vec![s.line("footway", (11.0, 5.0), (11.0, 75.0), 100.0)],
            Some(ground),
        );
        let m = run(&tile);
        assert!(m[GRADE].dist.count() > 50, "a sidewalk beside a street is in the population");
        assert_eq!(m[GRADE].violations(), 0);
    }

    #[test]
    fn a_footway_on_ground_below_its_street_is_caught() {
        // §0.3's defect: the bench stops at the shoulder and the sidewalk
        // drapes on whatever the hillside does.
        let (s, roads, ground) = street(97.5);
        let tile = s.scene(
            roads,
            Vec::new(),
            vec![s.line("footway", (11.0, 5.0), (11.0, 75.0), 97.5)],
            Some(ground),
        );
        let m = run(&tile);
        let worst = m[GRADE].worst_value().unwrap();
        assert!((worst - 2.5).abs() < 0.1, "expected a 2.5 m drop, got {worst}");
        assert!(m[GRADE].violations() > 0);
    }

    #[test]
    fn a_footway_stranded_on_the_bank_above_its_street_is_caught_too() {
        // The cutting side of the same missing cross-section, and the reason
        // the metric is unsigned: on the measured extract this side is the
        // larger half.
        let (s, roads, ground) = street(102.5);
        let tile = s.scene(
            roads,
            Vec::new(),
            vec![s.line("footway", (11.0, 5.0), (11.0, 75.0), 102.5)],
            Some(ground),
        );
        let m = run(&tile);
        let worst = m[GRADE].worst_value().unwrap();
        assert!((worst - 2.5).abs() < 0.1, "expected a 2.5 m climb, got {worst}");
        assert!(m[GRADE].violations() > 0);
    }

    #[test]
    fn a_footway_crossing_a_street_is_not_in_the_population() {
        // Six metres of way running *across* the kerb, entirely within the
        // attachment reach, so only the parallelism rule can reject it.
        let (s, roads, ground) = street(97.5);
        let tile = s.scene(
            roads,
            Vec::new(),
            vec![s.line("footway", (9.0, 40.0), (15.0, 40.0), 97.5)],
            Some(ground),
        );
        let m = run(&tile);
        assert!(m[GRADE].dist.is_empty(), "a crossing is not a sidewalk");
    }

    #[test]
    fn a_path_that_only_brushes_a_street_is_not_attached_to_it() {
        // Proximity at a point is not attachment. Without the coverage rule
        // this is a hillside path scored against a road it merely passes.
        let (s, roads, ground) = street(90.0);
        let tile = s.scene(
            roads,
            Vec::new(),
            vec![s.line("path", (11.0, 5.0), (60.0, 70.0), 90.0)],
            Some(ground),
        );
        let m = run(&tile);
        assert!(m[GRADE].dist.is_empty(), "a way that leaves the street is not its sidewalk");
    }

    #[test]
    fn a_staircase_is_not_in_the_population() {
        // A staircase's purpose is to change height beside what it runs along,
        // so counting it measures the class table rather than the defect.
        let (s, roads, ground) = street(97.5);
        let tile = s.scene(
            roads,
            Vec::new(),
            vec![s.line("steps", (11.0, 5.0), (11.0, 75.0), 97.5)],
            Some(ground),
        );
        let m = run(&tile);
        assert!(m[GRADE].dist.is_empty(), "steps climb on purpose");
    }

    #[test]
    fn a_way_drawn_over_the_asphalt_has_no_cross_section_to_measure() {
        // It is standing on the carriageway, not beside it.
        let (s, roads, ground) = street(90.0);
        let tile = s.scene(
            roads,
            Vec::new(),
            vec![s.line("footway", (4.0, 5.0), (4.0, 75.0), 100.0)],
            Some(ground),
        );
        let m = run(&tile);
        assert!(m[GRADE].dist.is_empty(), "a way over the band is not beside it");
    }

    #[test]
    fn a_walkway_band_answers_for_the_surface_before_the_terrain_does() {
        // The trap this exists to close: the day a pedestrian band is laid, it
        // takes a hole out of the terrain with it, and a check reading only
        // the ground would find nothing there and quietly empty its
        // population instead of measuring the kerb rise.
        let (s, mut roads, ground) = street(97.5);
        roads.push(s.band("walk_surface", 0, (9.5, 13.0, 0.0, 80.0), 100.12));
        let tile = s.scene(
            roads,
            Vec::new(),
            vec![s.line("footway", (11.0, 5.0), (11.0, 75.0), 100.12)],
            Some(ground),
        );
        let m = run(&tile);
        assert!(!m[GRADE].dist.is_empty(), "the walk band must be found");
        let worst = m[GRADE].worst_value().unwrap();
        assert!((worst - 0.12).abs() < 0.02, "expected the kerb rise, got {worst}");
        assert_eq!(m[GRADE].violations(), 0);
    }

    #[test]
    fn a_zoom_with_no_building_layer_reports_a_skip_rather_than_a_clean_column() {
        // A check that stopped running looks exactly like one that passed.
        let s = Site::new();
        let tile = s.scene(
            vec![s.band("road_surface", 0, (0.0, 8.0, 0.0, 80.0), 100.0)],
            Vec::new(),
            Vec::new(),
            None,
        );
        let m = run(&tile);
        assert!(m[OVERLAP].skipped.is_some(), "no footprint anywhere must read as a skip");
    }

    const RIM: &str = "contact.walk_rim";
    const FALL: &str = "slope.walk_crossfall";

    /// A pedestrian band over the plan rectangle `(x0, x1, y0, y1)` whose height
    /// ramps across its width, from `z0` at `y0` to `z1` at `y1`.
    fn tilted(s: &Site, class: &str, r: (f64, f64, f64, f64), z0: f64, z1: f64) -> RoadMesh {
        let mesh = SurfaceMesh::from_parts(
            vec![s.ux(r.0), s.ux(r.1), s.ux(r.1), s.ux(r.0)],
            vec![s.uy(r.2), s.uy(r.2), s.uy(r.3), s.uy(r.3)],
            vec![z0 as f32, z0 as f32, z1 as f32, z1 as f32],
            vec![0, 1, 2, 0, 2, 3],
        )
        .unwrap();
        RoadMesh { class: class.into(), level: 0, band: String::new(), fades: false, sheet: None, mesh }
    }

    /// The terrain's rim and the band's own height are one joint: the band's
    /// region is what cut the hole the rim bounds, so a bench under it makes
    /// the two the same number.
    #[test]
    fn a_band_flush_with_the_ground_reads_no_step_at_its_rim() {
        let s = Site::new();
        // Terrain stopping under the band's near edge, the band continuing east.
        let tile = s.scene(
            vec![s.band("path_surface", 0, (10.0, 24.0, 0.0, 40.0), 100.0)],
            Vec::new(),
            Vec::new(),
            Some(s.slab(-20.0, 11.0, 0.0, 40.0, 100.0)),
        );
        let m = run(&tile);
        assert!(m[RIM].dist.count() > 0, "the rim must be walked");
        assert_eq!(m[RIM].violations(), 0, "a joint at one height is not a step");
    }

    /// The defect: a band seated on its host's cross-section with the drawn
    /// ground still on the hillside under it.
    #[test]
    fn a_band_standing_above_its_ground_is_caught_at_the_rim() {
        let s = Site::new();
        let tile = s.scene(
            vec![s.band("walk_surface", 0, (10.0, 24.0, 0.0, 40.0), 101.5)],
            Vec::new(),
            Vec::new(),
            Some(s.slab(-20.0, 11.0, 0.0, 40.0, 100.0)),
        );
        let m = run(&tile);
        assert!(m[RIM].violations() > 0);
        let worst = m[RIM].worst_value().unwrap();
        assert!((worst - 1.5).abs() < 0.05, "a metre and a half of wall, got {worst}");
    }

    /// The same step with a drawn wall down its face is the drape guard's own
    /// verdict — a bench refused, an apron drawn instead — and the joint holds.
    #[test]
    fn a_walled_rim_is_a_joint_that_holds() {
        let s = Site::new();
        let apron = {
            let mesh = SurfaceMesh::from_parts(
                vec![s.ux(10.5), s.ux(11.5), s.ux(11.5), s.ux(10.5)],
                vec![s.uy(0.0), s.uy(0.0), s.uy(40.0), s.uy(40.0)],
                vec![101.5, 100.0, 100.0, 101.5],
                vec![0, 1, 2, 0, 2, 3],
            )
            .unwrap();
            RoadMesh {
                class: "walk_apron".into(),
                level: 0,
                band: String::new(),
                fades: false,
                sheet: None,
                mesh,
            }
        };
        let tile = s.scene(
            vec![s.band("walk_surface", 0, (10.0, 24.0, 0.0, 40.0), 101.5), apron],
            Vec::new(),
            Vec::new(),
            Some(s.slab(-20.0, 11.0, 0.0, 40.0, 100.0)),
        );
        let m = run(&tile);
        assert!(m[RIM].dist.count() > 0, "the rim must be walked");
        assert_eq!(m[RIM].violations(), 0, "a walled step is closed");
    }

    /// A path drawn on the drawn ground carries the hillside's cross-slope; the
    /// metric is read across the band, from one side edge inward.
    #[test]
    fn a_tilted_band_reads_the_fall_across_its_own_width() {
        let s = Site::new();
        // Two metres wide, a metre of rise across it: 50 %.
        let tile = s.scene(
            vec![tilted(&s, "path_surface", (0.0, 40.0, 0.0, 2.0), 100.0, 101.0)],
            Vec::new(),
            Vec::new(),
            None,
        );
        let m = run(&tile);
        assert!(m[FALL].dist.count() > 0, "the side edges must be walked");
        let worst = m[FALL].worst_value().unwrap();
        assert!((worst - 0.5).abs() < 0.02, "half a metre per metre across, got {worst}");
        // A sidewalk seated on its host's cross-section is flat across, and the
        // same walk says so rather than saying nothing.
        let flat = s.scene(
            vec![s.band("walk_surface", 0, (0.0, 40.0, 0.0, 2.0), 100.0)],
            Vec::new(),
            Vec::new(),
            None,
        );
        let m = run(&flat);
        assert!(m[FALL].dist.count() > 0);
        assert_eq!(m[FALL].violations(), 0);
    }

    /// An end edge is not a cross-section: inward there is *along* the way, so
    /// the reading would be its longitudinal grade. A band that is nothing but
    /// ends — three metres of it, so every edge fails the test — contributes
    /// nothing at all rather than contributing its grade.
    #[test]
    fn an_end_edge_is_not_a_cross_section() {
        let s = Site::new();
        let tile = s.scene(
            vec![tilted(&s, "path_surface", (0.0, 3.0, 0.0, 3.0), 100.0, 103.0)],
            Vec::new(),
            Vec::new(),
            None,
        );
        assert_eq!(run(&tile)[FALL].dist.count(), 0);
    }
}
