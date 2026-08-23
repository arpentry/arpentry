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
//! prior's half-width and nothing about junction plates, the casing rim or the
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

/// The at-grade *road* surface: the union's interior band and its casing rim.
///
/// Rail is deliberately out of the footprint population. A station roof stands
/// over its own platforms, which is a level relation the archive cannot state
/// and not asphalt through a wall — 2,772 m² of the Montreux extract, all of it
/// under station roofs.
fn is_road_band(r: &RoadMesh) -> bool {
    r.level == 0 && matches!(r.class.as_str(), "road_surface" | "road_casing")
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
        r.level == 0 && matches!(r.class.as_str(), "walk_surface" | "walk_casing")
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
struct Footprints<'a> {
    items: Vec<Footprint<'a>>,
    cells: HashMap<(i32, i32), Vec<usize>>,
    /// Cell size in unit plan space, per axis — about 20 m.
    cw: f64,
    ch: f64,
}

impl<'a> Footprints<'a> {
    fn build(tile: &'a TileScene) -> Footprints<'a> {
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
    fn depth(&self, px: f64, py: f64, scale: &Scale) -> Option<f64> {
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
    /// Both the interior band and its casing rim contribute. `road_surface` is
    /// an *inset* of the paved region by `PAVE_RIM_M`, so its silhouette is a
    /// third of a metre inside the true kerb; the casing carries the real one.
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
    grade: Dist,
    grade_worst: Worst,
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
            grade: Dist::new(0.0, 64.0),
            grade_worst: Worst::new(Sense::HigherIsWorse, opt.worst_k),
            grade_signed: Dist::metres(),
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
                let Some((_, k)) = kerbs.nearest(px, py, &tile.scale) else { return };
                let departure = (z - k.z).abs();
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
}

/// The drawn **sidewalk**: the band attached to a street, and its casing rim,
/// at grade.
///
/// `path_surface` is deliberately out. A path across a hillside stands on the
/// ground and belongs to no street, so measuring it against the nearest kerb
/// within [`WALK_ATTACH_M`] scores a relation that does not exist — the worst
/// site in the extract was a footpath 17.7 m up a slope from a road it passed
/// near. The two materials are separate for this reason (`priors::Surface`).
fn is_walk_band(r: &RoadMesh) -> bool {
    r.level == 0 && matches!(r.class.as_str(), "walk_surface" | "walk_casing")
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

impl Check for Street {
    fn visit(&mut self, tile: &TileScene, opt: &Options) {
        self.visit_overlap(tile, opt);
        self.visit_grade(tile, opt);
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
        }
        vec![
            Metric {
                id: "order.building_overlap".into(),
                invariant: Invariant::I3,
                title: "Drawn at-grade asphalt standing inside a building footprint".into(),
                population: "Every drawn at-grade road surface sample (road_surface and its \
                             road_casing rim) the tile owns, scored by how far inside a \
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
                         because it knows nothing of the junction plates, the casing rim and \
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
                             are outside this population altogether. And the far tail is not \
                             all this metric's defect: past a few metres it is a street on a \
                             terrace with a path along the foot of its wall, which is \
                             `contact.kerb_lip`'s question — at the worst site \
                             (6.8961,46.4649) the section shows a carriageway at 624 m and \
                             drawn ground at 613.5 m a metre away. Read the body, not the \
                             extreme."
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
                band: String::new(),
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
}
