//! The kerb line as one object: where the pavement beside a street stops
//! short of the pavement it should meet.
//!
//! Every pedestrian check in this harness scores a band that *exists* — its
//! seat, its rim, its cross-fall, whether it stands on the asphalt. None of
//! them can see a band that is missing, and missing is what a junction looks
//! like: each leg's sidewalk is drawn as an offset of its own street, so it
//! ends where the street ends, and the corner between two legs — or the whole
//! outer ring of a roundabout — is bare kerb between two pavements that each
//! stop a few metres short of it. `contact.walk_rim` reads 0.3 % on a
//! roundabout whose sidewalks visibly do not meet, because there is no rim to
//! score where nothing was drawn.
//!
//! [`street.kerb_gap`] walks the drawn kerb — the silhouette of the at-grade
//! road surface where its rim says it is a kerb rather than a deck handover or
//! a tile cut — as chains of one-metre stations, and marks each station
//! *served* when a drawn pedestrian surface stands beside it. A **gap** is a
//! bare run bounded by served runs on both ends and shorter than the reach a
//! sidewalk is expected to wrap a corner over (`priors::WALK_CORNER_MAX_M`,
//! the same number the band generator uses for the same idea). Every kerb
//! station is a sample: the length of the gap it lies in, or zero. Scoring the
//! zeros is what makes the rate mean something — it is the share of kerb
//! metres lying in a gap, and closing a gap moves the number instead of
//! removing samples.
//!
//! What is deliberately *not* a gap: a bare run longer than the corner reach
//! (a side street that has no pavement, and honestly draws none), and a bare
//! run whose far end is unknown — the chain left the tile, or reached a
//! handover cut, before another pavement was found. Those stations are out of
//! the population rather than scored zero, because "the pavement ends here" and
//! "we could not see where it resumes" are different answers from "there is no
//! gap".

use std::collections::HashMap;

use crate::priors;
use crate::verify::checks::{Check, Options};
use crate::verify::dist::Dist;
use crate::verify::mesh::EXTENT;
use crate::verify::scene::{RoadMesh, TileScene};
use crate::verify::{Invariant, Metric, Offender, Sense, Worst};

/// Station spacing along the kerb, metres. One metre resolves a gap of a few
/// metres — the corner-wrap defect this exists for — and a straight kerb is a
/// single silhouette edge after simplification, so the classification cannot
/// be per edge.
const STEP_M: f64 = 1.0;

/// Where a station looks for its pavement: this far *outward* from the road
/// surface's silhouette, metres. The silhouette measured is the interior
/// band's, which is inset [`priors::PAVE_RIM_M`] from the true kerb
/// (`synth::pave_mesh`), and a flush band begins at that kerb, so the probe
/// lands a third of a metre into a flush band. Outward rather than radial, so
/// the search does not reach *along* the kerb: a radius wide enough to admit a
/// band half a metre off the kerb also found a band a metre past its own end,
/// and every gap read two stations short.
const PROBE_M: f64 = priors::PAVE_RIM_M + 0.35;

/// How far from the probe point a pedestrian surface may stand and still
/// serve the station, metres — a band half a metre off the kerb line is still
/// this street's pavement (the same slack `order.walk_on_asphalt` allows);
/// along the kerb it means a gap reads half a station short at each end.
const SERVE_M: f64 = 0.5;

/// How far outward a building must stand clear of the silhouette for the
/// station to have room for a pavement at all, metres: the rim, the narrowest
/// band worth drawing and the clearance a facade keeps
/// (`priors::FACADE_CLEAR_M`). A station a footprint stands inside of within
/// this is a kerb the building pinches shut — the pavement ends there
/// honestly, on both sides of the model, and it is not a gap.
const PINCH_M: f64 = priors::PAVE_RIM_M + priors::WALK_MIN_WIDTH_M + priors::FACADE_CLEAR_M;

/// How far the road rim may stand from a silhouette station for the station
/// to be a kerb, metres — the rim's own width plus a station's rounding. A
/// silhouette edge without a rim beside it is a deck handover, which the
/// mesher deliberately leaves unedged ("edging it draws a line straight across
/// the carriageway"), or a tile cut.
const RIM_M: f64 = priors::PAVE_RIM_M + 0.15;

/// The longest bare run between two pavements that is still a corner the
/// pavement should have wrapped. Shared with the band generator
/// (`synth::walkway::free_bands`), where the same length decides that two
/// attached stretches of one way are one pavement with a corner between them.
const GAP_MAX_M: f64 = priors::WALK_CORNER_MAX_M;

/// Past this a bare stretch is a gap rather than station rounding at a band's
/// end: two stations, so a served run that starts a station late scores nothing.
const GAP_M: f64 = 2.0 * STEP_M;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum State {
    /// Kerb with a pedestrian surface beside it.
    Served,
    /// Kerb with nothing beside it.
    Bare,
    /// Not a question this tile can answer: outside the tile proper, on its
    /// edge, or not a kerb at all.
    Unknown,
}

/// How far outward the pavement is measured, and at what resolution: a
/// bracket every [`WIDTH_STEP_M`] out to [`WIDTH_MAX_M`], then three
/// bisections, so a width lands within a centimetre. Three metres is past
/// anything a pavement is drawn at.
///
/// The resolution is load-bearing: the wander below sums differences over ten
/// stations, so a measurement good to `e` carries `10·e` of its own noise into
/// every sample. At 5 cm that noise was half the threshold and the metric
/// mostly reported itself.
const WIDTH_MAX_M: f64 = 3.0;
const WIDTH_STEP_M: f64 = 0.4;
const WIDTH_BISECTIONS: u32 = 6;

/// Per-station change ignored before the wander is summed: twice the
/// measurement's own resolution, so a straight edge reads zero rather than
/// the march's rounding.
const WIDTH_DEADBAND_M: f64 = 0.02;

/// How far the pavement's edge may wander in and out over [`JOG_WINDOW`]
/// metres of kerb before it reads as a sawtooth.
///
/// **The wander, not the change.** A facade approaching a street narrows the
/// pavement monotonically over its own length, and a kerb turning a corner
/// sweeps its own normal: both change the width steadily, and neither is a
/// defect. What the eye reads as broken linework is the edge going out and
/// coming back. Total variation less the net change over a window is exactly
/// that quantity — zero for any monotone stretch however steep, and twice the
/// excursion for an edge that jumps a rung and returns. It is the same
/// reversal `slope.terrain_tearing` scores in the drawn ground, read along a
/// line instead of across a lattice.
const WIDTH_JOG_M: f64 = 0.25;

/// The window the wander is read over, in stations (a station is a metre).
/// Ten metres holds two full periods of the sawtooth a per-station width
/// ladder makes, whose plateaus are one ring station long.
const JOG_WINDOW: usize = 10;

/// One station along a silhouette chain.
struct Station {
    x: f64,
    y: f64,
    state: State,
    /// How wide the drawn pavement is here, measured outward from the kerb.
    /// `None` unless the station is [`State::Served`].
    width: Option<f64>,
}

pub struct Kerb {
    gap: Dist,
    gap_worst: Worst,
    /// The change in pavement width between neighbouring served stations —
    /// the pavement's outer edge read as a line rather than as a set of
    /// widths.
    jog: Dist,
    jog_worst: Worst,
    /// Kerb metres seen, served and bare, and the excluded remainder — the
    /// census `ARPT_DEBUG_KERB` prints before the metric is believed.
    kerb_m: f64,
    served_m: f64,
    excluded_m: f64,
    gaps: u64,
    /// Tiles at a zoom that meshes pedestrian bands at all.
    measured_tiles: u64,
    /// `ARPT_DEBUG_KERB`: every gap named as it is found, and the census at
    /// the end.
    debug: bool,
}

impl Kerb {
    pub fn new(opt: &Options) -> Kerb {
        Kerb {
            gap: Dist::new(0.0, 64.0),
            gap_worst: Worst::new(Sense::HigherIsWorse, opt.worst_k),
            jog: Dist::new(0.0, 4.0),
            jog_worst: Worst::new(Sense::HigherIsWorse, opt.worst_k),
            kerb_m: 0.0,
            served_m: 0.0,
            excluded_m: 0.0,
            gaps: 0,
            measured_tiles: 0,
            debug: std::env::var_os("ARPT_DEBUG_KERB").is_some(),
        }
    }
}

fn is_pedestrian_surface(r: &RoadMesh) -> bool {
    r.level == 0
        && matches!(r.class.as_str(), "walk_surface" | "walk_rim" | "path_surface" | "path_rim")
}

fn is_road_rim(r: &RoadMesh) -> bool {
    r.level == 0 && r.class == "road_rim"
}

fn is_road_surface(r: &RoadMesh) -> bool {
    r.level == 0 && r.class == "road_surface"
}

/// One point of a silhouette chain: its position in unit plan space and the
/// outward unit direction (in metres) of the edge leaving it — away from the
/// one triangle holding that edge, which is what says which side the ground
/// is on. The last point of a chain repeats the direction of the edge that
/// reached it.
type ChainPt = ((f64, f64), (f64, f64));

/// The silhouette of one mesh as chains. Open chains first (each from one
/// degree-one end to the other), then the closed loops; every boundary edge is
/// used exactly once.
fn chains(m: &crate::verify::mesh::SurfaceMesh, scale: &crate::verify::mesh::Scale) -> Vec<Vec<ChainPt>> {
    let key = |v: (f64, f64, f64)| ((v.0 * EXTENT).round() as i64, (v.1 * EXTENT).round() as i64);
    let edges: Vec<((f64, f64), (f64, f64), (i64, i64), (i64, i64), (f64, f64))> = m
        .boundary_edges()
        .into_iter()
        .map(|(a, b, opp)| {
            let (va, vb, vo) = (m.vertex(a), m.vertex(b), m.vertex(opp));
            let (mx, my) = ((va.0 + vb.0) * 0.5, (va.1 + vb.1) * 0.5);
            // **The edge's own perpendicular**, turned away from the triangle
            // that holds it — not the direction of that triangle's far
            // vertex, which is only normal to the edge when the triangle is
            // small. A surface meshed as two big triangles has its far vertex
            // at the opposite corner, and a probe sent that way walks along
            // the kerb instead of across it.
            let (ex, ey) = ((vb.0 - va.0) * scale.mx, (vb.1 - va.1) * scale.my);
            let elen = (ex * ex + ey * ey).sqrt().max(1e-12);
            let (mut nx, mut ny) = (ey / elen, -ex / elen);
            let (ox, oy) = ((mx - vo.0) * scale.mx, (my - vo.1) * scale.my);
            if nx * ox + ny * oy < 0.0 {
                nx = -nx;
                ny = -ny;
            }
            ((va.0, va.1), (vb.0, vb.1), key(va), key(vb), (nx, ny))
        })
        .collect();
    let mut at: HashMap<(i64, i64), Vec<usize>> = HashMap::new();
    for (i, e) in edges.iter().enumerate() {
        at.entry(e.2).or_default().push(i);
        at.entry(e.3).or_default().push(i);
    }
    let mut used = vec![false; edges.len()];
    let mut out = Vec::new();
    // Walk from `start` outward along unused edges, appending positions.
    let walk = |from: (i64, i64), first: usize, used: &mut Vec<bool>| -> Vec<ChainPt> {
        let mut chain: Vec<ChainPt> = Vec::new();
        let (mut here, mut e) = (from, first);
        loop {
            used[e] = true;
            let (pa, pb, ka, kb, out) = edges[e];
            let (p_here, p_next, k_next) = if ka == here { (pa, pb, kb) } else { (pb, pa, ka) };
            if chain.is_empty() {
                chain.push((p_here, out));
            } else if let Some(last) = chain.last_mut() {
                last.1 = out;
            }
            chain.push((p_next, out));
            here = k_next;
            match at.get(&here).and_then(|v| v.iter().copied().find(|&i| !used[i])) {
                Some(next) => e = next,
                None => break,
            }
        }
        chain
    };
    // Open chains start at a vertex with exactly one edge.
    let mut ends: Vec<(i64, i64)> = at.iter().filter(|(_, v)| v.len() == 1).map(|(k, _)| *k).collect();
    ends.sort_unstable();
    for k in ends {
        if let Some(&e) = at[&k].iter().find(|&&i| !used[i]) {
            out.push(walk(k, e, &mut used));
        }
    }
    // Whatever is left closes on itself.
    for i in 0..edges.len() {
        if !used[i] {
            let mut loop_ = walk(edges[i].2, i, &mut used);
            // A loop's walk ends back at its start; mark it closed by
            // repeating the first point, which the station walk reads.
            if loop_.first().map(|p| p.0) != loop_.last().map(|p| p.0) {
                let f = loop_[0];
                loop_.push(f);
            }
            out.push(loop_);
        }
    }
    out
}

/// How wide the drawn pavement is outward of a kerb station: the last point
/// along the outward normal still covered by a drawn pedestrian surface.
///
/// Marched, not queried, because the archive carries triangles and not the
/// region they came from: there is nothing to ask for a boundary. Bracketed
/// coarsely and then bisected, which costs about ten covering tests.
fn pavement_width(
    walks: &[&RoadMesh],
    tile: &TileScene,
    x: f64,
    y: f64,
    o: (f64, f64),
) -> Option<f64> {
    // **Containment, not proximity.** `span_near` measures the distance to a
    // triangle's *edges*, so a point deep inside a large triangle reads as far
    // from it — which is right for a wall and wrong for a floor. The width is
    // a question about what covers a point, so the point-in-triangle test
    // answers it.
    let covered = |d: f64| {
        let (px, py) = (x + o.0 * d / tile.scale.mx, y + o.1 * d / tile.scale.my);
        walks.iter().any(|r| r.mesh.height_at(px, py).is_some())
    };
    if !covered(PROBE_M) {
        return None;
    }
    let (mut lo, mut hi) = (PROBE_M, WIDTH_MAX_M);
    let mut d = PROBE_M;
    while d + WIDTH_STEP_M <= WIDTH_MAX_M {
        let next = d + WIDTH_STEP_M;
        if covered(next) {
            d = next;
        } else {
            hi = next;
            break;
        }
    }
    lo = lo.max(d);
    for _ in 0..WIDTH_BISECTIONS {
        let mid = 0.5 * (lo + hi);
        if covered(mid) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    // **A ray that never left the pavement measured no width.** At a corner
    // the outward normal of one kerb runs along the pavement wrapping it, and
    // at a plaza there is no far edge at all: both return the cap, and both
    // are a question about a cross-section that has none here.
    (lo < WIDTH_MAX_M - 0.05).then_some(lo)
}

impl Kerb {
    fn score_chain(&mut self, tile: &TileScene, stations: &[Station], closed: bool) {
        let n = stations.len();
        if n == 0 {
            return;
        }
        // The outer edge, read along the kerb: how far it wanders in and out
        // over a window, which is its total variation less its net change.
        // Scored before the runs, because a chain with no pavement at all has
        // none of this either way.
        let ends = if closed { n } else { n.saturating_sub(1) };
        let mut k = 0;
        while k < ends {
            // One maximal run of stations that all measured a width.
            let mut run: Vec<(usize, f64)> = Vec::new();
            while k < ends {
                match stations[k % n].width {
                    Some(w) => run.push((k % n, w)),
                    None => break,
                }
                k += 1;
            }
            k += 1;
            if run.len() < 3 {
                continue;
            }
            for i in 0..run.len() {
                let j = (i + JOG_WINDOW).min(run.len() - 1);
                if j - i < 2 {
                    break;
                }
                let tv: f64 = run[i..=j]
                    .windows(2)
                    .map(|w| ((w[1].1 - w[0].1).abs() - WIDTH_DEADBAND_M).max(0.0))
                    .sum();
                // Clamped, because the deadband is taken off the variation
                // and not off the net: a monotone taper reads a shade under
                // zero otherwise.
                let wander = (tv - (run[j].1 - run[i].1).abs()).max(0.0);
                self.jog.push(wander);
                if wander > WIDTH_JOG_M {
                    let s = &stations[run[i].0];
                    let (lon, lat) = tile.lonlat(s.x, s.y);
                    self.jog_worst.offer(Offender {
                        lon,
                        lat,
                        zoom: tile.z,
                        value: wander,
                        note: format!(
                            "the pavement's edge wanders {wander:.2} m in and out over \
                             {} m of kerb ({:.2} m wide here, {:.2} m there)",
                            j - i,
                            run[i].1,
                            run[j].1
                        ),
                    });
                }
            }
        }
        // Runs of one state, in chain order. A closed chain is rotated to
        // begin at a served station so a bare run wrapping its seam is one run.
        let start = if closed { stations.iter().position(|s| s.state == State::Served) } else { Some(0) };
        let Some(start) = start else {
            // A loop with no pavement at all — a roundabout's island, or a
            // street nobody paved. Nothing to be short of.
            self.excluded_m += n as f64 * STEP_M;
            return;
        };
        let order: Vec<usize> = (0..n).map(|i| (start + i) % n).collect();
        let mut runs: Vec<(State, usize, usize)> = Vec::new(); // (state, from, to) in `order`
        for (k, &i) in order.iter().enumerate() {
            match runs.last_mut() {
                Some((s, _, to)) if *s == stations[i].state => *to = k + 1,
                _ => runs.push((stations[i].state, k, k + 1)),
            }
        }
        for r in 0..runs.len() {
            let (state, from, to) = runs[r];
            let len_m = (to - from) as f64 * STEP_M;
            match state {
                State::Unknown => {
                    self.excluded_m += len_m;
                }
                State::Served => {
                    self.kerb_m += len_m;
                    self.served_m += len_m;
                    for _ in from..to {
                        self.gap.push(0.0);
                    }
                }
                State::Bare => {
                    let before = if r > 0 {
                        Some(runs[r - 1].0)
                    } else if closed {
                        Some(runs[runs.len() - 1].0)
                    } else {
                        None
                    };
                    let after = if r + 1 < runs.len() {
                        Some(runs[r + 1].0)
                    } else if closed {
                        Some(runs[0].0)
                    } else {
                        None
                    };
                    let bounded = before == Some(State::Served) && after == Some(State::Served);
                    if !bounded {
                        // The far end is unknown, or the run is the whole loop.
                        self.excluded_m += len_m;
                        continue;
                    }
                    self.kerb_m += len_m;
                    let v = if len_m <= GAP_MAX_M { len_m } else { 0.0 };
                    for _ in from..to {
                        self.gap.push(v);
                    }
                    if v > GAP_M {
                        self.gaps += 1;
                        let mid = &stations[order[(from + to) / 2]];
                        let (lon, lat) = tile.lonlat(mid.x, mid.y);
                        if self.debug {
                            eprintln!("[kerb] gap {len_m:5.1} m at {lon:.6},{lat:.6}");
                        }
                        self.gap_worst.offer(Offender {
                            lon,
                            lat,
                            zoom: tile.z,
                            value: v,
                            note: format!(
                                "the pavement stops for {v:.1} m of kerb between two pavements"
                            ),
                        });
                    }
                }
            }
        }
    }
}

impl Check for Kerb {
    fn visit(&mut self, tile: &TileScene, _opt: &Options) {
        if tile.z < priors::WALK_SURFACE_MIN_ZOOM {
            return;
        }
        self.measured_tiles += 1;
        let walks: Vec<&RoadMesh> = tile.roads.iter().filter(|r| is_pedestrian_surface(r)).collect();
        let footprints = crate::verify::checks::street::Footprints::build(tile);
        let rims: Vec<&RoadMesh> = tile.roads.iter().filter(|r| is_road_rim(r)).collect();
        let on_edge = |v: f64| v.abs() < 1e-6 || (v - 1.0).abs() < 1e-6;
        // ARPT_KERB_AT=lon,lat — every station within 10 m of the point,
        // with the road mesh it lies on and what it read.
        let probe_at: Option<(f64, f64)> = std::env::var("ARPT_KERB_AT").ok().and_then(|v| {
            let (a, b) = v.split_once(',')?;
            Some((a.trim().parse().ok()?, b.trim().parse().ok()?))
        });
        for road in tile.roads.iter().filter(|r| is_road_surface(r)) {
            for chain in chains(&road.mesh, &tile.scale) {
                let closed = chain.len() > 2 && chain.first().map(|p| p.0) == chain.last().map(|p| p.0);
                let mut stations: Vec<Station> = Vec::new();
                for w in chain.windows(2) {
                    let (((ax, ay), (ox, oy)), ((bx, by), _)) = (w[0], w[1]);
                    let len = tile.scale.dist(ax, ay, bx, by);
                    if len < 1e-9 {
                        continue;
                    }
                    let steps = (len / STEP_M).round().max(1.0) as usize;
                    for k in 0..steps {
                        let t = (k as f64 + 0.5) / steps as f64;
                        let (x, y) = (ax + (bx - ax) * t, ay + (by - ay) * t);
                        // The pavement is looked for outward of the kerb.
                        let (px, py) =
                            (x + ox * PROBE_M / tile.scale.mx, y + oy * PROBE_M / tile.scale.my);
                        let (fx, fy) =
                            (x + ox * PINCH_M / tile.scale.mx, y + oy * PINCH_M / tile.scale.my);
                        let state = if !tile.owns(x, y) || on_edge(x) || on_edge(y) {
                            State::Unknown
                        } else if !rims.iter().any(|r| r.mesh.span_near(x, y, &tile.scale, RIM_M).is_some()) {
                            State::Unknown
                        } else if footprints.depth(px, py, &tile.scale).is_some()
                            || footprints.depth(fx, fy, &tile.scale).is_some()
                        {
                            State::Unknown
                        } else if walks
                            .iter()
                            .any(|r| r.mesh.span_near(px, py, &tile.scale, SERVE_M).is_some())
                        {
                            State::Served
                        } else {
                            State::Bare
                        };
                        if let Some((plon, plat)) = probe_at {
                            let (lon, lat) = tile.lonlat(x, y);
                            let dm = ((lon - plon) * tile.scale.mx * tile.bounds.width()
                                / tile.bounds.width())
                            .hypot((lat - plat) * tile.scale.my / tile.bounds.height() * tile.bounds.height());
                            let _ = dm;
                            if (lon - plon).abs() < 0.0004 && (lat - plat).abs() < 0.0003 {
                                let w = (state == State::Served)
                                    .then(|| pavement_width(&walks, tile, x, y, (ox, oy)))
                                    .flatten();
                                eprintln!(
                                    "[kerb-at] {lon:.6},{lat:.6} sheet {:?} {state:?} width {}",
                                    road.sheet,
                                    w.map_or("-".to_string(), |w| format!("{w:.2}"))
                                );
                            }
                        }
                        let width = (state == State::Served)
                            .then(|| pavement_width(&walks, tile, x, y, (ox, oy)))
                            .flatten();
                        stations.push(Station { x, y, state, width });
                    }
                }
                self.score_chain(tile, &stations, closed);
            }
        }
    }

    fn finish(self: Box<Self>) -> Vec<Metric> {
        if self.debug {
            let q = |p: f64| self.gap.quantile(p).unwrap_or(f64::NAN);
            eprintln!(
                "[kerb] {:.0} m of kerb scored ({:.1} % served), {:.0} m excluded, {} gaps; \
                 gap length p50 {:.1} p90 {:.1} p99 {:.1} max {:.1}",
                self.kerb_m,
                100.0 * self.served_m / self.kerb_m.max(1e-9),
                self.excluded_m,
                self.gaps,
                q(0.5),
                q(0.9),
                q(0.99),
                self.gap.max().unwrap_or(f64::NAN),
            );
            for t in [2.0, 5.0, 10.0, 15.0, 20.0, 25.0] {
                eprintln!("    in a gap longer than {t:.0} m: {:.3} %", 100.0 - self.gap.pct_below(t));
            }
            let q = |p: f64| self.jog.quantile(p).unwrap_or(f64::NAN);
            eprintln!(
                "[kerb] width jog: n={} p50 {:.2} p90 {:.2} p99 {:.2} max {:.2}",
                self.jog.count(),
                q(0.5),
                q(0.9),
                q(0.99),
                self.jog.max().unwrap_or(f64::NAN),
            );
            for t in [0.1, 0.25, 0.4, 0.8] {
                eprintln!("    jog over {t:.2} m: {:.3} %", 100.0 - self.jog.pct_below(t));
            }
        }
        vec![
        Metric {
            id: "street.walk_width_step".into(),
            invariant: Invariant::I1,
            title: "The pavement's outer edge stepping sideways along the kerb".into(),
            population: "Every served one-metre kerb station (the `street.kerb_gap` walk) \
                         that measured a pavement width, scored as the total variation \
                         less the net change of that width over the next ten stations — \
                         how far the pavement's outer edge wanders in and out over ten \
                         metres of kerb. The width is marched outward from the kerb along \
                         the edge's own perpendicular to the last point a drawn at-grade \
                         pedestrian surface covers, out to 3 m and to about 5 cm: the \
                         archive carries triangles, not the region they were cut from, so \
                         there is no boundary to ask for. A station whose neighbours are \
                         bare or excluded starts a new run, and a run under three stations \
                         is out of the population; every other station scores, zeros \
                         included, so the rate is the share of the drawn kerb that saws. A \
                         taper and a corner sweep score zero by construction, being \
                         monotone. Measured only from `WALK_SURFACE_MIN_ZOOM`."
                .into(),
            detail: "One cross-section per street (docs/ROADS.md invariant 1) is a claim \
                     about the *line* a pavement's edge draws, not only about its width at \
                     a station. On screen this is a sawtooth down an otherwise straight \
                     pavement, and it is what a width ladder makes when it is resolved per \
                     station instead of per run: neighbouring stations land on different \
                     rungs and the edge jumps a rung out and back. The ring's own boundary \
                     is smooth — the union offset, cut by the facades — so anything here \
                     is something cutting it back station by station."
                .into(),
            sense: Sense::HigherIsWorse,
            threshold: WIDTH_JOG_M,
            skipped: (self.measured_tiles == 0)
                .then(|| "no tile at a zoom that meshes pedestrian bands (z16+)".to_string()),
            dist: self.jog,
            worst: self.jog_worst.into_vec(),
        },
        Metric {
            id: "street.kerb_gap".into(),
            invariant: Invariant::I1,
            title: "Kerb between two pavements that neither reaches".into(),
            population: "Every one-metre station along the silhouette of the drawn at-grade \
                         road surface (`road_surface`, level 0) the tile owns, where a \
                         `road_rim` stands beside the station — which is what tells a kerb \
                         from a deck handover or a tile cut, both of which the mesher leaves \
                         unedged — and no building footprint stands within 1.65 m outward \
                         of it (rim, narrowest band, facade clearance): a kerb a building \
                         pinches shut has no room for a pavement and is out of the \
                         population. A station is *served* when any drawn at-grade pedestrian \
                         surface (walk or path, band or rim) stands within half a metre of a \
                         probe 0.7 m outward of it — into where a flush band lies. \
                         Stations are walked in silhouette order as chains; a bare run \
                         bounded by served runs on both ends is a gap, and every station \
                         in it scores the gap's length, every other station zero. Out of \
                         the population, rather than zero: a bare run that reaches the \
                         tile's edge, a handover or the end of its chain before another \
                         pavement (its far end is unknown), and a loop with no pavement at \
                         all (an island, a street nobody paved). A bare run longer than \
                         `WALK_CORNER_MAX_M` (25 m) between two pavements scores zero: that \
                         is a side street without a sidewalk, drawn honestly. Measured only \
                         from `WALK_SURFACE_MIN_ZOOM` (z16), where pedestrian bands mesh; \
                         coarser rungs report a skip. Per tile, so a gap straddling a tile \
                         border is unseen from both sides — the same blindness every \
                         per-tile check has at a border."
                .into(),
            detail: "The sidewalk is drawn as an offset of each street, so it ends where \
                     its street ends: at a junction each leg's pavement stops a few metres \
                     short of the corner, and around a roundabout — a dozen short ring arcs \
                     no attachment survives on — the whole outer kerb between two legs is \
                     bare. On screen it is a pavement that dies in a stub at every mouth and \
                     never wraps a corner. `contact.walk_rim` and `slope.walk_crossfall` \
                     cannot see it because there is no band to score where none was drawn. \
                     The threshold is two stations, so a band that begins a station late \
                     scores nothing; the cap is the band generator's own corner reach, so \
                     the check and the model agree on what a corner is."
                .into(),
            sense: Sense::HigherIsWorse,
            threshold: GAP_M,
            skipped: (self.measured_tiles == 0)
                .then(|| "no tile at a zoom that meshes pedestrian bands (z16+)".to_string()),
            dist: self.gap,
            worst: self.gap_worst.into_vec(),
        }]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::project::Bounds;
    use crate::verify::mesh::{Scale, SurfaceMesh};

    /// A tile laid out in metres from its south-west corner (see the note in
    /// `street::tests::Site`).
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

        fn slab(&self, x0: f64, x1: f64, y0: f64, y1: f64, z: f64) -> SurfaceMesh {
            SurfaceMesh::from_parts(
                vec![self.ux(x0), self.ux(x1), self.ux(x1), self.ux(x0)],
                vec![self.uy(y0), self.uy(y0), self.uy(y1), self.uy(y1)],
                vec![z as f32; 4],
                vec![0, 1, 2, 0, 2, 3],
            )
            .unwrap()
        }

        fn band(&self, class: &str, r: (f64, f64, f64, f64)) -> RoadMesh {
            RoadMesh {
                class: class.into(),
                level: 0,
                band: String::new(),
                fades: false,
                sheet: None,
                mesh: self.slab(r.0, r.1, r.2, r.3, 100.0),
            }
        }

        fn scene(&self, z: u8, roads: Vec<RoadMesh>) -> TileScene {
            TileScene {
                z,
                x: 34000,
                y: 23000,
                scale: self.scale,
                bounds: self.bounds,
                terrain: None,
                roads,
                lines: Vec::new(),
                waters: Vec::new(),
                buildings: Vec::new(),
            }
        }
    }

    fn run(tile: &TileScene) -> Metric {
        by_id(tile, "street.kerb_gap")
    }

    fn by_id(tile: &TileScene, id: &str) -> Metric {
        let opt = Options { spacing_m: 1.0, ..Default::default() };
        let mut c = Box::new(Kerb::new(&opt));
        c.visit(tile, &opt);
        c.finish().into_iter().find(|m| m.id == id).expect("the metric is reported")
    }

    /// A street over x 10..18, y 10..90, with a rim all round it — so every
    /// silhouette station is a kerb — and the given pavements.
    fn street(walks: &[(f64, f64)]) -> TileScene {
        let s = Site::new();
        let mut roads = vec![
            s.band("road_surface", (10.0, 18.0, 10.0, 90.0)),
            s.band("road_rim", (18.0, 18.35, 10.0, 90.0)),
            s.band("road_rim", (9.65, 10.0, 10.0, 90.0)),
            s.band("road_rim", (10.0, 18.0, 9.65, 10.0)),
            s.band("road_rim", (10.0, 18.0, 90.0, 90.35)),
        ];
        for &(y0, y1) in walks {
            roads.push(s.band("walk_surface", (18.35, 20.35, y0, y1)));
        }
        s.scene(16, roads)
    }

    #[test]
    fn a_pavement_the_whole_way_along_scores_zero_rather_than_nothing() {
        let m = run(&street(&[(10.0, 90.0)]));
        assert!(m.dist.count() >= 80, "east kerb stations: {}", m.dist.count());
        assert_eq!(m.violations(), 0);
        assert_eq!(m.dist.max(), Some(0.0));
    }

    #[test]
    fn a_pavement_that_stops_and_resumes_scores_the_gap_it_leaves() {
        // Two pavements with 6 m of bare kerb between them.
        let m = run(&street(&[(10.0, 40.0), (46.0, 90.0)]));
        let worst = m.worst_value().unwrap();
        assert!((worst - 6.0).abs() <= 1.0, "gap length {worst}");
        // Six stations in the gap, out of eighty along the east kerb.
        assert!((5..=7).contains(&(m.violations() as i64)), "violations {}", m.violations());
        assert!(!m.worst.is_empty());
    }

    #[test]
    fn a_pavement_that_simply_ends_is_not_a_gap() {
        // Bare kerb from y=40 to the street's end at y=90, then round the end
        // and down the west kerb: no pavement is ever met again.
        let m = run(&street(&[(10.0, 40.0)]));
        assert_eq!(m.violations(), 0, "worst {:?}", m.worst_value());
    }

    #[test]
    fn a_side_street_without_pavement_is_not_a_gap() {
        // 40 m of bare kerb between two pavements: longer than the corner
        // reach, so it is a street with no sidewalk rather than a gap.
        let m = run(&street(&[(10.0, 30.0), (70.0, 90.0)]));
        assert_eq!(m.violations(), 0, "worst {:?}", m.worst_value());
        assert!(m.dist.count() >= 60);
    }

    #[test]
    fn a_pavement_of_one_width_has_no_jog() {
        let m = by_id(&street(&[(10.0, 90.0)]), "street.walk_width_step");
        assert!(m.dist.count() >= 70, "pairs {}", m.dist.count());
        assert_eq!(m.violations(), 0, "worst {:?}", m.worst_value());
    }

    #[test]
    fn a_pavement_that_pulses_between_two_widths_saws() {
        // Alternating 2 m and 1.2 m in four-metre blocks: every block boundary
        // is an edge that goes out and comes back.
        let s = Site::new();
        let mut roads = vec![
            s.band("road_surface", (10.0, 18.0, 10.0, 90.0)),
            s.band("road_rim", (18.0, 18.35, 10.0, 90.0)),
            s.band("road_rim", (9.65, 10.0, 10.0, 90.0)),
            s.band("road_rim", (10.0, 18.0, 9.65, 10.0)),
            s.band("road_rim", (10.0, 18.0, 90.0, 90.35)),
        ];
        let mut y = 10.0;
        let mut wide = true;
        while y < 90.0 {
            let far = if wide { 20.35 } else { 19.55 };
            roads.push(s.band("walk_surface", (18.35, far, y, (y + 4.0).min(90.0))));
            y += 4.0;
            wide = !wide;
        }
        let m = by_id(&s.scene(16, roads), "street.walk_width_step");
        assert!(m.violations() >= 8, "violations {}", m.violations());
        let worst = m.worst_value().unwrap();
        assert!(worst >= 1.5, "wander {worst}");
    }

    #[test]
    fn a_pavement_that_narrows_once_is_a_taper_and_not_a_jog() {
        // Two m of pavement for half the street, then 1.2 m: one jog of
        // 0.8 m where they meet, and nothing along either stretch.
        let s = Site::new();
        let mut roads = vec![
            s.band("road_surface", (10.0, 18.0, 10.0, 90.0)),
            s.band("road_rim", (18.0, 18.35, 10.0, 90.0)),
            s.band("road_rim", (9.65, 10.0, 10.0, 90.0)),
            s.band("road_rim", (10.0, 18.0, 9.65, 10.0)),
            s.band("road_rim", (10.0, 18.0, 90.0, 90.35)),
            s.band("walk_surface", (18.35, 20.35, 10.0, 50.0)),
            s.band("walk_surface", (18.35, 19.55, 50.0, 90.0)),
        ];
        roads.push(s.band("walk_surface", (9.65 - 2.0, 9.65, 10.0, 90.0)));
        let m = by_id(&s.scene(16, roads), "street.walk_width_step");
        assert_eq!(m.violations(), 0, "worst {:?}", m.worst_value());
        assert!(m.dist.count() >= 60, "triples {}", m.dist.count());
    }

    #[test]
    fn a_zoom_that_meshes_no_bands_reports_a_skip() {
        let s = Site::new();
        let tile = s.scene(15, vec![s.band("road_surface", (10.0, 18.0, 10.0, 90.0))]);
        let m = run(&tile);
        assert!(m.skipped.is_some());
    }

    #[test]
    fn a_ring_road_with_pavement_on_two_legs_scores_the_bare_arc_between_them() {
        // A square ring of road (outer 10..90, hole 30..70) with a rim on the
        // outer silhouette; pavement on the south and east outer kerbs only.
        // Going round the outer loop: south served, east served, north and
        // west bare — 160 m of bare kerb between two pavements is past the
        // cap, so make the bare stretch a corner instead: pavement on the
        // south kerb and on the east kerb from y=22 on, leaving a 12 m corner.
        let s = Site::new();
        let ring = SurfaceMesh::from_parts(
            vec![
                s.ux(10.0), s.ux(90.0), s.ux(90.0), s.ux(10.0), // outer
                s.ux(30.0), s.ux(70.0), s.ux(70.0), s.ux(30.0), // hole
            ],
            vec![
                s.uy(10.0), s.uy(10.0), s.uy(90.0), s.uy(90.0),
                s.uy(30.0), s.uy(30.0), s.uy(70.0), s.uy(70.0),
            ],
            vec![100.0; 8],
            vec![
                0, 1, 5, 0, 5, 4, // south
                1, 2, 6, 1, 6, 5, // east
                2, 3, 7, 2, 7, 6, // north
                3, 0, 4, 3, 4, 7, // west
            ],
        )
        .unwrap();
        let mut roads = vec![
            RoadMesh { class: "road_surface".into(), level: 0, band: String::new(), fades: false, sheet: None, mesh: ring },
            s.band("road_rim", (10.0, 90.0, 9.65, 10.0)),
            s.band("road_rim", (90.0, 90.35, 10.0, 90.0)),
            s.band("road_rim", (10.0, 90.0, 90.0, 90.35)),
            s.band("road_rim", (9.65, 10.0, 10.0, 90.0)),
            s.band("walk_surface", (10.0, 90.0, 7.65, 9.65)),
            s.band("walk_surface", (90.35, 92.35, 22.0, 90.0)),
        ];
        // And pavement on the north and west kerbs, so the only bare stretch
        // is the 12 m of east kerb at the south-east corner.
        roads.push(s.band("walk_surface", (10.0, 90.0, 90.35, 92.35)));
        roads.push(s.band("walk_surface", (7.65, 9.65, 10.0, 90.0)));
        let m = run(&s.scene(16, roads));
        let worst = m.worst_value().unwrap();
        assert!((worst - 12.0).abs() <= 1.5, "gap length {worst}");
        assert!(m.violations() >= 10, "violations {}", m.violations());
    }
}
