//! Whether the drawn world connects the pedestrian network the data maps.
//!
//! The mapped graph is connected: sidewalks run both sides of every street and
//! wrap its corners, crosswalks join them across the carriageway at shared
//! connectors, footways feed in. Every edge of that graph is somebody's route.
//! The drawn world re-derives it as bands — and every place the derivation
//! declines a piece (a crossing left as paint that was never painted, a corner
//! stretch under the shortest band worth building, a way ending short of the
//! kerb it joins) the *drawn* network is disconnected where the mapped one is
//! not. A person reads that as sidewalks that end in mid-air.
//!
//! **The archive cannot answer this.** From `WALK_SURFACE_MIN_ZOOM` the
//! pedestrian strokes are deleted — the band is the surface — so the mapped
//! line exists only in the model, and only the model knows which stretches it
//! deliberately did not band. So this runs against the model: the walk lines
//! against the same band sources the union will buffer
//! ([`CarriagewayModel`]), which are the drawn truth after every narrowing and
//! drop has had its say.
//!
//! The claim is two-tiered, because the failure is. Three metrics, one walk:
//!
//! - **`network.walk_cover`** — every station of every pedestrian line, scored
//!   as the plan distance to the nearest drawn *hard surface* of any kind — a
//!   band, a carriageway or formation, a junction's paved extent. This is
//!   surface connectivity: past the threshold a person on the mapped route
//!   stands on bare ground.
//! - **`network.walk_material`** — the same stations of the *non-crossing*
//!   lines, scored against **walkable bands only**. A route can be continuous
//!   as asphalt and still read as broken: the corner stretch that drowned in
//!   the junction plate, the kerb strip a band never reached. A crossing is
//!   excluded because riding the carriageway is what a crossing *is*.
//! - **`network.walk_reach`** — for every endpoint the data joins to something
//!   (a shared connector, or another pedestrian line ending at the same
//!   point), the bare metres walking in from that endpoint before any drawn
//!   hard surface appears. Connectivity, weighted at the joints, where the eye
//!   reads it.

use geo_types::Coord;

use crate::priors::Surface;
use crate::scene::DEG_M;
use crate::synth::carriageway::CarriagewayModel;
use crate::verify::dist::Dist;
use crate::verify::{Invariant, Metric, Offender, Sense, Worst};

use super::Model;

/// Sample spacing along a pedestrian line, in metres — the same metre
/// `assemble::walks` stations at, and finer than any band feature.
const STATION_M: f64 = 1.0;

/// Step of the inward march from an endpoint, in metres. The value it
/// measures is a strip width (a kerb stub, a dropped corner), so a quarter
/// metre resolves it; the cap only bounds an approach that never finds
/// anything.
const REACH_STEP_M: f64 = 0.25;
const REACH_CAP_M: f64 = 20.0;

/// Half-size of the source query box, in metres: the widest half-width any
/// band or carriageway carries, plus slack, so a query at a station finds
/// every source that could cover it — and, for the cover distance, anything
/// within the threshold's reporting range.
const QUERY_M: f64 = 24.0;

/// Bare hard surface past this is a violation, in metres.
///
/// Reasoned, not read: a station on any drawn surface scores exactly zero and
/// the boolean kernel and quantization move an edge by centimetres, so half a
/// metre is an order above the machinery and under anything a person reads as
/// a gap in pavement.
const BARE_M: f64 = 0.5;

/// A mapped walking stretch further than this from any walkable band is a
/// violation, in metres — for the stretches that are *not* street-attached,
/// whose band follows their own polyline and so should be on it.
const MATERIAL_M: f64 = 1.5;

/// How far an *attached* stretch's band may legitimately stand from the
/// mapped line, in metres. The seat re-plots a sidewalk onto its host's
/// cross-section — clamped between kerb and facade, at the run's mean offset
/// while the mapped line wanders about it — so the drawn band is the mapped
/// line's stand-in anywhere within the seat's own play: a carriageway
/// half-width of slide plus the band and its clearances. Past this there is
/// no band standing in for the stretch at all — the run built nothing here.
const STANDIN_M: f64 = 8.0;

/// Bare approach past this is a violation, in metres — wider than any kerb
/// strip a band legitimately stops short by.
const REACH_BARE_M: f64 = 1.0;

/// A connector this close to a way's end, as a fraction of its length, is
/// that end's connector — `assemble`'s own `END_AT_EPS` is exact-end only;
/// this is looser because a crossing's connector is often one vertex in.
const END_AT: f64 = 0.05;

/// Two pedestrian endpoints within this, in metres, are one joint even
/// without a shared connector id — hand-mapped ways meet without sharing
/// nodes routinely.
const JOIN_EPS_M: f64 = 0.75;

/// Sample cap, as in the facade walk: a continental extract still answers in
/// bounded time, and the metric says so when it bites.
const MAX_SAMPLES: usize = 2_000_000;

/// The two cover distances at a point: to the nearest drawn hard surface of
/// any kind, and to the nearest walkable band.
struct Cover {
    hard: f64,
    band: f64,
}

pub fn check(m: &Model<'_>) -> Vec<Metric> {
    let mut cover_dist = Dist::metres();
    let mut cover_worst = Worst::new(Sense::HigherIsWorse, 8);
    let mut mat_dist = Dist::metres();
    let mut mat_worst = Worst::new(Sense::HigherIsWorse, 8);
    let mut reach_dist = Dist::metres();
    let mut reach_worst = Worst::new(Sense::HigherIsWorse, 8);
    let mut scratch: Vec<u32> = Vec::new();
    let debug = std::env::var_os("ARPT_DEBUG_NETWORK").is_some();
    // Attribution by bare length, in the fix's own categories: a crossing, a
    // claimed stretch whose host built nothing near it, the gap *between*
    // two claims (a corner), and the free stretches (a path).
    let mut bare_by = [0.0f64; 5];
    const CATS: [&str; 5] = ["crossing", "-", "missing", "corner-gap", "free"];

    // The joints: how many features carry each connector id, over pedestrian
    // lines and corridors alike — a count of two is a joint. Plus every
    // pedestrian endpoint, so two ways meeting without a shared node still
    // count as joined (`JOIN_EPS_M`).
    let mut carriers: std::collections::HashMap<u64, u32> = std::collections::HashMap::new();
    for (line, _) in m.scene.walks.lines() {
        let mut ids: Vec<u64> = line.connectors.iter().map(|c| c.id).collect();
        ids.sort_unstable();
        ids.dedup();
        for id in ids {
            *carriers.entry(id).or_insert(0) += 1;
        }
    }
    for c in m.scene.corridors.iter() {
        let mut ids: Vec<u64> = c.connectors.iter().map(|&(id, _)| id).collect();
        ids.sort_unstable();
        ids.dedup();
        for id in ids {
            *carriers.entry(id).or_insert(0) += 1;
        }
    }
    let mut ends: Vec<Coord> = Vec::new();
    for (line, _) in m.scene.walks.lines() {
        if line.line.len() >= 2 {
            ends.push(line.line[0]);
            ends.push(line.line[line.line.len() - 1]);
        }
    }

    let mut samples = 0usize;
    let mut truncated = false;
    let mut lines_read = 0u32;
    for (line, attached) in m.scene.walks.lines() {
        if line.line.len() < 2 {
            continue;
        }
        lines_read += 1;
        let cos_lat = crate::scene::run_cos_lat(&line.line);
        let arc = crate::scene::cumulative_arc(&line.line);
        let total = *arc.last().unwrap_or(&0.0);
        if !(total > 0.0) {
            continue;
        }

        // The cover walk.
        let mut s = 0.0;
        while s <= total {
            let f = s / total;
            let p = point_at(&line.line, &arc, s);
            // Not on the ground here — a footbridge, a subway. The way carries
            // its own deck there; no band is owed and none is measured.
            let carried = line.spans.iter().any(|&(a, b)| f >= a && f <= b);
            if carried || !m.bounds.contains(p.x, p.y) {
                s += STATION_M;
                continue;
            }
            if samples >= MAX_SAMPLES {
                truncated = true;
                break;
            }
            samples += 1;
            // `ARPT_PROBE_NETWORK="lon,lat"`: at the station nearest the
            // point, compare the grid query against a brute-force scan, so a
            // silent indexing miss cannot masquerade as a bare stretch.
            if let Ok(spec) = std::env::var("ARPT_PROBE_NETWORK") {
                if let Some((plon, plat)) = spec.split_once(',').and_then(|(a, b)| {
                    Some((a.trim().parse::<f64>().ok()?, b.trim().parse::<f64>().ok()?))
                }) {
                    if metric_close(p, Coord { x: plon, y: plat }, cos_lat, 1.0) {
                        let (rx, ry) = (QUERY_M / (DEG_M * cos_lat), QUERY_M / DEG_M);
                        m.junctions
                            .sources_near((p.x - rx, p.y - ry, p.x + rx, p.y + ry), &mut scratch);
                        let grid_n = scratch.len();
                        let mut brute = f64::MAX;
                        let mut brute_c = u32::MAX;
                        for i in 0..m.junctions.source_count() as u32 {
                            let src = m.junctions.source(i);
                            if !matches!(src.surface, Surface::Walkway | Surface::Path) {
                                continue;
                            }
                            let (d0, t) = project_m(p, src.a, src.b, src.cos_lat);
                            let d = d0 - src.drawn_half_at(t);
                            if d < brute {
                                brute = d;
                                brute_c = src.corridor;
                            }
                        }
                        eprintln!(
                            "[network probe] station {:.5},{:.5} s={s:.0}: grid hits {grid_n}, \
                             brute nearest band {brute:.2} m (corridor {brute_c})",
                            p.x, p.y
                        );
                    }
                }
            }
            let c = cover_at(m.junctions, p, cos_lat, &mut scratch);
            cover_dist.push(c.hard);
            if c.hard > BARE_M {
                cover_worst.offer(Offender {
                    lon: p.x,
                    lat: p.y,
                    zoom: m.solved.z_ref,
                    value: c.hard,
                    note: format!(
                        "{} of a mapped {}{}, and the nearest drawn surface of any kind is {:.1} m away",
                        describe_station(line, attached, s),
                        class_name(line),
                        if line.tagged { " (tagged sidewalk)" } else { "" },
                        c.hard,
                    ),
                });
            }
            if !line.crosswalk {
                // The stand-in test. An attached stretch's band is re-seated
                // onto its host's cross-section, so the mapped line is stood
                // in for by any band its *own host* built within the seat's
                // play; everything else follows its own polyline and its band
                // should be on it.
                let host = attached
                    .iter()
                    .find(|a| s >= a.walk0 && s <= a.walk1)
                    .map(|a| a.host);
                let v = match host {
                    Some(host) => {
                        let own = own_host_band_m(m.junctions, p, cos_lat, host, &mut scratch);
                        if own <= STANDIN_M {
                            0.0
                        } else {
                            c.band.max(STANDIN_M)
                        }
                    }
                    None => c.band,
                };
                mat_dist.push(v);
                let bare = v > if host.is_some() { STANDIN_M } else { MATERIAL_M };
                if bare {
                    if debug {
                        let cat = match category(line, attached, s) {
                            1 => 2, // attached and the host built nothing here
                            c if c > 1 => c + 1,
                            c => c,
                        };
                        bare_by[cat] += STATION_M;
                    }
                    mat_worst.offer(Offender {
                        lon: p.x,
                        lat: p.y,
                        zoom: m.solved.z_ref,
                        value: v,
                        note: format!(
                            "{} of a mapped {}{}, drawn as no walkable band — the nearest is {:.1} m away",
                            describe_station(line, attached, s),
                            class_name(line),
                            if line.tagged { " (tagged sidewalk)" } else { "" },
                            v,
                        ),
                    });
                }
            } else if debug && c.hard > BARE_M {
                bare_by[0] += STATION_M;
            }
            s += STATION_M;
        }

        // The reach walk: both ends, where the data joins them to something.
        if line.crosswalk {
            continue; // a crossing's own ends are the stubs its registration owns
        }
        for end in [0usize, 1] {
            let (end_arc, at_end): (f64, Box<dyn Fn(f64) -> bool>) = if end == 0 {
                (0.0, Box::new(|at: f64| at <= END_AT))
            } else {
                (total, Box::new(|at: f64| at >= 1.0 - END_AT))
            };
            let p_end = point_at(&line.line, &arc, end_arc);
            if !m.bounds.contains(p_end.x, p_end.y) {
                continue;
            }
            let f_end = end_arc / total;
            if line.spans.iter().any(|&(a, b)| f_end >= a && f_end <= b) {
                continue; // an elevated end: the deck's business
            }
            let joined_by_id = line
                .connectors
                .iter()
                .any(|c| at_end(c.at) && carriers.get(&c.id).copied().unwrap_or(0) >= 2);
            let joined_by_touch = || {
                ends.iter()
                    .filter(|e| metric_close(**e, p_end, cos_lat, JOIN_EPS_M))
                    .count()
                    > 1 // itself, plus somebody else's end
            };
            if !joined_by_id && !joined_by_touch() {
                continue; // a true dead end owes nothing
            }
            let mut t = 0.0;
            let mut uncovered = REACH_CAP_M;
            while t <= REACH_CAP_M && t <= total {
                let p = point_at(&line.line, &arc, if end == 0 { t } else { total - t });
                if cover_at(m.junctions, p, cos_lat, &mut scratch).hard <= 0.0 {
                    uncovered = t;
                    break;
                }
                t += REACH_STEP_M;
            }
            reach_dist.push(uncovered);
            if uncovered > REACH_BARE_M {
                reach_worst.offer(Offender {
                    lon: p_end.x,
                    lat: p_end.y,
                    zoom: m.solved.z_ref,
                    value: uncovered,
                    note: format!(
                        "a mapped {}{} joins the network here and its first {uncovered:.1} m are bare ground",
                        class_name(line),
                        if line.tagged { " (tagged sidewalk)" } else { "" },
                    ),
                });
            }
        }
    }
    if debug {
        let t: f64 = bare_by.iter().sum();
        eprint!("[network] bare length {:.2} km  by:", t / 1000.0);
        for (i, name) in CATS.iter().enumerate() {
            eprint!("  {name} {:.1} %", if t > 0.0 { 100.0 * bare_by[i] / t } else { 0.0 });
        }
        eprintln!();
    }

    let mut cover_population = format!(
        "Every {STATION_M:.0} m of every mapped pedestrian line inside the bbox ({lines_read} \
         lines), on-ground stretches only, scored as the plan distance to the nearest drawn \
         hard surface of any kind — a walk or path band, a carriageway or formation, or a \
         junction's paved extent. Zero on any drawn surface, so the rate is the share of \
         mapped pedestrian length a person crosses bare ground for."
    );
    if truncated {
        cover_population.push_str(" Coverage: the sample cap bit; a full walk would find more.");
    }
    vec![
        Metric {
            id: "network.walk_cover".into(),
            invariant: Invariant::I4,
            title: "Mapped pedestrian ways with no drawn surface at all".into(),
            population: cover_population,
            detail: format!(
                "The mapped pedestrian graph is connected; the drawn world re-derives it as \
                 bands and loses pieces — a corner stretch under the shortest band worth \
                 building, a crossing's kerb stub, a dropped or refused band. Each loss draws \
                 as a route interrupted by bare ground. {BARE_M:.1} m is an order above the \
                 quantization and under anything a person reads as a hole in pavement."
            ),
            sense: Sense::HigherIsWorse,
            threshold: BARE_M,
            skipped: None,
            dist: cover_dist,
            worst: cover_worst.into_vec(),
        },
        Metric {
            id: "network.walk_material".into(),
            invariant: Invariant::I4,
            title: "Mapped walking routes drawn as no walkable band".into(),
            population: format!(
                "The same stations, non-crossing lines only. An attached stretch scores zero \
                 when its own host built a walk band within the seat's play ({STANDIN_M:.0} m \
                 — the band is the mapped line's re-seated stand-in), and its distance to the \
                 nearest band otherwise; an unattached stretch scores its distance to the \
                 nearest walkable band, which should be on its own polyline."
            ),
            detail: format!(
                "A route continuous as asphalt can still read as broken: the run a street \
                 claimed and then built nothing for, the corner that drowned in the junction \
                 plate, the kerb strip a band never reached. Past {MATERIAL_M:.1} m \
                 (unattached) or {STANDIN_M:.0} m (attached) no band stands in for the \
                 stretch and a person walking the mapped route walks on the carriageway or \
                 on bare ground."
            ),
            sense: Sense::HigherIsWorse,
            threshold: MATERIAL_M,
            skipped: None,
            dist: mat_dist,
            worst: mat_worst.into_vec(),
        },
        Metric {
            id: "network.walk_reach".into(),
            invariant: Invariant::I4,
            title: "Pedestrian ways that stop short of the network they join".into(),
            population: format!(
                "Every endpoint of a mapped pedestrian line that the data joins to something \
                 else — a connector another feature carries, or another pedestrian line ending \
                 within {JOIN_EPS_M} m — scored as the bare metres walking in from that \
                 endpoint before any drawn hard surface appears (capped at \
                 {REACH_CAP_M:.0} m). The joints are where the eye reads connectivity, and \
                 where a band that stopped short is a link that visibly is not there."
            ),
            detail: format!(
                "A way's band can stop short of its own endpoint — the seat ran out of room, \
                 the stretch was under the shortest band worth building, the crossing it joins \
                 was never drawn — and every such stop breaks the drawn network at exactly a \
                 place the mapped one connects. Past {REACH_BARE_M:.1} m the gap is wider than \
                 any kerb strip and reads as a dead end."
            ),
            sense: Sense::HigherIsWorse,
            threshold: REACH_BARE_M,
            skipped: None,
            dist: reach_dist,
            worst: reach_worst.into_vec(),
        },
    ]
}

/// The cover at a point: the plan distance to the nearest drawn hard surface
/// of any kind, and to the nearest walkable band, in metres — zero on one.
///
/// Cover is a source within its own drawn half-width, or (hard only) a
/// junction's paved extent. The half-width read is the source's `half_m`;
/// where a facade section narrowed one side the test is optimistic by that
/// narrowing, which is at most the walk band the narrowing made room for.
fn cover_at(
    junctions: &CarriagewayModel,
    p: Coord,
    cos_lat: f64,
    scratch: &mut Vec<u32>,
) -> Cover {
    let (rx, ry) = (QUERY_M / (DEG_M * cos_lat), QUERY_M / DEG_M);
    junctions.sources_near((p.x - rx, p.y - ry, p.x + rx, p.y + ry), scratch);
    let mut hard = f64::MAX;
    let mut band = f64::MAX;
    for &i in scratch.iter() {
        let s = junctions.source(i);
        let (d0, t) = project_m(p, s.a, s.b, s.cos_lat);
        // The width the band is *drawn* at. `half_m` became the run's chaining
        // key when the pavement turned into a side of a corridor, and reading
        // it here would report a strip the room narrowed as covering ground it
        // does not.
        let d = d0 - s.drawn_half_at(t);
        if d < hard {
            hard = d;
        }
        if matches!(s.surface, Surface::Walkway | Surface::Path) && d < band {
            band = d;
        }
        if hard <= 0.0 && band <= 0.0 {
            break;
        }
    }
    // Only where no source answers for it: a curb-return fillet is paved but
    // inside nobody's buffer, and a crossing at a junction mouth sits on it.
    if hard > 0.0 {
        for j in junctions.near((p.x - rx, p.y - ry, p.x + rx, p.y + ry)) {
            if j.area().contains(p) {
                hard = 0.0;
                break;
            }
        }
    }
    Cover {
        hard: hard.clamp(0.0, QUERY_M),
        band: band.clamp(0.0, QUERY_M),
    }
}

/// The distance from `p` to the nearest walk band built by `host`, in metres
/// — how far away the band standing in for an attached stretch actually is.
fn own_host_band_m(
    junctions: &CarriagewayModel,
    p: Coord,
    cos_lat: f64,
    host: u32,
    scratch: &mut Vec<u32>,
) -> f64 {
    let (rx, ry) = (QUERY_M / (DEG_M * cos_lat), QUERY_M / DEG_M);
    junctions.sources_near((p.x - rx, p.y - ry, p.x + rx, p.y + ry), scratch);
    let mut best = f64::MAX;
    for &i in scratch.iter() {
        let s = junctions.source(i);
        if s.surface != Surface::Walkway || s.corridor != host {
            continue;
        }
        let (d0, t) = project_m(p, s.a, s.b, s.cos_lat);
        let d = d0 - s.drawn_half_at(t);
        if d < best {
            best = d;
        }
        if best <= 0.0 {
            break;
        }
    }
    best.max(0.0)
}

/// Which raw category a bare station falls in: a crossing (0), a stretch some
/// street claimed (1 — the caller splits displaced from missing), the gap
/// between two claims (2, a corner), or the free stretches of a path (3).
fn category(
    line: &crate::assemble::walks::WalkLine,
    attached: &[crate::assemble::walks::Attachment],
    s: f64,
) -> usize {
    if line.crosswalk {
        return 0;
    }
    if attached.iter().any(|a| s >= a.walk0 && s <= a.walk1) {
        return 1;
    }
    let before = attached.iter().any(|a| a.walk1 < s);
    let after = attached.iter().any(|a| a.walk0 > s);
    if before && after {
        2
    } else {
        3
    }
}

/// A short description of where along its way a bare station sits.
fn describe_station(
    line: &crate::assemble::walks::WalkLine,
    attached: &[crate::assemble::walks::Attachment],
    s: f64,
) -> &'static str {
    match category(line, attached, s) {
        0 => "the crossing line",
        1 => "a street-attached stretch",
        2 => "the corner stretch between two attachments",
        _ => "a free stretch",
    }
}

fn class_name(line: &crate::assemble::walks::WalkLine) -> &'static str {
    if line.crosswalk {
        "crossing"
    } else {
        match line.kind {
            crate::priors::Kind::Road(crate::priors::RoadClass::Steps) => "stair",
            crate::priors::Kind::Road(crate::priors::RoadClass::Cycleway) => "cycleway",
            _ => "footway",
        }
    }
}

/// The point at arc `s` along a polyline with cumulative arc `arc`.
fn point_at(line: &[Coord], arc: &[f64], s: f64) -> Coord {
    let i = arc.partition_point(|&a| a < s).clamp(1, line.len() - 1);
    let (a0, a1) = (arc[i - 1], arc[i]);
    let t = if a1 > a0 { ((s - a0) / (a1 - a0)).clamp(0.0, 1.0) } else { 0.0 };
    Coord {
        x: line[i - 1].x + (line[i].x - line[i - 1].x) * t,
        y: line[i - 1].y + (line[i].y - line[i - 1].y) * t,
    }
}

/// Plan distance from `p` to the segment `a`–`b`, in metres.
/// `(distance in metres, parameter along a→b)` of the closest point to `p`.
fn project_m(p: Coord, a: Coord, b: Coord, cos_lat: f64) -> (f64, f64) {
    let m_lon = DEG_M * cos_lat;
    let (ex, ey) = ((b.x - a.x) * m_lon, (b.y - a.y) * DEG_M);
    let (qx, qy) = ((p.x - a.x) * m_lon, (p.y - a.y) * DEG_M);
    let len2 = ex * ex + ey * ey;
    let u = if len2 > 0.0 { ((qx * ex + qy * ey) / len2).clamp(0.0, 1.0) } else { 0.0 };
    ((qx - ex * u).hypot(qy - ey * u), u)
}

fn point_to_segment_m(p: Coord, a: Coord, b: Coord, cos_lat: f64) -> f64 {
    let m_lon = DEG_M * cos_lat;
    let (ex, ey) = ((b.x - a.x) * m_lon, (b.y - a.y) * DEG_M);
    let (qx, qy) = ((p.x - a.x) * m_lon, (p.y - a.y) * DEG_M);
    let len2 = ex * ex + ey * ey;
    let u = if len2 > 0.0 { ((qx * ex + qy * ey) / len2).clamp(0.0, 1.0) } else { 0.0 };
    let (fx, fy) = (qx - ex * u, qy - ey * u);
    fx.hypot(fy)
}

fn metric_close(a: Coord, b: Coord, cos_lat: f64, eps_m: f64) -> bool {
    let m_lon = DEG_M * cos_lat;
    let (dx, dy) = ((a.x - b.x) * m_lon, (a.y - b.y) * DEG_M);
    dx * dx + dy * dy <= eps_m * eps_m
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_point_walker_interpolates_between_vertices() {
        let line = vec![Coord { x: 6.0, y: 46.0 }, Coord { x: 6.0, y: 46.0 + 100.0 / DEG_M }];
        let arc = crate::scene::cumulative_arc(&line);
        let p = point_at(&line, &arc, 50.0);
        assert!((p.y - (46.0 + 50.0 / DEG_M)).abs() * DEG_M < 0.5, "{p:?}");
    }

    #[test]
    fn distance_to_a_segment_is_zero_on_it_and_lateral_off_it() {
        let cos = 46.0f64.to_radians().cos();
        let a = Coord { x: 6.0, y: 46.0 };
        let b = Coord { x: 6.0 + 100.0 / (DEG_M * cos), y: 46.0 };
        let on = Coord { x: 6.0 + 50.0 / (DEG_M * cos), y: 46.0 };
        let off = Coord { x: 6.0 + 50.0 / (DEG_M * cos), y: 46.0 + 7.0 / DEG_M };
        assert!(point_to_segment_m(on, a, b, cos) < 1e-6);
        assert!((point_to_segment_m(off, a, b, cos) - 7.0).abs() < 0.01);
    }
}
