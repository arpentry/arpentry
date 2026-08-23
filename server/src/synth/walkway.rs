//! The band a pedestrian way is drawn as (docs/ROADS.md invariant 1).
//!
//! A footway used to be a stroke and nothing else — `Surface::None`, no
//! corridor, no width to buffer — so at the detail zooms where every road
//! became a meshed surface a sidewalk stayed a cartographic line painted over
//! whatever it happened to cross. Here it becomes a surface like the rest: a
//! [`priors::Surface::Walkway`] band that joins the union, takes its own hole
//! out of the drawn ground, and wears an apron where it stands above it.
//!
//! **Two kinds of band, one material.**
//!
//! - *Attached* — the way runs alongside a street (`assemble::walks`). Its
//!   shape comes from the **host's** smoothed centerline, offset laterally, not
//!   from its own mapped polyline: that is what makes a sidewalk parallel to
//!   its kerb, constant in width, and free of the wobble a hand-drawn footway
//!   carries. The polyline decides *where a sidewalk is* and *which side*;
//!   it never decides what shape one has. Its height is the host's road
//!   surface plus [`priors::KERB_RISE_M`].
//! - *Unattached* — the rest of every pedestrian line, which is a path across
//!   open ground. It is buffered along its own polyline and stands on the
//!   ground, because there is nothing else for it to stand on.
//!
//! The two meet along one way with no seam: they are segments of one
//! `Surface::Walkway` run, the union merges them, and the height field blends
//! the kerb rise out over a band's width where one becomes the other — a
//! dropped kerb, which is what is really there.
//!
//! **The room is spent in order.** A band takes what is left between the drawn
//! kerb and [`priors::FACADE_CLEAR_M`] short of the facade, up to
//! [`priors::WALK_WIDTH_M`]. Where less than [`priors::WALK_MIN_WIDTH_M`] is
//! left there is no band, which is what a street too narrow for a sidewalk
//! looks like. That is the same room `synth::carriageway` allotted the asphalt
//! out of, read through the same `Room` (docs/ROADS.md invariant 1).

use geo_types::Coord;

use crate::assemble::facades::{Facades, Section};
use crate::assemble::walks::{Attachment, WalkLine};
use crate::priors;
use crate::scene::{Corridor, CorridorId, SceneGraph, SpanKind, DEG_M};
use crate::solve::SolvedModel;

use super::carriageway::{level_runs, sections_along, GradeRun, SourceSeg, RUN_EPS_M};

/// The corridor id a band with no host carries. Nothing resolves it to a
/// profile (`SolvedModel::profile` answers `None` past the end of its list),
/// which is exactly right: a path across a field has no road to ride, so its
/// height is the ground.
const NO_HOST: CorridorId = CorridorId::MAX;

/// Station spacing along an unattached path, in metres. The band follows the
/// way's own mapped line there, so this only bounds how coarsely a long
/// straight is buffered; an attached band is stationed by its host's profile
/// instead, like the carriageway.
const PATH_STATION_M: f64 = 8.0;

/// Shortest band worth building, in metres. Below this the buffer is a blob
/// whose casing is most of it.
const MIN_BAND_M: f64 = 4.0;

/// Builds every walkway band in the extract, as sources for the union.
///
/// They are *not* handed to `synth::sheets`: a band's sheet is its host's, and
/// letting a sidewalk vote on the grade-separation layering would let the thing
/// standing on a surface decide what that surface is.
pub fn bake(
    scene: &SceneGraph,
    solved: &SolvedModel,
    facades: &Facades,
    grade_runs: &[GradeRun],
) -> Vec<SourceSeg> {
    let mut out = Vec::new();
    if std::env::var_os("ARPT_NO_WALK_BAND").is_some() {
        return out;
    }
    let mut scratch: Vec<u32> = Vec::new();
    for (line, attached) in scene.walks.lines() {
        if line.crosswalk || line.line.len() < 2 {
            continue; // a crossing is paint on the carriageway, not a band
        }
        for a in attached {
            attached_band(scene, solved, facades, grade_runs, a, &mut out, &mut scratch);
        }
        path_bands(line, attached, &mut out);
    }
    out
}

/// The band of one attachment: the host's own centerline, offset to the side
/// the way is on, over the stretch of it the way covers.
fn attached_band(
    scene: &SceneGraph,
    solved: &SolvedModel,
    facades: &Facades,
    grade_runs: &[GradeRun],
    a: &Attachment,
    out: &mut Vec<SourceSeg>,
    scratch: &mut Vec<u32>,
) {
    let Some(c) = scene.corridors.get(a.host as usize) else { return };
    let Some(half_m) = super::carriageway::corridor_half_width_m(c) else { return };
    let profile = solved.profile(c.id);
    for (r0, r1, level, kind) in level_runs(c) {
        // **Only where the host is on the ground.** Over a bridge or in a bore
        // the sidewalk is carried by the structure itself (`synth::carried`),
        // and a band drawn there would be a second one floating beside the
        // deck — 1.5 % of attached host arc, measured in phase 3.
        if kind != SpanKind::Grade {
            continue;
        }
        let (lo, hi) = (a.arc0.max(r0), a.arc1.min(r1));
        if hi - lo < MIN_BAND_M {
            continue;
        }
        let layer = grade_runs
            .iter()
            .find(|g| g.corridor == c.id && g.arc0 <= lo + RUN_EPS_M && g.arc1 >= hi - RUN_EPS_M)
            .map_or(0, |g| g.layer);
        // The host's own stations, so the band is sampled on the same curve the
        // asphalt is buffered around and the two stay parallel by construction.
        let stations: Vec<f64> = match profile {
            Some(p) => p.arc().iter().copied().filter(|&s| s > lo && s < hi).collect(),
            None => c.arc.iter().copied().filter(|&s| s > lo && s < hi).collect(),
        };
        let mut stops: Vec<f64> = vec![lo];
        for s in stations.into_iter().chain(std::iter::once(hi)) {
            if s - stops[stops.len() - 1] > RUN_EPS_M {
                stops.push(s);
            }
        }
        if stops.len() < 2 {
            continue;
        }
        let point = |arc: f64| match profile {
            Some(p) => p.smooth_at_arc(arc),
            None => super::carriageway::raw_point_at_arc(c, arc),
        };
        let pts: Vec<Coord> = stops.iter().map(|&s| point(s)).collect();
        // The asphalt's own cross-section here — the kerb the band starts at.
        let sections = sections_along(c, &stops, &pts, half_m, facades, false, scratch);
        let side = a.side as usize;
        // Where each station's band sits: its centre offset from the host
        // centerline, and its half-width. `None` where the room ran out.
        let seats: Vec<Option<(f64, f64)>> = (0..stops.len())
            .map(|i| seat(c, &stops, &pts, i, sections[i], side, a.offset_m, facades, scratch))
            .collect();
        let height = |arc: f64| profile.map_or(0.0, |p| p.road_at_arc(arc));
        for i in 0..stops.len() - 1 {
            let (Some((oa, ha)), Some((ob, hb))) = (seats[i], seats[i + 1]) else {
                continue; // the band stops where the room does
            };
            // One half-width per segment, so the run chains: the union strokes
            // a polyline at a constant width, and a taper would be a stack of
            // one-segment runs. The narrower of the two ends keeps the band
            // clear of the facade at both.
            let half = ha.min(hb);
            if 2.0 * half < priors::WALK_MIN_WIDTH_M {
                continue;
            }
            let normal_a = normal_at(&pts, i, c.cos_lat, side);
            let normal_b = normal_at(&pts, i + 1, c.cos_lat, side);
            out.push(SourceSeg {
                a: offset(pts[i], normal_a, oa, c.cos_lat),
                b: offset(pts[i + 1], normal_b, ob, c.cos_lat),
                cos_lat: c.cos_lat,
                half_m: half,
                sect_a: Section::uniform(half),
                sect_b: Section::uniform(half),
                level,
                layer,
                cut_a: None,
                cut_b: None,
                height_a: height(stops[i]) + priors::KERB_RISE_M,
                height_b: height(stops[i + 1]) + priors::KERB_RISE_M,
                corridor: c.id,
                surface: priors::Surface::Walkway,
                rise_m: priors::KERB_RISE_M,
            });
        }
    }
}

/// Where one station's band sits: `(offset from the centerline, half-width)`,
/// or `None` where there is not [`priors::WALK_MIN_WIDTH_M`] of room for one.
///
/// **Seated on the mapped way, pushed clear of the kerb, clipped at the
/// facade.** Three rules, in that priority:
///
/// - The band is [`priors::WALK_WIDTH_M`] wide, narrowing only where the strip
///   between kerb and facade is narrower than that.
/// - It sits where the way was measured (`Attachment::offset_m`). A band pinned
///   to the kerb regardless would be right for the median sidewalk — phase 3
///   measured p50 1.1 m clear of the kerb — and wrong for the p90 at 3.1 m and
///   the tail past 5 m, drawing the sidewalk metres from where the mapper put
///   it while its own stroke is gone.
/// - It never overlaps the carriageway and never crosses
///   [`priors::FACADE_CLEAR_M`] short of a wall. Those two are the bounds the
///   seat is clamped into, so a way mapped *inside* its own street's kerb — a
///   footway digitized down the middle of a lane — still comes out beside it
///   rather than on it.
#[allow(clippy::too_many_arguments)]
fn seat(
    c: &Corridor,
    stops: &[f64],
    pts: &[Coord],
    i: usize,
    section: Section,
    side: usize,
    want_m: f64,
    facades: &Facades,
    scratch: &mut Vec<u32>,
) -> Option<(f64, f64)> {
    let kerb = section.on(side);
    // Look far enough out to seat the band where the way is *and* clear its
    // far edge — never less than a kerb-hugging band's own reach, or open
    // ground would report exactly the room a band needs and the seat would
    // have nowhere to slide.
    let want = want_m.max(kerb);
    let reach = (want + priors::WALK_WIDTH_M * 0.5 + priors::FACADE_CLEAR_M)
        .max(kerb + priors::WALK_WIDTH_M + priors::FACADE_CLEAR_M);
    let room = room_at(c, stops, pts, i, side, reach, facades, scratch);
    let avail = room - priors::FACADE_CLEAR_M - kerb;
    if avail < priors::WALK_MIN_WIDTH_M {
        return None;
    }
    let half = avail.min(priors::WALK_WIDTH_M) * 0.5;
    Some((want.clamp(kerb + half, kerb + avail - half), half))
}

/// The room on `side` at station `i`, read exactly as the carriageway read it
/// (`synth::carriageway::sections_along`) so the two allocations are one
/// measurement of one street.
#[allow(clippy::too_many_arguments)]
fn room_at(
    c: &Corridor,
    stops: &[f64],
    pts: &[Coord],
    i: usize,
    side: usize,
    reach: f64,
    facades: &Facades,
    scratch: &mut Vec<u32>,
) -> f64 {
    if facades.is_empty() {
        return reach;
    }
    let m_lon = DEG_M * c.cos_lat;
    let (j, k) = (i.saturating_sub(1), (i + 1).min(stops.len() - 1));
    if j == k {
        return reach;
    }
    let (dx, dy) = ((pts[k].x - pts[j].x) * m_lon, (pts[k].y - pts[j].y) * DEG_M);
    let len = dx.hypot(dy);
    if !(len > 0.0) {
        return reach;
    }
    let window = (stops[k] - stops[j])
        .max(super::carriageway::ROOM_WINDOW_MIN_M)
        .min(super::carriageway::ROOM_WINDOW_MAX_M);
    facades.room(pts[i], c.cos_lat, (dx / len, dy / len), reach, window, scratch).on(side)
}

/// The unit normal at station `i`, pointing to `side` (0 left, 1 right) — the
/// same handedness `assemble::walks` measured the offset with.
fn normal_at(pts: &[Coord], i: usize, cos_lat: f64, side: usize) -> (f64, f64) {
    let m_lon = DEG_M * cos_lat;
    let (j, k) = (i.saturating_sub(1), (i + 1).min(pts.len() - 1));
    let (dx, dy) = ((pts[k].x - pts[j].x) * m_lon, (pts[k].y - pts[j].y) * DEG_M);
    let len = dx.hypot(dy);
    if !(len > 0.0) {
        return (0.0, 0.0);
    }
    // Left of the direction of travel is `(-dy, dx)`; side 1 is the other.
    let s = if side == 0 { 1.0 } else { -1.0 };
    (-s * dy / len, s * dx / len)
}

/// `p` moved `d` metres along a local east/north unit vector.
fn offset(p: Coord, n: (f64, f64), d: f64, cos_lat: f64) -> Coord {
    let m_lon = DEG_M * cos_lat;
    Coord { x: p.x + n.0 * d / m_lon, y: p.y + n.1 * d / DEG_M }
}

/// The bands for the stretches of a way that attached to nothing: a path,
/// buffered along its own line and standing on the ground.
fn path_bands(line: &WalkLine, attached: &[Attachment], out: &mut Vec<SourceSeg>) {
    let cos_lat = crate::scene::run_cos_lat(&line.line);
    let arc = crate::scene::cumulative_arc(&line.line);
    let total = *arc.last().unwrap_or(&0.0);
    // The complement of the attached stretches, in the way's own arc. They come
    // in walk-arc order along one line, so one sweep is enough.
    // What the band must skip: the stretches a sidewalk already covers, and the
    // stretches that are not on the ground at all. Both are "something else is
    // the walkway here", and both are cut out the same way.
    let mut taken: Vec<(f64, f64)> = attached.iter().map(|a| (a.walk0, a.walk1)).collect();
    taken.extend(line.spans.iter().map(|&(s, e)| (s * total, e * total)));
    taken.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut cursor = 0.0f64;
    let mut gaps: Vec<(f64, f64)> = Vec::new();
    for (w0, w1) in taken.into_iter().chain(std::iter::once((total, total))) {
        // The ranges come from two measurements of the same line — station
        // counts and level-run fractions — so neither is guaranteed to land
        // inside `total`. Order the clamp so a range past the end closes the
        // sweep rather than inverting it.
        let lo = cursor.min(total);
        let hi = w0.clamp(lo, total);
        cursor = cursor.max(w1);
        if hi - lo > MIN_BAND_M {
            gaps.push((lo, hi));
        }
    }
    let half = priors::WALK_WIDTH_M * 0.5;
    for (g0, g1) in gaps {
        let pts = resample(&line.line, &arc, g0, g1, PATH_STATION_M);
        for w in pts.windows(2) {
            out.push(SourceSeg {
                a: w[0],
                b: w[1],
                cos_lat,
                half_m: half,
                sect_a: Section::uniform(half),
                sect_b: Section::uniform(half),
                level: 0,
                layer: 0,
                cut_a: None,
                cut_b: None,
                // A path stands on the ground. The height field reads that off
                // `NO_HOST`'s absent profile, so these are only what
                // `synth::sheets` would have compared — and it never sees them.
                height_a: 0.0,
                height_b: 0.0,
                corridor: NO_HOST,
                surface: priors::Surface::Path,
                rise_m: 0.0,
            });
        }
    }
}

/// The stretch `[from_m, to_m]` of a polyline, stationed at most `step_m`
/// apart and keeping every mapped vertex in between.
fn resample(line: &[Coord], arc: &[f64], from_m: f64, to_m: f64, step_m: f64) -> Vec<Coord> {
    let at = |s: f64| -> Coord {
        let i = arc.partition_point(|&a| a < s).clamp(1, line.len() - 1);
        let (a0, a1) = (arc[i - 1], arc[i]);
        let t = if a1 > a0 { (s - a0) / (a1 - a0) } else { 0.0 };
        Coord {
            x: line[i - 1].x + (line[i].x - line[i - 1].x) * t,
            y: line[i - 1].y + (line[i].y - line[i - 1].y) * t,
        }
    };
    let mut stops: Vec<f64> = vec![from_m];
    for (i, &a) in arc.iter().enumerate() {
        if a <= from_m || a >= to_m {
            continue;
        }
        // Keep the vertex, and fill the straight leading to it.
        let prev = stops[stops.len() - 1];
        let n = ((a - prev) / step_m).floor() as usize;
        for k in 1..=n {
            stops.push(prev + step_m * k as f64);
        }
        if a - stops[stops.len() - 1] > RUN_EPS_M {
            stops.push(a);
        }
        let _ = i;
    }
    let prev = stops[stops.len() - 1];
    let n = ((to_m - prev) / step_m).floor() as usize;
    for k in 1..=n {
        stops.push(prev + step_m * k as f64);
    }
    if to_m - stops[stops.len() - 1] > RUN_EPS_M {
        stops.push(to_m);
    }
    stops.into_iter().map(at).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::priors::{Kind, RoadClass};

    const LAT: f64 = 46.44;

    fn east(m: f64) -> f64 {
        m / (DEG_M * LAT.to_radians().cos())
    }

    fn line(from_m: f64, to_m: f64, north_m: f64) -> Vec<Coord> {
        vec![
            Coord { x: 6.9 + east(from_m), y: LAT + north_m / DEG_M },
            Coord { x: 6.9 + east(to_m), y: LAT + north_m / DEG_M },
        ]
    }

    fn walk_line(pts: Vec<Coord>) -> WalkLine {
        WalkLine {
            source: 1,
            line: pts,
            kind: Kind::Road(RoadClass::Footway),
            tagged: true,
            crosswalk: false,
            connectors: Vec::new(),
            spans: Vec::new(),
        }
    }

    #[test]
    fn a_path_with_no_attachment_is_banded_along_its_whole_length() {
        let w = walk_line(line(0.0, 40.0, 0.0));
        let mut out = Vec::new();
        path_bands(&w, &[], &mut out);
        assert!(!out.is_empty());
        assert!(out.iter().all(|s| s.corridor == NO_HOST && s.rise_m == 0.0));
        let span: f64 = out
            .iter()
            .map(|s| crate::scene::metric_len(s.a, s.b, s.cos_lat))
            .sum();
        assert!((span - 40.0).abs() < 0.5, "{span}");
    }

    #[test]
    fn the_attached_stretch_is_left_to_the_sidewalk() {
        // 100 m of way, attached from 20 to 60: the path bands are the two ends.
        let w = walk_line(line(0.0, 100.0, 0.0));
        let a = Attachment {
            walk: 1,
            line: 0,
            walk0: 20.0,
            walk1: 60.0,
            kind: Kind::Road(RoadClass::Footway),
            host: 0,
            side: 0,
            arc0: 20.0,
            arc1: 60.0,
            offset_m: 5.0,
            spread_m: 0.0,
            evidence: crate::assemble::walks::Evidence::Tag,
        };
        let mut out = Vec::new();
        path_bands(&w, &[a], &mut out);
        let span: f64 = out
            .iter()
            .map(|s| crate::scene::metric_len(s.a, s.b, s.cos_lat))
            .sum();
        assert!((span - 60.0).abs() < 0.5, "20 + 40 metres of path, got {span}");
    }

    #[test]
    fn a_nick_of_a_gap_is_not_worth_a_band() {
        let w = walk_line(line(0.0, 100.0, 0.0));
        let stub = |w0: f64, w1: f64| Attachment {
            walk: 1,
            line: 0,
            walk0: w0,
            walk1: w1,
            kind: Kind::Road(RoadClass::Footway),
            host: 0,
            side: 0,
            arc0: w0,
            arc1: w1,
            offset_m: 5.0,
            spread_m: 0.0,
            evidence: crate::assemble::walks::Evidence::Tag,
        };
        // Two attachments 2 m apart: the gap between them is not a path.
        let mut out = Vec::new();
        path_bands(&w, &[stub(0.0, 49.0), stub(51.0, 100.0)], &mut out);
        assert!(out.is_empty(), "{out:?}", out = out.len());
    }

    #[test]
    fn the_normal_points_to_the_side_the_way_is_on() {
        // An east-running line: side 0 (left) is north.
        let pts = vec![Coord { x: 6.9, y: LAT }, Coord { x: 6.9 + east(10.0), y: LAT }];
        let n = normal_at(&pts, 0, LAT.to_radians().cos(), 0);
        assert!(n.1 > 0.9, "left of east is north: {n:?}");
        let n = normal_at(&pts, 0, LAT.to_radians().cos(), 1);
        assert!(n.1 < -0.9, "right of east is south: {n:?}");
    }

    #[test]
    fn the_band_takes_the_room_it_has_and_slides_to_meet_the_way() {
        let sect = Section::uniform(3.0);
        let c = corridor();
        let stops = vec![0.0, 50.0];
        let pts = vec![c.nodes[0], c.nodes[1]];
        let mut scratch = Vec::new();
        // Open ground: the full width, seated where the way was measured.
        let (off, half) =
            seat(&c, &stops, &pts, 0, sect, 0, 5.0, &Facades::empty(), &mut scratch).unwrap();
        assert!((half - 1.0).abs() < 1e-9, "the nominal 2 m band: {half}");
        assert!((off - 5.0).abs() < 1e-9, "seated on the mapped offset: {off}");
        // A way mapped right against the kerb — or inside it — is pushed out
        // to make room for its own width, so the band never sits on asphalt.
        for want in [3.0, 1.0, -2.0] {
            let (off, half) =
                seat(&c, &stops, &pts, 0, sect, 0, want, &Facades::empty(), &mut scratch).unwrap();
            assert!(
                (off - half - 3.0).abs() < 1e-9,
                "the band's inner edge is the kerb, for want={want}: {off} ± {half}"
            );
        }
    }

    #[test]
    fn a_facade_too_close_to_the_kerb_leaves_no_sidewalk() {
        let c = corridor();
        let stops = vec![0.0, 50.0];
        let pts = vec![c.nodes[0], c.nodes[1]];
        let mut scratch = Vec::new();
        // A wall 3.6 m out from a centerline whose asphalt already reaches
        // 3.0 m: 0.1 m of strip after the facade clearance, under the minimum.
        let wall = Facades::from_edges([[
            Coord { x: 6.89, y: LAT + 3.6 / DEG_M },
            Coord { x: 6.91, y: LAT + 3.6 / DEG_M },
        ]]);
        let seated =
            seat(&c, &stops, &pts, 0, Section::uniform(3.0), 0, 4.0, &wall, &mut scratch);
        assert!(seated.is_none(), "{seated:?}");
        // And the other side of the same street is untouched.
        assert!(seat(&c, &stops, &pts, 0, Section::uniform(3.0), 1, 4.0, &wall, &mut scratch)
            .is_some());
    }

    fn corridor() -> Corridor {
        let nodes = vec![Coord { x: 6.9, y: LAT }, Coord { x: 6.9 + east(50.0), y: LAT }];
        Corridor {
            id: 0,
            arc: crate::scene::cumulative_arc(&nodes),
            nodes,
            cos_lat: LAT.to_radians().cos(),
            kind: Kind::Road(RoadClass::Residential),
            class_key: "residential".into(),
            link: false,
            width_m: Some(5.5),
            spans: Vec::new(),
            segments: Vec::new(),
            connectors: Vec::new(),
        }
    }
}
