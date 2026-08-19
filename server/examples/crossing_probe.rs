//! Probe: where the road height field has two sources at one place that
//! disagree, and by how much.
//!
//! `synth::height` blends every carriageway source covering a point, keyed by
//! (level, grade-separation layer). Two sources on one key whose solved heights
//! differ produce a *ramp* across their overlap — continuous, but metres of
//! rise over metres of run, which is the bump. This enumerates the overlapping
//! pairs and splits them by what relates the two sources, so the fix can be
//! aimed at the population that actually carries the disagreement.
//!
//! Usage: cargo run --release --example crossing_probe -- <segment.parquet> \
//!            <w,s,e,n> [terrain.pmtiles]

use std::collections::HashSet;

use arpentry_server::assemble;
use arpentry_server::assemble::grid::GridIndex;
use arpentry_server::project::Bounds;
use arpentry_server::scene::{CorridorId, SceneGraph, DEG_M};
use arpentry_server::solve::{self, SolvedModel};
use arpentry_server::synth::carriageway::{self, SourceSeg};
use geo_types::Coord;

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let seg = std::path::PathBuf::from(&a[0]);
    let b: Vec<f64> = a[1].split(',').map(|s| s.parse().unwrap()).collect();
    let bbox = Bounds { west: b[0], south: b[1], east: b[2], north: b[3] };
    let terrain = a.get(2).map(std::path::PathBuf::from);

    let mut scene = assemble::run(&seg, None, &bbox).expect("assemble");
    let solved = solve::run(&mut scene, terrain.as_deref(), 16, 8).expect("solve");
    eprintln!("corridors {} profiles {}", scene.corridors.len(), solved.solved_count());

    let model = carriageway::bake(&scene, &solved);
    let n = model.source_count();
    eprintln!("carriageway sources: {n}");

    // How the sheet layering cuts the carriageways. A layer change between two
    // stretches that *join end to end* is a region boundary drawn across a
    // road that physically carries on — the casing rims it and the apron walls
    // it, which is the road arriving in pieces. One per genuine sheet change is
    // the physical truth; a count near the source count is the defect.
    {
        let (mut runs, mut seams, mut lifted) = (0usize, 0usize, 0usize);
        let mut where_: Vec<(Coord, u32, u32)> = Vec::new();
        for i in 0..n {
            let s = model.source(i as u32);
            lifted += usize::from(s.layer > 0);
            if i == 0 {
                runs += 1;
                continue;
            }
            let p = model.source(i as u32 - 1);
            if p.corridor != s.corridor || p.b != s.a {
                runs += 1; // a different stretch, not a seam in this one
            } else if p.layer != s.layer {
                seams += 1;
                if bbox.contains(s.a.x, s.a.y) {
                    where_.push((s.a, p.layer, s.layer));
                }
            }
        }
        // ARPT_AT=lon,lat dumps everything the surface at one place is built
        // from: the plates that pin it and the carriageway stretches that
        // blend into it, with the layer each one carries. A junction that does
        // not come out flat is one of these disagreeing.
        if let Ok(at) = std::env::var("ARPT_AT") {
            let v: Vec<f64> = at.split(',').map(|s| s.parse().unwrap()).collect();
            let (lon, lat) = (v[0], v[1]);
            let pad = 40.0 / DEG_M;
            println!("== at {lon:.6},{lat:.6} ==");
            for j in model.near((lon - pad, lat - pad, lon + pad, lat + pad)) {
                let p = j.point();
                let d = ((p.x - lon) * DEG_M * lat.to_radians().cos()).hypot((p.y - lat) * DEG_M);
                println!(
                    "    plate  {:.6},{:.6}  {:.1} m away  layer {}  height {}",
                    p.x,
                    p.y,
                    d,
                    j.layer(),
                    j.height().map_or("none".into(), |h| format!("{h:.2}"))
                );
            }
            let mut ids = Vec::new();
            model.sources_near((lon - pad, lat - pad, lon + pad, lat + pad), &mut ids);
            let mut rows: Vec<(f64, String)> = Vec::new();
            for &i in &ids {
                let s = model.source(i);
                let m = mid(s);
                let d = ((m.x - lon) * DEG_M * lat.to_radians().cos()).hypot((m.y - lat) * DEG_M);
                rows.push((
                    d,
                    format!(
                        "    source c{:<6} level {:>2}  layer {}  {:.2} -> {:.2} m  half {:.1} m  \
                         {:.1} m away",
                        s.corridor, s.level, s.layer, s.height_a, s.height_b, s.half_m, d
                    ),
                ));
            }
            rows.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
            for (_, r) in rows.iter().take(16) {
                println!("{r}");
            }

            // The welds themselves: every mapped junction near, and which
            // corridor arcs it ties together. A node pulled off its neighbours
            // is usually one of these tying it to something much higher.
            for j in &scene.junctions {
                let d = ((j.point.x - lon) * DEG_M * lat.to_radians().cos())
                    .hypot((j.point.y - lat) * DEG_M);
                if d > 40.0 {
                    continue;
                }
                let members: Vec<String> = j
                    .members
                    .iter()
                    .map(|m| format!("c{}@{:.1}", m.corridor, m.arc))
                    .collect();
                println!("    weld   {:.1} m away  {}", d, members.join("  "));
            }

            // The crossings, which are what raise a road that no junction ties
            // to anything: clearance over whatever the DAG put underneath it.
            for x in &solved.crossings {
                let d = ((x.point.x - lon) * DEG_M * lat.to_radians().cos())
                    .hypot((x.point.y - lat) * DEG_M);
                if d > 40.0 {
                    continue;
                }
                println!(
                    "    cross  {:.1} m away  upper c{}@{:.1} (level {})  lower {}  (level {})",
                    d,
                    x.upper,
                    x.upper_arc,
                    x.upper_level,
                    x.lower.map_or("terrain".into(), |c| format!("c{c} {:?}", x.lower_kind)),
                    x.lower_level,
                );
            }

            // The solved profile of every corridor passing near, node by node:
            // the raw terrain it was solved over beside the road height it came
            // out at. A road that spikes while the terrain under it does not has
            // been pulled off the ground by a constraint; one that follows a
            // terrain spike is reading a bad sample.
            let mut near: Vec<CorridorId> =
                ids.iter().map(|&i| model.source(i).corridor).collect();
            near.sort_unstable();
            near.dedup();
            for c in near {
                let Some(pr) = solved.profile(c) else { continue };
                let (arc, road, terr, pts) = (pr.arc(), pr.road_m(), pr.terrain_m(), pr.nodes());
                let class = &scene.corridors[c as usize].class_key;
                let mut shown = false;
                for k in 0..arc.len() {
                    let d = ((pts[k].x - lon) * DEG_M * lat.to_radians().cos())
                        .hypot((pts[k].y - lat) * DEG_M);
                    if d > 40.0 {
                        continue;
                    }
                    if !shown {
                        println!("    -- c{c} ({class})");
                        shown = true;
                    }
                    let step = if k == 0 {
                        String::new()
                    } else {
                        let ds = arc[k] - arc[k - 1];
                        format!("  {:+.2} m over {:.1} m", road[k] - road[k - 1], ds)
                    };
                    println!(
                        "       n{k:<3} arc {:7.1}  terrain {:8.2}  road {:8.2}  deck {:8.2}  \
                         {:?}{step}",
                        arc[k],
                        terr[k],
                        road[k],
                        pr.deck_m()[k],
                        scene.corridors[c as usize].spans.iter().find(|s| {
                            arc[k] >= s.arc0 - 1e-6 && arc[k] <= s.arc1 + 1e-6
                        }).map(|s| (s.kind, s.level)),
                    );
                }
            }
            println!();
        }

        // How the asphalt actually partitions: one entry per layer, biggest
        // first. A healthy answer is one huge sheet — the connected ground
        // network — plus a tail of small ones that genuinely stand apart.
        let mut per_layer: std::collections::BTreeMap<u32, usize> = Default::default();
        for i in 0..n {
            *per_layer.entry(model.source(i as u32).layer).or_default() += 1;
        }
        let mut sizes: Vec<(u32, usize)> = per_layer.into_iter().collect();
        sizes.sort_by_key(|&(l, c)| (std::cmp::Reverse(c), l));

        println!("== sheet layering ==");
        println!("    sources {n}, lifted {lifted}, runs {runs}, seams inside a run {seams}");
        print!("    layers by size:");
        for (l, c) in sizes.iter().take(6) {
            print!("  {l}:{c}");
        }
        println!("   ({} layers in all)", sizes.len());
        for (p, a, b) in where_.iter().take(12) {
            println!("    seam {a} -> {b}  --at {:.6},{:.6}", p.x, p.y);
        }
        println!();
    }

    // Index every source by its buffered box, then test every pair whose bands
    // can reach each other.
    let mut grid = GridIndex::new();
    for i in 0..n {
        let s = model.source(i as u32);
        let pad = s.half_m / DEG_M;
        grid.insert(
            (
                s.a.x.min(s.b.x) - pad,
                s.a.y.min(s.b.y) - pad,
                s.a.x.max(s.b.x) + pad,
                s.a.y.max(s.b.y) + pad,
            ),
            i as u32,
        );
    }

    // Sanity floor: how steep are the solved profiles themselves? A bump in the
    // drawn road can only be the field's doing if the profile under it is
    // smooth, so that has to be ruled out first.
    {
        let mut g: Vec<f64> = Vec::new();
        let mut steep: Vec<(f64, Coord, String)> = Vec::new();
        for c in &scene.corridors {
            let Some(pr) = solved.profile(c.id) else { continue };
            let (arc, road, pts) = (pr.arc(), pr.road_m(), pr.nodes());
            for k in 1..arc.len() {
                let ds = arc[k] - arc[k - 1];
                if ds < 1e-6 || !bbox.contains(pts[k].x, pts[k].y) {
                    continue;
                }
                let gr = (road[k] - road[k - 1]).abs() / ds;
                g.push(gr);
                if gr > 0.5 {
                    steep.push((
                        gr,
                        pts[k],
                        format!(
                            "{} c{} n{k} {:+.2} m over {:.2} m",
                            c.class_key,
                            c.id,
                            road[k] - road[k - 1],
                            ds
                        ),
                    ));
                }
            }
        }
        println!("== longitudinal grade of the solved profiles ==");
        hist(&mut g, &[0.05, 0.10, 0.15, 0.30, 0.60, 1.00, 2.00]);
        steep.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());
        // How many of these stand next to a crossing? A clearance raise applied
        // at one node is a ramp the road has no length to climb, and that is a
        // different defect from a road that is simply steep.
        let mut at_crossing = 0usize;
        for (_, p, _) in &steep {
            let near = solved.crossings.iter().any(|x| {
                let d = ((x.point.x - p.x) * DEG_M * 46.4f64.to_radians().cos())
                    .hypot((x.point.y - p.y) * DEG_M);
                d < 30.0
            });
            at_crossing += usize::from(near);
        }
        println!(
            "    {} nodes steeper than 0.5 m/m, {at_crossing} of them within 30 m of a crossing",
            steep.len()
        );
        for (gr, p, c) in steep.iter().take(8) {
            println!("    {gr:6.2} m/m  {:.6},{:.6}  {c}", p.x, p.y);
        }
        println!();
    }

    let nodes = node_index(&scene, &solved);
    let mut rows: Vec<Row> = Vec::new();
    let mut cand = Vec::new();
    let mut seen: HashSet<(u32, u32)> = HashSet::new();
    for i in 0..n {
        let s = *model.source(i as u32);
        let pad = s.half_m / DEG_M;
        grid.query(
            (
                s.a.x.min(s.b.x) - pad,
                s.a.y.min(s.b.y) - pad,
                s.a.x.max(s.b.x) + pad,
                s.a.y.max(s.b.y) + pad,
            ),
            &mut cand,
        );
        for &j in cand.iter() {
            if j as usize <= i {
                continue;
            }
            let t = *model.source(j);
            if s.level != t.level {
                continue;
            }
            if s.layer != t.layer && std::env::var_os("SHOW_SEPARATED").is_none() {
                continue; // the field already partitions these
            }
            // Adjacent segments of one corridor always overlap; that is the
            // road being continuous, not a disagreement.
            if s.corridor == t.corridor && adjacent(&scene, s.corridor, &s, &t) {
                continue;
            }
            let mid_s = mid(&s);
            let mid_t = mid(&t);
            let d = seg_distance_m(&s, &t);
            if d > s.half_m + t.half_m {
                continue; // bands do not meet
            }
            if !seen.insert((i as u32, j)) {
                continue;
            }
            let (Some(ps), Some(pt)) = (solved.profile(s.corridor), solved.profile(t.corridor))
            else {
                continue;
            };
            let p = Coord { x: 0.5 * (mid_s.x + mid_t.x), y: 0.5 * (mid_s.y + mid_t.y) };
            // Each source's own height *at its own arc*. A plan lookup
            // (`height_at`) collapses a corridor that stacks over itself to one
            // answer, which is exactly the case this is trying to see.
            let dh = (arc_height(&nodes, ps, s.corridor, s.a)
                - arc_height(&nodes, pt, t.corridor, t.a))
            .abs();
            // Row groups spill far past the bbox, and a corridor assembled from
            // a partial row group has a truncated node list: its arcs and
            // profile are not the ones the tiler would build. Only the extent
            // actually tiled is measurable.
            if !bbox.contains(p.x, p.y) {
                continue;
            }
            rows.push(Row {
                point: p,
                dh,
                overlap_m: (s.half_m + t.half_m - d).max(0.0),
                kind: classify(&scene, s.corridor, t.corridor, p),
                la: s.layer,
                lb: t.layer,
                arc_gap: if s.corridor == t.corridor {
                    let (ka, kb) = (
                        nodes.get(&(s.corridor, s.a.x.to_bits(), s.a.y.to_bits())).copied(),
                        nodes.get(&(t.corridor, t.a.x.to_bits(), t.a.y.to_bits())).copied(),
                    );
                    match (ka, kb) {
                        // Profile arc, indexed by profile node — the same array
                        // `nodes` was built over.
                        (Some(x), Some(y)) => Some((ps.arc()[x] - ps.arc()[y]).abs()),
                        _ => None,
                    }
                } else {
                    None
                },
                class_a: scene.corridors[s.corridor as usize].class_key.clone(),
                class_b: scene.corridors[t.corridor as usize].class_key.clone(),
            });
        }
    }

    // --near lon,lat,radius_m: dump every pair around one place, in full.
    if let Some(spec) = std::env::var_os("NEAR") {
        let v: Vec<f64> =
            spec.to_string_lossy().split(',').map(|s| s.parse().unwrap()).collect();
        let (at, r) = (Coord { x: v[0], y: v[1] }, v[2]);
        let cos = at.y.to_radians().cos();
        println!("== pairs within {r} m of {},{} ==", at.x, at.y);
        let mut near: Vec<&Row> = rows
            .iter()
            .filter(|w| point_to_segment_m(w.point, at, at, cos) <= r)
            .collect();
        near.sort_by(|a, b| b.dh.partial_cmp(&a.dh).unwrap());
        for w in near.iter().take(30) {
            println!(
                "  Δh {:7.3} m  overlap {:5.2} m  L{}/{}  {:?}  {} × {}  at {:.6},{:.6}",
                w.dh, w.overlap_m, w.la, w.lb, w.kind, w.class_a, w.class_b, w.point.x, w.point.y
            );
        }
        println!("  ({} pairs total there)", near.len());
        return;
    }

    println!("overlapping same-key source pairs: {}", rows.len());
    {
        let self_bad: Vec<&Row> = rows
            .iter()
            .filter(|r| r.kind == Kind::SelfOverlap && r.dh > 0.5)
            .collect();
        println!(
            "\n== self-stacking: {} of {} SelfOverlap pairs disagree by > 0.5 m ==",
            self_bad.len(),
            rows.iter().filter(|r| r.kind == Kind::SelfOverlap).count()
        );
        let mut w: Vec<&&Row> = self_bad.iter().collect();
        w.sort_by(|a, b| b.dh.partial_cmp(&a.dh).unwrap());
        for r in w.iter().take(10) {
            println!(
                "    Δh {:6.2} m  overlap {:5.2} m  arc apart {:7.1} m  {:.6},{:.6}  {}",
                r.dh,
                r.overlap_m,
                r.arc_gap.unwrap_or(f64::NAN),
                r.point.x,
                r.point.y,
                r.class_a
            );
        }
    }
    for k in [Kind::SelfOverlap, Kind::Junction, Kind::NearJunction, Kind::Unrelated] {
        let sub: Vec<&Row> = rows.iter().filter(|r| r.kind == k).collect();
        println!("\n== {k:?}: {} pairs ==", sub.len());
        println!("  |Δh| between the two sources:");
        let mut v: Vec<f64> = sub.iter().map(|r| r.dh).collect();
        hist(&mut v, &[0.05, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0]);
        println!("  implied ramp gradient (|Δh| / overlap width):");
        let mut g: Vec<f64> =
            sub.iter().filter(|r| r.overlap_m > 0.05).map(|r| r.dh / r.overlap_m).collect();
        hist(&mut g, &[0.05, 0.15, 0.3, 0.6, 1.0, 2.0, 4.0]);
        let mut worst: Vec<&&Row> = sub.iter().filter(|r| r.overlap_m > 0.05).collect();
        worst.sort_by(|a, b| {
            (b.dh / b.overlap_m).partial_cmp(&(a.dh / a.overlap_m)).unwrap()
        });
        println!("  worst 12 by gradient:");
        for r in worst.iter().take(12) {
            println!(
                "    {:6.2} m/m  ({:6.3} m over {:5.2} m)  {:.6},{:.6}  {} × {}",
                r.dh / r.overlap_m,
                r.dh,
                r.overlap_m,
                r.point.x,
                r.point.y,
                r.class_a,
                r.class_b
            );
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Kind {
    /// One corridor overlapping itself: a hairpin's two arms, a loop ramp.
    SelfOverlap,
    /// The overlap sits at an intersection the two share — legitimately one
    /// paved surface, and the junction pin reconciles it.
    Junction,
    /// The two meet somewhere, but not here: the overlap is out along the
    /// roads, past where the weld holds them together.
    NearJunction,
    /// Two corridors that never meet: a grade separation the data never
    /// annotated, or two roads mapped closer than their widths.
    Unrelated,
}

/// How near a shared connector an overlap still counts as the intersection
/// itself, in metres — a couple of carriageway widths.
const AT_JUNCTION_M: f64 = 15.0;

struct Row {
    point: Coord,
    dh: f64,
    overlap_m: f64,
    kind: Kind,
    la: u32,
    lb: u32,
    arc_gap: Option<f64>,
    class_a: String,
    class_b: String,
}

fn classify(scene: &SceneGraph, a: CorridorId, b: CorridorId, at: Coord) -> Kind {
    if a == b {
        return Kind::SelfOverlap;
    }
    let (ca, cb) = (&scene.corridors[a as usize], &scene.corridors[b as usize]);
    let shared: Vec<u64> = ca
        .connectors
        .iter()
        .map(|&(id, _)| id)
        .filter(|id| cb.connectors.binary_search_by(|(o, _)| o.cmp(id)).is_ok())
        .collect();
    if shared.is_empty() {
        return Kind::Unrelated;
    }
    // A shared connector reconciles the overlap only if it is *here*.
    let near = scene.junctions.iter().any(|j| {
        shared.contains(&j.connector)
            && point_to_segment_m(at, j.point, j.point, ca.cos_lat) <= AT_JUNCTION_M
    });
    if near {
        Kind::Junction
    } else {
        Kind::NearJunction
    }
}

/// Whether two sources of one corridor are consecutive along it (their shared
/// node makes them touch by construction).
fn adjacent(_scene: &SceneGraph, _c: CorridorId, s: &SourceSeg, t: &SourceSeg) -> bool {
    let same = |p: Coord, q: Coord| (p.x - q.x).abs() < 1e-12 && (p.y - q.y).abs() < 1e-12;
    same(s.b, t.a) || same(t.b, s.a) || same(s.a, t.a) || same(s.b, t.b)
}

fn mid(s: &SourceSeg) -> Coord {
    Coord { x: 0.5 * (s.a.x + s.b.x), y: 0.5 * (s.a.y + s.b.y) }
}

/// Exact-coordinate index from a corridor node position to its node number, so
/// a source's height can be read at *its own* arc. A plan lookup collapses a
/// corridor that doubles back on itself to a single answer, which is exactly
/// the case being measured.
type NodeIndex = std::collections::HashMap<(CorridorId, u64, u64), usize>;

fn node_index(scene: &SceneGraph, solved: &SolvedModel) -> NodeIndex {
    let mut m = NodeIndex::default();
    for c in &scene.corridors {
        // `road_m` is indexed by the *profile's* densified node list, not the
        // corridor's; the corridor's own nodes survive inside it, so an exact
        // coordinate match finds the right slot.
        let Some(p) = solved.profile(c.id) else { continue };
        for (k, n) in p.nodes().iter().enumerate() {
            m.entry((c.id, n.x.to_bits(), n.y.to_bits())).or_insert(k);
        }
    }
    m
}

fn arc_height(
    idx: &NodeIndex,
    p: &arpentry_server::solve::Profile,
    corridor: CorridorId,
    at: Coord,
) -> f64 {
    match idx.get(&(corridor, at.x.to_bits(), at.y.to_bits())) {
        Some(&k) if k < p.road_m().len() => p.road_m()[k],
        _ => p.height_at(at.x, at.y),
    }
}

/// Minimum distance in metres between two centerline segments.
fn seg_distance_m(s: &SourceSeg, t: &SourceSeg) -> f64 {
    let d = |p: Coord, a: Coord, b: Coord| point_to_segment_m(p, a, b, s.cos_lat);
    d(s.a, t.a, t.b)
        .min(d(s.b, t.a, t.b))
        .min(d(t.a, s.a, s.b))
        .min(d(t.b, s.a, s.b))
}

fn point_to_segment_m(p: Coord, a: Coord, b: Coord, cos_lat: f64) -> f64 {
    let m_lon = DEG_M * cos_lat;
    let (px, py) = ((p.x - a.x) * m_lon, (p.y - a.y) * DEG_M);
    let (ex, ey) = ((b.x - a.x) * m_lon, (b.y - a.y) * DEG_M);
    let len2 = ex * ex + ey * ey;
    if len2 < 1e-18 {
        return (px * px + py * py).sqrt();
    }
    let t = ((px * ex + py * ey) / len2).clamp(0.0, 1.0);
    let (dx, dy) = (px - ex * t, py - ey * t);
    (dx * dx + dy * dy).sqrt()
}

fn hist(v: &mut Vec<f64>, edges: &[f64]) {
    if v.is_empty() {
        println!("  (empty)");
        return;
    }
    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let n = v.len();
    let q = |p: f64| v[((n as f64 - 1.0) * p).round() as usize];
    println!(
        "  n={n}  p50={:.3}  p90={:.3}  p99={:.3}  max={:.3}",
        q(0.5),
        q(0.9),
        q(0.99),
        v[n - 1]
    );
    let mut prev = 0.0;
    for &e in edges {
        let c = v.iter().filter(|&&x| x >= prev && x < e).count();
        println!("  [{prev:>5.2}, {e:>5.2})  {c:7}  {:5.1}%", 100.0 * c as f64 / n as f64);
        prev = e;
    }
    let c = v.iter().filter(|&&x| x >= prev).count();
    println!("  [{prev:>5.2},   inf)  {c:7}  {:5.1}%", 100.0 * c as f64 / n as f64);
}

#[allow(unused)]
fn unused(_: &SolvedModel) {}
