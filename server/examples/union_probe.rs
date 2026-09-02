//! Which stretches of a corridor's centerline the unioned pavement actually
//! covers — the instrument for a band with a hole in it.
//!
//! `seam.band_deck_bare` names a place where drawn ground interrupts the
//! asphalt; the scorecard cannot say *why* the union left it out. This walks
//! the corridor nearest the site at one-metre steps, asks the baked
//! [`arpentry_server::synth::pavement`] model whether each step is inside a
//! paved region of the corridor's own (level, surface), and prints the
//! covered/uncovered intervals plus every region key the chunk holds — so a
//! missing source, a subtracted yield and a mis-keyed region read differently.
//!
//! Usage:
//!   cargo run --release --example union_probe -- \
//!       <segment.parquet> <w,s,e,n> <terrain.pmtiles> <lon,lat>

use arpentry_server::assemble;
use arpentry_server::ground;
use arpentry_server::project::Bounds;
use arpentry_server::solve;
use arpentry_server::synth;

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let bb: Vec<f64> = a[1].split(',').map(|s| s.parse().unwrap()).collect();
    let bbox = Bounds { west: bb[0], south: bb[1], east: bb[2], north: bb[3] };
    let terrain = std::path::PathBuf::from(&a[2]);
    let at: Vec<f64> = a[3].split(',').map(|s| s.parse().unwrap()).collect();
    let (lon0, lat0) = (at[0], at[1]);

    let mut scene = assemble::run(std::path::Path::new(&a[0]), None, &bbox).expect("assemble");
    let solved = solve::run(&mut scene, Some(&terrain), 16, 0).expect("solve");
    let facades = assemble::facades::Facades::empty();
    let junctions = synth::carriageway::bake(&scene, &solved, &facades, Vec::new());
    let pavement = synth::pavement::bake(&junctions, 1, None);

    let m_lat = 110_540.0;
    let m_lon = 111_320.0 * lat0.to_radians().cos();
    let dist = |c: geo_types::Coord| -> f64 {
        let dx = (c.x - lon0) * m_lon;
        let dy = (c.y - lat0) * m_lat;
        (dx * dx + dy * dy).sqrt()
    };

    // The corridor standing nearest the site.
    let Some((_, cid)) = scene
        .corridors
        .iter()
        .filter_map(|c| {
            c.nodes
                .iter()
                .map(|&n| dist(n))
                .min_by(f64::total_cmp)
                .map(|d| (d, c.id))
        })
        .min_by(|a, b| a.0.total_cmp(&b.0))
    else {
        println!("no corridor near the site");
        return;
    };
    let c = &scene.corridors[cid as usize];
    println!(
        "corridor #{cid} {:?} {} m — spans: {}",
        c.kind,
        c.total().round(),
        c.spans
            .iter()
            .map(|s| format!("{:?}@L{}[{:.0}..{:.0}]", s.kind, s.level, s.arc0, s.arc1))
            .collect::<Vec<_>>()
            .join(" ")
    );
    // Span boundaries in plan, so a drawn gap can be compared with where the
    // partition believes the handover arcs are.
    let plan_at = |a: f64| -> (f64, f64) {
        let k = c.arc.partition_point(|&x| x < a).min(c.nodes.len() - 1).max(1);
        let (a0, a1) = (c.arc[k - 1], c.arc[k]);
        let t = if a1 > a0 { ((a - a0) / (a1 - a0)).clamp(0.0, 1.0) } else { 0.0 };
        let (p0, p1) = (c.nodes[k - 1], c.nodes[k]);
        (p0.x + (p1.x - p0.x) * t, p0.y + (p1.y - p0.y) * t)
    };
    for sp in &c.spans {
        let (x0, y0) = plan_at(sp.arc0);
        let (x1, y1) = plan_at(sp.arc1);
        println!(
            "  span {:?}[{:.1}..{:.1}] plan {:.6},{:.6} -> {:.6},{:.6}",
            sp.kind, sp.arc0, sp.arc1, x0, y0, x1, y1
        );
    }

    // Every region key of the chunk under the site.
    let tile = solve::tile_containing(16, lon0, lat0);
    let Some(levels) = pavement.chunk_for(&tile) else {
        println!("no chunk under the site");
        return;
    };
    for ls in levels {
        let verts: usize = ls.shapes.iter().flat_map(|sh| sh.iter()).map(Vec::len).sum();
        println!(
            "  region level {} layer {} {:?}: {} shapes, {verts} verts",
            ls.level, ls.layer, ls.surface, ls.shapes.len()
        );
    }

    // The corridor's own sources and their layers, so coverage can be read on
    // the sheet the band actually belongs to rather than on whatever region
    // covers the plan (the road passing underneath answers a 2-D test too).
    let mut own_layers: Vec<u32> = Vec::new();
    let mut ids = Vec::new();
    junctions.sources_near((bbox.west, bbox.south, bbox.east, bbox.north), &mut ids);
    for &i in &ids {
        let s = junctions.source(i);
        if s.corridor == cid {
            own_layers.push(s.layer);
        }
    }
    own_layers.sort_unstable();
    own_layers.dedup();
    println!("  corridor's own source layers: {own_layers:?}");

    // March the centerline; even–odd point-in-shape per region, reporting the
    // set of (layer) keys of the corridor's material covering each step.
    let covering = |lon: f64, lat: f64| -> Vec<u32> {
        levels
            .iter()
            .filter(|ls| ls.level == 0 && ls.surface == c.kind.prior().surface)
            .filter(|ls| {
                ls.shapes.iter().any(|sh| {
                    let mut winds = false;
                    for ring in sh {
                        let n = ring.len();
                        for i in 0..n {
                            let (p, q) = (ring[i], ring[(i + 1) % n]);
                            if (p.y > lat) != (q.y > lat)
                                && lon < p.x + (q.x - p.x) * (lat - p.y) / (q.y - p.y)
                            {
                                winds = !winds;
                            }
                        }
                    }
                    winds
                })
            })
            .map(|ls| ls.layer)
            .collect()
    };
    let total = c.total();
    let mut prev: Option<Vec<u32>> = None;
    let mut start = 0.0f64;
    let mut arc = 0.0f64;
    while arc <= total {
        let p = profile_point(&scene.corridors[cid as usize], arc);
        let now = covering(p.x, p.y);
        if prev.as_ref().is_some_and(|w| *w != now) {
            println!("  [{start:>6.1} .. {arc:>6.1}] layers {:?}", prev.as_ref().unwrap());
            start = arc;
        }
        prev = Some(now);
        arc += 1.0;
    }
    println!("  [{start:>6.1} .. {total:>6.1}] layers {:?}", prev.as_ref().unwrap());

    // The per-tile half: what the mesher hands the terrain CDT for the site
    // tile — each paved entry, whether its hole region survived, and whether
    // that region contains the site in the CDT's own quantized coordinates.
    let stack = std::sync::Arc::new(ground::derive(
        &scene,
        &solved,
        &facades,
        &[],
        Some(&terrain),
        1,
    ));
    let dem = arpentry_server::dem::Dem::open(&terrain).ok();
    let mut sampler = ground::sampler::GroundSampler::new(
        dem,
        stack,
        16,
        ground::sampler::MeshOptions::default(),
    );
    let field = synth::height::HeightField::for_tile(&junctions, &solved, 16, &tile);
    println!("  tile hole gate: cuts_hole = {}", sampler.cuts_hole(16));
    let qx = 16384.0 + (lon0 - tile.west) / tile.width() * 32768.0;
    let qy = 16384.0 + (lat0 - tile.south) / tile.height() * 32768.0;
    for paved in synth::pave_mesh::tile_meshes(levels, &field, &mut sampler, 16, 16, &tile, true, &[])
    {
        println!(
            "  paved level {} {:?}: region empty = {}, contains site = {}",
            paved.level,
            paved.material,
            paved.region.is_empty(),
            paved.region.contains((qx, qy)),
        );
    }
}

/// The corridor's mapped point at arc `d` (the raw line — near enough for a
/// coverage march; the buffered band is metres wide).
fn profile_point(c: &arpentry_server::scene::Corridor, d: f64) -> geo_types::Coord {
    let arc = &c.arc;
    let i = match arc.binary_search_by(|v| v.partial_cmp(&d).expect("finite")) {
        Ok(i) => i.min(c.nodes.len() - 1),
        Err(i) => i.saturating_sub(1).min(c.nodes.len().saturating_sub(2)),
    };
    if i + 1 >= c.nodes.len() {
        return c.nodes[c.nodes.len() - 1];
    }
    let span = arc[i + 1] - arc[i];
    let t = if span > 0.0 { ((d - arc[i]) / span).clamp(0.0, 1.0) } else { 0.0 };
    geo_types::Coord {
        x: c.nodes[i].x + (c.nodes[i + 1].x - c.nodes[i].x) * t,
        y: c.nodes[i].y + (c.nodes[i + 1].y - c.nodes[i].y) * t,
    }
}
