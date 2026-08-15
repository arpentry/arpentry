//! How far does a corridor's smoothed sweep line sit from the raw line the
//! at-grade asphalt is buffered around?
//!
//! Two curves claim to be the same road. Structures are swept along the
//! *smoothed* one (`Profile::deck_nodes` → `smooth_point`), and so is the paint
//! that rides them (`synth::road::bake`'s snap). The unioned at-grade surface is
//! buffered around the **raw** corridor nodes
//! (`synth::carriageway::carriageway_sources` reads `Corridor::nodes`). Wherever
//! the two disagree, a centre line generated at offset zero is off the centre of
//! its own asphalt by exactly that much — and no marking-side fix can close it,
//! because both ends of the discrepancy are correct in their own terms.
//!
//! This measures the disagreement directly from the model, with no tiling: for
//! every node of every corridor with a profile, the plan distance from the node
//! to the smoothed line carried through `smooth_at`.
//!
//! Usage: cargo run --release --example smooth_offset -- <segment.parquet> <w,s,e,n> <terrain.pmtiles>

use arpentry_server::assemble;
use arpentry_server::project::Bounds;
use arpentry_server::solve;

const DEG_M: f64 = 111_320.0;

/// Plan distance in metres from a point to a polyline.
fn to_polyline(pts: &[geo_types::Coord], cos_lat: f64, q: geo_types::Coord) -> f64 {
    let (qx, qy) = (q.x * cos_lat * DEG_M, q.y * DEG_M);
    let mut best = f64::INFINITY;
    for w in pts.windows(2) {
        let (ax, ay) = (w[0].x * cos_lat * DEG_M, w[0].y * DEG_M);
        let (bx, by) = (w[1].x * cos_lat * DEG_M, w[1].y * DEG_M);
        let (ex, ey) = (bx - ax, by - ay);
        let len2 = ex * ex + ey * ey;
        let t = if len2 > 0.0 {
            (((qx - ax) * ex + (qy - ay) * ey) / len2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let (dx, dy) = (qx - (ax + ex * t), qy - (ay + ey * t));
        best = best.min((dx * dx + dy * dy).sqrt());
    }
    best
}

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    if a.len() < 3 {
        eprintln!("usage: smooth_offset <segment.parquet> <w,s,e,n> <terrain.pmtiles>");
        std::process::exit(2);
    }
    let bb: Vec<f64> = a[1].split(',').map(|s| s.parse().expect("bbox number")).collect();
    let bbox = Bounds { west: bb[0], south: bb[1], east: bb[2], north: bb[3] };
    let terrain = std::path::PathBuf::from(&a[2]);

    let mut scene = assemble::run(std::path::Path::new(&a[0]), None, &bbox).expect("assemble");
    let solved = solve::run(&mut scene, Some(&terrain), 16, 0).expect("solve");

    let mut all: Vec<f64> = Vec::new();
    // Split by class: markings only exist on the engineered ladder
    // (`priors::has_centre_line` / `has_edge_lines`), and those roads are
    // already smooth, so a median over every track and service road in the
    // extract says nothing about the paint.
    let mut painted: Vec<f64> = Vec::new();
    let mut worst: Vec<(f64, u32, f64, f64)> = Vec::new();
    let mut uncarried = 0usize;
    for c in &scene.corridors {
        let Some(p) = solved.profile(c.id) else { continue };
        let cos_lat = c.cos_lat;
        for &n in &c.nodes {
            match p.smooth_at(n.x, n.y, 6.0) {
                Some(s) => {
                    // The *lateral* disagreement only: the distance from the
                    // carried point to the raw polyline itself. Smoothing moves
                    // a node along its own path as well as across it, and only
                    // the across part takes paint off the centre of its asphalt.
                    let d = to_polyline(&c.nodes, cos_lat, s);
                    all.push(d);
                    if matches!(
                        c.class_key.as_str(),
                        "motorway" | "trunk" | "primary" | "secondary" | "tertiary"
                    ) {
                        painted.push(d);
                    }
                    worst.push((d, c.id, n.x, n.y));
                }
                // The ends, and anything the 6 m gate refuses: the paint is
                // left on the raw line there, which is where the asphalt is.
                None => uncarried += 1,
            }
        }
    }
    if all.is_empty() {
        println!("no corridor node carried onto a sweep line");
        return;
    }
    all.sort_by(f64::total_cmp);
    let q = |p: f64| all[((all.len() - 1) as f64 * p) as usize];
    println!(
        "{} corridor nodes carried, {uncarried} left on the raw line (ends and the 6 m gate)",
        all.len()
    );
    println!(
        "lateral gap, smoothed sweep line → raw line (m): p05 {:.2}  p25 {:.2}  median {:.2}  p75 {:.2}  \
         p90 {:.2}  p95 {:.2}  p99 {:.2}  max {:.2}",
        q(0.05),
        q(0.25),
        q(0.5),
        q(0.75),
        q(0.90),
        q(0.95),
        q(0.99),
        q(1.0)
    );
    let over = |t: f64| all.iter().filter(|&&d| d > t).count();
    for t in [0.25, 0.5, 1.0, 2.0] {
        println!(
            "  {:>6} nodes ({:.2} %) sit more than {t} m from their own asphalt's centerline",
            over(t),
            100.0 * over(t) as f64 / all.len() as f64
        );
    }
    if !painted.is_empty() {
        painted.sort_by(f64::total_cmp);
        let q = |p: f64| painted[((painted.len() - 1) as f64 * p) as usize];
        println!(
            "\nof those, the {} nodes on classes that carry paint (motorway/trunk/primary/\
             secondary/tertiary): p05 {:.2}  p25 {:.2}  median {:.2}  p75 {:.2}  p90 {:.2}  \
             p95 {:.2}  max {:.2}",
            painted.len(),
            q(0.05),
            q(0.25),
            q(0.5),
            q(0.75),
            q(0.90),
            q(0.95),
            q(1.0)
        );
        let over = |t: f64| painted.iter().filter(|&&d| d > t).count();
        for t in [0.25, 0.5, 1.0, 2.0] {
            println!(
                "  {:>6} ({:.2} %) more than {t} m off their own asphalt's centerline",
                over(t),
                100.0 * over(t) as f64 / painted.len() as f64
            );
        }
    }

    worst.sort_by(|a, b| b.0.total_cmp(&a.0));
    for (d, id, lon, lat) in worst.iter().take(8) {
        println!("    {d:.2} m  corridor #{id}  at {lon:.6},{lat:.6}");
    }
}
