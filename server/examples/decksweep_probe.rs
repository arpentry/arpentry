//! Reproduce one bridge span's deck sweep inputs: the piece line cut from the
//! corridor, and the `deck_nodes` sections the sweep would build — so a drawn
//! deck that starts short of its span boundary can be blamed on the right
//! stage.
//!
//! Usage: decksweep_probe <segment.parquet> <w,s,e,n> <terrain.pmtiles> <lon,lat>

use arpentry_server::assemble;
use arpentry_server::project::Bounds;
use arpentry_server::scene::SpanKind;
use arpentry_server::solve;

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let bb: Vec<f64> = a[1].split(',').map(|s| s.parse().unwrap()).collect();
    let bbox = Bounds { west: bb[0], south: bb[1], east: bb[2], north: bb[3] };
    let site: Vec<f64> = a[3].split(',').map(|s| s.parse().unwrap()).collect();

    let mut scene = assemble::run(std::path::Path::new(&a[0]), None, &bbox).expect("assemble");
    let solved = solve::run(&mut scene, Some(std::path::Path::new(&a[2]).as_ref()), 16, 0)
        .expect("solve");

    // Nearest corridor with a Bridge span near the site.
    let dist = |c: &arpentry_server::scene::Corridor| -> f64 {
        c.nodes
            .iter()
            .map(|n| {
                let dx = (n.x - site[0]) * c.cos_lat;
                let dy = n.y - site[1];
                dx * dx + dy * dy
            })
            .fold(f64::INFINITY, f64::min)
    };
    let c = scene
        .corridors
        .iter()
        .filter(|c| c.spans.iter().any(|s| s.kind == SpanKind::Bridge))
        .min_by(|a, b| dist(a).total_cmp(&dist(b)))
        .expect("a corridor");
    println!("corridor #{} spans: {}", c.id, c.spans.iter()
        .map(|s| format!("{:?}[{:.1}..{:.1}]", s.kind, s.arc0, s.arc1))
        .collect::<Vec<_>>().join(" "));
    let p = solved.profile(c.id).expect("profile");

    for seg in &c.segments {
        for piece in c.pieces_in(seg, &c.spans) {
            if piece.kind != SpanKind::Bridge {
                continue;
            }
            let first = piece.line[0];
            let last = piece.line[piece.line.len() - 1];
            println!(
                "piece span#{} kind={:?} nodes={} line {:.6},{:.6} -> {:.6},{:.6}",
                piece.span, piece.kind, piece.line.len(), first.x, first.y, last.x, last.y
            );
            // Manual full-range nearest-edge projection of each piece node
            // onto the profile polyline, so a windowed-walk artifact and a
            // genuinely divergent polyline read differently.
            let (pn, pa) = (p.nodes(), p.arc());
            for (k, q) in piece.line.iter().enumerate() {
                let (mut bi, mut bt, mut bd) = (0usize, 0.0, f64::INFINITY);
                for i in 0..pn.len() - 1 {
                    let (ax, ay) = (pn[i].x, pn[i].y);
                    let (ex, ey) = ((pn[i + 1].x - ax) * c.cos_lat, pn[i + 1].y - ay);
                    let (dx, dy) = ((q.x - ax) * c.cos_lat, q.y - ay);
                    let l2 = ex * ex + ey * ey;
                    let t = if l2 > 0.0 { (dx * ex + dy * ey) / l2 } else { 0.0 }.clamp(0.0, 1.0);
                    let (rx, ry) = (dx - t * ex, dy - t * ey);
                    let d = rx * rx + ry * ry;
                    if d < bd {
                        (bi, bt, bd) = (i, t, d);
                    }
                }
                let arc = pa[bi] + (pa[bi + 1] - pa[bi]) * bt;
                println!(
                    "  node[{k}] {:.6},{:.6} -> profile edge {bi} arc {arc:.1} dist {:.2} m",
                    q.x, q.y, bd.sqrt() * 111132.0
                );
            }
            println!("  profile nodes={} arc [{:.1}..{:.1}]", pn.len(), pa[0], pa[pa.len()-1]);
            // The real sweep densifies to ~4 m before deck_nodes — replicate,
            // so the probe measures the pipeline's own projection.
            let mut dense = vec![piece.line[0]];
            for w in piece.line.windows(2) {
                let de = (w[1].x - w[0].x) * c.cos_lat * 111_320.0;
                let dn = (w[1].y - w[0].y) * 111_132.0;
                let len = (de * de + dn * dn).sqrt();
                let steps = ((len / 4.0).ceil() as usize).max(1);
                for i in 1..=steps {
                    let t = i as f64 / steps as f64;
                    dense.push(geo_types::Coord {
                        x: w[0].x + (w[1].x - w[0].x) * t,
                        y: w[0].y + (w[1].y - w[0].y) * t,
                    });
                }
            }
            let dn = p.deck_nodes(&dense);
            println!("  densified {} pts:", dense.len());
            for (k, d) in dn.iter().enumerate() {
                if k < 4 || k + 2 >= dn.len() {
                    println!(
                        "  dsect[{k}] {:.6},{:.6} arc {:.1} h {:.2}",
                        d.lon, d.lat, d.arc_m, d.height_m
                    );
                }
            }
            let nodes = p.deck_nodes(&piece.line);
            for (k, d) in nodes.iter().enumerate() {
                if k < 3 || k + 3 >= nodes.len() {
                    println!(
                        "  sect[{k}] {:.6},{:.6} arc {:.1} h {:.2}",
                        d.lon, d.lat, d.arc_m, d.height_m
                    );
                }
            }
        }
    }
}
