//! Scratch probe: which field answered for the asphalt triangles that stand
//! up as vertical curtains (carriageway triangles spanning more than `dz` m).
//!
//!   cargo run --release --example curtain_probe -- ZONE w,s,e,n x0,y0,x1,y1 [dz]

use arpentry_server::dem::Dem;
use arpentry_server::project::Bounds;
use arpentry_world::field::Field;
use arpentry_world::pipeline::{self, Sources};
use arpentry_world::poly;
use arpentry_world::step::Step;
use arpentry_world::world::{Profiles, World};

/// The same ranges `copies::of_axes` builds a sheet's field from, one profile per call.
fn ranges(p: &arpentry_world::world::Profile, spans: &[(usize, f64, f64)], pi: usize) -> Vec<(usize, usize)> {
    let mut r = Vec::new();
    for &(profile, a0, a1) in spans {
        if profile != pi {
            continue;
        }
        let first = p.stations.iter().position(|st| st.s >= a0 - 1e-9);
        let last = p.stations.iter().rposition(|st| st.s <= a1 + 1e-9);
        if let (Some(k0), Some(k1)) = (first, last) {
            if k0 <= k1 {
                r.push((k0.saturating_sub(1), (k1 + 1).min(p.stations.len() - 1)));
            }
        }
    }
    r
}

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let zone = std::path::PathBuf::from(&a[0]);
    let b: Vec<f64> = a[1].split(',').map(|s| s.parse().unwrap()).collect();
    let w: Vec<f64> = a[2].split(',').map(|s| s.parse().unwrap()).collect();
    let dz: f64 = a.get(3).map(|s| s.parse().unwrap()).unwrap_or(8.0);
    let mut dem = Dem::open(&zone.join("terrain.pmtiles")).unwrap();
    let mut world = World::new(Bounds { west: b[0], south: b[1], east: b[2], north: b[3] });
    let (seg, bld) = (zone.join("segment.parquet"), zone.join("building.parquet"));
    let mut src = Sources { dem: &mut dem, segments: &seg, buildings: Some(&bld), spacing: 1.0, max_vertices: 2_000_000 };
    pipeline::upto(&mut world, Step::Bench, &mut src, &mut |s, sum| eprintln!("{s:?} {sum}")).unwrap();

    let profiles: &Profiles = world.solved().unwrap();
    let sheets = world.sheets.as_ref().unwrap();
    // Which sheet holds a point: asked of the sheets' own regions, since the
    // one mesh carries no per-vertex sheet.
    let held: Vec<poly::Indexed> = sheets.sheets.iter().map(|s| poly::Indexed::new(&s.shapes)).collect();
    let sheet_at = |p: [f64; 3]| held.iter().position(|i| i.contains([p[0], p[1]])).map_or(u32::MAX, |k| k as u32);
    let bench = world.bench.as_ref().unwrap();
    let over = poly::Indexed::new(&world.arrangement.as_ref().unwrap().over);
    let inside = |p: [f64; 3]| p[0] >= w[0] && p[0] <= w[2] && p[1] >= w[1] && p[1] <= w[3];

    // `decks`: for every deck face of the arrangement's second layer, the
    // asphalt heights over its probe point — the gap between the deck and
    // the ground layer under it. A gap near zero is two coplanar surfaces.
    if a.get(4).map(String::as_str) == Some("decks") {
        let c = &bench.carriageway;
        let mut gaps: Vec<(f64, [f64; 2], f64)> = Vec::new();
        for d in &world.arrangement.as_ref().unwrap().decks {
            let Some(q) = poly::inside(&d.shape) else { continue };
            let mut zs: Vec<f64> = Vec::new();
            for t in c.indices.chunks_exact(3) {
                let v: Vec<[f64; 3]> = t.iter().map(|&i| c.positions[i as usize]).collect();
                let s = |a: [f64; 3], b: [f64; 3]| (b[0] - a[0]) * (q[1] - a[1]) - (b[1] - a[1]) * (q[0] - a[0]);
                let e = [s(v[0], v[1]), s(v[1], v[2]), s(v[2], v[0])];
                if e.iter().all(|x| *x >= 0.0) || e.iter().all(|x| *x <= 0.0) {
                    zs.push((v[0][2] + v[1][2] + v[2][2]) / 3.0);
                }
            }
            zs.sort_by(f64::total_cmp);
            let gap = if zs.len() >= 2 { zs[zs.len() - 1] - zs[0] } else { f64::NAN };
            gaps.push((gap, q, poly::area(std::slice::from_ref(&d.shape))));
        }
        gaps.sort_by(|x, y| x.0.total_cmp(&y.0));
        for (g, q, m2) in &gaps {
            println!("gap {g:7.2} m at [{:.1},{:.1}] {m2:.1} m2", q[0], q[1]);
        }
        return;
    }

    // With a point: every surface covering it, and at what height.
    if let Some(qs) = a.get(4) {
      for q in qs.split(';') {
        let q: Vec<f64> = q.split(',').map(|s| s.parse().unwrap()).collect();
        println!("== {q:?}");
        let st = world.structure.as_ref();
        let layers: Vec<(&str, &arpentry_world::world::Tri)> = vec![
            ("carriageway", &bench.carriageway),
            ("pavement", &bench.pavement),
            ("ballast", &bench.ballast),
            ("ground", &bench.ground),
            ("kerb", &bench.kerb),
            ("wall", &bench.wall),
        ];
        let _ = st;
        for (name, t) in layers {
            for tr in t.indices.chunks_exact(3) {
                let v: Vec<[f64; 3]> = tr.iter().map(|&i| t.positions[i as usize]).collect();
                let s = |a: [f64; 3], b: [f64; 3]| (b[0] - a[0]) * (q[1] - a[1]) - (b[1] - a[1]) * (q[0] - a[0]);
                let (d0, d1, d2) = (s(v[0], v[1]), s(v[1], v[2]), s(v[2], v[0]));
                let inside = (d0 >= 0.0 && d1 >= 0.0 && d2 >= 0.0) || (d0 <= 0.0 && d1 <= 0.0 && d2 <= 0.0);
                if inside && d0.abs() + d1.abs() + d2.abs() > 0.0 {
                    println!("{name}: z={:.2}/{:.2}/{:.2}", v[0][2], v[1][2], v[2][2]);
                }
            }
        }
        for f in &world.arrangement.as_ref().unwrap().faces {
            if poly::contains(&vec![f.shape.clone()], [q[0], q[1]]) {
                println!("face {:?} sheet={:?} spanned={} near={}", f.material, f.sheet, f.spanned, f.near);
            }
        }
      }
        return;
    }

    let c = &bench.carriageway;
    let mut low: std::collections::BTreeMap<u32, Vec<usize>> = Default::default();
    let mut tris = 0;
    for t in c.indices.chunks_exact(3) {
        let v: Vec<[f64; 3]> = t.iter().map(|&i| c.positions[i as usize]).collect();
        if !v.iter().all(|&p| inside(p)) {
            continue;
        }
        let zs: Vec<f64> = v.iter().map(|p| p[2]).collect();
        let (lo, hi) = (zs.iter().cloned().fold(f64::MAX, f64::min), zs.iter().cloned().fold(f64::MIN, f64::max));
        if hi - lo < dz {
            continue;
        }
        tris += 1;
        for &i in t {
            let s = sheet_at(c.positions[i as usize]);
            low.entry(s).or_default().push(i as usize);
        }
    }
    println!("curtain triangles in window: {tris}");
    for (s, mut vs) in low {
        vs.sort();
        vs.dedup();
        let Some(sh) = sheets.sheets.get(s as usize) else { continue };
        let ax: std::collections::BTreeSet<usize> = sh.axes.iter().map(|x| x.0).collect();
        let ch: std::collections::BTreeSet<usize> = sh.chords.iter().map(|x| x.0).collect();
        println!(
            "\n== sheet {s} group={} spanning={} regions={} area={:.0} m2 spans_area={:.0} axes={} chords={:?}",
            sh.group,
            sh.spanning,
            sh.shapes.len(),
            poly::area(&sh.shapes),
            poly::area(&sh.spans),
            ax.len(),
            ch.iter().map(|&i| format!("{}[{}]", profiles.profiles[i].id, profiles.profiles[i].class)).collect::<Vec<_>>()
        );
        // Per-profile fields, so the answer can be attributed to an axis.
        let one = |pi: usize, spans: &[(usize, f64, f64)]| {
            let p = &profiles.profiles[pi];
            Field::of_stations(std::iter::once((p, ranges(p, spans, pi))))
        };
        let grounds: Vec<(usize, Field)> = ax.iter().map(|&pi| (pi, one(pi, &sh.axes))).collect();
        let chords: Vec<(usize, Field)> = ch.iter().map(|&pi| (pi, one(pi, &sh.chords))).collect();
        let mut shown = 0;
        let mut tally: std::collections::BTreeMap<String, usize> = Default::default();
        for &i in &vs {
            let p = c.positions[i];
            let q = [p[0], p[1]];
            let masked = over.contains(q);
            let best = |fs: &[(usize, Field)]| {
                fs.iter()
                    .filter_map(|(pi, f)| f.at(q).map(|ft| (*pi, ft)))
                    .min_by(|x, y| x.1.d.total_cmp(&y.1.d))
            };
            let g = best(&grounds);
            let k = best(&chords);
            let name = |pi: usize| format!("{}[{}]", profiles.profiles[pi].id, profiles.profiles[pi].class);
            let key = format!(
                "masked={masked} ground_axis={} chord_axis={} chord_wins={}",
                g.map_or("-".into(), |x| name(x.0)),
                k.map_or("-".into(), |x| name(x.0)),
                match (g, k) {
                    (Some(g), Some(k)) => masked && k.1.d <= g.1.d,
                    (None, Some(_)) => masked,
                    _ => false,
                }
            );
            *tally.entry(key).or_default() += 1;
            if shown < 12 {
                shown += 1;
                println!(
                    "  v=[{:.1},{:.1}] z={:.2} masked={masked} ground={} chord={}",
                    p[0],
                    p[1],
                    p[2],
                    g.map_or("-".into(), |(pi, f)| format!("{} h={:.2} d={:.2}", name(pi), f.h, f.d)),
                    k.map_or("-".into(), |(pi, f)| format!("{} h={:.2} d={:.2}", name(pi), f.h, f.d)),
                );
            }
        }
        for (k, n) in tally {
            println!("  {n:6}  {k}");
        }
    }
}
