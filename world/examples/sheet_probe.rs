//! Scratch probe: which sheet the pieces near a point are in, and why.
//!
//!   cargo run --release --example sheet_probe -- ZONE w,s,e,n lon,lat [r]

use arpentry_server::dem::Dem;
use arpentry_server::project::Bounds;
use arpentry_world::pipeline::{self, Sources};
use arpentry_world::step::Step;
use arpentry_world::world::{Kind, World};

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let zone = std::path::PathBuf::from(&a[0]);
    let b: Vec<f64> = a[1].split(',').map(|s| s.parse().unwrap()).collect();
    let at: Vec<f64> = a[2].split(',').map(|s| s.parse().unwrap()).collect();
    let r: f64 = a.get(3).map(|s| s.parse().unwrap()).unwrap_or(25.0);
    let mut dem = Dem::open(&zone.join("terrain.pmtiles")).unwrap();
    let mut world = World::new(Bounds { west: b[0], south: b[1], east: b[2], north: b[3] });
    let (seg, bld) = (zone.join("segment.parquet"), zone.join("building.parquet"));
    let mut src = Sources {
        dem: &mut dem,
        segments: &seg,
        buildings: Some(&bld),
        spacing: 1.0,
        max_vertices: 2_000_000,
    };
    pipeline::upto(&mut world, Step::Sheet, &mut src, &mut |_, _| {}).unwrap();

    let p = world.extent.frame.to_local(at[0], at[1]);
    println!("probe [{:.1},{:.1}] r={r}\n", p[0], p[1]);
    let roads = world.network().unwrap();
    let group = &world.partition.as_ref().unwrap().groups.of;
    let near = |pts: &[[f64; 2]]| pts.iter().any(|q| (q[0] - p[0]).hypot(q[1] - p[1]) < r);

    println!("{:<5} {:<38} {:<12} {:<9} {:>5} {:>6} {:>6}", "idx", "id", "class", "kind", "grp", "len", "width");
    let mut mine: Vec<usize> = Vec::new();
    for (i, pc) in roads.pieces().enumerate() {
        if !near(&pc.pts) {
            continue;
        }
        mine.push(i);
        let kind = match pc.kind {
            Kind::Ground => "ground".into(),
            Kind::Bridge(l) => format!("deck{l:+}"),
            Kind::Tunnel(l) => format!("bore{l:+}"),
            k => format!("{k:?}"),
        };
        let len: f64 = pc.pts.windows(2).map(|w| (w[1][0] - w[0][0]).hypot(w[1][1] - w[0][1])).sum();
        println!(
            "{:<5} {:<38} {:<12} {:<9} {:>5} {:>6.1} {:>6.1}",
            i, pc.id, pc.class, kind, group[i], len, pc.width_m
        );
    }

    println!("\nvertex meetings among them (E=end, I=interior):");
    let pieces: Vec<_> = roads.pieces().collect();
    for (n, &i) in mine.iter().enumerate() {
        for &j in &mine[n + 1..] {
            let (a, b) = (pieces[i], pieces[j]);
            for (ka, va) in a.pts.iter().enumerate() {
                for (kb, vb) in b.pts.iter().enumerate() {
                    if (va[0] - vb[0]).hypot(va[1] - vb[1]) > 0.02 {
                        continue;
                    }
                    let ea = if ka == 0 || ka == a.pts.len() - 1 { "E" } else { "I" };
                    let eb = if kb == 0 || kb == b.pts.len() - 1 { "E" } else { "I" };
                    println!(
                        "  {i}({ea}) x {j}({eb}) at [{:.1},{:.1}]  groups {} / {}{}",
                        va[0], va[1], group[i], group[j],
                        if group[i] == group[j] { "" } else { "   <-- NOT UNIONED" }
                    );
                }
            }
        }
    }

    println!("\nsheets near the probe:");
    let sheets = world.sheets.as_ref().unwrap();
    for (k, sh) in sheets.sheets.iter().enumerate() {
        if !sh.shapes.iter().flatten().any(|ring| near(ring)) {
            continue;
        }
        println!(
            "  sheet {k}: family={:?} group={} regions={} m2={:.0} spanning={} holds={}",
            sh.family,
            sh.group,
            sh.shapes.len(),
            arpentry_world::poly::area(&sh.shapes),
            sh.spanning,
            mine.iter().filter(|&&i| group[i] == sh.group).count()
        );
    }

    println!("\nthe surface's own regions near the probe (the legs carriageway):");
    let f = world.legs.as_ref().unwrap();
    for (k, region) in f.surface.carriageway.iter().enumerate() {
        if region.iter().any(|ring| near(ring)) {
            println!("  region {k}: rings={} m2={:.0}", region.len(), arpentry_world::poly::area(&vec![region.clone()]));
        }
    }
}
