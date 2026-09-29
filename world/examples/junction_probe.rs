//! Scratch probe: the solved profile of every way near a point.
//!
//!   cargo run --release --example junction_probe -- ZONE w,s,e,n lon,lat [r]

use arpentry_server::dem::Dem;
use arpentry_server::project::Bounds;
use arpentry_world::pipeline::{self, Sources};
use arpentry_world::step::Step;
use arpentry_world::world::{connector, Kind, World};

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let zone = std::path::PathBuf::from(&a[0]);
    let b: Vec<f64> = a[1].split(',').map(|s| s.parse().unwrap()).collect();
    let at: Vec<f64> = a[2].split(',').map(|s| s.parse().unwrap()).collect();
    let r: f64 = a.get(3).map(|s| s.parse().unwrap()).unwrap_or(30.0);
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
    pipeline::upto(&mut world, Step::Crossing, &mut src, &mut |s, sum| println!("{s:?} {sum}"))
        .unwrap();

    let p = world.extent.frame.to_local(at[0], at[1]);
    println!("\nprobe [{:.1},{:.1}] r={r}\n", p[0], p[1]);
    let near = |q: &[f64; 2]| (q[0] - p[0]).hypot(q[1] - p[1]) < r;

    for pr in &world.solved().unwrap().profiles {
        if !pr.stations.iter().any(|st| near(&st.p)) {
            continue;
        }
        println!("== {} [{}] w={:.1} spans={:?}", pr.id, pr.class, pr.width_m, pr.spans);
        for (k, st) in pr.stations.iter().enumerate() {
            let tag = if near(&st.p) { "*" } else { " " };
            if !near(&st.p) && k % 10 != 0 {
                continue;
            }
            println!(
                "  {tag} k={k:<4} s={:7.1} h={:8.2} ground={:8.2} ref={:8.2} h-g={:+6.2} {:?} conn={:?}",
                st.s,
                st.h,
                st.ground,
                st.reference,
                st.h - st.ground,
                st.solved,
                connector(st.p)
            );
        }
    }

    // Which connectors are shared, and what each way puts there.
    println!("\nshared connectors near the probe:");
    let mut by: std::collections::HashMap<(i64, i64), Vec<(String, String, f64, f64, bool, bool)>> =
        Default::default();
    for pr in &world.solved().unwrap().profiles {
        let runs = pr.runs();
        for (k, st) in pr.stations.iter().enumerate() {
            if !near(&st.p) {
                continue;
            }
            let terminal = k == 0 || k == pr.stations.len() - 1;
            let structural = runs
                .iter()
                .any(|&(k0, k1, kind)| k >= k0 && k <= k1 && kind.is_structure());
            by.entry(connector(st.p)).or_default().push((
                pr.id.clone(),
                pr.class.clone(),
                st.h,
                st.ground,
                terminal,
                structural,
            ));
        }
    }
    let mut keys: Vec<_> = by.keys().copied().collect();
    keys.sort();
    for key in keys {
        let v = &by[&key];
        if v.len() < 2 {
            continue;
        }
        let lo = v.iter().map(|x| x.2).fold(f64::INFINITY, f64::min);
        let hi = v.iter().map(|x| x.2).fold(f64::NEG_INFINITY, f64::max);
        println!("  {key:?} spread={:.2}", hi - lo);
        for (id, class, h, g, terminal, structural) in v {
            println!(
                "     {:8} {:<14} h={:8.2} ground={:8.2} {} {}",
                &id[..8],
                class,
                h,
                g,
                if *terminal { "END " } else { "MID " },
                if *structural { "SPAN" } else { "grnd" }
            );
        }
    }

    // The span mask the bench reads, and what it covers near the probe.
    pipeline::upto(&mut world, Step::Sheet, &mut src, &mut |_, _| {}).unwrap();
    let sheets = world.sheets.as_ref().unwrap();
    println!("\nsheets and their span masks:");
    for (k, sh) in sheets.sheets.iter().enumerate() {
        let bbox = |regions: &Vec<Vec<Vec<[f64; 2]>>>| {
            let mut b = [f64::MAX, f64::MAX, f64::MIN, f64::MIN];
            for r in regions.iter().flatten() {
                for q in r {
                    b[0] = b[0].min(q[0]);
                    b[1] = b[1].min(q[1]);
                    b[2] = b[2].max(q[0]);
                    b[3] = b[3].max(q[1]);
                }
            }
            b
        };
        if sh.spans.is_empty() {
            continue;
        }
        let b = bbox(&sh.spans);
        println!(
            "  sheet {k} family={:?} group={} spans: rings={} m2={:.0} bbox=[{:.1},{:.1} .. {:.1},{:.1}] chords={:?}",
            sh.family,
            sh.group,
            sh.spans.len(),
            arpentry_world::poly::area(&sh.spans),
            b[0], b[1], b[2], b[3],
            sh.chords
        );
        if sh.family != arpentry_world::width::Family::Carriageway {
            continue;
        }
        for (r, region) in sh.spans.iter().enumerate() {
            for (j, ring) in region.iter().enumerate() {
                let b = bbox(&vec![vec![ring.clone()]]);
                println!(
                    "     region {r} ring {j}: n={} m2={:.1} bbox=[{:.1},{:.1} .. {:.1},{:.1}]",
                    ring.len(),
                    arpentry_world::poly::area(&vec![vec![ring.clone()]]),
                    b[0], b[1], b[2], b[3]
                );
                if ring.len() <= 40 {
                    let s: Vec<String> = ring.iter().map(|q| format!("{:.1},{:.1}", q[0], q[1])).collect();
                    println!("        {}", s.join("  "));
                }
            }
        }
    }

    // Does the pavement cross the asphalt, as polygons?
    let room = world.room.as_ref().unwrap();
    let fillet = world.legs.as_ref().unwrap();
    let surface = world.surface.as_ref().unwrap();
    let sheet_car: Vec<Vec<Vec<[f64; 2]>>> = sheets
        .sheets
        .iter()
        .filter(|s| s.family == arpentry_world::width::Family::Carriageway)
        .flat_map(|s| s.shapes.iter().cloned())
        .collect();
    let a = |s: &Vec<Vec<Vec<[f64; 2]>>>| arpentry_world::poly::area(s);
    let x = |p: &Vec<Vec<Vec<[f64; 2]>>>, q: &Vec<Vec<Vec<[f64; 2]>>>| {
        arpentry_world::poly::area(&arpentry_world::poly::intersect(p, q))
    };
    println!("\npavement against asphalt, as polygons:");
    println!("  room.pavement          m2={:.1}", a(&room.surface.walk));
    println!("  surface.carriageway    m2={:.1}  overlap={:.2}", a(&surface.carriageway), x(&surface.carriageway, &room.surface.walk));
    println!("  legs.carriageway       m2={:.1}  overlap={:.2}", a(&fillet.surface.carriageway), x(&fillet.surface.carriageway, &room.surface.walk));
    println!("  sheets' carriageway    m2={:.1}  overlap={:.2}", a(&sheet_car), x(&sheet_car, &room.surface.walk));
    println!("  surface.ballast        m2={:.1}  overlap={:.2}", a(&surface.ballast), x(&surface.ballast, &room.surface.walk));

    let facade = world.facade.as_ref().unwrap();
    // The pavement near the probe, stage by stage: how much of the disc
    // around it each step leaves paved, so the step that cuts a notch is
    // the one the number drops at.
    {
        let disc = arpentry_world::poly::dilate(
            &vec![vec![vec![
                [p[0] - 0.05, p[1] - 0.05],
                [p[0] + 0.05, p[1] - 0.05],
                [p[0] + 0.05, p[1] + 0.05],
                [p[0] - 0.05, p[1] + 0.05],
            ]]],
            r,
        );
        let a = |s: &Vec<Vec<Vec<[f64; 2]>>>| {
            arpentry_world::poly::area(&arpentry_world::poly::intersect(s, &disc))
        };
        let kerb = world.kerb.as_ref().unwrap();
        println!("\npaved area within {r} m of the probe, stage by stage:");
        println!("  surface.walk      {:8.1}", a(&surface.walk));
        println!("  kerb.pavement     {:8.1}", a(&kerb.surface.walk));
        println!("  legs.pavement     {:8.1}", a(&fillet.surface.walk));
        println!("  room.pavement     {:8.1}", a(&room.surface.walk));
        println!("  surface.carriage  {:8.1}", a(&surface.carriageway));
        println!("  surface.ballast   {:8.1}", a(&surface.ballast));
        println!("  surface.spanned   {:8.1}", a(&surface.spanned));
        for (name, s) in [
            ("surface.walk", &surface.walk),
            ("kerb.pavement", &kerb.surface.walk),
            ("legs.pavement", &fillet.surface.walk),
            ("room.pavement", &room.surface.walk),
        ] {
            let near = arpentry_world::poly::intersect(s, &disc);
            println!("  {name} regions near the probe:");
            for (k, region) in near.iter().enumerate() {
                let ar = arpentry_world::poly::area(&vec![region.clone()]);
                let per: f64 = region
                    .iter()
                    .flat_map(|ring| {
                        (0..ring.len()).map(move |i| {
                            let (u, v) = (ring[i], ring[(i + 1) % ring.len()]);
                            (v[0] - u[0]).hypot(v[1] - u[1])
                        })
                    })
                    .sum();
                if ar > 8.0 {
                    continue;
                }
                let mut b = [f64::MAX, f64::MAX, f64::MIN, f64::MIN];
                for q in region.iter().flatten() {
                    b[0] = b[0].min(q[0]);
                    b[1] = b[1].min(q[1]);
                    b[2] = b[2].max(q[0]);
                    b[3] = b[3].max(q[1]);
                }
                // What lies just outside each stretch of its boundary.
                // Outside an outer (counter-clockwise) ring is to the
                // RIGHT of each edge. Taking the left samples the region
                // itself, which reads "open" everywhere and is a lie.
                let outside = |q: [f64; 2], t: [f64; 2]| {
                    [q[0] + t[1] * 0.05, q[1] - t[0] * 0.05]
                };
                let mut share = [0.0f64; 5];
                for ring in region {
                    for i in 0..ring.len() {
                        let (u, v) = (ring[i], ring[(i + 1) % ring.len()]);
                        let d = (v[0] - u[0]).hypot(v[1] - u[1]);
                        if d <= 0.0 {
                            continue;
                        }
                        let t = [(v[0] - u[0]) / d, (v[1] - u[1]) / d];
                        let m = outside([(u[0] + v[0]) / 2.0, (u[1] + v[1]) / 2.0], t);
                        let hit = if arpentry_world::poly::contains(&facade.solid, m) {
                            0
                        } else if arpentry_world::poly::contains(&fillet.surface.carriageway, m) {
                            1
                        } else if arpentry_world::poly::contains(&surface.ballast, m) {
                            2
                        } else if arpentry_world::poly::contains(&surface.spanned, m) {
                            3
                        } else {
                            4
                        };
                        share[hit] += d;
                    }
                }
                let pct = |x: f64| if per > 0.0 { 100.0 * x / per } else { 0.0 };
                println!(
                    "     small region {k}: m2={ar:6.2} width~{:.2} at [{:.1},{:.1} .. {:.1},{:.1}]  \
                     wall {:3.0}% asphalt {:3.0}% ballast {:3.0}% span {:3.0}% open {:3.0}%",
                    if per > 0.0 { 2.0 * ar / per } else { 0.0 },
                    b[0], b[1], b[2], b[3],
                    pct(share[0]), pct(share[1]), pct(share[2]), pct(share[3]), pct(share[4])
                );
            }
        }
    }

    // Where does a span's ribbon stick out past the ground asphalt?
    println!("\nspan ribbon against the ground asphalt:");
    let ribbons = world.ribbons.as_ref().unwrap();
    for (fam, group, span) in ribbons.spans.iter().cloned() {
        if fam != arpentry_world::width::Family::Carriageway {
            continue;
        }
        let over = arpentry_world::poly::difference(&span, &surface.carriageway);
        if arpentry_world::poly::area(&over) < 0.01 {
            continue;
        }
        let masked = ribbons
            .masks
            .iter()
            .cloned()
            .find(|(f, g, _)| *f == fam && *g == group)
            .map(|(.., s)| s)
            .unwrap_or_default();
        let over_sq = arpentry_world::poly::difference(&masked, &surface.carriageway);
        println!(
            "  group {group}: span_m2={:.1} past the asphalt round={:.2} square={:.2}",
            arpentry_world::poly::area(&span),
            arpentry_world::poly::area(&over),
            arpentry_world::poly::area(&over_sq)
        );
        for (k, region) in over.iter().enumerate() {
            let a = arpentry_world::poly::area(&vec![region.clone()]);
            let mut b = [f64::MAX, f64::MAX, f64::MIN, f64::MIN];
            for q in region.iter().flatten() {
                b[0] = b[0].min(q[0]);
                b[1] = b[1].min(q[1]);
                b[2] = b[2].max(q[0]);
                b[3] = b[3].max(q[1]);
            }
            println!("     lobe {k}: m2={a:.2} bbox=[{:.1},{:.1} .. {:.1},{:.1}]", b[0], b[1], b[2], b[3]);
        }
    }

    let roads = world.network().unwrap();
    println!("\npieces near the probe:");
    for (i, pc) in roads.pieces().enumerate() {
        if !pc.pts.iter().any(near) {
            continue;
        }
        let kind = match pc.kind {
            Kind::Ground => "ground".into(),
            k => format!("{k:?}"),
        };
        println!("  {i:<4} {} {:<14} {kind}", &pc.id[..8], pc.class);
    }
}
