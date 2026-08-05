//! Which corridors the solve drove furthest below their own ground, and what
//! asked them to.
//!
//! A scratch instrument for the clearance dip: splitting a crossing's deficit
//! between the two sides put a narrow-gauge railway 288 m under the terrain,
//! and the interesting question is not that it is deep but *what kept asking*.
//! Prints the worst corridors by `terrain − road`, then every derived crossing
//! that touches one, from both sides.
//!
//! Usage: cargo run --release --example dip_probe -- <segment.parquet> <w,s,e,n> <terrain.pmtiles>

use arpentry_server::assemble;
use arpentry_server::project::Bounds;
use arpentry_server::solve;

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let bb: Vec<f64> = a[1].split(',').map(|s| s.parse().unwrap()).collect();
    let bbox = Bounds { west: bb[0], south: bb[1], east: bb[2], north: bb[3] };
    let terrain = std::path::PathBuf::from(&a[2]);

    let mut scene = assemble::run(std::path::Path::new(&a[0]), None, &bbox).expect("assemble");
    let solved = solve::run(&mut scene, Some(&terrain), 16, 0).expect("solve");

    // With a corridor id, dump that one corridor's profile instead.
    if let Some(id) = a.get(3).and_then(|s| s.parse::<u32>().ok()) {
        let c = &scene.corridors[id as usize];
        let p = solved.profile(id).expect("profiled");
        println!("corridor {id} {} spans {:?}", c.class_key, c.spans);
        let (arc, road, terr, ag) = (p.arc(), p.road_m(), p.terrain_m(), p.at_grade());
        for i in (0..arc.len()).step_by(4) {
            println!(
                "  {:7.0} m  road {:8.1}  terrain {:8.1}  gap {:8.1}  {}",
                arc[i], road[i], terr[i], road[i] - terr[i],
                if ag[i] { "grade" } else { "structure" }
            );
        }
        return;
    }

    let mut worst: Vec<(f64, u32, usize)> = Vec::new();
    for c in &scene.corridors {
        let Some(p) = solved.profile(c.id) else { continue };
        let (road, terr) = (p.road_m(), p.terrain_m());
        let mut deepest = (0.0f64, 0usize);
        for i in 0..road.len() {
            let d = terr[i] - road[i];
            if d > deepest.0 {
                deepest = (d, i);
            }
        }
        worst.push((deepest.0, c.id, deepest.1));
    }
    worst.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());

    println!("\nthe deepest corridors, by how far the road ended below the raw terrain\n");
    for &(depth, id, node) in worst.iter().take(10) {
        let c = &scene.corridors[id as usize];
        let p = solved.profile(id).expect("profiled");
        let pt = p.nodes()[node];
        println!(
            "\ncorridor {id}  {} ({:?})  {:.0} m deep at {:.5},{:.5}  ({} nodes, {:.0} m long)",
            c.class_key,
            c.kind.stratum(),
            depth,
            pt.x,
            pt.y,
            p.arc().len(),
            p.arc().last().copied().unwrap_or(0.0)
        );
        // Every crossing that touches it, and which side it is on.
        for x in solved.crossings.iter().filter(|x| x.upper == id || x.lower == Some(id)) {
            let side = if x.upper == id { "above" } else { "below" };
            let other = if x.upper == id { x.lower } else { Some(x.upper) };
            let (oclass, ostratum) = other.map_or(("—".to_string(), String::new()), |o| {
                let oc = &scene.corridors[o as usize];
                (oc.class_key.clone(), format!("{:?}", oc.kind.stratum()))
            });
            let up_h = solved.profile(x.upper).map(|p| p.road_at_arc(x.upper_arc));
            let lo_h = x.lower.and_then(|l| solved.profile(l)).map(|p| p.road_at_arc(x.lower_arc));
            println!(
                "    {side} {oclass} ({ostratum})  L{} over L{}  upper {:?} lower {:?}  at {:.5},{:.5}",
                x.upper_level,
                x.lower_level,
                up_h.map(|h| (h * 10.0).round() / 10.0),
                lo_h.map(|h| (h * 10.0).round() / 10.0),
                x.point.x,
                x.point.y
            );
        }
    }
}
