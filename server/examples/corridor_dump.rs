//! Every node of one corridor: what the terrain said, what the solve returned,
//! and what was attached to it there.
//!
//! `site_probe` names the corridor standing too high; this says *where along
//! it* the height leaves the ground, which is the difference between a profile
//! that was lifted at one node and one that was chorded end to end.
//!
//! Usage: cargo run --release --example corridor_dump -- <segment.parquet> <w,s,e,n> <terrain.pmtiles> <corridor_id>

use arpentry_server::assemble;
use arpentry_server::priors::DECK_THICKNESS_M;
use arpentry_server::project::Bounds;
use arpentry_server::solve;

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let bb: Vec<f64> = a[1].split(',').map(|s| s.parse().unwrap()).collect();
    let bbox = Bounds { west: bb[0], south: bb[1], east: bb[2], north: bb[3] };
    let terrain = std::path::PathBuf::from(&a[2]);
    let want: u32 = a[3].parse().unwrap();

    let mut scene = assemble::run(std::path::Path::new(&a[0]), None, &bbox).expect("assemble");
    let solved = solve::run(&mut scene, Some(&terrain), 16, 0).expect("solve");

    let c = &scene.corridors[want as usize];
    let p = solved.profile(want).expect("no profile");
    println!(
        "corridor #{want}  {:?}  class {}  width {:.1} m  link {}  mode {:?}",
        c.kind,
        c.class_key,
        c.width_m.unwrap_or(f64::NAN),
        c.link,
        solve::Mode::for_kind(c.kind),
    );
    println!("  deviation_m {:.2}  clearance_over_m {:.2}", c.kind.prior().deviation_m, c.kind.prior().clearance_over_m);
    for s in &c.spans {
        println!("  span {:?} level {} arc {:.1}..{:.1}", s.kind, s.level, s.arc0, s.arc1);
    }

    // Junctions this corridor is a member of, and what else meets there.
    println!("\nJUNCTIONS");
    for (ji, j) in scene.junctions.iter().enumerate() {
        let Some(me) = j.members.iter().find(|m| m.corridor == want) else { continue };
        let others: Vec<String> = j
            .members
            .iter()
            .filter(|m| m.corridor != want)
            .map(|m| {
                let oc = &scene.corridors[m.corridor as usize];
                let h = solved
                    .profile(m.corridor)
                    .map(|op| format!("{:.2}", op.road_at_arc(m.arc)))
                    .unwrap_or("—".into());
                format!("#{} {:?} {} @arc {:.0} h {h}", m.corridor, oc.kind, oc.class_key, m.arc)
            })
            .collect();
        println!(
            "  j{ji} at {:.6},{:.6}  my arc {:.1}  solved junction h {}  | {}",
            j.point.x,
            j.point.y,
            me.arc,
            solved.junction_height(ji).map_or("—".into(), |h| format!("{h:.2}")),
            others.join(" ; ")
        );
    }

    // Crossings where this corridor is the upper side — the demands on it.
    println!("\nCROSSINGS WHERE #{want} IS UPPER");
    for x in &solved.crossings {
        if x.upper != want {
            continue;
        }
        let extra = x.lower_kind.prior().clearance_over_m + DECK_THICKNESS_M;
        let lh = x.lower.and_then(|l| solved.profile(l).map(|lp| lp.road_at_arc(x.lower_arc)));
        println!(
            "  arc {:.1} at {:.6},{:.6}  lower {} {:?} L{}  lower h {}  demand {}",
            x.upper_arc,
            x.point.x,
            x.point.y,
            x.lower.map_or("(ground)".into(), |l| format!("#{l}")),
            x.lower_kind,
            x.lower_level,
            lh.map_or("—".into(), |h| format!("{h:.2}")),
            lh.map_or("—".into(), |h| format!("{:.2}", h + extra)),
        );
    }

    println!("\nPROFILE  (standoff = road − reference terrain)");
    println!("   k     arc      lon        lat        terrain     road   standoff  at_grade");
    let (arc, road, terr, nodes, ag) =
        (p.arc(), p.road_m(), p.terrain_m(), p.nodes(), p.at_grade());
    for k in 0..arc.len() {
        println!(
            "  {k:>3} {:>8.1}  {:.6}  {:.6}  {:>8.2} {:>8.2}  {:>+8.2}   {}",
            arc[k],
            nodes[k].x,
            nodes[k].y,
            terr[k],
            road[k],
            road[k] - terr[k],
            if ag[k] { "yes" } else { "NO" }
        );
    }
}
