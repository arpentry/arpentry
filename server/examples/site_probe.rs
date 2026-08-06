//! What lifted the road *here*?
//!
//! The scorecard names a coordinate; this says which corridor is standing at
//! it, how far its solved profile left its own reference terrain, and which
//! derived crossing wrote the demand that put it there. One site, everything
//! about it, so a screenshot's "that road is too high" becomes a corridor id
//! and a constraint.
//!
//! Usage: cargo run --release --example site_probe -- <segment.parquet> <w,s,e,n> <terrain.pmtiles> <lon,lat> [radius_m]

use arpentry_server::assemble;
use arpentry_server::priors::DECK_THICKNESS_M;
use arpentry_server::project::Bounds;
use arpentry_server::scene::SpanKind;
use arpentry_server::solve;

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let bb: Vec<f64> = a[1].split(',').map(|s| s.parse().unwrap()).collect();
    let bbox = Bounds { west: bb[0], south: bb[1], east: bb[2], north: bb[3] };
    let terrain = std::path::PathBuf::from(&a[2]);
    let at: Vec<f64> = a[3].split(',').map(|s| s.parse().unwrap()).collect();
    let (lon0, lat0) = (at[0], at[1]);
    let radius = a.get(4).and_then(|s| s.parse().ok()).unwrap_or(60.0);

    let mut scene = assemble::run(std::path::Path::new(&a[0]), None, &bbox).expect("assemble");
    let solved = solve::run(&mut scene, Some(&terrain), 16, 0).expect("solve");

    let m_lat = 110_540.0;
    let m_lon = 111_320.0 * lat0.to_radians().cos();
    let dist = |c: geo_types::Coord| -> f64 {
        let dx = (c.x - lon0) * m_lon;
        let dy = (c.y - lat0) * m_lat;
        (dx * dx + dy * dy).sqrt()
    };

    println!("site {lon0:.6},{lat0:.6}  radius {radius:.0} m\n");

    // ---- corridors standing at the site -------------------------------------
    println!("CORRIDORS WITHIN RADIUS");
    let mut hits: Vec<(f64, u32)> = Vec::new();
    for c in &scene.corridors {
        let Some((d, _)) = c
            .nodes
            .iter()
            .map(|&n| (dist(n), n))
            .min_by(|a, b| a.0.total_cmp(&b.0))
        else {
            continue;
        };
        if d <= radius {
            hits.push((d, c.id));
        }
    }
    hits.sort_by(|a, b| a.0.total_cmp(&b.0));
    for &(d, id) in &hits {
        let c = &scene.corridors[id as usize];
        let i = (0..c.nodes.len()).min_by(|&x, &y| dist(c.nodes[x]).total_cmp(&dist(c.nodes[y])));
        let Some(i) = i else { continue };
        let spans: Vec<String> = c
            .spans
            .iter()
            .map(|s| format!("{:?}@L{}[{:.0}..{:.0}]", s.kind, s.level, s.arc0, s.arc1))
            .collect();
        match solved.profile(id) {
            Some(p) => {
                let (road, terr) = (p.road_m()[i], p.terrain_m()[i]);
                println!(
                    "  #{id:<5} {:<28} {:<22} {d:>5.1} m away, node {i}/{}\n\
                     \x20        arc {:>8.1}  road {:>8.2}  ref-terrain {:>8.2}  standoff {:>+7.2}  at_grade {}\n\
                     \x20        spans: {}",
                    format!("{:?}", c.kind),
                    c.class_key,
                    c.nodes.len(),
                    p.arc()[i],
                    road,
                    terr,
                    road - terr,
                    p.at_grade()[i],
                    if spans.is_empty() { "none (all at grade)".into() } else { spans.join(" ") },
                );
            }
            None => println!(
                "  #{id:<5} {:<28} {:<22} {d:>5.1} m away — NO PROFILE",
                format!("{:?}", c.kind),
                c.class_key
            ),
        }
    }

    // ---- crossings touching those corridors ---------------------------------
    let near: std::collections::HashSet<u32> = hits.iter().map(|&(_, id)| id).collect();
    println!("\nDERIVED CROSSINGS TOUCHING THEM");
    let mut any = false;
    for c in &solved.crossings {
        let touches = near.contains(&c.upper) || c.lower.is_some_and(|l| near.contains(&l));
        if !touches || dist(c.point) > radius * 3.0 {
            continue;
        }
        any = true;
        let uc = &scene.corridors[c.upper as usize];
        let extra = c.lower_kind.prior().clearance_over_m + DECK_THICKNESS_M;
        let up = solved.profile(c.upper);
        let (lower_desc, lower_h, lower_bore) = match c.lower {
            Some(l) => {
                let lc = &scene.corridors[l as usize];
                let kind_at = lc
                    .spans
                    .iter()
                    .find(|s| c.lower_arc >= s.arc0 && c.lower_arc <= s.arc1)
                    .map_or(SpanKind::Grade, |s| s.kind);
                let h = solved.profile(l).map(|p| p.road_at_arc(c.lower_arc));
                (format!("#{l} {:?} {}", lc.kind, lc.class_key), h, kind_at == SpanKind::Tunnel)
            }
            None => ("(unprofiled — ground)".into(), None, false),
        };
        let demand = lower_h.map(|h| h + extra);
        let solved_up = up.map(|p| p.road_at_arc(c.upper_arc));
        println!(
            "  {:.6},{:.6}\n\
             \x20   upper #{} {:?} {}  L{}   solved {:>8.2}\n\
             \x20   lower {lower_desc}  L{}   {}{}\n\
             \x20   clearance_over {:.2} + slab {:.2} = {extra:.2}   demand {}   lift {}",
            c.point.x,
            c.point.y,
            c.upper,
            uc.kind,
            uc.class_key,
            c.upper_level,
            solved_up.unwrap_or(f64::NAN),
            c.lower_level,
            lower_h.map_or("(none)".into(), |h| format!("solved {h:>8.2}")),
            if lower_bore { "   [LOWER IS IN A BORE]" } else { "" },
            c.lower_kind.prior().clearance_over_m,
            DECK_THICKNESS_M,
            demand.map_or("—".into(), |d| format!("{d:>8.2}")),
            match (demand, solved_up) {
                (Some(d), Some(s)) => format!("{:>+7.2}", d - s),
                _ => "—".into(),
            },
        );
    }
    if !any {
        println!("  none");
    }

    println!(
        "\nrelaxation: {} sweeps, {} demands dropped (worst {:.2} m)",
        solved.relaxed.sweeps, solved.relaxed.demands_dropped, solved.relaxed.worst_dropped_m
    );
}
