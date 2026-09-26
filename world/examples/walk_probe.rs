//! Scratch probe: what the bench does to a walk's pavement, and why.
//!
//! For every walk piece near a point, the natural terrain along it, the
//! road axis that answers for it and how far off that axis is, and the
//! height the room therefore takes.
//!
//!   cargo run --release --example walk_probe -- ZONE w,s,e,n lon,lat [r]

use arpentry_server::dem::Dem;
use arpentry_server::project::Bounds;
use arpentry_world::bench::{Field, KERB_RISE_M, ROOM_REACH_M};
use arpentry_world::pipeline::{self, Sources};
use arpentry_world::step::Step;
use arpentry_world::terrain::height_at;
use arpentry_world::width::{self, Family};
use arpentry_world::world::World;

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let zone = std::path::PathBuf::from(&a[0]);
    let b: Vec<f64> = a[1].split(',').map(|s| s.parse().unwrap()).collect();
    let at: Vec<f64> = a[2].split(',').map(|s| s.parse().unwrap()).collect();
    let r: f64 = a.get(3).map(|s| s.parse().unwrap()).unwrap_or(40.0);
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
    pipeline::upto(&mut world, Step::Bench, &mut src, &mut |_, _| {}).unwrap();

    let p = world.extent.frame.to_local(at[0], at[1]);
    println!("probe [{:.1},{:.1}] r={r}", p[0], p[1]);
    let near = |q: &[f64; 2]| (q[0] - p[0]).hypot(q[1] - p[1]) < r;

    let terrain = world.terrain.as_ref().unwrap();
    let profiles = world.profile.as_ref().unwrap();
    let rail = |q: &&arpentry_world::world::Profile| width::family(&q.class) == Family::Rail;
    let field = Field::grounded(profiles.profiles.iter().filter(|q| !rail(q)));

    let roads = world.roads.as_ref().unwrap();
    println!(
        "\n{:<38} {:<10} {:>7} {:>9} {:>9} {:>7} {:>7} {:>8}",
        "walk piece", "class", "s", "terrain", "road h", "dist", "half_w", "room h"
    );
    for pc in roads.pieces() {
        if width::family(&pc.class) != Family::Walk || !pc.pts.iter().any(near) {
            continue;
        }
        let mut s = 0.0;
        for (i, q) in pc.pts.iter().enumerate() {
            if i > 0 {
                s += (q[0] - pc.pts[i - 1][0]).hypot(q[1] - pc.pts[i - 1][1]);
            }
            if !near(q) {
                continue;
            }
            let ground = height_at(terrain, q[0], q[1]);
            match field.at(*q) {
                Some(f) => {
                    let room = f.h + KERB_RISE_M;
                    let reach = f.half_w + ROOM_REACH_M;
                    println!(
                        "{:<38} {:<10} {s:>7.1} {ground:>9.2} {:>9.2} {:>7.2} {:>7.2} {:>8.2}  {}{}",
                        &pc.id[..8.min(pc.id.len())],
                        pc.class,
                        f.h,
                        f.d,
                        f.half_w,
                        room,
                        if f.d > reach { "PAST THE ROOM, batters " } else { "in the room " },
                        format!("{:+.2} off the terrain", room - ground),
                    );
                }
                None => println!(
                    "{:<38} {:<10} {s:>7.1} {ground:>9.2} {:>9} {:>7} {:>7} {:>8}  no road within the field",
                    &pc.id[..8.min(pc.id.len())],
                    pc.class,
                    "-",
                    "-",
                    "-",
                    "-"
                ),
            }
        }
    }
}
