//! Scratch probe: why the terrain has a hole where it has one.
//!
//! The bench cuts the ground as `rect − outline`, and
//!
//!     on_ground = ⋃ sheets.shapes  −  dilate(sheets.spanned(), OVER_RIM_M)
//!     outline   = on_ground ∪ room.pavement ∪ galleries
//!
//! so a hole is exactly a point in `outline`. This walks that chain over a
//! window and prints one character per sample, then the full membership of
//! one named point.
//!
//!   cargo run --release --example hole_probe -- ZONE w,s,e,n x0,y0,x1,y1 [step] [px,py]

use arpentry_server::dem::Dem;
use arpentry_server::project::Bounds;
use arpentry_world::bench::{Portals, OVER_RIM_M};
use arpentry_world::pipeline::{self, Sources};
use arpentry_world::poly;
use arpentry_world::step::Step;
use arpentry_world::width::Family;
use arpentry_world::world::World;

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let zone = std::path::PathBuf::from(&a[0]);
    let b: Vec<f64> = a[1].split(',').map(|s| s.parse().unwrap()).collect();
    let w: Vec<f64> = a[2].split(',').map(|s| s.parse().unwrap()).collect();
    let step: f64 = a.get(3).map(|s| s.parse().unwrap()).unwrap_or(1.0);
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

    let sheets = world.sheets.as_ref().unwrap();
    let room = world.room.as_ref().unwrap();
    let roads = world.roads.as_ref().unwrap();
    let profiles = world.profile.as_ref().unwrap();

    // Exactly the bench's own three lines.
    let paved: poly::Shapes = sheets.sheets.iter().flat_map(|s| s.shapes.iter().cloned()).collect();
    let paved = poly::union_all(&paved);
    let spanned = poly::dilate(&sheets.spanned(), OVER_RIM_M);
    let (_, galleries) = Portals::new(&roads.spans, profiles);
    let on_ground = poly::difference(&paved, &spanned);
    let carried = poly::intersect(&room.surface.walk, &poly::dilate(&sheets.spanned(), arpentry_world::bench::ROOM_REACH_M));
    let walk_on_ground = poly::difference(&room.surface.walk, &carried);
    let outline = poly::union_of(&[&on_ground, &walk_on_ground, &galleries]);

    // The carriageway sheets alone, so a hole can be attributed.
    let car: poly::Shapes = poly::union_all(
        &sheets.of(Family::Carriageway).flat_map(|s| s.shapes.iter().cloned()).collect(),
    );

    println!("legend:  . ground   # hole from asphalt   w hole from pavement   g hole from a gallery");
    println!("         D deck   C walk a deck carries (no hole)   ? hole nothing explains\n");
    let mut y = w[3];
    while y >= w[1] {
        print!("{y:7.1} ");
        let mut x = w[0];
        while x <= w[2] {
            let p = [x, y];
            let c = if !poly::contains(&outline, p) {
                if poly::contains(&spanned, p) && poly::contains(&paved, p) {
                    'D'
                } else if poly::contains(&carried, p) {
                    'C'
                } else {
                    '.'
                }
            } else if poly::contains(&on_ground, p) {
                if poly::contains(&car, p) { '#' } else { 'o' }
            } else if poly::contains(&walk_on_ground, p) {
                'w'
            } else if poly::contains(&galleries, p) {
                'g'
            } else {
                '?'
            };
            print!("{c}");
            x += step;
        }
        println!();
        y -= step;
    }
    println!("\n{:7} {}", "", (w[0] as i64..=w[2] as i64).step_by(10).map(|v| format!("{v:<10}")).collect::<String>());

    if let Some(at) = a.get(4) {
        let q: Vec<f64> = at.split(',').map(|s| s.parse().unwrap()).collect();
        let p = [q[0], q[1]];
        println!("\nat [{:.1},{:.1}]:", p[0], p[1]);
        for (name, shape) in [
            ("sheets' paving", &paved),
            ("  of which carriageway", &car),
            ("sheets.spanned() + rim", &spanned),
            ("on_ground = paving - spanned", &on_ground),
            ("room.pavement", &room.surface.walk),
            ("  of which carried by a deck", &carried),
            ("  of which on the ground", &walk_on_ground),
            ("galleries", &galleries),
            ("OUTLINE (a hole is here)", &outline),
        ] {
            println!("  {name:<30} {}", poly::contains(shape, p));
        }
        // And what the bench actually put there.
        let bench = world.bench.as_ref().unwrap();
        let terrain = world.terrain.as_ref().unwrap();
        let nat = arpentry_world::terrain::height_at(terrain, p[0], p[1]);
        let nearest = |t: &arpentry_world::world::Tri| {
            t.positions
                .iter()
                .map(|q| ((q[0] - p[0]).hypot(q[1] - p[1]), q[2]))
                .min_by(|a, b| a.0.total_cmp(&b.0))
        };
        println!("  natural terrain                {nat:.2}");
        for (name, t) in [
            ("nearest pavement vertex", &bench.pavement),
            ("nearest carriageway vertex", &bench.carriageway),
            ("nearest ground vertex", &bench.ground),
        ] {
            match nearest(t) {
                Some((d, z)) => println!("  {name:<30} z={z:.2} ({d:.2} m away, {:+.2} off the terrain)", z - nat),
                None => println!("  {name:<30} none"),
            }
        }
    }
}
