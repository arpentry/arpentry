//! Scratch probe: what each conditioning pass did along one way's axis.
//!
//!   cargo run --release --example ref_probe -- ZONE w,s,e,n ID_PREFIX...

use arpentry_server::dem::Dem;
use arpentry_server::project::Bounds;
use arpentry_world::pipeline::{self, Sources};
use arpentry_world::reference;
use arpentry_world::step::Step;
use arpentry_world::world::World;

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let zone = std::path::PathBuf::from(&a[0]);
    let b: Vec<f64> = a[1].split(',').map(|s| s.parse().unwrap()).collect();
    let want: Vec<&str> = a[2..].iter().map(|s| s.as_str()).collect();
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
    pipeline::upto(&mut world, Step::Reference, &mut src, &mut |_, _| {}).unwrap();

    let roads = world.roads.as_ref().unwrap();
    let axes = &world.reference.as_ref().unwrap().axes;
    for ax in axes {
        let w = &roads.ways[ax.way];
        if !want.iter().any(|p| w.id.starts_with(p)) {
            continue;
        }
        let closed = reference::close_notches(&ax.s, &ax.ground);
        let opened = reference::open_bumps(&ax.s, &closed);
        println!(
            "== {} [{}] len={:.1} spans={:?}",
            w.id,
            w.class,
            ax.len(),
            w.spans.iter().map(|s| (s.a0, s.a1, s.kind)).collect::<Vec<_>>()
        );
        println!(
            "  refused_notch={:?} refused_crest={:?}",
            ax.refused_notch, ax.refused_crest
        );
        println!(
            "{:>9} {:>9} {:>9} {:>9} {:>9} {:>8} {:>8} {:>6} {:>8}",
            "s", "ground", "closed", "opened", "ref(h)", "fill", "shave", "blind", "spanned"
        );
        for k in 0..ax.s.len() {
            println!(
                "{:>9.3} {:>9.2} {:>9.2} {:>9.2} {:>9.2} {:>+8.2} {:>+8.2} {:>6} {:>8}",
                ax.s[k],
                ax.ground[k],
                closed[k],
                opened[k],
                ax.h[k],
                closed[k] - ax.ground[k],
                opened[k] - closed[k],
                ax.blind[k],
                ax.spanned[k]
            );
        }
    }
}
