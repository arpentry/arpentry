//! What every probe starts from: a cut zone's world, built up to one step.

use std::path::Path;

use arpentry_server::dem::Dem;
use arpentry_server::project::Bounds;
use arpentry_world::pipeline::{self, Sources};
use arpentry_world::step::Step;
use arpentry_world::world::World;

/// The world over `bbox` (`w,s,e,n` in degrees) from the cut zone at `zone`,
/// built through `until` on a one-metre lattice, each step's line on stderr.
pub fn world(zone: &Path, bbox: &str, until: Step) -> World {
    let b: Vec<f64> = bbox.split(',').map(|s| s.parse().expect("w,s,e,n")).collect();
    let mut dem = Dem::open(&zone.join("terrain.pmtiles")).expect("the zone's terrain");
    let mut world = World::new(Bounds { west: b[0], south: b[1], east: b[2], north: b[3] });
    let (segments, buildings) = (zone.join("segment.parquet"), zone.join("building.parquet"));
    let mut src = Sources {
        dem: &mut dem,
        segments: &segments,
        buildings: buildings.exists().then_some(buildings.as_path()),
        spacing: 1.0,
        max_vertices: 2_000_000,
    };
    pipeline::upto(&mut world, until, &mut src, &mut |step, line| eprintln!("{} {line}", step.name()))
        .expect("the zone builds");
    world
}

/// `lon,lat` in degrees as a point of `world`'s local frame.
#[allow(dead_code)]
pub fn point(world: &World, lonlat: &str) -> [f64; 2] {
    let at: Vec<f64> = lonlat.split(',').map(|s| s.parse().expect("lon,lat")).collect();
    world.extent.frame.to_local(at[0], at[1])
}
