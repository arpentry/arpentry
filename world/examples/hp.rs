use arpentry_server::dem::Dem;
use arpentry_server::project::Bounds;
use arpentry_world::pipeline::{self, Sources};
use arpentry_world::step::Step;
use arpentry_world::world::World;
fn main() {
    let mut dem = Dem::open(std::path::Path::new("../data/zones/montreux/terrain.pmtiles")).unwrap();
    let mut w = World::new(Bounds { west: 6.91304, south: 46.42968, east: 6.91712, north: 46.43268 });
    let (seg, bld) = (std::path::PathBuf::from("../data/zones/montreux/segment.parquet"), std::path::PathBuf::from("../data/zones/montreux/building.parquet"));
    let mut src = Sources { dem: &mut dem, segments: &seg, buildings: Some(&bld), spacing: 1.0, max_vertices: 2_000_000 };
    pipeline::upto(&mut w, Step::Partition, &mut src, &mut |_, _| {}).unwrap();
    for p in &w.profile.as_ref().unwrap().profiles {
        if !p.id.starts_with("f4667303") && !p.id.starts_with("9083e905") { continue; }
        println!("{} [{}]", p.id, p.class);
        for st in &p.stations {
            println!("   s={:6.1} h={:7.2} ground={:7.2} ref={:7.2} h-ref={:+5.2} {:?}", st.s, st.h, st.ground, st.reference, st.h - st.reference, st.solved);
        }
    }
}
