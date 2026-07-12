//! Per-zoom compressed byte totals of an .arpa archive.
//! Usage: cargo run --release --example zsizes -- <archive>

use arpentry_server::archive::Archive;

fn main() {
    let path = std::env::args().nth(1).unwrap();
    let data = std::fs::read(&path).unwrap();
    let reader = Archive::open(&data).unwrap();
    let mut per_zoom: std::collections::BTreeMap<u8, (usize, usize, usize)> = Default::default();
    for e in reader.entries() {
        let blob = reader.get(e.z, e.x, e.y).unwrap();
        let ent = per_zoom.entry(e.z).or_default();
        ent.0 += 1;
        ent.1 += blob.len();
        ent.2 = ent.2.max(blob.len());
    }
    println!("{:>4} {:>7} {:>12} {:>12}", "z", "tiles", "bytes", "max tile");
    for (z, (n, total, max)) in per_zoom {
        println!("{:>4} {:>7} {:>12} {:>12}", z, n, total, max);
    }
}
