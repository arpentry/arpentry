//! Quantiles of the `water.descends` ascent population — the calibration read
//! for `ASCENT_M`. Usage: cargo run --release --example water_quantiles -- <archive.arpa>

use arpentry_server::verify::checks::{self, Options};
use arpentry_server::verify::scene::ArchiveScan;

fn main() {
    let path = std::env::args().nth(1).expect("archive path");
    let bytes = std::fs::read(&path).unwrap();
    let scan = ArchiveScan::open(&bytes).unwrap();
    let card = checks::run(&scan, &Options::default());
    let m = card.metrics.iter().find(|m| m.id == "water.descends").expect("metric");
    println!("samples {}", m.dist.count());
    for p in [0.5, 0.75, 0.9, 0.95, 0.99, 0.999, 1.0] {
        println!("p{:<5} {:8.3} m", p * 100.0, m.dist.quantile(p).unwrap());
    }
    for t in [0.25, 0.5, 1.0, 2.0, 4.0] {
        let over = m.dist.count() - m.dist.count_below(t);
        println!("> {t:4} m  {:8} samples  {:6.3} %", over, over as f64 / m.dist.count() as f64 * 100.0);
    }
}
