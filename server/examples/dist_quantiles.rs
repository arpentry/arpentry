//! Quantiles of one metric's population — the calibration read for a
//! threshold (the census discipline: histogram before gating).
//! Usage: cargo run --release --example dist_quantiles -- <archive.arpa> <metric-id>

use arpentry_server::verify::checks::{self, Options};
use arpentry_server::verify::scene::ArchiveScan;

fn main() {
    let path = std::env::args().nth(1).expect("archive path");
    let id = std::env::args().nth(2).expect("metric id");
    let bytes = std::fs::read(&path).unwrap();
    let scan = ArchiveScan::open(&bytes).unwrap();
    let card = checks::run(&scan, &Options::default());
    let m = card.metrics.iter().find(|m| m.id == id).expect("metric");
    println!("samples {}", m.dist.count());
    for p in [0.5, 0.75, 0.9, 0.95, 0.99, 0.999, 1.0] {
        println!("p{:<5} {:10.4}", p * 100.0, m.dist.quantile(p).unwrap());
    }
    for t in [0.01, 0.02, 0.05, 0.1, 0.2, 0.25, 0.5, 1.0, 2.0, 4.0] {
        let over = m.dist.count() - m.dist.count_below(t);
        println!(
            "> {t:5} {:8} samples  {:6.3} %",
            over,
            over as f64 / m.dist.count() as f64 * 100.0
        );
    }
}
