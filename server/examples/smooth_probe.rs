//! How far does the smoothed sweep line slide *along* the alignment from the
//! raw profile node whose height it is paired with?
//!
//! `ground::corridor_earthworks` builds its bench edges along `Profile::smooth`
//! but takes `target_a`/`target_b` from `road[k]`, indexed by the unsmoothed
//! node. Any displacement between the two is a height error equal to the
//! displacement times the grade — invisible at 6 % and metres at 57 %.
//!
//! Usage: cargo run --release --example smooth_probe -- <segment.parquet> <w,s,e,n> <terrain.pmtiles> <corridor_id>
use arpentry_server::assemble;
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
    let p = solved.profile(want).expect("profiled");
    let (raw, sm, road, arc) = (p.nodes(), p.smooth(), p.road_m(), p.arc());
    let cos = raw[0].y.to_radians().cos();
    println!("  k     arc    |smooth-raw|   road    grade   height error implied");
    for k in 0..raw.len() {
        let dx = (sm[k].x - raw[k].x) * 111_320.0 * cos;
        let dy = (sm[k].y - raw[k].y) * 110_540.0;
        let d = (dx * dx + dy * dy).sqrt();
        let g = if k + 1 < road.len() && arc[k + 1] > arc[k] {
            (road[k + 1] - road[k]) / (arc[k + 1] - arc[k])
        } else {
            0.0
        };
        println!("{k:>3} {:>8.1} {d:>12.2} m {:>8.2} {:>7.1}% {:>12.2} m", arc[k], road[k], g * 100.0, d * g.abs());
    }
}
