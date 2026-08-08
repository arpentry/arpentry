//! What is the R layer doing at one point: is the ground there a bench band
//! (some corridor's roadbed) or a batter face reaching in from elsewhere?
//! Usage: cargo run --release --example fill_probe -- <segment.parquet> <w,s,e,n> <terrain.pmtiles> <lon,lat>
use arpentry_server::assemble;
use arpentry_server::ground;
use arpentry_server::priors::Stratum;
use arpentry_server::project::Bounds;
use arpentry_server::solve;

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let bb: Vec<f64> = a[1].split(',').map(|s| s.parse().unwrap()).collect();
    let bbox = Bounds { west: bb[0], south: bb[1], east: bb[2], north: bb[3] };
    let terrain = std::path::PathBuf::from(&a[2]);
    let at: Vec<f64> = a[3].split(',').map(|s| s.parse().unwrap()).collect();
    let (lon, lat) = (at[0], at[1]);

    let mut scene = assemble::run(std::path::Path::new(&a[0]), None, &bbox).expect("assemble");
    let solved = solve::run(&mut scene, Some(&terrain), 16, 0).expect("solve");
    let stack = ground::derive(&scene, &solved, Some(&terrain), 0);
    let mut scratch: Vec<u32> = Vec::new();
    let r = stack.layer(Stratum::R).expect("an R layer");
    let ew = r.earthworks();
    println!("R layer: {} edges", ew.len());
    println!("bench target here: {:?}", ew.target_at(lon, lat, &mut scratch));
    println!("covers here: {}", ew.covers(lon, lat, &mut scratch));
    // Every R edge whose declared reach covers the point.
    let mut hits = 0;
    for (i, e) in ew.edges().iter().enumerate() {
        let dx = (e.a.x - lon) * 111_320.0 * lat.to_radians().cos();
        let dy = (e.a.y - lat) * 110_540.0;
        let d = (dx * dx + dy * dy).sqrt();
        if d < 40.0 {
            hits += 1;
            if hits <= 8 {
                println!(
                    "  edge {i}: node {:.1} m away  target {:.2}..{:.2}  half_width {:.2}  batter [{:.1},{:.1}]  carve {}",
                    d, e.target_a, e.target_b, e.half_width_m, e.batter_m[0], e.batter_m[1], e.carve
                );
            }
        }
    }
    println!("  ({hits} R edges within 40 m)");
}
