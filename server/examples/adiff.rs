//! Compare two .arpa archives tile by tile: count identical / differing /
//! missing blobs. Usage: cargo run --release --example adiff -- <a> <b>

use arpentry_server::archive::Archive;

fn main() {
    let mut args = std::env::args().skip(1);
    let (pa, pb) = (args.next().unwrap(), args.next().unwrap());
    let da = std::fs::read(&pa).unwrap();
    let db = std::fs::read(&pb).unwrap();
    let a = Archive::open(&da).unwrap();
    let b = Archive::open(&db).unwrap();
    let (mut same, mut diff, mut missing) = (0usize, 0usize, 0usize);
    let mut examples = Vec::new();
    for e in a.entries() {
        match b.get(e.z, e.x, e.y) {
            None => missing += 1,
            Some(blob_b) => {
                let blob_a = a.get(e.z, e.x, e.y).unwrap();
                if blob_a == blob_b {
                    same += 1;
                } else {
                    diff += 1;
                    if examples.len() < 8 {
                        examples.push((e.z, e.x, e.y, blob_a.len(), blob_b.len()));
                    }
                }
            }
        }
    }
    println!("same {same}, diff {diff}, missing-in-b {missing}");
    for (z, x, y, la, lb) in examples {
        println!("  {z}/{x}/{y}: {la} vs {lb} bytes");
    }
}
