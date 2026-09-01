fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let bytes = std::fs::read(&a[0]).unwrap();
    let ar = arpentry_server::archive::Archive::open(&bytes).unwrap();
    let b = ar.bounds();
    println!("zooms {}..{}  tiles {}  bounds {:.4},{:.4},{:.4},{:.4}",
        ar.min_zoom(), ar.max_zoom(), ar.tile_count(), b.west, b.south, b.east, b.north);
}
