//! Diagnostic probe: normal-field continuity of one tile's terrain mesh.
//!
//! For every triangle edge, the angle between its endpoints' stored normals;
//! reported separately for edges touching the tile-border lattice row and for
//! interior edges. A shading crease along the border shows up as border-edge
//! angles far above the interior distribution while the heights seam at zero.
//!
//! ```sh
//! cargo run --release --example norm_probe -- archive.arpa 16 34029 49668
//! ```

use arpentry_server::verify::scene::ArchiveScan;

const EXTENT: i64 = 32768;

fn decode(p: (i8, i8)) -> (f64, f64, f64) {
    let nx = p.0 as f64 / 127.0;
    let ny = p.1 as f64 / 127.0;
    let nz = (1.0 - (nx * nx + ny * ny)).max(0.0).sqrt();
    (nx, ny, nz)
}

fn angle(a: (i8, i8), b: (i8, i8)) -> f64 {
    let (ax, ay, az) = decode(a);
    let (bx, by, bz) = decode(b);
    (ax * bx + ay * by + az * bz).clamp(-1.0, 1.0).acos().to_degrees()
}

fn stats(name: &str, mut v: Vec<f64>) {
    if v.is_empty() {
        println!("{name}: no edges");
        return;
    }
    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let n = v.len();
    let mean = v.iter().sum::<f64>() / n as f64;
    let p95 = v[((n as f64 * 0.95) as usize).min(n - 1)];
    let over15 = v.iter().filter(|&&a| a > 15.0).count();
    println!(
        "{name}: n={n} mean={mean:.2}° p95={p95:.2}° max={:.2}° >15°: {over15}",
        v[n - 1]
    );
}

fn main() {
    let mut args = std::env::args().skip(1);
    let path = args.next().expect("archive path");
    let z: u8 = args.next().expect("z").parse().unwrap();
    let x: u32 = args.next().expect("x").parse().unwrap();
    let y: u32 = args.next().expect("y").parse().unwrap();
    let data = std::fs::read(&path).expect("read archive");
    let scan = ArchiveScan::open(&data).expect("open archive");
    let id = scan
        .tiles_at(z)
        .into_iter()
        .find(|&(_, tx, ty, _)| tx == x && ty == y)
        .map(|(_, _, _, id)| id)
        .expect("tile listed");
    let tile = scan.decode(z, x, y, id).expect("tile decodes");
    let mesh = tile.terrain.as_ref().expect("terrain mesh");
    let nverts = mesh.vertex_count();
    let mut on_border = vec![false; nverts];
    let mut have_normal = 0usize;
    for i in 0..nverts {
        let (px, py, _) = mesh.vertex(i);
        let qx = (px * EXTENT as f64).round() as i64;
        let qy = (py * EXTENT as f64).round() as i64;
        on_border[i] = qx == 0 || qx == EXTENT || qy == 0 || qy == EXTENT;
        if mesh.normal(i).is_some() {
            have_normal += 1;
        }
    }
    println!(
        "{path} {z}/{x}/{y}: {nverts} verts ({have_normal} with normals), {} tris, {} border verts",
        mesh.triangle_count(),
        on_border.iter().filter(|&&b| b).count()
    );
    let mut border_edges = Vec::new();
    let mut interior_edges = Vec::new();
    let mut seen = std::collections::HashSet::new();
    for t in 0..mesh.triangle_count() {
        let [a, b, c] = mesh.triangle_indices(t);
        for &(i, j) in &[(a, b), (b, c), (c, a)] {
            let key = (i.min(j), i.max(j));
            if !seen.insert(key) {
                continue;
            }
            let (Some(ni), Some(nj)) = (mesh.normal(i as usize), mesh.normal(j as usize)) else {
                continue;
            };
            let ang = angle(ni, nj);
            if on_border[i as usize] != on_border[j as usize] {
                border_edges.push(ang);
            } else if !on_border[i as usize] {
                interior_edges.push(ang);
            }
        }
    }
    stats("border-to-interior edges", border_edges.clone());
    stats("interior-to-interior edges", interior_edges);
    // The worst offenders, decoded: is the border normal near-vertical on a
    // steep slope (a flat analytic field), or tilted the other way?
    let mut worst: Vec<(f64, u32, u32)> = Vec::new();
    let mut seen2 = std::collections::HashSet::new();
    for t in 0..mesh.triangle_count() {
        let [a, b, c] = mesh.triangle_indices(t);
        for &(i, j) in &[(a, b), (b, c), (c, a)] {
            let key = (i.min(j), i.max(j));
            if !seen2.insert(key) { continue; }
            if on_border[i as usize] == on_border[j as usize] { continue; }
            let (Some(ni), Some(nj)) = (mesh.normal(i as usize), mesh.normal(j as usize)) else { continue };
            let (bi, ii) = if on_border[i as usize] { (i, j) } else { (j, i) };
            let _ = (ni, nj);
            worst.push((angle(mesh.normal(bi as usize).unwrap(), mesh.normal(ii as usize).unwrap()), bi, ii));
        }
    }
    worst.sort_by(|x, y| y.0.partial_cmp(&x.0).unwrap());
    for &(ang, bi, ii) in worst.iter().take(6) {
        let nb = decode(mesh.normal(bi as usize).unwrap());
        let nn = decode(mesh.normal(ii as usize).unwrap());
        let (bx, by, bz) = mesh.vertex(bi as usize);
        let (ix, iy, iz) = mesh.vertex(ii as usize);
        println!(
            "  {ang:6.1}°  border v{bi} ({bx:.4},{by:.4},z={bz:.1}) n=({:.2},{:.2},{:.2})  interior v{ii} ({ix:.4},{iy:.4},z={iz:.1}) n=({:.2},{:.2},{:.2})",
            nb.0, nb.1, nb.2, nn.0, nn.1, nn.2
        );
    }
}
