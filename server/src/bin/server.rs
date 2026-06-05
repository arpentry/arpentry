//! Arpentry tile server (Rust port of the C `server/`).
//!
//! Serves the same HTTP API the C server exposes to the viewer:
//!
//! ```text
//!   GET /{z}/{x}/{y}.arpt   tile      (application/x-arpt, br)
//!   GET /index.arpi         tileset   (application/x-arpi, br)
//!   GET /style.arps         style     (application/x-arps, br)
//!   GET /models.arpm        models    (application/x-arpm, br)
//! ```
//!
//! Two tile sources, chosen by the first argument:
//!   * an `.arpa` archive — tiles are read by Hilbert-id binary search; a missing
//!     tile falls back to a flat-terrain tile so the client always renders;
//!   * any other path — tiles are synthesised on the fly from procedural noise
//!     (terrain, biomes, town, trees, POIs), matching `gen/world.c`.
//!
//! Like the C server it is a blocking, thread-per-connection HTTP/1.1 server
//! (`Connection: close`); concurrency comes from a fixed worker pool.

use std::process::exit;
use std::sync::Arc;

use arpentry_server::archive::Archive;
use arpentry_server::geom::GeometryType;
use arpentry_server::project::Bounds;
use arpentry_server::tileset::{self, LayerInfo, TilesetInfo};
use arpentry_server::{gen, models, style, terrain, tile_build};

use tiny_http::{Header, Method, Request, Response, Server};

/// Grid resolution for the flat fallback tile (matches the tiler pipeline).
const FALLBACK_TERRAIN_GRID: u32 = 16;

/// Tile source backing the server.
enum Source {
    /// Serve from a memory-resident `.arpa` archive (leaked to `'static`).
    Archive(Archive<'static>),
    /// Synthesise tiles procedurally.
    Procedural,
}

/// Immutable server state shared across worker threads.
struct State {
    source: Source,
    style_path: std::path::PathBuf,
    index_arpi: Vec<u8>,
    models_arpm: Vec<u8>,
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.len() < 2 || args.len() > 4 {
        eprintln!("Usage: arpentry_server <tile_dir|archive.arpa> <style_file> [port] [threads]");
        exit(1);
    }
    let tile_dir = &args[0];
    let style_file = std::path::PathBuf::from(&args[1]);
    let port: u16 = args.get(2).map(|p| p.parse().unwrap_or(8090)).unwrap_or(8090);
    let nthreads: usize = args.get(3).and_then(|t| t.parse().ok()).unwrap_or(8).max(1);

    // Open an archive if the path looks like one; otherwise generate procedurally.
    let source = if tile_dir.ends_with(".arpa") {
        match std::fs::read(tile_dir) {
            Ok(bytes) => {
                // Leak the bytes so the zero-copy Archive view can be 'static.
                let leaked: &'static [u8] = Box::leak(bytes.into_boxed_slice());
                match Archive::open(leaked) {
                    Ok(a) => {
                        println!("Serving tiles from archive {tile_dir} ({} tiles)", a.tile_count());
                        Source::Archive(a)
                    }
                    Err(e) => {
                        eprintln!("Failed to open archive {tile_dir}: {e}");
                        exit(1);
                    }
                }
            }
            Err(e) => {
                eprintln!("Failed to read archive {tile_dir}: {e}");
                exit(1);
            }
        }
    } else {
        println!("Serving generated tiles from {tile_dir}");
        Source::Procedural
    };

    // Clamp advertised max level to the archive's range (matches resp_tileset.c).
    let archive_max_zoom = match &source {
        Source::Archive(a) => Some(a.max_zoom()),
        Source::Procedural => None,
    };
    let index_arpi = tileset::build(&generated_tileset(archive_max_zoom));
    let models_arpm = models::build();

    let state = Arc::new(State { source, style_path: style_file, index_arpi, models_arpm });

    let addr = format!("0.0.0.0:{port}");
    let server = match Server::http(&addr) {
        Ok(s) => Arc::new(s),
        Err(e) => {
            eprintln!("Failed to bind {addr}: {e}");
            exit(1);
        }
    };
    println!("Listening on {addr} ({nthreads} thread{})", if nthreads > 1 { "s" } else { "" });

    let mut workers = Vec::with_capacity(nthreads);
    for _ in 0..nthreads {
        let server = Arc::clone(&server);
        let state = Arc::clone(&state);
        workers.push(std::thread::spawn(move || {
            for request in server.incoming_requests() {
                handle(request, &state);
            }
        }));
    }
    for w in workers {
        let _ = w.join();
    }
}

/// The outcome of routing a request: either a body to serve or a status code.
enum Routed {
    Blob { content_type: &'static str, body: Vec<u8> },
    Status(u16),
}

/// Dispatches one request and sends its response.
fn handle(request: Request, state: &State) {
    // Strip any query string; we only route on the path.
    let url = request.url().to_string();
    let path = url.split('?').next().unwrap_or("");

    match route(request.method(), path, state) {
        Routed::Blob { content_type, body } => respond_blob(request, content_type, &body),
        Routed::Status(code) => respond_status(request, code),
    }
}

/// Pure request router: maps a method + path to a response. Kept free of the
/// socket layer so the full dispatch (archive lookup, fallback, procedural
/// generation, errors) is unit-testable without binding a port.
fn route(method: &Method, path: &str, state: &State) -> Routed {
    if *method != Method::Get {
        return Routed::Status(405);
    }

    if let Some((z, x, y)) = parse_tile_path(path) {
        return Routed::Blob { content_type: "application/x-arpt", body: tile_body(state, z, x, y) };
    }
    match path {
        "/index.arpi" => Routed::Blob { content_type: "application/x-arpi", body: state.index_arpi.clone() },
        "/style.arps" => match style::build_from_file(&state.style_path) {
            Ok(blob) => Routed::Blob { content_type: "application/x-arps", body: blob },
            Err(e) => {
                eprintln!("{e}");
                Routed::Status(500)
            }
        },
        "/models.arpm" => Routed::Blob { content_type: "application/x-arpm", body: state.models_arpm.clone() },
        _ => Routed::Status(404),
    }
}

/// Produces the `.arpt` body for a tile: archive blob, flat fallback, or
/// procedurally generated tile.
fn tile_body(state: &State, z: u8, x: u32, y: u32) -> Vec<u8> {
    match &state.source {
        Source::Archive(archive) => match archive.get(z, x, y) {
            Some(blob) => blob.to_vec(),
            // A miss means the archive has no tile here; the client still needs a
            // terrain layer to render, so synthesise a flat one. A burst of these
            // for land tiles signals an archive/addressing mismatch.
            None => {
                eprintln!("[tile] {z}/{x}/{y} not in archive -> flat fallback");
                build_fallback_tile(z, x, y)
            }
        },
        Source::Procedural => gen::world::generate_terrain(z, x, y),
    }
}

/// Builds a minimal flat-terrain tile for archive misses (Brotli-compressed),
/// so the client always has a `terrain` layer to render.
fn build_fallback_tile(z: u8, x: u32, y: u32) -> Vec<u8> {
    let bounds = Bounds::of_tile(z, x, y);
    let mesh = terrain::flat_mesh(FALLBACK_TERRAIN_GRID);
    tile_build::build_tile(&bounds, Some(&mesh), &[])
}

/// The "Generated Terrain" tileset metadata (port of `resp_build_tileset`).
fn generated_tileset(archive_max_zoom: Option<u8>) -> TilesetInfo {
    let max_level = archive_max_zoom.map_or(19, |m| m.min(19));
    let layer = |name: &str, gt: GeometryType, min: u8, max: u8| LayerInfo {
        name: name.to_string(),
        geometry_types: vec![gt],
        min_level: min,
        max_level: max,
    };
    TilesetInfo {
        name: Some("Generated Terrain".to_string()),
        bounds: Bounds { west: -180.0, south: -90.0, east: 180.0, north: 90.0 },
        elevation_range: (-500.0, 4800.0),
        min_level: 0,
        max_level: max_level,
        root_error: 400_000.0,
        // Decode-priority order (FORMAT.md §9).
        layers: vec![
            layer("terrain", GeometryType::Mesh, 0, 19),
            layer("surface", GeometryType::Polygon, 0, 19),
            layer("transportation", GeometryType::Line, 8, 19),
            layer("building", GeometryType::Polygon, 13, 19),
            layer("tree", GeometryType::Point, 13, 19),
        ],
    }
}

/// Parses `/{z}/{x}/{y}.arpt` with the C server's validation (`tile_path.c`).
/// Returns `None` for any non-tile or out-of-range path.
fn parse_tile_path(path: &str) -> Option<(u8, u32, u32)> {
    let rest = path.strip_prefix('/')?;
    let rest = rest.strip_suffix(".arpt")?;
    let mut parts = rest.split('/');
    let z: u32 = parts.next()?.parse().ok()?;
    let x: u32 = parts.next()?.parse().ok()?;
    let y: u32 = parts.next()?.parse().ok()?;
    if parts.next().is_some() {
        return None; // too many segments
    }
    // Level in [0, 21] (address-space limit); x, y in [0, 2^z - 1].
    if z > 21 {
        return None;
    }
    let max = (1u32 << z) - 1;
    if x > max || y > max {
        return None;
    }
    Some((z as u8, x, y))
}

// ── Response helpers ─────────────────────────────────────────────────────

fn header(name: &str, value: &str) -> Header {
    Header::from_bytes(name.as_bytes(), value.as_bytes()).expect("valid header")
}

/// Responds with a Brotli-compressed blob and permissive CORS, like the C server.
///
/// The chunked threshold is raised past any tile size so every response carries a
/// `Content-Length` and is never sent with `Transfer-Encoding: chunked`. tiny_http
/// otherwise chunks bodies over 32 KiB, which the client's minimal HTTP reader
/// (it splits on the header terminator and feeds the rest straight to Brotli)
/// cannot decode — so large tiles would fail to load. The C server always sent
/// `Content-Length`, so this keeps the wire behaviour identical.
fn respond_blob(request: Request, content_type: &str, body: &[u8]) {
    let response = Response::from_data(body.to_vec())
        .with_chunked_threshold(usize::MAX)
        .with_header(header("Content-Type", content_type))
        .with_header(header("Content-Encoding", "br"))
        .with_header(header("Access-Control-Allow-Origin", "*"));
    let _ = request.respond(response);
}

fn respond_status(request: Request, status: u16) {
    let response = Response::from_string(status_text(status))
        .with_status_code(status)
        .with_header(header("Content-Type", "text/plain"))
        .with_header(header("Access-Control-Allow-Origin", "*"));
    let _ = request.respond(response);
}

fn status_text(status: u16) -> &'static str {
    match status {
        404 => "Not Found",
        405 => "Method Not Allowed",
        500 => "Internal Server Error",
        _ => "Error",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn procedural_state() -> State {
        State {
            source: Source::Procedural,
            style_path: std::path::PathBuf::from("../style.json"),
            index_arpi: tileset::build(&generated_tileset(None)),
            models_arpm: models::build(),
        }
    }

    fn decompress(data: &[u8]) -> Vec<u8> {
        let mut out = Vec::new();
        let mut input = data;
        brotli::BrotliDecompress(&mut input, &mut out).unwrap();
        out
    }

    fn blob(routed: Routed) -> (&'static str, Vec<u8>) {
        match routed {
            Routed::Blob { content_type, body } => (content_type, body),
            Routed::Status(c) => panic!("expected blob, got status {c}"),
        }
    }

    #[test]
    fn routes_each_endpoint_to_a_decodable_blob() {
        let s = procedural_state();
        // The four resource endpoints decode to the right FlatBuffer identifier.
        for (path, ct, ident) in [
            ("/index.arpi", "application/x-arpi", b"arpi"),
            ("/style.arps", "application/x-arps", b"arps"),
            ("/models.arpm", "application/x-arpm", b"arpm"),
            ("/4/0/0.arpt", "application/x-arpt", b"arpt"),
        ] {
            let (content_type, body) = blob(route(&Method::Get, path, &s));
            assert_eq!(content_type, ct, "content-type for {path}");
            let raw = decompress(&body);
            assert_eq!(&raw[4..8], ident, "identifier for {path}");
        }
    }

    #[test]
    fn non_get_and_unknown_paths_get_status_codes() {
        let s = procedural_state();
        assert!(matches!(route(&Method::Post, "/index.arpi", &s), Routed::Status(405)));
        assert!(matches!(route(&Method::Get, "/favicon.ico", &s), Routed::Status(404)));
        assert!(matches!(route(&Method::Get, "/22/0/0.arpt", &s), Routed::Status(404)));
    }

    #[test]
    fn archive_misses_fall_back_to_a_flat_terrain_tile() {
        // An empty in-memory archive: every tile request misses and falls back.
        use arpentry_server::archive::{ArchiveMeta, ArchiveWriter};
        let meta = ArchiveMeta {
            min_zoom: 0,
            max_zoom: 6,
            bounds: Bounds { west: -180.0, south: -90.0, east: 180.0, north: 90.0 },
            root_error: 400_000.0,
        };
        let bytes = ArchiveWriter::new(std::io::Cursor::new(Vec::new()), meta)
            .unwrap()
            .finish(b"")
            .unwrap()
            .into_inner();
        let leaked: &'static [u8] = Box::leak(bytes.into_boxed_slice());
        let archive = Archive::open(leaked).unwrap();
        let s = State {
            source: Source::Archive(archive),
            style_path: std::path::PathBuf::from("../style.json"),
            index_arpi: tileset::build(&generated_tileset(Some(6))),
            models_arpm: models::build(),
        };

        let (_, body) = blob(route(&Method::Get, "/3/1/2.arpt", &s));
        let raw = decompress(&body);
        // Fallback still carries a renderable terrain mesh as layer 0.
        assert_eq!(&raw[4..8], b"arpt");
        let tile = arpentry_server::fb::tile::arpentry::tiles::root_as_tile(&raw).unwrap();
        assert_eq!(tile.layers().unwrap().get(0).name(), "terrain");
    }

    /// Serves a real tile from the checked-in archive. Ignored by default (the
    /// 120 MB `naturalearth.arpa` is not always present); run with
    /// `cargo test -- --ignored`.
    #[test]
    #[ignore]
    fn serves_a_real_archive_tile() {
        let bytes = std::fs::read("naturalearth.arpa").expect("naturalearth.arpa present");
        let leaked: &'static [u8] = Box::leak(bytes.into_boxed_slice());
        let archive = Archive::open(leaked).unwrap();
        let max_zoom = archive.max_zoom();
        let first = archive.entries().next().expect("archive has tiles");
        let s = State {
            source: Source::Archive(archive),
            style_path: std::path::PathBuf::from("../style.json"),
            index_arpi: tileset::build(&generated_tileset(Some(max_zoom))),
            models_arpm: models::build(),
        };
        let path = format!("/{}/{}/{}.arpt", first.z, first.x, first.y);
        let (_, body) = blob(route(&Method::Get, &path, &s));
        let raw = decompress(&body);
        assert_eq!(&raw[4..8], b"arpt");
    }

    #[test]
    fn parses_valid_tile_paths() {
        assert_eq!(parse_tile_path("/0/0/0.arpt"), Some((0, 0, 0)));
        assert_eq!(parse_tile_path("/5/31/17.arpt"), Some((5, 31, 17)));
        assert_eq!(parse_tile_path("/14/9000/4096.arpt"), Some((14, 9000, 4096)));
    }

    #[test]
    fn rejects_bad_tile_paths() {
        assert_eq!(parse_tile_path("/index.arpi"), None);
        assert_eq!(parse_tile_path("/5/31/17.png"), None); // wrong extension
        assert_eq!(parse_tile_path("/22/0/0.arpt"), None); // level out of range
        assert_eq!(parse_tile_path("/1/2/0.arpt"), None); // x out of range at z1
        assert_eq!(parse_tile_path("/5/1/2/3.arpt"), None); // too many segments
        assert_eq!(parse_tile_path("/a/b/c.arpt"), None); // non-numeric
    }
}
