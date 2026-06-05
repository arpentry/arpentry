#!/usr/bin/env bash
# Tile Natural Earth 10m data, serve it, and view the globe.
#
# Uses the Rust tiler and server (tiler-rs/, built with Cargo); the client is
# the C build (CMake).
#
# Run scripts/download-naturalearth.py first to fetch the data:
#   python3 scripts/download-naturalearth.py
#
# Usage:
#   ./scripts/run-naturalearth.sh [--screenshot /tmp/globe.png]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$SCRIPT_DIR/.."
BUILD_DIR="$ROOT_DIR/build"
DATA_DIR="$ROOT_DIR/data/naturalearth"

# Tiler and server are the Rust reimplementation (tiler-rs), built with Cargo
# below; only the client comes from the C build.
TILER="$ROOT_DIR/tiler-rs/target/release/arpentry_tiler"
SERVER="$ROOT_DIR/tiler-rs/target/release/arpentry_server"
CLIENT="$BUILD_DIR/client/arpentry_client"

# World bbox
BBOX="-180,-85,180,85"

# Zoom range.
# NOTE: the Rust tiler is currently single-threaded and clipping the global
# Natural Earth land polygon at every zoom is the hot spot, so high MAX_ZOOM
# over the whole world is slow. Lower it for quick tests; raise for detail.
MIN_ZOOM=0
MAX_ZOOM=8

# Archive path encodes the zoom so bumping MAX_ZOOM doesn't reuse a stale
# lower-zoom archive via the skip-if-exists check below.
ARCHIVE="/tmp/naturalearth-z${MAX_ZOOM}.arpa"

# Parse arguments
SCREENSHOT=""
while [ $# -gt 0 ]; do
    case "$1" in
        --screenshot)    SCREENSHOT="$2"; shift 2 ;;
        *) shift ;;
    esac
done

# ── Build ─────────────────────────────────────────────────────────────────────

if [ ! -f "$BUILD_DIR/CMakeCache.txt" ]; then
    echo "Configuring build..."
    cmake -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release
fi

echo "Building client (C)..."
# Only the client comes from the C build; the tiler and server are Rust.
cmake --build "$BUILD_DIR" --target arpentry_client

echo "Building Rust tiler + server..."
( cd "$ROOT_DIR/tiler-rs" && cargo build --release )

# ── Check data ────────────────────────────────────────────────────────────────

if [ ! -d "$DATA_DIR" ]; then
    echo "Data not found. Downloading Natural Earth 10m..."
    python3 "$SCRIPT_DIR/download-naturalearth.py"
fi

# ── Tile ──────────────────────────────────────────────────────────────────────

if [ ! -f "$ARCHIVE" ]; then
    echo ""
    echo "Tiling to $ARCHIVE..."
    echo "  bbox=$BBOX  zoom=$MIN_ZOOM-$MAX_ZOOM"

    # Layer assignment matches tiler/src/layers.h:
    #   1 land_cover   | 3 water | 4 land
    #   5 transportation | 6 land_use
    # Line features (coastline, river, boundaries, roads) go to
    # transportation since water/land are texture-only in the style.
    # The tiler prepends a flat terrain mesh (layer 0) to every tile so the
    # client renders it; per-class min-zoom keeps low-zoom tiles light.
    "$TILER" \
        --output "$ARCHIVE" \
        --bbox "$BBOX" \
        --min-zoom "$MIN_ZOOM" \
        --max-zoom "$MAX_ZOOM" \
        --mem $((64 * 1024 * 1024)) \
        --input "4:$DATA_DIR/land.parquet" \
        --input "3:$DATA_DIR/lake.parquet" \
        --input "1:$DATA_DIR/glacier.parquet" \
        --input "1:$DATA_DIR/ice_shelf.parquet" \
        --input "1:$DATA_DIR/reef.parquet" \
        --input "6:$DATA_DIR/urban.parquet" \
        --input "5:$DATA_DIR/coastline.parquet" \
        --input "5:$DATA_DIR/river.parquet" \
        --input "5:$DATA_DIR/boundary.parquet" \
        --input "5:$DATA_DIR/admin1_boundary.parquet" \
        --input "5:$DATA_DIR/road.parquet" \
        --input "5:$DATA_DIR/geographic_lines.parquet"

    echo "Archive: $(du -h "$ARCHIVE" | cut -f1)"
else
    echo ""
    echo "Using existing archive: $ARCHIVE ($(du -h "$ARCHIVE" | cut -f1))"
fi

# ── Kill any previous instances ──────────────────────────────────────────────

pkill -x arpentry_server 2>/dev/null || true
pkill -x arpentry_client 2>/dev/null || true
sleep 0.5

# ── Start server ─────────────────────────────────────────────────────────────

echo ""
echo "Starting server with $ARCHIVE..."
"$SERVER" "$ARCHIVE" "$ROOT_DIR/style-naturalearth.json" &
SERVER_PID=$!
echo "Server started (pid $SERVER_PID)"
sleep 1

# ── Start client ─────────────────────────────────────────────────────────────

cleanup() {
    kill "$SERVER_PID" 2>/dev/null || true
}
trap cleanup EXIT

# Globe view centered on Europe
LON=10.0
LAT=45.0
ALT=5000000

if [ -n "$SCREENSHOT" ]; then
    echo "Capturing screenshot to $SCREENSHOT..."
    "$CLIENT" --lon "$LON" --lat "$LAT" --alt "$ALT" \
              --bearing 0 --tilt 0 --screenshot "$SCREENSHOT"
    echo "Done: $SCREENSHOT"
else
    echo "Opening viewer centered on globe..."
    "$CLIENT" --lon "$LON" --lat "$LAT" --alt "$ALT"
fi
