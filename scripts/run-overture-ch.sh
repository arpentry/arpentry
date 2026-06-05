#!/usr/bin/env bash
# Download Overture Maps data for Switzerland, tile it, serve it, and view it.
#
# Prerequisites:
#   pip install overturemaps
#
# Usage:
#   ./scripts/run-overture-ch.sh [--skip-download] [--screenshot /tmp/ch.png]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$SCRIPT_DIR/.."
BUILD_DIR="$ROOT_DIR/build"
DATA_DIR="$ROOT_DIR/data/overture-ch"
ARCHIVE="$DATA_DIR/switzerland.arpa"

# Tiler and server are the Rust reimplementation (server), built with Cargo
# below; only the client comes from the C build.
TILER="$ROOT_DIR/server/target/release/arpentry_tiler"
SERVER="$ROOT_DIR/server/target/release/arpentry_server"
CLIENT="$BUILD_DIR/client/arpentry_client"

# Switzerland bbox (approx)
BBOX="5.9,45.8,10.5,47.9"

# Zoom range
MIN_ZOOM=0
MAX_ZOOM=14

# Parse arguments
SKIP_DOWNLOAD=false
SCREENSHOT=""
while [ $# -gt 0 ]; do
    case "$1" in
        --skip-download) SKIP_DOWNLOAD=true; shift ;;
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
cmake --build "$BUILD_DIR" --target arpentry_client

echo "Building Rust tiler + server..."
( cd "$ROOT_DIR/server" && cargo build --release )

# ── Download Overture data ────────────────────────────────────────────────────

mkdir -p "$DATA_DIR"

download_type() {
    local type="$1"
    local output="$DATA_DIR/${type}.parquet"
    if [ "$SKIP_DOWNLOAD" = true ] && [ -f "$output" ]; then
        echo "Skipping $type (already downloaded)"
        return
    fi
    echo "Downloading $type for Switzerland..."
    overturemaps download \
        --bbox="$BBOX" \
        -f geoparquet \
        --type="$type" \
        -o "$output"
    echo "  -> $(du -h "$output" | cut -f1)"
}

download_type water
download_type land_cover
download_type land_use
download_type segment
download_type building
download_type place

# ── Tile ──────────────────────────────────────────────────────────────────────

echo ""
echo "Tiling to $ARCHIVE..."
echo "  bbox=$BBOX  zoom=$MIN_ZOOM-$MAX_ZOOM"

"$TILER" \
    --output "$ARCHIVE" \
    --bbox "$BBOX" \
    --min-zoom "$MIN_ZOOM" \
    --max-zoom "$MAX_ZOOM" \
    --mem $((256 * 1024 * 1024)) \
    --input "1:$DATA_DIR/water.parquet" \
    --input "1:$DATA_DIR/land_cover.parquet" \
    --input "1:$DATA_DIR/land_use.parquet" \
    --input "2:$DATA_DIR/segment.parquet" \
    --input "3:$DATA_DIR/building.parquet" \
    --input "5:$DATA_DIR/place.parquet"

echo "Archive: $(du -h "$ARCHIVE" | cut -f1)"

# ── Kill any previous instances ──────────────────────────────────────────────

pkill -x arpentry_server 2>/dev/null || true
pkill -x arpentry_client 2>/dev/null || true
sleep 0.5

# ── Start server ─────────────────────────────────────────────────────────────

echo ""
echo "Starting server with $ARCHIVE..."
"$SERVER" "$ARCHIVE" "$ROOT_DIR/style.json" &
SERVER_PID=$!
echo "Server started (pid $SERVER_PID)"
sleep 1

# ── Start client ─────────────────────────────────────────────────────────────

cleanup() {
    kill "$SERVER_PID" 2>/dev/null || true
}
trap cleanup EXIT

# Center on Switzerland: Bern area
LON=7.45
LAT=46.95
ALT=200000

if [ -n "$SCREENSHOT" ]; then
    echo "Capturing screenshot to $SCREENSHOT..."
    "$CLIENT" --lon "$LON" --lat "$LAT" --alt "$ALT" \
              --bearing 0 --tilt 0 --screenshot "$SCREENSHOT"
    echo "Done: $SCREENSHOT"
else
    echo "Opening viewer centered on Switzerland..."
    "$CLIENT" --lon "$LON" --lat "$LAT" --alt "$ALT"
fi
