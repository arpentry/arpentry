#!/usr/bin/env bash
# Download Overture Maps land_cover+bathymetry+water data, tile as a globe, serve and view.
#
# Prerequisites:
#   pip install overturemaps
#
# Usage:
#   ./scripts/run-overture-globe.sh [--skip-download] [--screenshot /tmp/globe.png]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$SCRIPT_DIR/.."
BUILD_DIR="$ROOT_DIR/build"
DATA_DIR="$ROOT_DIR/data/overture-globe"
ARCHIVE="/tmp/overture-globe.arpa"

# Tiler and server are the Rust reimplementation (tiler-rs), built with Cargo
# below; only the client comes from the C build.
TILER="$ROOT_DIR/tiler-rs/target/release/arpentry_tiler"
SERVER="$ROOT_DIR/tiler-rs/target/release/arpentry_server"
CLIENT="$BUILD_DIR/client/arpentry_client"

# World bbox
BBOX="-180,-85,180,85"

# Zoom range
MIN_ZOOM=0
MAX_ZOOM=12

# Layers enabled by default (index:name pairs)
LAYER_LAND=true
LAYER_LAND_COVER=true
LAYER_BATHYMETRY=true
LAYER_WATER=true
LAYER_TRANSPORTATION=true
LAYER_BUILDING=true

# Parse arguments
SKIP_DOWNLOAD=false
SCREENSHOT=""
while [ $# -gt 0 ]; do
    case "$1" in
        --skip-download) SKIP_DOWNLOAD=true; shift ;;
        --screenshot)    SCREENSHOT="$2"; shift 2 ;;
        --no-land)           LAYER_LAND=false; shift ;;
        --no-land-cover)     LAYER_LAND_COVER=false; shift ;;
        --no-bathymetry)     LAYER_BATHYMETRY=false; shift ;;
        --no-water)          LAYER_WATER=false; shift ;;
        --no-transportation) LAYER_TRANSPORTATION=false; shift ;;
        --no-building)       LAYER_BUILDING=false; shift ;;
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
( cd "$ROOT_DIR/tiler-rs" && cargo build --release )

# ── Download Overture base-theme data ────────────────────────────────────────

mkdir -p "$DATA_DIR"

download_type() {
    local type="$1"
    local output="$DATA_DIR/${type}.parquet"
    if [ -f "$output" ]; then
        echo "Skipping $type (already downloaded)"
        return
    fi
    echo "Downloading $type (global)..."
    overturemaps download \
        --bbox="$BBOX" \
        -f geoparquet \
        --type="$type" \
        -o "$output"
    echo "  -> $(du -h "$output" | cut -f1)"
}

$LAYER_LAND           && download_type land
$LAYER_LAND_COVER     && download_type land_cover
$LAYER_BATHYMETRY     && download_type bathymetry
$LAYER_WATER          && download_type water
$LAYER_TRANSPORTATION && download_type segment
$LAYER_BUILDING       && download_type building

# ── Tile ──────────────────────────────────────────────────────────────────────

if [ ! -f "$ARCHIVE" ]; then
    echo ""
    echo "Tiling to $ARCHIVE..."
    echo "  bbox=$BBOX  zoom=$MIN_ZOOM-$MAX_ZOOM"

    SEGMENT_FILE="$DATA_DIR/segment.parquet"

    TILER_INPUTS=()
    $LAYER_LAND_COVER     && TILER_INPUTS+=(--input "1:$DATA_DIR/land_cover.parquet")
    $LAYER_BATHYMETRY     && TILER_INPUTS+=(--input "2:$DATA_DIR/bathymetry.parquet")
    $LAYER_WATER          && TILER_INPUTS+=(--input "3:$DATA_DIR/water.parquet")
    $LAYER_LAND           && TILER_INPUTS+=(--input "4:$DATA_DIR/land.parquet")
    $LAYER_TRANSPORTATION && TILER_INPUTS+=(--input "5:$SEGMENT_FILE")
    $LAYER_BUILDING       && TILER_INPUTS+=(--input "6:$DATA_DIR/building.parquet")

    "$TILER" \
        --output "$ARCHIVE" \
        --bbox "$BBOX" \
        --min-zoom "$MIN_ZOOM" \
        --max-zoom "$MAX_ZOOM" \
        --mem $((256 * 1024 * 1024)) \
        "${TILER_INPUTS[@]}"

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
"$SERVER" "$ARCHIVE" "$ROOT_DIR/style-overture-globe.json" &
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
