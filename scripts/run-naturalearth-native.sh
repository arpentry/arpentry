#!/usr/bin/env bash
# Tile Natural Earth 10m data, serve it, and view the globe.
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
ARCHIVE="/tmp/naturalearth.arpa"
DEM="$DATA_DIR/etopo1.tif"

SERVER="$BUILD_DIR/server/arpentry_server"
CLIENT="$BUILD_DIR/client/arpentry_client"
TILER="$BUILD_DIR/tiler/arpentry_tiler"

# World bbox
BBOX="-180,-85,180,85"

# Zoom range
MIN_ZOOM=0
MAX_ZOOM=8

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

echo "Building..."
cmake --build "$BUILD_DIR"

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

    DEM_FLAG=""
    if [ -f "$DEM" ]; then
        DEM_FLAG="--dem $DEM"
    else
        echo "  (no DEM found at $DEM, using flat terrain)"
    fi

    "$TILER" \
        --output "$ARCHIVE" \
        --bbox "$BBOX" \
        --min-zoom "$MIN_ZOOM" \
        --max-zoom "$MAX_ZOOM" \
        --mem $((64 * 1024 * 1024)) \
        $DEM_FLAG \
        --input "1:$DATA_DIR/land.parquet" \
        --input "1:$DATA_DIR/coastline.parquet" \
        --input "1:$DATA_DIR/lake.parquet" \
        --input "1:$DATA_DIR/glacier.parquet" \
        --input "1:$DATA_DIR/ice_shelf.parquet" \
        --input "1:$DATA_DIR/reef.parquet" \
        --input "1:$DATA_DIR/urban.parquet" \
        --input "2:$DATA_DIR/river.parquet" \
        --input "2:$DATA_DIR/boundary.parquet" \
        --input "2:$DATA_DIR/admin1_boundary.parquet" \
        --input "2:$DATA_DIR/road.parquet" \
        --input "2:$DATA_DIR/geographic_lines.parquet" \
        --input "5:$DATA_DIR/places.parquet"

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
