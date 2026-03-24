#!/usr/bin/env bash
# Tile Natural Earth 10m data, serve it, and view the globe in the browser.
#
# Run scripts/download-naturalearth.py first to fetch the data:
#   python3 scripts/download-naturalearth.py
#
# Run scripts/setup-web.sh first to configure the Emscripten build:
#   ./scripts/setup-web.sh
#
# Usage:
#   ./scripts/run-naturalearth-web.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$SCRIPT_DIR/.."
BUILD_DIR="$ROOT_DIR/build"
WEB_BUILD_DIR="$ROOT_DIR/build-web"
WEB_DIR="$WEB_BUILD_DIR/client"
DATA_DIR="$ROOT_DIR/data/naturalearth"
ARCHIVE="/tmp/naturalearth.arpa"
DEM="$DATA_DIR/etopo1.tif"
HTTP_PORT=8080

SERVER="$BUILD_DIR/server/arpentry_server"
TILER="$BUILD_DIR/tiler/arpentry_tiler"

# World bbox
BBOX="-180,-85,180,85"

# Zoom range
MIN_ZOOM=0
MAX_ZOOM=8

# ── Check native build is configured ─────────────────────────────────────────

if [ ! -f "$BUILD_DIR/CMakeCache.txt" ]; then
    echo "Configuring native build..."
    cmake -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release
fi

# ── Check web build is configured ────────────────────────────────────────────

if [ ! -d "$WEB_BUILD_DIR" ]; then
    echo "Error: build-web not configured. Run ./scripts/setup-web.sh first." >&2
    exit 1
fi

# ── Build ─────────────────────────────────────────────────────────────────────

echo "Building native..."
cmake --build "$BUILD_DIR"

echo "Building web client..."
cmake --build "$WEB_BUILD_DIR"

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
        --input "2:$DATA_DIR/river.parquet" \
        --input "2:$DATA_DIR/boundary.parquet" \
        --input "5:$DATA_DIR/places.parquet"

    echo "Archive: $(du -h "$ARCHIVE" | cut -f1)"
else
    echo ""
    echo "Using existing archive: $ARCHIVE ($(du -h "$ARCHIVE" | cut -f1))"
fi

# ── Kill any previous instances ──────────────────────────────────────────────

pkill -x arpentry_server 2>/dev/null || true
pkill -f "http.server $HTTP_PORT" 2>/dev/null || true
sleep 0.5

# ── Start server in background ───────────────────────────────────────────────

echo ""
echo "Starting server with $ARCHIVE..."
"$SERVER" "$ARCHIVE" "$ROOT_DIR/style-naturalearth.json" &
SERVER_PID=$!
echo "Server started (pid $SERVER_PID)"

# ── Start HTTP server for WebAssembly client (foreground) ────────────────────

echo "Serving WebAssembly client at http://localhost:$HTTP_PORT"
trap "kill $SERVER_PID 2>/dev/null || true" EXIT
python3 - "$HTTP_PORT" "$WEB_DIR" <<'EOF'
import sys, http.server, functools

class NoCacheHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

port = int(sys.argv[1])
directory = sys.argv[2]
Handler = functools.partial(NoCacheHandler, directory=directory)
http.server.HTTPServer(("localhost", port), Handler).serve_forever()
EOF
