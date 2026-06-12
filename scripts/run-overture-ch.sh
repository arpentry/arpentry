#!/usr/bin/env bash
# Download Overture Maps data for Switzerland, tile it, serve it, and view it.
#
# Prerequisites:
#   pip install overturemaps
#   pmtiles CLI (for terrain elevation): https://github.com/protomaps/go-pmtiles
#
# Usage:
#   ./scripts/run-overture-ch.sh [options]
#
# Options:
#   --skip-download        Don't fetch missing layers (use whatever is present)
#   --skip-build           Don't rebuild the client / Rust binaries
#   --skip-tile            Reuse the existing archive (implies --skip-download)
#   --serve-only           Start the server and wait; don't launch the client
#   --screenshot <path>    Capture a PNG instead of opening the interactive viewer
#   --bbox <w,s,e,n>       Override the geographic bounds
#   --min-zoom <z>         Override the minimum zoom (default 0)
#   --max-zoom <z>         Override the maximum zoom (default 14)
#   --port <n>             Server port (default 8090)
#   --no-terrain           Don't add Mapterhorn elevation (flat terrain mesh)
#   --terrain-file <path>  Use an existing terrain PMTiles instead of extracting
#
# Downloads are idempotent: a layer is fetched only when its .parquet is missing.
#
# Terrain: elevation comes from Mapterhorn (https://mapterhorn.com), a global
# Terrarium DEM in PMTiles. Rather than download the ~30 GB planet file, the
# bbox + zooms this run needs are extracted from it over HTTP with the `pmtiles`
# CLI into data/overture-ch/terrain.pmtiles (idempotent, like the layer
# downloads). The tiler then samples it for per-vertex elevation.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$SCRIPT_DIR/.."
BUILD_DIR="$ROOT_DIR/build"
DATA_DIR="$ROOT_DIR/data/overture-ch"
ARCHIVE="$DATA_DIR/switzerland.arpa"
STYLE="$ROOT_DIR/style-overture-ch.json"

# Mapterhorn terrain (Terrarium DEM PMTiles). planet.pmtiles covers z0-12; we
# extract just the run's bbox + zooms into TERRAIN_PMTILES over HTTP range reads.
MAPTERHORN_URL="https://download.mapterhorn.com/planet.pmtiles"
MAPTERHORN_MAX_ZOOM=12
TERRAIN_PMTILES="$DATA_DIR/terrain.pmtiles"

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

# Server port
PORT=8090

# Parse arguments
SKIP_DOWNLOAD=false
SKIP_BUILD=false
SKIP_TILE=false
SERVE_ONLY=false
SCREENSHOT=""
USE_TERRAIN=true
TERRAIN_FILE=""
while [ $# -gt 0 ]; do
    case "$1" in
        --skip-download) SKIP_DOWNLOAD=true; shift ;;
        --skip-build)    SKIP_BUILD=true; shift ;;
        --skip-tile)     SKIP_TILE=true; SKIP_DOWNLOAD=true; shift ;;
        --serve-only)    SERVE_ONLY=true; shift ;;
        --screenshot)    SCREENSHOT="$2"; shift 2 ;;
        --bbox)          BBOX="$2"; shift 2 ;;
        --min-zoom)      MIN_ZOOM="$2"; shift 2 ;;
        --max-zoom)      MAX_ZOOM="$2"; shift 2 ;;
        --port)          PORT="$2"; shift 2 ;;
        --no-terrain)    USE_TERRAIN=false; shift ;;
        --terrain-file)  TERRAIN_FILE="$2"; shift 2 ;;
        *) echo "Unknown option: $1" >&2; exit 1 ;;
    esac
done

# ── Build ─────────────────────────────────────────────────────────────────────

if [ "$SKIP_BUILD" = false ]; then
    if [ ! -f "$BUILD_DIR/CMakeCache.txt" ]; then
        echo "Configuring build..."
        cmake -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release
    fi
    echo "Building client (C)..."
    cmake --build "$BUILD_DIR" --target arpentry_client
    echo "Building Rust tiler + server..."
    ( cd "$ROOT_DIR/server" && cargo build --release )
fi

# ── Download + tile ──────────────────────────────────────────────────────────

if [ "$SKIP_TILE" = true ]; then
    if [ ! -f "$ARCHIVE" ]; then
        echo "ERROR: --skip-tile set but archive not found: $ARCHIVE" >&2
        exit 1
    fi
    echo "Reusing existing archive: $ARCHIVE ($(du -h "$ARCHIVE" | cut -f1))"
else
    mkdir -p "$DATA_DIR"

    # Fetch a layer only when its parquet is missing (downloads are idempotent).
    download_type() {
        local type="$1"
        local output="$DATA_DIR/${type}.parquet"
        if [ -f "$output" ]; then
            echo "Skipping $type (already present: $(du -h "$output" | cut -f1))"
            return
        fi
        if [ "$SKIP_DOWNLOAD" = true ]; then
            echo "WARNING: $type missing and --skip-download set; continuing without it"
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

    download_type land_cover
    download_type land_use
    download_type water
    download_type segment
    download_type building
    download_type place
    download_type division_boundary

    # Add an input only when its parquet exists, so partial data still tiles.
    # Indices match the tiler's layer scheme (and style-overture-ch.json's
    # source_layer names): 1=land_cover 3=water 5=transportation 6=land_use
    # 7=building 8=poi 9=boundary.
    TILER_INPUTS=()
    add_input() {
        local idx="$1" file="$2"
        if [ -f "$file" ]; then
            TILER_INPUTS+=(--input "$idx:$file")
        else
            echo "Skipping layer $idx (missing $(basename "$file"))"
        fi
    }
    add_input 1 "$DATA_DIR/land_cover.parquet"        # land_cover
    add_input 6 "$DATA_DIR/land_use.parquet"          # land_use
    add_input 3 "$DATA_DIR/water.parquet"             # water
    add_input 5 "$DATA_DIR/segment.parquet"           # transportation
    add_input 7 "$DATA_DIR/building.parquet"          # building (extrusions)
    add_input 8 "$DATA_DIR/place.parquet"             # poi (labels)
    add_input 9 "$DATA_DIR/division_boundary.parquet" # boundary (country/canton)

    if [ ${#TILER_INPUTS[@]} -eq 0 ]; then
        echo "ERROR: no input parquet files found under $DATA_DIR" >&2
        exit 1
    fi

    # Extract the Mapterhorn terrain for this bbox + zoom range (idempotent).
    # Sets TERRAIN_ARG to (--terrain <file>) when elevation is available.
    TERRAIN_ARG=()
    if [ "$USE_TERRAIN" = true ]; then
        terrain_src="$TERRAIN_FILE"
        if [ -z "$terrain_src" ]; then
            terrain_src="$TERRAIN_PMTILES"
            if [ ! -f "$terrain_src" ]; then
                if [ "$SKIP_DOWNLOAD" = true ]; then
                    echo "WARNING: terrain missing and --skip-download set; tiling flat"
                    terrain_src=""
                elif ! command -v pmtiles >/dev/null 2>&1; then
                    echo "WARNING: pmtiles CLI not found; tiling flat (install" \
                         "https://github.com/protomaps/go-pmtiles or use --no-terrain)"
                    terrain_src=""
                else
                    # Mapterhorn tops out at z12; don't ask for more.
                    tzoom=$MAX_ZOOM
                    [ "$tzoom" -gt "$MAPTERHORN_MAX_ZOOM" ] && tzoom=$MAPTERHORN_MAX_ZOOM
                    echo "Extracting Mapterhorn terrain (bbox=$BBOX zoom=$MIN_ZOOM-$tzoom)..."
                    pmtiles extract "$MAPTERHORN_URL" "$terrain_src" \
                        --bbox="$BBOX" --minzoom="$MIN_ZOOM" --maxzoom="$tzoom"
                    echo "  -> $(du -h "$terrain_src" | cut -f1)"
                fi
            else
                echo "Reusing terrain: $terrain_src ($(du -h "$terrain_src" | cut -f1))"
            fi
        fi
        [ -n "$terrain_src" ] && TERRAIN_ARG=(--terrain "$terrain_src")
    fi

    echo ""
    echo "Tiling to $ARCHIVE..."
    echo "  bbox=$BBOX  zoom=$MIN_ZOOM-$MAX_ZOOM"
    "$TILER" \
        --output "$ARCHIVE" \
        --bbox "$BBOX" \
        --min-zoom "$MIN_ZOOM" \
        --max-zoom "$MAX_ZOOM" \
        --mem $((256 * 1024 * 1024)) \
        ${TERRAIN_ARG[@]+"${TERRAIN_ARG[@]}"} \
        "${TILER_INPUTS[@]}"
    echo "Archive: $(du -h "$ARCHIVE" | cut -f1)"
fi

# ── Kill any previous instances ──────────────────────────────────────────────

pkill -x arpentry_server 2>/dev/null || true
pkill -x arpentry_client 2>/dev/null || true
sleep 0.5

# ── Start server ─────────────────────────────────────────────────────────────

echo ""
echo "Starting server on port $PORT with $ARCHIVE..."
"$SERVER" "$ARCHIVE" "$STYLE" "$PORT" &
SERVER_PID=$!
cleanup() { kill "$SERVER_PID" 2>/dev/null || true; }
trap cleanup EXIT
echo "Server started (pid $SERVER_PID)"
sleep 1

if [ "$SERVE_ONLY" = true ]; then
    echo "Serving on http://localhost:$PORT (Ctrl-C to stop)..."
    wait "$SERVER_PID"
    exit 0
fi

# ── Start client ─────────────────────────────────────────────────────────────

# Center on Switzerland: Bern area
LON=7.45
LAT=46.95
ALT=200000

if [ -n "$SCREENSHOT" ]; then
    echo "Capturing screenshot to $SCREENSHOT..."
    "$CLIENT" --url "http://localhost:$PORT" \
              --lon "$LON" --lat "$LAT" --alt "$ALT" \
              --bearing 0 --tilt 0 --screenshot "$SCREENSHOT"
    echo "Done: $SCREENSHOT"
else
    echo "Opening viewer centered on Switzerland..."
    "$CLIENT" --url "http://localhost:$PORT" \
              --lon "$LON" --lat "$LAT" --alt "$ALT"
fi
