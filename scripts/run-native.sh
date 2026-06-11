#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$SCRIPT_DIR/.."
BUILD_DIR="$ROOT_DIR/build"
TILE_DIR="$ROOT_DIR/tiles"

# Server is the Rust reimplementation (server), built with Cargo below; only
# the client comes from the C build (CMake). The server synthesises tiles
# procedurally when pointed at a non-archive path (here, the tile directory).
SERVER="$ROOT_DIR/server/target/release/arpentry_server"
CLIENT="$BUILD_DIR/client/arpentry_client"

# ── Check native build is configured ─────────────────────────────────────────

if [ ! -f "$BUILD_DIR/CMakeCache.txt" ]; then
    echo "Configuring native build..."
    cmake -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Debug
fi

# ── Build ─────────────────────────────────────────────────────────────────────

echo "Building client (C)..."
cmake --build "$BUILD_DIR" --target arpentry_client

echo "Building Rust server..."
( cd "$ROOT_DIR/server" && cargo build --release )

# ── Kill any previous instances ──────────────────────────────────────────────

pkill -x arpentry_server 2>/dev/null || true
pkill -x arpentry_client 2>/dev/null || true

# ── Ensure tile directory exists ─────────────────────────────────────────────

mkdir -p "$TILE_DIR"

# ── Start server in background ───────────────────────────────────────────────

"$SERVER" "$TILE_DIR" "$ROOT_DIR/style.json" &
SERVER_PID=$!
echo "arpentry_server started (pid $SERVER_PID)"

# Wait until the server accepts connections. The client fetches style.arps,
# models.arpm, and index.arpi exactly once at startup; if it launches before
# the server listens, those fetches fail and it silently falls back to the
# built-in defaults (green background, no models).
for _ in $(seq 1 100); do
    if nc -z 127.0.0.1 8090 >/dev/null 2>&1; then break; fi
    if ! kill -0 "$SERVER_PID" 2>/dev/null; then
        echo "Error: arpentry_server exited before accepting connections" >&2
        exit 1
    fi
    sleep 0.1
done

# ── Start client (foreground) ─────────────────────────────────────────────────

"$CLIENT"

# ── Clean up server when client exits ────────────────────────────────────────

kill "$SERVER_PID" 2>/dev/null || true
