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

# ── Start client (foreground) ─────────────────────────────────────────────────

"$CLIENT"

# ── Clean up server when client exits ────────────────────────────────────────

kill "$SERVER_PID" 2>/dev/null || true
