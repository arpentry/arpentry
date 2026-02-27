#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"
WEB_BUILD_DIR="$SCRIPT_DIR/build-web"
WEB_DIR="$WEB_BUILD_DIR/client"
TILE_DIR="$SCRIPT_DIR/tiles"
HTTP_PORT=8080

SERVER="$BUILD_DIR/server/arpentry_server"

# ── Build ─────────────────────────────────────────────────────────────────────

cmake --build "$BUILD_DIR"
cmake --build "$WEB_BUILD_DIR"

# ── Kill any previous instances ──────────────────────────────────────────────

pkill -x arpentry_server 2>/dev/null || true
pkill -f "http.server $HTTP_PORT" 2>/dev/null || true

# ── Ensure tile directory exists ─────────────────────────────────────────────

mkdir -p "$TILE_DIR"

# ── Start server in background ───────────────────────────────────────────────

"$SERVER" "$TILE_DIR" &
SERVER_PID=$!
echo "arpentry_server started (pid $SERVER_PID)"

# ── Start HTTP server for WebAssembly client (foreground) ────────────────────

echo "Serving WebAssembly client at http://localhost:$HTTP_PORT"
trap "kill $SERVER_PID 2>/dev/null || true" EXIT
python3 -m http.server "$HTTP_PORT" --bind localhost -d "$WEB_DIR"
