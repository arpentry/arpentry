#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"
TILE_DIR="$SCRIPT_DIR/tiles"

SERVER="$BUILD_DIR/server/arpentry_server"
CLIENT="$BUILD_DIR/client/arpentry_client"

# ── Build ─────────────────────────────────────────────────────────────────────

cmake --build "$BUILD_DIR"

# ── Kill any previous instances ──────────────────────────────────────────────

pkill -x arpentry_server 2>/dev/null || true
pkill -x arpentry_client 2>/dev/null || true

# ── Ensure tile directory exists ─────────────────────────────────────────────

mkdir -p "$TILE_DIR"

# ── Start server in background ───────────────────────────────────────────────

"$SERVER" "$TILE_DIR" &
SERVER_PID=$!
echo "arpentry_server started (pid $SERVER_PID)"

# ── Start client (foreground) ─────────────────────────────────────────────────

"$CLIENT"

# ── Clean up server when client exits ────────────────────────────────────────

kill "$SERVER_PID" 2>/dev/null || true
