#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$SCRIPT_DIR/.."
BUILD_DIR="$ROOT_DIR/build"
WEB_BUILD_DIR="$ROOT_DIR/build-web"
WEB_DIR="$WEB_BUILD_DIR/client"
TILE_DIR="$ROOT_DIR/tiles"
HTTP_PORT=8080

SERVER="$BUILD_DIR/server/arpentry_server"

# ── Check web build is configured ────────────────────────────────────────────

if [ ! -d "$WEB_BUILD_DIR" ]; then
    echo "Error: build-web not configured. Run ./scripts/setup-web.sh first." >&2
    exit 1
fi

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
