#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$SCRIPT_DIR/.."
BUILD_NATIVE_DIR="$ROOT_DIR/build-native"
WEB_BUILD_DIR="$ROOT_DIR/build-web"
FLATCC="$BUILD_NATIVE_DIR/_deps/flatcc-src/bin/flatcc"

# ── Check prerequisites ───────────────────────────────────────────────────────

if ! command -v emcmake &>/dev/null; then
    echo "Error: emcmake not found." >&2
    echo "Install Emscripten: https://emscripten.org/docs/getting_started/downloads.html" >&2
    exit 1
fi

# ── Build native flatcc host compiler ────────────────────────────────────────

if [ ! -f "$FLATCC" ]; then
    echo "Configuring native build for flatcc host compiler..."
    cmake -B "$BUILD_NATIVE_DIR" -DCMAKE_BUILD_TYPE=Release
    cmake --build "$BUILD_NATIVE_DIR" --target flatcc_cli
fi

# ── Configure web build ───────────────────────────────────────────────────────

echo "Configuring web build..."
emcmake cmake -B "$WEB_BUILD_DIR" -DFLATCC_HOST_COMPILER="$FLATCC"

echo ""
echo "Web build configured. Run ./scripts/run-web.sh to build and launch."
