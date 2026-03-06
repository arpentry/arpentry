#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$SCRIPT_DIR/.."
BUILD_DIR="$ROOT_DIR/build"

# ── Configure (only if needed) ───────────────────────────────────────────

if [ ! -f "$BUILD_DIR/CMakeCache.txt" ]; then
    echo "==> Configuring (Debug + sanitizers)..."
    cmake -B "$BUILD_DIR" \
        -DCMAKE_BUILD_TYPE=Debug \
        -DARPENTRY_SANITIZERS=ON
fi

# ── Build ────────────────────────────────────────────────────────────────

echo "==> Building..."
cmake --build "$BUILD_DIR" 2>&1

# ── Test ─────────────────────────────────────────────────────────────────

echo "==> Running tests..."
ctest --test-dir "$BUILD_DIR" --output-on-failure

echo "==> All tests passed."
