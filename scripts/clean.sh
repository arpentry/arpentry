#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

rm -rf "$SCRIPT_DIR/../build" "$SCRIPT_DIR/../build-native" "$SCRIPT_DIR/../build-web"
echo "Removed build directories."
