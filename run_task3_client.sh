#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CLIENT_DIR="$ROOT_DIR/qt_client"
BUILD_DIR="$CLIENT_DIR/build"
TARGET="$BUILD_DIR/qt_imu_client"

if [[ ! -x "$TARGET" ]]; then
  cmake -S "$CLIENT_DIR" -B "$BUILD_DIR"
  cmake --build "$BUILD_DIR" -j2
fi

exec "$TARGET"
