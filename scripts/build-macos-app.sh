#!/usr/bin/env bash
# Builds AutoPathingFTC.app from app/macos/main.swift + app/web/index.html.
#
# Usage:
#   ./scripts/build-macos-app.sh            # build into ./build/
#   ./scripts/build-macos-app.sh --install  # build, then copy into $INSTALL_DIR
#
# Optional overrides live in .env (copy .env.example → .env).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

if [[ -f .env ]]; then
  set -a; source .env; set +a
fi

APP_NAME="${APP_NAME:-AutoPathingFTC}"
BUNDLE_ID="${BUNDLE_ID:-}"
SIGN_IDENTITY="${SIGN_IDENTITY:--}"          # "-" = ad-hoc signature (fine for personal use)
INSTALL_DIR="${INSTALL_DIR:-/Applications}"
MIN_MACOS="${MIN_MACOS:-12.0}"

BUILD_DIR="$ROOT/build"
APP_DIR="$BUILD_DIR/$APP_NAME.app"

command -v swiftc >/dev/null || { echo "error: swiftc not found — run: xcode-select --install" >&2; exit 1; }

echo "▶ Cleaning $APP_DIR"
rm -rf "$APP_DIR"
mkdir -p "$APP_DIR/Contents/MacOS" "$APP_DIR/Contents/Resources"

echo "▶ Compiling Swift wrapper"
swiftc -O \
  -target "$(uname -m)-apple-macos$MIN_MACOS" \
  -framework Cocoa -framework WebKit \
  "$ROOT/app/macos/main.swift" \
  -o "$APP_DIR/Contents/MacOS/$APP_NAME"

echo "▶ Bundling resources"
cp "$ROOT/app/macos/Info.plist" "$APP_DIR/Contents/Info.plist"
cp "$ROOT/app/web/index.html"   "$APP_DIR/Contents/Resources/index.html"

plutil -replace CFBundleExecutable  -string "$APP_NAME" "$APP_DIR/Contents/Info.plist"
plutil -replace CFBundleName        -string "$APP_NAME" "$APP_DIR/Contents/Info.plist"
plutil -replace CFBundleDisplayName -string "$APP_NAME" "$APP_DIR/Contents/Info.plist"
if [[ -n "$BUNDLE_ID" ]]; then
  plutil -replace CFBundleIdentifier -string "$BUNDLE_ID" "$APP_DIR/Contents/Info.plist"
fi

echo "▶ Code signing (identity: $SIGN_IDENTITY)"
codesign --force --deep --sign "$SIGN_IDENTITY" "$APP_DIR"

echo "✓ Built $APP_DIR"

if [[ "${1:-}" == "--install" ]]; then
  echo "▶ Installing to $INSTALL_DIR"
  rm -rf "$INSTALL_DIR/$APP_NAME.app"     # avoid nesting a .app inside an existing bundle
  cp -R "$APP_DIR" "$INSTALL_DIR/"
  echo "✓ Installed $INSTALL_DIR/$APP_NAME.app"
fi
