#!/usr/bin/env sh
# Serve the docs book on all interfaces (LAN-visible).
set -eu
cd "$(dirname "$0")"
exec mdbook serve --hostname 0.0.0.0 --port "${PORT:-3001}"
