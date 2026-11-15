#!/usr/bin/env bash
# Download a file through the proxy in proxy.env (falls back to direct).
#
#   ./tools/fetch.sh <url> [output]
#
# Exists because release assets (the xiaozhi images, ESP-IDF tools) drop from
# ~14 s to ~1.3 s per request through the LAN proxy, and a stalled curl in the
# middle of a flash session is how a device ends up half-written.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
[ -f "$HERE/proxy.env" ] && set -a && . "$HERE/proxy.env" && set +a

URL=${1:?usage: fetch.sh <url> [output]}
OUT=${2:-$(basename "$URL")}

PROXY=${MRRC_HTTP_PROXY:-}
CURL=(curl -L --retry 3 --retry-delay 2 --connect-timeout 15 -o "$OUT")
if [ -n "$PROXY" ]; then
    CURL+=(-x "$PROXY")
    [ -n "${MRRC_NO_PROXY:-}" ] && CURL+=(--noproxy "$(echo "$MRRC_NO_PROXY" | tr ',' ',')")
    echo "proxy: $PROXY"
else
    echo "proxy: none (direct)"
fi

echo "get:   $URL"
"${CURL[@]}" -w 'done: %{http_code} %{size_download} B in %{time_total}s\n' "$URL"
