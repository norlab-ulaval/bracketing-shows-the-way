#!/usr/bin/env bash
# Download and extract the sample data into data/bstw_sample/.
# Set BSTW_SAMPLE_ARCHIVE=/path/to/bstw_sample_v1.tar.zst to use a local copy.
set -euo pipefail
cd "$(dirname "$0")/.."

URL="https://github.com/norlab-ulaval/bracketing-shows-the-way/releases/download/v1.0/bstw_sample_v1.tar.zst"
SHA256="68d6e8b638a2f5c0b4e9a474692e993cf9e47bd08180fd2888ffed0a60eb5d69"
ARCHIVE="${BSTW_SAMPLE_ARCHIVE:-data/bstw_sample_v1.tar.zst}"

mkdir -p data
if [ ! -f "$ARCHIVE" ]; then
  echo "Downloading sample data (1.3 GB)..."
  curl -L --fail -o "$ARCHIVE" "$URL"
fi
echo "$SHA256  $ARCHIVE" | sha256sum --check --quiet
tar --zstd -xf "$ARCHIVE" -C data
echo "Sample data ready in data/bstw_sample/"
