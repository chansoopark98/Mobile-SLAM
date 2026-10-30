#!/usr/bin/env bash
# Fetch the Ceres commit the public reference runtime tree pins (MODULE.bazel `ceres`:
# 2b32b321242b32f6364108694e0069f87e56326c, 2021-02-17, 2.0.0 + 17 commits) from the official
# ceres-solver GitHub repository, verify it, and unpack it for -DSLAM_REFACT_CERES_COMMIT=<sha>.
#
#   bash slam_refact/engine/tools/fetch_ceres_pin.sh
#
# Network: one HTTPS download of the GitHub archive of that commit (skipped when the archive is
# already at build/_deps/ceres-2b32b321.tar.gz). The archive must match the sha256 recorded
# below, otherwise nothing is unpacked.
# Output: build/_deps/ceres-2b32b321-src with SLAM_REFACT_SOURCE (version, commit, tree, archive
# sha256, url), which the engine configure checks against SLAM_REFACT_CERES_COMMIT.
set -euo pipefail

COMMIT=2b32b321242b32f6364108694e0069f87e56326c
TREE=35768b4a4826fa0d9047f855f4fabc8ad3a0598c
SHA256=31514b8a1dc04436ad59cf16ea1d44880dd35c2593f6055dbba07362f5a981a9
URL="https://github.com/ceres-solver/ceres-solver/archive/${COMMIT}.tar.gz"

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
# Worktrees keep build/ in the main checkout.
if [ ! -d "$ROOT/build/_deps" ] && [ -n "${SLAM_REFACT_MAIN_ROOT:-}" ]; then ROOT=$SLAM_REFACT_MAIN_ROOT; fi
DEPS="$ROOT/build/_deps"
ARCHIVE="$DEPS/ceres-${COMMIT:0:8}.tar.gz"
SRC="$DEPS/ceres-${COMMIT:0:8}-src"

mkdir -p "$DEPS"
if [ ! -f "$ARCHIVE" ]; then
  curl -fsSL -o "$ARCHIVE.part" "$URL"
  mv "$ARCHIVE.part" "$ARCHIVE"
fi
echo "$SHA256  $ARCHIVE" | sha256sum -c -
# GitHub archives carry the commit id in the pax global header.
header="$(gzip -dc "$ARCHIVE" 2>/dev/null | head -c 1024 | tr -d '\0' || true)"
if [[ "$header" != *"$COMMIT"* ]]; then
  echo "archive header does not name commit $COMMIT" >&2
  exit 1
fi

rm -rf "$SRC.tmp"
mkdir -p "$SRC.tmp"
tar xzf "$ARCHIVE" -C "$SRC.tmp" --strip-components=1
grep -q "<version>2.0.0</version>" "$SRC.tmp/package.xml"
printf 'version 2.0.0\ncommit %s\ntree %s\narchive_sha256 %s\nurl %s\n' \
  "$COMMIT" "$TREE" "$SHA256" "$URL" > "$SRC.tmp/SLAM_REFACT_SOURCE"
rm -rf "$SRC"
mv "$SRC.tmp" "$SRC"
echo "Ceres ${COMMIT} unpacked at $SRC"
