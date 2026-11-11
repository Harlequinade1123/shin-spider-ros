#!/usr/bin/env bash
# Build and run the Java golden harness around the ORIGINAL Processing logic.
# Needs a JDK (javac). Falls back to the JDK bundled with Processing 4 under /opt.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
PDE="${PDE_DIR:-/home/ntlab/king_ws/src/ExercisesInCreativeRobotics2-main/Processing/original_spider_controller}"
OUT="${1:-$HERE/../../test/golden/golden.txt}"
BUILD="$HERE/build"

JAVAC="$(command -v javac || true)"
if [ -z "$JAVAC" ]; then
  for c in /usr/lib/jvm/*/bin/javac /opt/processing-*/java/bin/javac; do
    [ -x "$c" ] && JAVAC="$c" && break
  done
fi
[ -n "$JAVAC" ] || { echo "no javac found" >&2; exit 1; }
JAVA="$(dirname "$JAVAC")/java"

rm -rf "$BUILD"; mkdir -p "$BUILD/classes" "$(dirname "$OUT")"
python3 -I "$HERE/transform.py" "$PDE" "$HERE/Golden.inc" "$BUILD/Sketch.java"
"$JAVAC" -encoding UTF-8 -nowarn -d "$BUILD/classes" "$HERE/Stub.java" "$BUILD/Sketch.java"
"$JAVA" -cp "$BUILD/classes" Sketch "$OUT"
echo "wrote $OUT ($(grep -c '^CASE ' "$OUT") cases)"
