#!/usr/bin/env bash
# Clearpath Husky A200 xacro -> URDF for the Isaac Sim URDF importer (isaacsim/assets/bases/husky/).
# The source package stays outside the repo; only the imported USD is committed.
# Mesh paths are rewritten to plain absolute paths (no package://).
#
# needs: ROS 2 Jazzy (xacro), git. No build or install: husky_description is registered
#        for $(find ...) with a local ament index under $HUSKY_WS/husky_prefix.
#
# usage:
#   source /opt/ros/jazzy/setup.bash
#   isaacsim/scripts/build_husky_urdf.sh            # -> $HUSKY_WS/husky_out/husky.urdf
#   HUSKY_WS=~/other_ws isaacsim/scripts/build_husky_urdf.sh
# then import that URDF in the Isaac Sim GUI (isaacsim/assets/bases/husky/README.md).
set -euo pipefail

HUSKY_WS="${HUSKY_WS:-$HOME/husky_ws}"
REPO_URL="https://github.com/husky/husky.git"
BRANCH="humble-devel"
COMMIT="729f8aa45ccd86fa33a05e07ef698c52c451cd9c"   # the commit the committed USD was built from

SRC="$HUSKY_WS/husky"
PREFIX="$HUSKY_WS/husky_prefix"
OUT_DIR="$HUSKY_WS/husky_out"
OUT="$OUT_DIR/husky.urdf"
PKG="$SRC/husky_description"

command -v xacro >/dev/null || { echo "[ERROR] xacro not found. source /opt/ros/jazzy/setup.bash" >&2; exit 1; }

if [ ! -d "$SRC/.git" ]; then
  mkdir -p "$HUSKY_WS"
  git clone -b "$BRANCH" "$REPO_URL" "$SRC"
fi
HEAD="$(git -C "$SRC" rev-parse HEAD)"
if [ "$HEAD" != "$COMMIT" ]; then
  echo "[ERROR] $SRC is at $HEAD, expected $COMMIT. Run: git -C $SRC checkout $COMMIT" >&2
  exit 1
fi

# register husky_description for xacro $(find husky_description) (link only, no build)
mkdir -p "$PREFIX/share/ament_index/resource_index/packages"
touch "$PREFIX/share/ament_index/resource_index/packages/husky_description"
ln -sfn "$PKG" "$PREFIX/share/husky_description"
export AMENT_PREFIX_PATH="$PREFIX${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"

mkdir -p "$OUT_DIR"
# defaults = no laser / realsense / sensor arch; gazebo_controllers is only read when is_sim
xacro "$PKG/urdf/husky.urdf.xacro" gazebo_controllers:=none urdf_extras:="$PKG/urdf/empty.urdf" > "$OUT.tmp"
sed "s#package://husky_description/#$PKG/#g" "$OUT.tmp" > "$OUT"
rm -f "$OUT.tmp"

if grep -q "package://" "$OUT"; then
  echo "[ERROR] package:// paths left in $OUT:" >&2
  grep -o 'package://[^"]*' "$OUT" | sort -u >&2
  exit 1
fi
missing=0
for f in $(grep -o 'filename="[^"]*"' "$OUT" | cut -d'"' -f2 | sort -u); do
  [ -f "$f" ] || { echo "[ERROR] missing mesh: $f" >&2; missing=1; }
done
[ "$missing" = 0 ] || exit 1

echo "[OK] $OUT (husky $BRANCH $COMMIT)"
if command -v check_urdf >/dev/null 2>&1; then
  check_urdf "$OUT" | head -3
fi
