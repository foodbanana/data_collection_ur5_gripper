#!/usr/bin/env bash
# xacro -> URDF for the Isaac Sim URDF importer.
# All mesh paths are rewritten to plain absolute paths (no package://, no file://).
#
# needs (ROS 2 Jazzy):
#   sudo apt install ros-jazzy-xacro ros-jazzy-ur-description ros-jazzy-realsense2-description
#   (realsense2_description from ~/realsense_ws also works if you source it)
#
# usage:
#   source /opt/ros/jazzy/setup.bash
#   ./scripts/build_urdf.sh [xacro args...]
#   e.g.
#   ./scripts/build_urdf.sh mount_mesh:=$PWD/meshes/mount/wrist_mount.stl mount_thickness:=0.012 \
#                           kinematics_params:=$HOME/my_robot_calibration.yaml
set -euo pipefail

HERE="$(cd "$(dirname "$0")/.." && pwd)"
OUT_DIR="$HERE/out"
OUT="$OUT_DIR/ur5_rh_p12_d435i.urdf"
mkdir -p "$OUT_DIR"

for pkg in ur_description realsense2_description; do
  if ! ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
    echo "[ERROR] ROS package '$pkg' not found. Install it or source its workspace." >&2
    exit 1
  fi
done

xacro "$HERE/urdf/ur5_rh_p12_d435i.urdf.xacro" pkg_dir:="$HERE" "$@" > "$OUT.tmp"

python3 - "$OUT.tmp" "$OUT" <<'PY'
import re, subprocess, sys, os
src, dst = sys.argv[1], sys.argv[2]
txt = open(src).read()

def pkg_share(pkg):
    prefix = subprocess.check_output(["ros2", "pkg", "prefix", pkg], text=True).strip()
    return os.path.join(prefix, "share", pkg)

cache = {}
def repl_pkg(m):
    pkg = m.group(1)
    if pkg not in cache:
        cache[pkg] = pkg_share(pkg)
    return cache[pkg] + "/"

txt = re.sub(r"package://([^/\"]+)/", repl_pkg, txt)
txt = txt.replace("file://", "")

# every mesh must exist
missing = [p for p in re.findall(r'filename="([^"]+)"', txt) if not os.path.isfile(p)]
if missing:
    sys.exit("[ERROR] missing mesh files:\n  " + "\n  ".join(sorted(set(missing))))

open(dst, "w").write(txt)
os.remove(src)
PY

echo "[OK] $OUT"
if command -v check_urdf >/dev/null 2>&1; then
  check_urdf "$OUT" | head -n 5
fi
