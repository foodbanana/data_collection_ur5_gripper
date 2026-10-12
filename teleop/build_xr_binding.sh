#!/usr/bin/env bash
# =============================================================
# build_xr_binding.sh — XRoboToolkit Python 바인딩 (xrobotoolkit_sdk) 빌드 (docs/PLAN.md 5-B, 0-5)
#
#   모든 것을 ~/xrobotoolkit/ 안에만 만든다. sudo 를 쓰지 않는다. 시스템 pip·~/.bashrc·conda·Isaac Sim python 은 건드리지 않는다
#     ~/xrobotoolkit/venv/            전용 venv (시스템 python3 3.12, --system-site-packages: apt 패키지는 읽기만, pip 는 venv 안에만)
#     ~/xrobotoolkit/src/pybind/      바인딩 소스 (커밋 고정)
#     ~/xrobotoolkit/src/pcsvc/       PC-Service 레포에서 nlohmann/json 헤더 폴더만 (sparse, 약 1 MB)
#     ~/xrobotoolkit/BUILD_INFO.txt   버전·커밋·지우는 법
#   pip 캐시를 쓰지 않는다 (~/.cache/pip 에 남기지 않음)
#
#   공식 setup_ubuntu.sh 와 다른 점 (2026-10-12 확인):
#     - PC-Service 소스 (gRPC 포함) 를 빌드하지 않는다. deb 가 설치한 미리 빌드된 SDK
#       (/opt/apps/roboticsservice/SDK/x64/libPXREARobotSDK.so, 헤더는 소스와 같음, 표준 C/C++ 런타임 외 의존성 없음) 에 링크
#     - SDK 경로를 모듈에 RUNPATH 로 넣는다 → 실행할 때 LD_LIBRARY_PATH 불필요
#     - venv 안에만 설치 (공식 스크립트는 venv 밖이면 python 에 직접 설치하고, pip install pybind11 -y 는 잘못된 옵션)
#
#   주의: 서비스가 없어도 init() 은 에러 없이 끝나고 모든 값이 0 이다 (자세 7 개 0, 타임스탬프 0).
#         쓰는 쪽에서 "쿼터니언 크기 0 또는 타임스탬프 정체 = 입력 없음" 으로 판정할 것
#
# 실행 (PC-Service deb 설치 뒤, conda·venv 를 끈 터미널에서. 레포 최상위에서):
#   bash teleop/build_xr_binding.sh
# 다시 빌드: rm -rf ~/xrobotoolkit/venv ~/xrobotoolkit/src 뒤 다시 실행
# 지우기:    rm -rf ~/xrobotoolkit   (deb 는 sudo dpkg -r roboticsservice)
# =============================================================
set -euo pipefail

ROOT="${XR_ROOT:-$HOME/xrobotoolkit}"
SDK="${XR_SDK:-/opt/apps/roboticsservice/SDK}"
PY="${XR_PYTHON:-/usr/bin/python3}"
PY_MINOR="${XR_PY_MINOR:-3.12}"            # rclpy (ROS 2 Jazzy) 와 같은 버전이어야 한 프로세스에서 같이 쓸 수 있다

PYBIND_REPO=https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind
PYBIND_COMMIT=c64ccf6acd577a333e03b66fafe8efeeceb511b1
SVC_REPO=https://github.com/XR-Robotics/XRoboToolkit-PC-Service
SVC_COMMIT=85bac4dbc1fd5cef42c74a160d9c30aa3491f122
JSON_DIR=RoboticsService/PXREARobotSDK/nlohmann
PYBIND11_VER=3.1.0
# setuptools 는 80 미만: venv 가 --system-site-packages 라 시스템의 colcon-core (setuptools<80 요구) 가 보이고,
#   80 이상을 넣으면 pip 가 의존성 충돌을 알린다 (2026-10-12 데스크탑, 84.0.0 에서 확인. 시스템 colcon 자체에는 영향 없음)
SETUPTOOLS_VER=79.0.1

export PIP_NO_CACHE_DIR=1 PIP_DISABLE_PIP_VERSION_CHECK=1

die() { echo "[ERROR] $*" >&2; exit 1; }
step() { echo; echo "== $*"; }

# ---------- 0. 시작 전 검사 (아무것도 만들지 않음) ----------
step "0. 검사"
[ "$(id -u)" -ne 0 ] || die "sudo 로 실행하지 말 것"
[ -z "${VIRTUAL_ENV:-}" ] || die "venv 가 켜져 있음 ($VIRTUAL_ENV). deactivate 뒤 실행"
[ -z "${CONDA_PREFIX:-}" ] || die "conda 환경이 켜져 있음 ($CONDA_PREFIX). conda deactivate 뒤 실행"
[ -f "$SDK/x64/libPXREARobotSDK.so" ] || die "SDK 라이브러리 없음: $SDK/x64/libPXREARobotSDK.so (PC-Service deb 를 먼저 설치)"
[ -f "$SDK/include/PXREARobotSDK.h" ] || die "SDK 헤더 없음: $SDK/include/PXREARobotSDK.h"
"$PY" -c "import sys; v='%d.%d' % sys.version_info[:2]; assert v == '$PY_MINOR', v" 2>/dev/null \
  || die "$PY 가 Python $PY_MINOR 이 아님: $("$PY" --version 2>&1)"
"$PY" -c "import sysconfig, os, sys; sys.exit(0 if os.path.exists(sysconfig.get_paths()['include'] + '/Python.h') else 1)" \
  || die "Python.h 없음 (python3-dev 필요)"
for t in git cmake g++ readelf; do command -v "$t" >/dev/null || die "$t 없음"; done
for d in "$ROOT/venv" "$ROOT/src"; do
  [ ! -e "$d" ] || die "$d 가 이미 있음. 다시 빌드하려면: rm -rf $ROOT/venv $ROOT/src"
done
echo "OK: python $("$PY" --version 2>&1 | cut -d' ' -f2), SDK $SDK, 설치 위치 $ROOT"

mkdir -p "$ROOT/src"

# ---------- 1. 소스 (커밋 고정) ----------
step "1. 바인딩 소스 ($PYBIND_COMMIT)"
git init -q "$ROOT/src/pybind"
git -C "$ROOT/src/pybind" fetch -q --depth 1 "$PYBIND_REPO" "$PYBIND_COMMIT"
git -C "$ROOT/src/pybind" -c advice.detachedHead=false checkout -q FETCH_HEAD

step "1. nlohmann/json 헤더 (PC-Service $SVC_COMMIT 에서 이 폴더만)"
git init -q "$ROOT/src/pcsvc"
git -C "$ROOT/src/pcsvc" sparse-checkout set --no-cone "/$JSON_DIR/"
git -C "$ROOT/src/pcsvc" fetch -q --depth 1 --filter=blob:none "$SVC_REPO" "$SVC_COMMIT"
git -C "$ROOT/src/pcsvc" -c advice.detachedHead=false checkout -q FETCH_HEAD
[ -f "$ROOT/src/pcsvc/$JSON_DIR/json.hpp" ] || die "json.hpp 를 받지 못함"

# 바인딩 빌드가 찾는 위치 (CMakeLists: include/, lib/) 에 SDK 헤더·라이브러리 복사
mkdir -p "$ROOT/src/pybind/include" "$ROOT/src/pybind/lib"
cp "$SDK/include/PXREARobotSDK.h" "$ROOT/src/pybind/include/"
cp -r "$ROOT/src/pcsvc/$JSON_DIR" "$ROOT/src/pybind/include/nlohmann"
cp "$SDK/x64/libPXREARobotSDK.so" "$ROOT/src/pybind/lib/"

# ---------- 2. venv ----------
step "2. venv ($ROOT/venv, --system-site-packages)"
"$PY" -m venv --system-site-packages "$ROOT/venv"
VPY="$ROOT/venv/bin/python"
# ROS 를 source 한 터미널이면 PYTHONPATH 에 /opt/ros 가 들어 있다 → 빌드·설치에는 쓰지 않는다
env -u PYTHONPATH "$VPY" -m pip install -q "pybind11==$PYBIND11_VER" "setuptools==$SETUPTOOLS_VER"

# ---------- 3. 빌드·설치 (venv 안에만) ----------
step "3. 바인딩 빌드"
CMAKE_DIR="$(env -u PYTHONPATH "$VPY" -m pybind11 --cmakedir)"
env -u PYTHONPATH CMAKE_PREFIX_PATH="$CMAKE_DIR" LDFLAGS="-Wl,-rpath,$SDK/x64" \
  "$VPY" -m pip install -q --no-build-isolation --no-deps "$ROOT/src/pybind"

# ---------- 4. 확인 ----------
step "4. 확인"
MOD="$(env -u PYTHONPATH "$VPY" -c 'import importlib.util as u; print(u.find_spec("xrobotoolkit_sdk").origin)')"
case "$MOD" in "$ROOT/venv/"*) ;; *) die "모듈이 venv 밖에 있음: $MOD" ;; esac
readelf -d "$MOD" | grep -q "RUNPATH.*$SDK/x64" || die "RUNPATH 에 $SDK/x64 가 없음"
echo "모듈: $MOD"
env -u PYTHONPATH -u LD_LIBRARY_PATH "$VPY" - <<'PY'
import xrobotoolkit_sdk as x
need = ["init", "close", "get_right_controller_pose", "get_left_controller_pose", "get_right_trigger",
        "get_right_grip", "get_A_button", "get_B_button", "get_time_stamp_ns"]
miss = [n for n in need if not hasattr(x, n)]
assert not miss, f"없는 함수: {miss}"
print(f"import OK, 함수 {sum(1 for n in dir(x) if not n.startswith('_'))} 개")
PY
if [ -f /opt/ros/jazzy/setup.bash ]; then
  # ROS 를 source 한 상태에서 rclpy 와 같은 프로세스로 import (텔레옵 노드의 조건)
  bash -c "source /opt/ros/jazzy/setup.bash && '$VPY' -c 'import rclpy, xrobotoolkit_sdk; print(\"rclpy + xrobotoolkit_sdk 같은 프로세스 import OK\")'"
else
  echo "(ROS 2 Jazzy 없음: rclpy 확인 건너뜀)"
fi

cat > "$ROOT/BUILD_INFO.txt" <<EOF
XRoboToolkit Python 바인딩 빌드 기록 ($(date '+%Y-%m-%d %H:%M:%S'))
  스크립트        build_xr_binding.sh
  python          $("$VPY" --version 2>&1)  ($PY)
  venv            $ROOT/venv  (--system-site-packages)
  모듈            $MOD
  SDK (RUNPATH)   $SDK/x64/libPXREARobotSDK.so
  pybind 소스     $PYBIND_REPO @ $PYBIND_COMMIT
  json 헤더       $SVC_REPO @ $SVC_COMMIT ($JSON_DIR)
  pip             pybind11==$PYBIND11_VER setuptools==$SETUPTOOLS_VER (캐시 안 씀)
지우기: rm -rf $ROOT   (PC-Service deb: sudo dpkg -r roboticsservice, 서비스 로그: ~/.local/share/PICOBusinessSuitData)
EOF
echo
echo "[OK] 완료. 기록: $ROOT/BUILD_INFO.txt"
echo "     쓰는 법: $VPY <스크립트>   (venv 를 activate 하지 않아도 된다)"
