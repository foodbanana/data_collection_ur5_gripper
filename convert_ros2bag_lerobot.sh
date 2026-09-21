#!/bin/bash
# =============================================================
# convert_ros2bag_lerobot.sh
#
# ros2 bag 하나를 받아서, 한 번에 아래 3가지를 순차 실행한다:
#   1단계 (ROS2 환경)        : bag -> 중간파일(npy+png)
#   2단계 v3.0 (conda:lerobot_v1, py3.12+lerobot0.6.1) : 중간파일 -> v3.0 데이터셋
#   2단계 v2.1 (conda:lerobot_v2, py3.10+lerobot0.3.3) : 중간파일 -> v2.1 데이터셋
#
# repo_id 는 bag 이름을 포함해 자동 생성되므로, bag 마다 다른 폴더가 만들어진다:
#   repo_id = <namespace>/<project>_<bag이름>
#   예: foodbanana/ur5_gripper_drone_20260916_141816_test_1_fake_node
#
# 사용:
#   ./convert_ros2bag_lerobot.sh <bag폴더> [--task "문장"] [--fps 숫자] \
#                                          [--namespace 이름] [--project 이름]
# 예:
#   ./convert_ros2bag_lerobot.sh \
#     ~/data_collection_ur5_gripper/bags/20260916_141816_test_1_fake_node \
#     --task "pick up the drone" --fps 25
#
#   # namespace/project 는 기본값이 있어 생략 가능. 바꾸고 싶을 때만:
#   ./convert_ros2bag_lerobot.sh <bag> --project drone_grasp --namespace myname
#
# 결과:
#   <데이터수집>/bag_lerobot_intermediate/<bag이름>/                      (중간파일)
#   <데이터수집>/lerobot_dataset_v30/<namespace>/<project>_<bag이름>/     (v3.0)
#   <데이터수집>/lerobot_dataset_v21/<namespace>/<project>_<bag이름>/     (v2.1)
# =============================================================

set -e  # 어느 단계든 실패하면 즉시 중단

# ── 사용자 환경 설정 ──
CONDA_SH="${HOME}/miniconda3/etc/profile.d/conda.sh"
ENV_V30="lerobot_v1"     # py3.12 + lerobot 0.6.1 (v3.0)
ENV_V21="lerobot_v2"     # py3.10 + lerobot 0.3.3 (v2.1)

# 이 스크립트가 있는 폴더 (스크립트들이 같은 폴더에 있다고 가정)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STAGE1="${SCRIPT_DIR}/lerobot_stage1_extract_bag.py"
STAGE2_V30="${SCRIPT_DIR}/lerobot_stage2_build_dataset_v30.py"
STAGE2_V21="${SCRIPT_DIR}/lerobot_stage2_build_dataset_v21.py"

usage() {
  echo "사용법: $0 <bag폴더> [--task \"문장\"] [--fps 숫자] [--namespace 이름] [--project 이름]"
  echo "  <bag폴더>          (필수) ros2 bag 폴더 경로"
  echo "  --task \"문장\"      (선택) 언어 명령. 기본: \"pick up the drone\""
  echo "  --fps 숫자         (선택) 리샘플 목표 fps. 기본: 25"
  echo "  --namespace 이름   (선택) repo_id 앞부분. 기본: foodbanana"
  echo "  --project 이름     (선택) repo_id 프로젝트명. 기본: ur5_gripper_drone"
  echo ""
  echo "  repo_id = <namespace>/<project>_<bag이름>  (bag마다 다른 폴더가 생성됨)"
  echo ""
  echo "예: $0 ~/.../bags/20260916_141816_test_1_fake_node --task \"pick up the drone\" --fps 25"
}

# ── 기본값 ──
BAG_PATH=""
TASK="pick up the drone"
FPS="25"
NAMESPACE="foodbanana"
PROJECT="ur5_gripper_drone"

# ── 인자 파싱 (bag은 위치 인자, 나머지는 --옵션) ──
while [ $# -gt 0 ]; do
  case "$1" in
    --task)      TASK="$2"; shift 2 ;;
    --fps)       FPS="$2"; shift 2 ;;
    --namespace) NAMESPACE="$2"; shift 2 ;;
    --project)   PROJECT="$2"; shift 2 ;;
    -h|--help)   usage; exit 0 ;;
    -*)          echo "알 수 없는 옵션: $1"; usage; exit 1 ;;
    *)
      if [ -z "${BAG_PATH}" ]; then
        BAG_PATH="$1"; shift
      else
        echo "인자가 너무 많음: $1"; usage; exit 1
      fi ;;
  esac
done

if [ -z "${BAG_PATH}" ]; then
  echo "[오류] bag 폴더를 지정하세요."; usage; exit 1
fi
if [ ! -d "${BAG_PATH}" ]; then
  echo "[오류] bag 폴더가 아님: ${BAG_PATH}"; exit 1
fi
if ! echo "${FPS}" | grep -Eq '^[0-9]+([.][0-9]+)?$'; then
  echo "[오류] --fps 는 숫자여야 함 (예: 25). 입력값: ${FPS}"; exit 1
fi

# 절대경로 정규화 + bag 이름 추출
BAG_PATH="$(cd "${BAG_PATH}" && pwd)"
BAG_NAME="$(basename "${BAG_PATH}")"

# repo_id 조립 = <namespace>/<project>_<bag이름>
REPO_ID="${NAMESPACE}/${PROJECT}_${BAG_NAME}"

# 경로 계산
BAGS_DIR="$(dirname "${BAG_PATH}")"              # .../bags
COLLECTION_DIR="$(dirname "${BAGS_DIR}")"        # .../data_collection_ur5_gripper
INTER_DIR="${COLLECTION_DIR}/bag_lerobot_intermediate/${BAG_NAME}"

echo "============================================================"
echo " bag        : ${BAG_PATH}"
echo " task       : ${TASK}"
echo " fps        : ${FPS}"
echo " repo_id    : ${REPO_ID}"
echo " 중간파일   : ${INTER_DIR}"
echo "============================================================"

# =============================================================
# 1단계 — ROS2 환경 (rosbag2_py) : bag -> 중간파일
# =============================================================
echo ""
echo "### [1/3] 1단계: bag -> 중간파일 (ROS2 환경) ###"
conda deactivate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
python3 "${STAGE1}" "${BAG_PATH}" --fps "${FPS}"

# =============================================================
# conda 준비 (2,3단계 공통)
# =============================================================
source "${CONDA_SH}"

# =============================================================
# 2단계 v3.0 — conda:lerobot_v1 : 중간파일 -> v3.0
# =============================================================
echo ""
echo "### [2/3] 2단계: 중간파일 -> v3.0 (conda:${ENV_V30}) ###"
conda activate "${ENV_V30}"
python "${STAGE2_V30}" "${INTER_DIR}" --task "${TASK}" --fps "${FPS}" --repo-id "${REPO_ID}"
conda deactivate

# =============================================================
# 2단계 v2.1 — conda:lerobot_v2 : 중간파일 -> v2.1
# =============================================================
echo ""
echo "### [3/3] 2단계: 중간파일 -> v2.1 (conda:${ENV_V21}) ###"
conda activate "${ENV_V21}"
env -u PYTHONPATH python "${STAGE2_V21}" "${INTER_DIR}" --task "${TASK}" --fps "${FPS}" --repo-id "${REPO_ID}"
conda deactivate

echo ""
echo "============================================================"
echo " 완료. 생성물:"
echo "   중간파일 : ${INTER_DIR}"
echo "   v3.0     : ${COLLECTION_DIR}/lerobot_dataset_v30/${REPO_ID}"
echo "   v2.1     : ${COLLECTION_DIR}/lerobot_dataset_v21/${REPO_ID}"
echo "============================================================"
