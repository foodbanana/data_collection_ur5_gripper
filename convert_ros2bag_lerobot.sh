#!/bin/bash
# =============================================================
# convert_ros2bag_lerobot.sh
#
# ros2 bag 하나를 받아서, 한 번에 아래 2가지를 순차 실행한다:
#   1단계 (ROS2 환경)        : bag -> 중간파일(npy+png)
#   2단계 v2.1 (conda:lerobot_v2, py3.10+lerobot0.3.3) : 중간파일 -> v2.1 데이터셋
# ※ v3.0 변환은 뺐다 (2026-10-05, PLAN 4-4): 학습은 v2.1 만 쓰고, lerobot_stage2_build_dataset_v30.py 는
#   옛 중간파일 형식 (head/, 그리퍼 raw) 용이라 지금 1단계 출력 (wrist/, 0~1, base_imu) 을 읽지 못한다
# 에피소드 여러 개를 하나로 합치려면 1단계만 에피소드마다 돌리고 lerobot_merge_episodes_v21.py (dataset_merge.md)
#
# repo_id 는 bag 이름을 포함해 자동 생성되므로, bag 마다 다른 폴더가 만들어진다:
#   repo_id = <namespace>/<project>_<bag이름>
#   예: foodbanana/ur5_gripper_drone_20260916_141816_test_1_fake_node
#
# 사용:
#   ./convert_ros2bag_lerobot.sh <bag폴더> --arm-action command|next_state --base-imu const|topic \
#                                [--task "문장"] [--fps 숫자] [--namespace 이름] [--project 이름]
#   --arm-action, --base-imu 는 필수 (1단계에 그대로 넘김. 텔레옵·sim = command, 고정 베이스 = const)
# 예:
#   ./convert_ros2bag_lerobot.sh ~/data_collection_ur5_gripper/bags/20261005_212640_stage1test \
#     --arm-action command --base-imu const --task "pick up the drone" --fps 25
#
#   # namespace/project 는 기본값이 있어 생략 가능. 바꾸고 싶을 때만:
#   ./convert_ros2bag_lerobot.sh <bag> --project drone_grasp --namespace myname
#
# 결과:
#   <데이터수집>/bag_lerobot_intermediate/<bag이름>/                      (중간파일)
#   <데이터수집>/lerobot_dataset_v21/<namespace>/<project>_<bag이름>/     (v2.1)
# =============================================================

set -e  # 어느 단계든 실패하면 즉시 중단

# ── 사용자 환경 설정 ──
CONDA_SH="${HOME}/miniconda3/etc/profile.d/conda.sh"
ENV_V21="lerobot_v2"     # py3.10 + lerobot 0.3.3 (v2.1)

# 이 스크립트가 있는 폴더 (스크립트들이 같은 폴더에 있다고 가정)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STAGE1="${SCRIPT_DIR}/lerobot_stage1_extract_bag.py"
STAGE2_V21="${SCRIPT_DIR}/lerobot_stage2_build_dataset_v21.py"

usage() {
  echo "사용법: $0 <bag폴더> --arm-action command|next_state --base-imu const|topic [--task \"문장\"] [--fps 숫자] [--namespace 이름] [--project 이름]"
  echo "  <bag폴더>          (필수) ros2 bag 폴더 경로 (record_toggle.py 로 녹화)"
  echo "  --arm-action 값    (필수) 팔 action: command = /joint_command (텔레옵·sim), next_state = states[i+1] (freedrive)"
  echo "  --base-imu 값      (필수) base_imu: const = 고정 베이스 상수, topic = /base/imu"
  echo "  --task \"문장\"      (선택) 언어 명령. 기본: \"pick up the drone\""
  echo "  --fps 숫자         (선택) 리샘플 목표 fps. 기본: 25"
  echo "  --namespace 이름   (선택) repo_id 앞부분. 기본: foodbanana"
  echo "  --project 이름     (선택) repo_id 프로젝트명. 기본: ur5_gripper_drone"
  echo ""
  echo "  repo_id = <namespace>/<project>_<bag이름>  (bag마다 다른 폴더가 생성됨)"
  echo ""
  echo "예: $0 ~/.../bags/<bag이름> --arm-action command --base-imu const --task \"pick up the drone\" --fps 25"
}

# ── 기본값 ──
BAG_PATH=""
TASK="pick up the drone"
FPS="25"
NAMESPACE="foodbanana"
PROJECT="ur5_gripper_drone"
ARM_ACTION=""
BASE_IMU=""

# ── 인자 파싱 (bag은 위치 인자, 나머지는 --옵션) ──
while [ $# -gt 0 ]; do
  case "$1" in
    --task)      TASK="$2"; shift 2 ;;
    --fps)       FPS="$2"; shift 2 ;;
    --namespace) NAMESPACE="$2"; shift 2 ;;
    --project)   PROJECT="$2"; shift 2 ;;
    --arm-action) ARM_ACTION="$2"; shift 2 ;;
    --base-imu)  BASE_IMU="$2"; shift 2 ;;
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
if [ -z "${ARM_ACTION}" ] || [ -z "${BASE_IMU}" ]; then
  echo "[오류] --arm-action 과 --base-imu 를 지정하세요 (기본값 없음: 팔 action 의미가 달라 섞이면 안 됨)."; usage; exit 1
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
echo "### [1/2] 1단계: bag -> 중간파일 (ROS2 환경) ###"
conda deactivate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
python3 "${STAGE1}" "${BAG_PATH}" --fps "${FPS}" --arm-action "${ARM_ACTION}" --base-imu "${BASE_IMU}"

source "${CONDA_SH}"

# =============================================================
# 2단계 v2.1 — conda:lerobot_v2 : 중간파일 -> v2.1
# =============================================================
echo ""
echo "### [2/2] 2단계: 중간파일 -> v2.1 (conda:${ENV_V21}) ###"
conda activate "${ENV_V21}"
env -u PYTHONPATH python "${STAGE2_V21}" "${INTER_DIR}" --task "${TASK}" --repo-id "${REPO_ID}"
conda deactivate

echo ""
echo "============================================================"
echo " 완료. 생성물:"
echo "   중간파일 : ${INTER_DIR}"
echo "   v2.1     : ${COLLECTION_DIR}/lerobot_dataset_v21/${REPO_ID}"
echo "============================================================"
