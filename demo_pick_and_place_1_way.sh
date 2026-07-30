#!/usr/bin/env bash

set -Eeuo pipefail

ROOT="${HOME}/IK_solver_MuJoCo"

FULL_SEQUENCE="${ROOT}/src/drok_arm_control/scripts/execute_cylinder_pick_place_full.sh"

CONFIRMATION_TOKEN="EXECUTE_MUJOCO_CYLINDER_PICK_PLACE"

EXECUTE=0
MUJOCO_PID=""

usage()
{
  cat <<EOF
Pick and Place 1 Way — MuJoCo Demo

실행:

  cd ${ROOT}
  ./demo_pick_and_place_1_way.sh --execute

동작 순서:

  HOME
  → 그리퍼 개방
  → 원기둥 접근
  → 파지
  → 들어 올리기
  → 반대편 받침대로 운반
  → 내려놓기
  → 그리퍼 개방
  → 후퇴

주의:
  MuJoCo 시뮬레이션 전용입니다.
  실제 로봇에서는 실행하지 마십시오.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --execute)
      EXECUTE=1
      shift
      ;;

    --help|-h)
      usage
      exit 0
      ;;

    *)
      echo "알 수 없는 인자: $1" >&2
      usage
      exit 2
      ;;
  esac
done

if [[ "${EXECUTE}" -ne 1 ]]; then
  usage
  exit 0
fi

cd "${ROOT}"

if [[ ! -f "${ROOT}/install/setup.bash" ]]; then
  echo "[실패] install/setup.bash가 없습니다."
  echo
  echo "먼저 빌드하십시오:"
  echo
  echo "  cd ${ROOT}"
  echo "  source /opt/ros/humble/setup.bash"
  echo "  colcon build --symlink-install"
  exit 1
fi

if [[ ! -x "${FULL_SEQUENCE}" ]]; then
  echo "[실패] 전체 실행기를 찾지 못했습니다:"
  echo "${FULL_SEQUENCE}"
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "${ROOT}/install/setup.bash"
set -u

mkdir -p "${ROOT}/run_logs"

STAMP="$(date +%Y%m%d_%H%M%S)"

LAUNCH_LOG="${ROOT}/run_logs/pick_place_1_way_mujoco_${STAMP}.log"

cleanup()
{
  if [[ -n "${MUJOCO_PID}" ]] &&
    kill -0 "${MUJOCO_PID}" 2>/dev/null
  then
    kill "${MUJOCO_PID}" 2>/dev/null || true
    wait "${MUJOCO_PID}" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

echo "============================================================"
echo "Pick and Place 1 Way"
echo "============================================================"
echo
echo "[시작 대기] MuJoCo 실행"

ros2 launch \
  drok_arm_mujoco \
  drok_arm_mujoco.launch.py \
  > "${LAUNCH_LOG}" 2>&1 &

MUJOCO_PID=$!

READY=0
TIMEOUT_SEC=20

echo "[시작 대기] 제어기 준비"

for ((attempt=1; attempt<=TIMEOUT_SEC; attempt++)); do
  if ! kill -0 "${MUJOCO_PID}" 2>/dev/null; then
    echo
    echo "[실패] MuJoCo launch가 종료되었습니다."
    echo "[로그] ${LAUNCH_LOG}"
    exit 1
  fi

  ACTIONS="$(
    ros2 action list \
      2>/dev/null \
      || true
  )"

  if grep -Fxq \
    "/arm_controller/follow_joint_trajectory" \
    <<< "${ACTIONS}" &&
    grep -Fxq \
    "/gripper_controller/follow_joint_trajectory" \
    <<< "${ACTIONS}"
  then
    READY=1
    break
  fi

  sleep 1
done

if [[ "${READY}" -ne 1 ]]; then
  echo
  echo "[실패] 제어기가 ${TIMEOUT_SEC}초 안에 준비되지 않았습니다."
  echo "[로그] ${LAUNCH_LOG}"
  exit 1
fi

echo "[시작] Pick and Place 1 Way"

set +e

"${FULL_SEQUENCE}" \
  --execute \
  --confirmation "${CONFIRMATION_TOKEN}"

SEQUENCE_STATUS=$?

set -e

if [[ "${SEQUENCE_STATUS}" -ne 0 ]]; then
  echo
  echo "[실패] Pick and Place 1 Way"
  echo "[상태 코드] ${SEQUENCE_STATUS}"
  echo "[MuJoCo 로그] ${LAUNCH_LOG}"
  exit "${SEQUENCE_STATUS}"
fi

echo
echo "[종료] Pick and Place 1 Way 완료"
echo "[현재 위치] PLACE_RETREAT 종료점"
echo
echo "MuJoCo는 계속 실행 중입니다."
echo "종료하려면 이 터미널에서 Ctrl+C를 누르십시오."
echo
echo "[MuJoCo 로그] ${LAUNCH_LOG}"

wait "${MUJOCO_PID}" || true
