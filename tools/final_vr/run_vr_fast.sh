#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
SPEED="${1:-2.5}"
MODE="${2:-fresh}"
EXTRA=()
case "$MODE" in
  fresh) EXTRA+=(--fresh-mujoco) ;;
  preserve) ;;
  *) echo "MODE는 fresh 또는 preserve"; exit 2 ;;
esac
source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"
exec python3 "$ROOT/src/drok_arm_control/scripts/vr_pick_place_fast_node.py" \
  --speed-scale "$SPEED" \
  "${EXTRA[@]}" \
  --execute \
  --confirmation EXECUTE_MUJOCO_VR_PICK_PLACE
