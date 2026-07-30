#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/humble/setup.bash
cd "$ROOT"
if command -v rosdep >/dev/null 2>&1; then
  rosdep install --from-paths src --ignore-src -r -y
fi
colcon build --symlink-install
