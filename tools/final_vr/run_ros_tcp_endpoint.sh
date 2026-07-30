#!/usr/bin/env bash
set -Eeo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
ROS_IP_VALUE="${ROS_IP_OVERRIDE:-$(hostname -I | awk '{print $1}')}"
ROS_TCP_PORT_VALUE="${ROS_TCP_PORT_OVERRIDE:-10000}"
source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"
echo "ROS-TCP IP=$ROS_IP_VALUE PORT=$ROS_TCP_PORT_VALUE"
exec ros2 run ros_tcp_endpoint default_server_endpoint \
  --ros-args \
  -p ROS_IP:="$ROS_IP_VALUE" \
  -p ROS_TCP_PORT:="$ROS_TCP_PORT_VALUE"
