#!/usr/bin/env bash
set -e
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$workspace_dir/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-74}"
export GZ_PARTITION="balance_yaw_manual_$$"
result_dir="$workspace_dir/results_yaw/manual_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$result_dir"
export ROS_LOG_DIR="$result_dir/ros_logs"
exec ros2 launch balance_yaw yaw.launch.py gui:=true "log_path:=$result_dir/yaw.csv" "$@"
