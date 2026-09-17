#!/usr/bin/env bash
# 从任意目录启动。只设置当前进程环境，不安装组件、不修改系统配置。
set -e
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
if [[ ! -f "$workspace_dir/install/setup.bash" ]]; then
  echo "请先在 $workspace_dir 中执行 colcon build --symlink-install --packages-select balance_sim"
  exit 1
fi
source "$workspace_dir/install/setup.bash"
run_dir="$workspace_dir/results/interactive_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$run_dir"
export ROS_LOG_DIR="$run_dir/ros_logs"
exec ros2 launch balance_sim balance.launch.py "log_path:=$run_dir/state.csv" "$@"
