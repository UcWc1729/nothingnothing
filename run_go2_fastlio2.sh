#!/bin/bash
# 一键启动：Go2 仿真 + 键盘控制节点 + FastLIO2 建图
# 使用前请先执行 build_fastlio2.sh 并 source ros2_ws/install/setup.bash

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 加载 ROS2 与工作空间
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "[ERROR] 未找到 /opt/ros/humble/setup.bash，请先安装 ROS2 Humble 或修改本脚本中的 ROS 路径"
    exit 1
fi
if [ -f ros2_ws/install/setup.bash ]; then
    source ros2_ws/install/setup.bash
else
    echo "[ERROR] 未找到 ros2_ws/install/setup.bash，请先运行 ./build_fastlio2.sh"
    exit 1
fi

# 退出时杀掉本脚本启动的后台进程（Go2 + 静态 TF）
GO2_PID=""
TF_PID=""
cleanup() {
    echo ""
    echo "[INFO] 正在关闭所有节点..."
    [ -n "$GO2_PID" ] && kill $GO2_PID 2>/dev/null || true
    [ -n "$TF_PID" ] && kill $TF_PID 2>/dev/null || true
    wait $GO2_PID 2>/dev/null || true
    wait $TF_PID 2>/dev/null || true
    exit 0
}
trap cleanup SIGINT SIGTERM EXIT

echo "=============================================="
echo "一键启动 Go2 + 键盘控制 + FastLIO2 建图"
echo "=============================================="
echo "若修改过 src/fastlio2_config/*.yaml，请先运行: ./sync_fastlio2_config.sh"
echo ""
echo "[1/3] 启动 Go2 仿真环境（雷达: mid360，后台）..."
python3 src/robots/play_go2_ros2.py --lidar mid360 &
GO2_PID=$!
sleep 5
echo "[1b] 发布静态 TF odom -> camera_init（与仿真 odom->imu->lidar 连成一体，RViz 中可见 odom/imu/lidar/camera_init）..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom camera_init &
TF_PID=$!
sleep 3
echo "[2/3] 键盘控制：请在【另一终端】运行以下命令后，用 i/j/k/l 等控制 Go2 移动："
echo "      source $(pwd)/ros2_ws/install/setup.bash"
echo "      ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo ""
echo "[3/3] 启动 FastLIO2 建图（前台，按 Ctrl+C 结束全部）..."
echo "RViz：Fixed Frame 选 camera_init；若报 queue is full，选 camera_init 即可。"
echo "----------------------------------------------"
ros2 launch fast_lio mapping.launch.py config_file:=go2_fastlio2.yaml
