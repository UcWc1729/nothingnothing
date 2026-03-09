#!/bin/bash
# 检查 Go2 + FastLIO2 仿真环境与配置
# 用法: ./check_env.sh  或  source ros2_ws/install/setup.bash && ./check_env.sh

set +e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

OK="[OK]"
FAIL="[FAIL]"
WARN="[WARN]"

echo "=============================================="
echo "  Go2 + FastLIO2 仿真环境检查"
echo "=============================================="
echo ""

# ---- 1. 基础环境 ----
echo "【1】基础环境"
if command -v python3 &>/dev/null; then
    echo "  $OK python3: $(python3 --version 2>&1)"
else
    echo "  $FAIL python3 未找到"
fi
if python3 -c "import mujoco" 2>/dev/null; then
    echo "  $OK mujoco 已安装"
else
    echo "  $FAIL mujoco 未安装 (pip3 install mujoco)"
fi
if python3 -c "import onnxruntime" 2>/dev/null; then
    echo "  $OK onnxruntime 已安装"
else
    echo "  $WARN onnxruntime 未安装 (play_go2_ros2 需要: pip3 install onnxruntime)"
fi
echo ""

# ---- 2. MuJoCo-LiDAR（雷达仿真） ----
echo "【2】MuJoCo-LiDAR（雷达仿真）"
BUILD_DIR="$SCRIPT_DIR/.mujoco_lidar_build"
if [ -d "$BUILD_DIR" ]; then
    echo "  $OK 源码目录存在: .mujoco_lidar_build"
    if [ -d "$BUILD_DIR/mujoco_lidar/core_ti" ]; then
        echo "  $OK core_ti (GPU 后端) 存在"
    else
        echo "  $WARN core_ti 不存在，仅可使用 --backend cpu"
    fi
    if [ -d "$BUILD_DIR/mujoco_lidar/core_cpu" ]; then
        echo "  $OK core_cpu (CPU 后端) 存在"
    else
        echo "  $FAIL core_cpu 不存在，请重新运行 ./install_mujoco_lidar_from_source.sh"
    fi
else
    echo "  $FAIL .mujoco_lidar_build 不存在，请运行 ./install_mujoco_lidar_from_source.sh"
fi
# 能否 import（会优先用 .mujoco_lidar_build 若在 path 中）
if python3 -c "
import sys
import os
p = os.path.join('$SCRIPT_DIR', '.mujoco_lidar_build')
if os.path.isdir(p):
    sys.path.insert(0, p)
import mujoco_lidar
d = os.path.dirname(mujoco_lidar.__file__)
print('  $OK mujoco_lidar 可导入，路径:', d)
" 2>/dev/null; then
    :
else
    echo "  $WARN mujoco_lidar 导入失败或路径异常"
fi
echo ""

# ---- 3. ROS2 与工作空间 ----
echo "【3】ROS2 与工作空间"
if [ -f /opt/ros/humble/setup.bash ]; then
    echo "  $OK ROS2 Humble: /opt/ros/humble/setup.bash"
else
    echo "  $FAIL 未找到 /opt/ros/humble/setup.bash"
fi
if [ -f "$SCRIPT_DIR/ros2_ws/install/setup.bash" ]; then
    echo "  $OK ros2_ws/install/setup.bash 存在"
else
    echo "  $FAIL ros2_ws/install 未找到，请先运行 ./build_fastlio2.sh"
fi
if [ -f "$SCRIPT_DIR/ros2_ws/install/setup.bash" ]; then
    source /opt/ros/humble/setup.bash 2>/dev/null
    source "$SCRIPT_DIR/ros2_ws/install/setup.bash" 2>/dev/null
    if ros2 pkg list 2>/dev/null | grep -q "^fast_lio$"; then
        echo "  $OK 包 fast_lio 已安装"
    else
        echo "  $FAIL 包 fast_lio 未找到 (请 colcon build --packages-select fast_lio)"
    fi
    if ros2 pkg list 2>/dev/null | grep -q "teleop_twist_keyboard"; then
        echo "  $OK teleop_twist_keyboard 已安装"
    else
        echo "  $WARN teleop_twist_keyboard 未安装 (可选: sudo apt install ros-humble-teleop-twist-keyboard)"
    fi
fi
echo ""

# ---- 4. FastLIO 配置（点云关键） ----
echo "【4】FastLIO 配置（lidar_type 影响是否有点云）"
SRC_CFG="$SCRIPT_DIR/src/fastlio2_config/go2_fastlio2.yaml"
WS_CFG="$SCRIPT_DIR/ros2_ws/src/FAST_LIO/config/go2_fastlio2.yaml"
for name in "src/fastlio2_config" "ros2_ws/src/FAST_LIO/config"; do
    f="$SCRIPT_DIR/$name/go2_fastlio2.yaml"
    if [ -f "$f" ]; then
        lidar_type=$(grep -E "^\s*lidar_type:" "$f" 2>/dev/null | head -1 | sed 's/.*:\s*//')
        if [ "$lidar_type" = "0" ]; then
            echo "  $OK $name/go2_fastlio2.yaml 存在，lidar_type=$lidar_type (适合仿真)"
        else
            echo "  $WARN $name/go2_fastlio2.yaml 存在，lidar_type=$lidar_type (仿真建议为 0，否则易无点云)"
        fi
    else
        echo "  $FAIL $name/go2_fastlio2.yaml 不存在"
    fi
done
echo ""

# ---- 5. 运行时可选项：话题检查 ----
echo "【5】运行时话题（需先启动 Go2 仿真 + FastLIO 后再检查）"
if command -v ros2 &>/dev/null; then
    source /opt/ros/humble/setup.bash 2>/dev/null
    source "$SCRIPT_DIR/ros2_ws/install/setup.bash" 2>/dev/null
    if ros2 topic list 2>/dev/null | grep -q "/lidar_points"; then
        echo "  $OK /lidar_points 存在"
        hz=$(timeout 3 ros2 topic hz /lidar_points 2>&1 | grep "average rate" || true)
        [ -n "$hz" ] && echo "     $hz"
    else
        echo "  $WARN /lidar_points 未发现（请先运行 Go2 仿真）"
    fi
    if ros2 topic list 2>/dev/null | grep -q "/imu"; then
        echo "  $OK /imu 存在"
    else
        echo "  $WARN /imu 未发现（请先运行 Go2 仿真）"
    fi
else
    echo "  (跳过：ros2 不可用)"
fi
echo ""

# ---- 总结与建议 ----
echo "=============================================="
echo "  建议"
echo "=============================================="
if [ ! -d "$BUILD_DIR/mujoco_lidar/core_cpu" ] 2>/dev/null; then
    echo "  • 雷达仿真: 运行 ./install_mujoco_lidar_from_source.sh"
fi
if [ ! -f "$SCRIPT_DIR/ros2_ws/install/setup.bash" ]; then
    echo "  • FastLIO: 运行 ./build_fastlio2.sh"
fi
if [ -f "$WS_CFG" ]; then
    lidar_type=$(grep -E "^\s*lidar_type:" "$WS_CFG" 2>/dev/null | head -1 | sed 's/.*:\s*//')
    if [ "$lidar_type" != "0" ]; then
        echo "  • 无点云时: 运行 ./sync_fastlio2_config.sh 后重新启动建图"
    fi
fi
echo "  • 一键启动: ./run_go2_fastlio2.sh"
echo "  • RViz 看点云: Add → PointCloud2 → /lidar_points 或 /Laser_map，Fixed Frame 选 camera_init"
echo ""
