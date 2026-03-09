#!/bin/bash
# 从源码安装 MuJoCo-LiDAR（包含 core_cpu 与 core_ti 后端）
# PyPI 的 mujoco-lidar 不包含后端实现，必须从源码安装后才能运行 LiDAR 仿真

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORK_DIR="${SCRIPT_DIR}/.mujoco_lidar_build"

echo "=============================================="
echo "从源码安装 MuJoCo-LiDAR"
echo "=============================================="

# 克隆或更新仓库
if [ -d "$WORK_DIR" ]; then
    echo "[1/3] 更新已有仓库: $WORK_DIR"
    cd "$WORK_DIR"
    git pull --rebase || true
else
    echo "[1/3] 克隆 MuJoCo-LiDAR 到: $WORK_DIR"
    git clone https://github.com/TATP-233/MuJoCo-LiDAR.git "$WORK_DIR"
    cd "$WORK_DIR"
fi

# 安装（常规安装；editable 需要 PEP 660 支持，该项目暂不支持）
echo "[2/3] 安装 mujoco_lidar（含 CPU + Taichi 后端）..."
pip3 install ".[taichi]"

echo "[3/3] 验证安装..."
python3 -c "
import mujoco_lidar
import os
p = os.path.dirname(mujoco_lidar.__file__)
core_ti = os.path.exists(os.path.join(p, 'core_ti'))
core_cpu = os.path.exists(os.path.join(p, 'core_cpu'))
print('mujoco_lidar 路径:', p)
print('core_ti 存在:', core_ti)
print('core_cpu 存在:', core_cpu)
if core_ti or core_cpu:
    print('安装成功，可以运行 LiDAR 仿真。')
else:
    print('警告: 未找到 core_ti/core_cpu，请检查安装。')
    exit(1)
"

echo ""
echo "安装完成。请在本项目目录下运行："
echo "  python3 src/lidar_sim_native.py          # GPU 后端"
echo "  python3 src/lidar_sim_native.py --backend cpu   # CPU 后端"
echo "  python3 src/lidar_sim_ros2.py"
echo ""
