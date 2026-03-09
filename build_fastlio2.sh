#!/bin/bash

set -e 

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

git submodule update --init --recursive

source /opt/ros/humble/setup.bash
cd ros2_ws

PROJECT_ROOT=$(pwd)
log_info "ws path: $PROJECT_ROOT"

log_info "step 1/6: 检查 cmake..."
if ! command -v cmake &>/dev/null; then
  log_warn "未检测到 cmake，尝试安装（需要 sudo 密码）..."
  sudo apt update && sudo apt install -y cmake || log_warn "跳过 cmake 安装，请手动安装后重试"
else
  log_info "cmake 已安装: $(cmake --version | head -1)"
fi

log_info "step 2/6: 编译 Livox-SDK2..."
cd src/Livox-SDK2
mkdir -p build && cd build
cmake .. && make -j$(nproc)
# livox_ros_driver2 使用自带的 prebuild SDK，不依赖系统安装；若需系统级安装可取消下行注释并输入 sudo 密码
sudo make install 2>/dev/null || log_warn "跳过 Livox-SDK2 系统安装（不影响后续构建，livox_ros_driver2 使用自带 SDK）"

log_info "step 3/6: 编译 livox_ros_driver2..."
cd "$PROJECT_ROOT"
colcon build --symlink-install --packages-select livox_ros_driver2

cd "$PROJECT_ROOT"

log_info "step 4/6: 加载Livox环境..."
source install/setup.bash

log_info "step 5/6: 复制FAST_LIO配置文件..."

cp -r ../src/fastlio2_config/* src/FAST_LIO/config/
log_info "✓ 配置文件已复制到 src/FAST_LIO/config/"

log_info "step 6/6: 编译 FAST_LIO..."
if command -v rosdepc &>/dev/null; then
  rosdepc install --from-paths src --ignore-src -y
else
  rosdep install --from-paths src --ignore-src -y
fi
colcon build --symlink-install --packages-select fast_lio

log_info "✓ 全部构建完成！"