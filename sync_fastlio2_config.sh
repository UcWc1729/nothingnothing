#!/bin/bash
# 将 src/fastlio2_config 同步到 ros2_ws/src/FAST_LIO/config 并重新编译 fast_lio
# 用法: ./sync_fastlio2_config.sh

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'
log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }

SRC_CFG="$SCRIPT_DIR/src/fastlio2_config"
DST_CFG="$SCRIPT_DIR/ros2_ws/src/FAST_LIO/config"

if [ ! -d "$SRC_CFG" ]; then
    echo "错误: 源配置目录不存在: $SRC_CFG"
    exit 1
fi
if [ ! -d "$DST_CFG" ]; then
    echo "错误: FastLIO 配置目录不存在: $DST_CFG（请先运行 ./build_fastlio2.sh）"
    exit 1
fi

log_info "同步配置: $SRC_CFG -> $DST_CFG"
cp -r "$SRC_CFG"/* "$DST_CFG/"
log_info "✓ 配置已复制"

log_info "重新编译 fast_lio..."
source /opt/ros/humble/setup.bash
cd "$SCRIPT_DIR/ros2_ws"
if [ -f install/setup.bash ]; then
    source install/setup.bash
fi
colcon build --symlink-install --packages-select fast_lio

log_info "✓ 完成。重启 FastLIO2 节点后新配置生效。"
