#!/bin/bash

# ROS1到ROS2数据包转换脚本
# 支持将ROS1 bag格式转换为ROS2 bag格式

set -e

# 颜色定义
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[1;33m'
readonly CYAN='\033[0;36m'
readonly NC='\033[0m'

log_info() {
    echo -e "${CYAN}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

# 检查rosbags工具
if ! command -v rosbags-convert &> /dev/null; then
    log_error "未找到 rosbags-convert 工具"
    echo "安装方法:"
    echo "  pip3 install rosbags"
    exit 1
fi

# 参数检查
if [[ $# -lt 1 ]]; then
    log_error "使用方法: $0 <数据集名称>"
    echo "示例:"
    echo "  $0 10m_easy"
    echo "  $0 300m_medium"
    exit 1
fi

DATASET_NAME="$1"
WORKSPACE_ROOT="/home/lonestarl/ws/mapping_ws"
DATA_PATH="$WORKSPACE_ROOT/src/outdoor_mapping/data"

# 输入文件路径
ROS1_BAG="$DATA_PATH/${DATASET_NAME}.bag"
ROS2_DIR="$DATA_PATH/${DATASET_NAME}"

log_info "开始数据包格式转换..."
log_info "输入: $ROS1_BAG"
log_info "输出: $ROS2_DIR"

# 检查输入文件
if [[ ! -f "$ROS1_BAG" ]]; then
    log_error "未找到ROS1数据包: $ROS1_BAG"
    exit 1
fi

# 检查输出目录是否已存在
if [[ -d "$ROS2_DIR" ]]; then
    log_warning "输出目录已存在: $ROS2_DIR"
    read -p "是否覆盖? [y/N]: " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        log_info "取消转换"
        exit 1
    fi
    rm -rf "$ROS2_DIR"
fi

# 执行转换
log_info "正在转换数据包格式..."
rosbags-convert --src "$ROS1_BAG" --dst "$ROS2_DIR"

# 验证转换结果
if [[ -d "$ROS2_DIR" ]] && [[ -f "$ROS2_DIR/metadata.yaml" ]]; then
    log_success "数据包转换成功!"
    log_info "转换后文件:"
    ls -lh "$ROS2_DIR"
else
    log_error "数据包转换失败"
    exit 1
fi

log_success "转换完成，可以继续运行建图系统"
echo ""
echo "═══════════════════════════════════════════════"
echo "转换完成的ROS2数据包路径:"
echo "  $ROS2_DIR"
echo "═══════════════════════════════════════════════"