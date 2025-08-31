#!/bin/bash

set -e  # 遇到错误立即退出

# 数据集选择
DATASET_INPUT=${1:-easy}

# 智能数据集映射表
declare -A DATASET_MAP=(
    # 简写映射
    ["easy"]="10m_easy"
    ["medium"]="300m_medium"
    ["hard"]="degenerative_hard"
    ["degen"]="degenerative_hard"
    ["short"]="A_short_hard"
    
    # 数字简写
    ["10m"]="10m_easy"
    ["300m"]="300m_medium"
    
    # 全名支持
    ["10m_easy"]="10m_easy"
    ["300m_medium"]="300m_medium"
    ["degenerative_hard"]="degenerative_hard"
    ["A_short_hard"]="A_short_hard"

    # ROS1
    ["10m_easy.bag"]="10m_easy"
    ["300m_medium.bag"]="300m_medium"
    ["degenerative_hard.bag"]="degenerative_hard"
    ["A_short_hard.bag"]="A_short_hard"
)

# 输出配置
SAVE_FORMAT=${2:-pcd}          # 保存格式: pcd/ply
SAVE_INTERVAL=${3:-30.0}     # 自动保存间隔(秒)
PLAY_RATE=${4:-auto}         # 播放速率(auto表示智能适配)

# 颜色定义 - 增强终端显示
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[1;33m'
readonly BLUE='\033[0;34m'
readonly CYAN='\033[0;36m'
readonly MAGENTA='\033[0;35m'
readonly WHITE='\033[1;37m'
readonly NC='\033[0m' # No Color

# 日志函数
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_step() {
    echo -e "${CYAN}[STEP $1/7]${NC} $2"
}

# 智能数据集验证和映射
validate_dataset() {
    local input_name="$1"
    
    log_info "开始验证数据集输入: '$input_name'"
    
    # 检查映射
    if [[ -n "${DATASET_MAP[$input_name]}" ]]; then
        DATASET="${DATASET_MAP[$input_name]}"
        log_info "✓ 输入 '$input_name' → 映射为 '$DATASET'"
        
        # 验证映射结果不为空
        if [[ -z "$DATASET" ]]; then
            log_error "映射结果为空字符串，请检查映射表配置"
            exit 1
        fi
    else
        log_error "无效的数据集 '$input_name'"
        echo "支持的简写:"
        echo "  简单: easy, 10m"
        echo "  中等: medium, 300m"
        echo "  困难: hard, degen, degenerative_hard"
        echo "  短场景: short, A_short_hard"
        echo ""
        echo "完整支持列表:"
        for key in "${!DATASET_MAP[@]}"; do
            echo "  $key → ${DATASET_MAP[$key]}"
        done
        exit 1
    fi
    
    return 0
}

# 路径配置

# 虚拟机环境路径配置
WORKSPACE_ROOT="/home/lonestarl/ws/mapping_ws"
log_info "使用虚拟机环境路径: $WORKSPACE_ROOT"

PACKAGE_NAME="outdoor_mapping"

# 基于环境的路径配置
CONFIG_PATH="$WORKSPACE_ROOT/src/$PACKAGE_NAME/config"
RVIZ_PATH="$WORKSPACE_ROOT/src/$PACKAGE_NAME/rviz"
DATA_ROOT="$WORKSPACE_ROOT/src/$PACKAGE_NAME/data"
MAPS_ROOT="$WORKSPACE_ROOT/maps"

# 具体文件路径 - 使用数据集名称
# 注意：DATASET变量必须在validate_dataset函数调用后才会被设置
# 这些变量将在main函数中重新设置
CONFIG_FILE=""
RVIZ_FILE=""
DATASET_PATH=""

# 调试信息将在验证数据集后显示
# 环境检查
check_environment() {
    log_step 1 "环境检查 - 虚拟机优化"
    
    # 检查ROS2环境
    if [[ -z "$ROS_DISTRO" ]]; then
        log_error "ROS2环境未激活"
        echo "虚拟机解决方案:"
        echo "  source /opt/ros/foxy/setup.bash"
        echo "  或 source /opt/ros/humble/setup.bash"
        exit 1
    fi
    log_success "ROS2环境: $ROS_DISTRO"
    
    # 检查必需工具
    local tools=("colcon" "python3" "ros2")
    for tool in "${tools[@]}"; do
        if ! command -v "$tool" &> /dev/null; then
            log_error "未找到工具: $tool"
            echo "虚拟机安装:"
            [[ "$tool" == "colcon" ]] && echo "  pip3 install colcon-common-extensions"
            [[ "$tool" == "python3" ]] && echo "  sudo apt install python3"
            [[ "$tool" == "ros2" ]] && echo "  检查ROS2安装"
            exit 1
        fi
    done
    log_success "所有必需工具已就绪"
}

# 文件验证
validate_files() {
    log_step 2 "文件验证 - 支持ROS2 bag"
    
    # 验证配置文件
    local config_file="$CONFIG_PATH/$CONFIG_FILE"
    log_info "检查配置文件: $config_file"
    
    # 强制修正：确保使用正确的配置文件名
    if [[ "$CONFIG_FILE" == "_mapping.yaml" ]]; then
        log_error "检测到错误配置文件名: $CONFIG_FILE"
        log_error "这表示数据集映射失败，当前数据集: '$DATASET'"
        log_info "请检查数据集输入参数是否正确"
        exit 1
    fi
    
    if [[ ! -f "$config_file" ]]; then
        log_error "配置文件不存在: $config_file"
        log_info "当前数据集: $DATASET"
        log_info "期望配置文件: $CONFIG_FILE"
        log_info "配置目录: $CONFIG_PATH"
        log_info "可用配置文件:"
        
        if [[ -d "$CONFIG_PATH" ]]; then
            for file in "$CONFIG_PATH"/*.yaml; do
                if [[ -f "$file" ]]; then
                    log_info "  - $(basename "$file")"
                fi
            done
        else
            log_error "配置目录不存在: $CONFIG_PATH"
        fi
        exit 1
    fi
    log_success "配置文件: $CONFIG_FILE"
    
    # 验证RViz配置（可选）
    local rviz_file="$RVIZ_PATH/$RVIZ_FILE"
    if [[ ! -f "$rviz_file" ]]; then
        log_warning "RViz配置不存在，将使用默认配置"
        RVIZ_FILE=""
    else
        log_success "RViz配置: $RVIZ_FILE"
    fi
    
    # 验证数据集 - 支持ROS2和ROS1 bag格式
    local ros1_bag_file="$DATA_ROOT/$DATASET.bag"
    local ros2_dir="$DATA_ROOT/$DATASET"

    log_info "检查数据集格式..."

    # 检查ROS2格式
    if [[ -d "$ros2_dir" ]]; then
        local db3_files=($(find "$ros2_dir" -name "*.db3" -type f))
        local metadata_file="$ros2_dir/metadata.yaml"

        if [[ ${#db3_files[@]} -gt 0 ]] && [[ -f "$metadata_file" ]]; then
            DATASET_PATH="$ros2_dir"
            log_success "检测到ROS2格式数据包"
            log_info "  .db3文件数量: ${#db3_files[@]}"
            log_info "  总大小: $(du -sh "$ros2_dir" | cut -f1)"
            return 0
        fi
    fi

    # 检查ROS1格式
    if [[ -f "$ros1_bag_file" ]]; then
        log_info "检测到ROS1格式数据包: $ros1_bag_file"
        log_info "需要转换为ROS2格式..."

        # 检查转换工具
        if ! command -v rosbags-convert &> /dev/null; then
            log_error "未找到 rosbags-convert 工具"
            echo "安装方法:"
            echo "  pip3 install rosbags"
            echo "或运行:"
            echo "  ./convert_bag.sh $DATASET"
            exit 1
        fi

        # 检查输出目录
        if [[ -d "$ros2_dir" ]]; then
            log_warning "ROS2格式已存在，跳过转换"
            DATASET_PATH="$ros2_dir"
        else
            log_info "正在转换数据包格式..."
            
            # 在新终端中运行转换脚本，显示详细过程
            if [[ -f "./convert_bag.sh" ]]; then
                log_info "在新终端中启动数据包转换..."
                log_info "转换命令: ./convert_bag.sh $DATASET"
                
                # 使用gnome-terminal或xterm启动新终端
                if command -v gnome-terminal &> /dev/null; then
                    gnome-terminal -- bash -c "./convert_bag.sh $DATASET; echo '按任意键继续...'; read -n 1 -s"
                elif command -v xterm &> /dev/null; then
                    xterm -e "./convert_bag.sh $DATASET; echo '按任意键继续...'; read -n 1 -s" &
                elif command -v konsole &> /dev/null; then
                    konsole -e "./convert_bag.sh $DATASET; echo '按任意键继续...'; read -n 1 -s" &
                else
                    # 如果没有图形终端，直接运行
                    log_warning "未找到图形终端，在当前终端运行转换..."
                    ./convert_bag.sh "$DATASET"
                fi
                
                # 等待转换完成
                while [[ ! -d "$ros2_dir" ]] || [[ ! -f "$ros2_dir/metadata.yaml" ]]; do
                    sleep 30
                    log_info "等待转换完成..."
                done
                
                log_success "数据包转换完成"
                DATASET_PATH="$ros2_dir"
            else
                log_info "使用内置转换..."
                rosbags-convert --src "$ros1_bag_file" --dst "$ros2_dir"
                
                if [[ $? -ne 0 ]]; then
                    log_error "数据包转换失败"
                    exit 1
                fi
                
                log_success "数据包转换完成"
                DATASET_PATH="$ros2_dir"
            fi
        fi

        # 验证转换结果
        local db3_files=($(find "$ros2_dir" -name "*.db3" -type f))
        local metadata_file="$ros2_dir/metadata.yaml"
        
        if [[ ${#db3_files[@]} -eq 0 ]] || [[ ! -f "$metadata_file" ]]; then
            log_error "转换后的数据包格式不正确"
            exit 1
        fi

        log_info "  .db3文件数量: ${#db3_files[@]}"
        log_info "  总大小: $(du -sh "$ros2_dir" | cut -f1)"
        return 0
    fi

    log_error "未找到数据集文件"
    log_info "期望路径:"
    log_info "  ROS2格式: $ros2_dir/"
    log_info "  ROS1格式: $ros1_bag_file"
    exit 1
}

# 播放速率
calculate_play_rate() {
    local dataset="$1"
    local rate="$2"
    
    if [[ "$rate" != "auto" ]]; then
        echo "$rate"
        return
    fi
    
    # 根据数据集智能选择播放速率
    case "$dataset" in
        "10m_easy")
            echo "2.0"  # 简单场景，快速播放
            ;;
        "300m_medium")
            echo "1.5"  # 中等场景，中速播放
            ;;
        "degenerative_hard"|"A_short_hard")
            echo "1.0"  # 困难场景，标准播放
            ;;
        *)
            echo "1.0"  # 默认标准播放
            ;;
    esac
}

# 工作空间构建
build_workspace() {
    log_step 3 "工作空间构建 - 智能环境优化"
    
    cd "$WORKSPACE_ROOT"
    
    # 检查是否需要重新构建
    if [[ -d "install/$PACKAGE_NAME" ]]; then
        log_info "工作空间已构建，跳过重新构建"
        return 0
    fi
    
    log_info "开始构建工作空间..."
    log_info "构建目录: $(pwd)"
    
    # 虚拟机环境构建参数
    local parallel_workers=$(nproc 2>/dev/null || echo 2)
    colcon build \
        --symlink-install \
        --packages-select "$PACKAGE_NAME" \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --parallel-workers $parallel_workers
    
    if [[ $? -ne 0 ]]; then
        log_error "构建失败"
        echo "解决方案:"
        echo "  1. 清理构建: rm -rf build/ install/ log/"
        echo "  2. 检查依赖: pip3 install -r src/$PACKAGE_NAME/requirements.txt"
        echo "  3. 重新构建"
        exit 1
    fi
    
    log_success "工作空间构建完成"
}

# 环境加载
load_environment() {
    log_step 4 "环境加载"
    
    # 设置ROS2环境变量 - 隐藏冗长输出
    export RCUTILS_LOGGING_BUFFERED_STREAM=1
    export RCUTILS_LOGGING_USE_STDOUT=1
    export COLCON_TRACE=${COLCON_TRACE:-0}
    export RCUTILS_LOGGING_SEVERITY_THRESHOLD=WARN
    export COLCON_LOG_LEVEL=ERROR
    export ROS_LOCALHOST_ONLY=1
    
    # 加载工作空间环境 - 静默模式
    local setup_file="$WORKSPACE_ROOT/install/setup.bash"
    if [[ -f "$setup_file" ]]; then
        # 临时禁用调试输出
        local old_trace="$COLCON_TRACE"
        export COLCON_TRACE=0
        source "$setup_file" > /dev/null 2>&1
        export COLCON_TRACE="$old_trace"
        log_success "ROS2环境加载完成"
    else
        log_error "未找到环境设置文件: $setup_file"
        log_info "请确保工作空间已构建: colcon build"
        exit 1
    fi
}

# 输出目录准备
prepare_output() {
    log_step 5 "输出目录准备"
    
    local output_dir="$MAPS_ROOT/$DATASET"
    mkdir -p "$output_dir"
    
    log_success "输出目录: $output_dir"
    
    # 清理旧地图文件
    if [[ -d "$output_dir" ]]; then
        local old_maps=$(find "$output_dir" -name "*.pcd" -o -name "*.ply" | wc -l)
        if [[ $old_maps -gt 0 ]]; then
            log_info "发现 $old_maps 个旧地图文件"
            log_info "新地图将保存在: $output_dir"
        fi
    fi
}

# 启动建图系统
start_mapping() {
    log_step 6 "启动建图系统"
    
    local play_rate=$(calculate_play_rate "$DATASET" "$PLAY_RATE")
    
    # 设置全局日志级别以减少输出
    export RCUTILS_LOGGING_SEVERITY_THRESHOLD=WARN
    export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] {message}"
    export QT_QPA_PLATFORM=xcb  # 强制使用X11模式
    export ROS_PYTHON_LOG_CONFIG_FILE=""  # 禁用Python日志配置
    
    # 显示启动信息
    echo ""
    echo "配置信息:"
    echo "  数据集: $DATASET"
    echo "  播放速率: ${play_rate}x"
    echo "  保存格式: $SAVE_FORMAT"
    echo "  保存间隔: ${SAVE_INTERVAL}秒"
    echo "  启动时间: $(date)"
    echo "  日志级别: WARN (减少调试信息)"
    echo ""
    
    # 启动SLAM系统
    log_info "启动SLAM节点和可视化..."
    
    # 使用launch文件启动完整系统
    ros2 launch "$PACKAGE_NAME" data_specific_mapping.launch.py \
        dataset:="$DATASET" \
        save_format:="$SAVE_FORMAT" \
        save_interval:="$SAVE_INTERVAL" \
        play_rate:="$play_rate" \
        config_file:="$CONFIG_FILE" \
        rviz_file:="$RVIZ_FILE" &
    
    SLAM_PID=$!
    sleep 10  # 等待系统完全启动
    
    log_success "SLAM系统启动完成 (PID: $SLAM_PID)"
    
    # 数据包播放
    log_info "开始播放数据包..."
    local metadata_file="$DATASET_PATH/metadata.yaml"
    
    if [[ -f "$metadata_file" ]]; then
        log_info "使用ROS2 bag播放: $DATASET_PATH"
        log_info "播放速率: ${play_rate}x"
        
        # 播放数据包
        ros2 bag play "$DATASET_PATH" \
            --remap /livox/pointcloud_livox_frame_front:=/livox/pointcloud_livox_frame_front \
            --remap /livox/imu_livox_frame_front:=/livox/imu_lixox_frame_front \
            --rate "$play_rate" \
            --clock &
        
        BAG_PID=$!
        log_success "数据包播放启动完成 (PID: $BAG_PID)"
    else
        log_error "未找到metadata.yaml文件，无法播放数据包"
        exit 1
    fi
}

# 清理标志，防止重复执行
cleanup_executed=false

cleanup() {
    # 如果已经执行过清理，直接返回
    if [[ "$cleanup_executed" == "true" ]]; then
        return 0
    fi
    cleanup_executed=true
    
    log_step 7 "系统清理"
    
    echo ""
    echo "正在执行彻底清理..."
    
    # 获取地图文件数量
    local output_dir="$MAPS_ROOT/$DATASET"
    local map_count=0
    if [[ -d "$output_dir" ]]; then
        map_count=$(find "$output_dir" -name "*.pcd" -o -name "*.ply" | wc -l)
    fi
    
    # 获取当前会话的所有相关进程ID
    local pids_to_kill=$(pgrep -f "ros2|rviz|lidar_slam|outdoor_mapping" | tr '\n' ' ')
    
    # 显示当前运行的进程（已移除，保持输出简洁）
    
    # 1. 首先停止数据包播放（防止继续输入数据）
    if [[ -n "$BAG_PID" ]] && kill -0 "$BAG_PID" 2>/dev/null; then
        log_info "停止数据包播放进程 (PID: $BAG_PID)..."
        kill -TERM "$BAG_PID" 2>/dev/null || true
        sleep 2
        kill -KILL "$BAG_PID" 2>/dev/null || true
    fi
    
    # 2. 停止RViz（先关闭可视化，避免界面卡顿）
    log_info "停止RViz..."
    pkill -TERM -f "rviz2" 2>/dev/null || true
    sleep 2
    pkill -KILL -f "rviz2" 2>/dev/null || true
    
    # 3. 等待SLAM节点自动完成最终地图保存
    local output_dir="$MAPS_ROOT/$DATASET"
    
    log_info "等待SLAM节点自动完成最终地图保存..."
    
    # 获取当前地图文件数量作为参考
    local map_count_before=0
    if [[ -d "$output_dir" ]]; then
        map_count_before=$(find "$output_dir" -name "*.pcd" -o -name "*.ply" | wc -l)
    fi
    
    # 给SLAM节点一些时间完成自动保存
    sleep 5
    
    # 显示最终地图统计
    if [[ -d "$output_dir" ]]; then
        local map_count_after=$(find "$output_dir" -name "*.pcd" -o -name "*.ply" | wc -l)
        local final_maps=$(find "$output_dir" -name "final_map_*" | wc -l)
        log_info "地图文件统计: 建图前 $map_count_before → 建图后 $map_count_after 个文件"
        [[ $final_maps -gt 0 ]] && log_success "已生成 $final_maps 个最终地图文件"
    fi
    
    # 4. 逐个关闭主要进程，避免终止脚本本身
    log_info "关闭SLAM相关进程..."
    # 临时禁用信号处理，避免进程关闭触发清理
    trap - TERM INT
    
    # 5. 关闭所有ROS2相关进程
    log_info "关闭所有ROS2相关进程..."
    # 临时禁用信号处理，避免进程关闭触发清理
    trap - TERM INT
    
    # 优先关闭数据包播放进程
    if [[ -n "$BAG_PID" ]] && kill -0 "$BAG_PID" 2>/dev/null; then
        log_info "停止数据包播放进程 (PID: $BAG_PID)..."
        kill -TERM "$BAG_PID" 2>/dev/null || true
        sleep 2
    fi
    
    # 关闭其他相关进程
    pkill -TERM -f "rviz2" 2>/dev/null || true
    pkill -TERM -f "ros2 launch" 2>/dev/null || true
    pkill -TERM -f "ros2 bag" 2>/dev/null || true
    pkill -TERM -f "lidar_slam" 2>/dev/null || true
    
    # 等待进程退出
    sleep 3
    
    # 6. 强制清理剩余进程 - 使用更精确的进程识别
    log_info "强制清理剩余进程..."
    local processes=("rviz2" "ros2" "lidar_slam" "outdoor_mapping" "python3.*outdoor_mapping")
    
    for proc in "${processes[@]}"; do
        if pgrep -f "$proc" > /dev/null; then
            log_info "强制关闭 $proc 相关进程..."
            pkill -KILL -f "$proc" 2>/dev/null || true
            sleep 1
        fi
    done
    
    # 确保所有ROS2节点停止
    ros2 daemon stop 2>/dev/null || true
    
    # 8. 检查并清理任何剩余的Python进程
    local python_pids=$(pgrep -f "python3.*ros" | tr '\n' ' ')
    if [[ -n "$python_pids" ]]; then
        log_info "清理剩余Python进程: $python_pids"
        kill -KILL $python_pids 2>/dev/null || true
    fi
    
    # 9. 清理可能的僵尸进程
    wait 2>/dev/null || true
    
    # 10. 确认所有进程已关闭
    sleep 2
    local remaining_processes=$(ps aux | grep -E "(ros2|rviz|lidar_slam|outdoor_mapping)" | grep -v grep | wc -l)
    if [[ $remaining_processes -eq 0 ]]; then
        log_success "所有ROS2相关进程已正确关闭"
    else
        log_warning "检测到 $remaining_processes 个进程仍在运行，执行最终清理..."
        pkill -KILL -f "ros2|rviz|lidar_slam|outdoor_mapping" 2>/dev/null || true
        sleep 2
    fi
    
    # 11. 清理完成，显示成功消息
    log_success "建图任务完成"
    
    # 显示结果
    echo ""
    echo "结果统计:"
    echo "  数据集: $DATASET"
    
    # 重新统计地图文件数量
    if [[ -d "$output_dir" ]]; then
        map_count=$(find "$output_dir" -name "*.pcd" -o -name "*.ply" | wc -l)
        final_maps=$(find "$output_dir" -name "final_map_*" | wc -l)
    fi
    
    echo "  地图文件: $map_count 个"
    echo "  最终地图: $final_maps 个"
    echo "  输出目录: $MAPS_ROOT/$DATASET"
    echo "  完成时间: $(date)"
    echo ""
    
    # 最终进程状态确认
    local remaining_processes=$(ps aux | grep -E "(ros2|rviz|lidar_slam|outdoor_mapping)" | grep -v grep | wc -l)
    if [[ $remaining_processes -gt 0 ]]; then
        log_warning "检测到 $remaining_processes 个进程仍在运行，执行最终清理..."
        pkill -KILL -f "ros2|rviz|lidar_slam|outdoor_mapping" 2>/dev/null || true
        sleep 2
    else
        log_success "所有进程已成功清理"
    fi
    
    if [[ -d "$output_dir" ]]; then
        echo ""
        echo "地图文件列表:"
        echo "--- 最终地图 ---"
        ls -lh "$output_dir"/final_map_* 2>/dev/null || echo "  (无最终地图文件)"
        echo ""
        echo "--- 所有地图文件 ---"
        ls -lh "$output_dir"/*.pcd "$output_dir"/*.ply 2>/dev/null || echo "  (目录为空)"
    fi
}

# 设置信号处理，确保Ctrl+C能够立即响应
setup_signal_handlers() {
    trap 'echo -e "\n${RED}[INTERRUPT]${NC} 检测到中断信号，开始清理..."; cleanup; exit 130' INT
    trap 'echo -e "\n${RED}[TERMINATE]${NC} 检测到终止信号，开始清理..."; cleanup; exit 143' TERM
    trap 'cleanup' EXIT
}

# 主流程
main() {
    # 设置信号处理
    setup_signal_handlers
    
    # 执行主流程
    validate_dataset "$DATASET_INPUT"
    
    # 在验证数据集后设置路径变量
    CONFIG_FILE="${DATASET}_mapping.yaml"
    RVIZ_FILE="${DATASET}_mapping.rviz"
    DATASET_PATH="$DATA_ROOT/$DATASET"
    
    # 重新生成调试信息
    log_info "=== 路径调试信息 ==="
    log_info "工作空间根目录: $WORKSPACE_ROOT"
    log_info "当前工作目录: $(pwd)"
    log_info "原始输入: $DATASET_INPUT"
    log_info "映射后数据集: $DATASET"
    log_info "配置文件名: $CONFIG_FILE"
    log_info "配置文件完整路径: $CONFIG_PATH/$CONFIG_FILE"
    log_info "配置目录是否存在: $(test -d "$CONFIG_PATH" && echo "是" || echo "否")"
    log_info "数据集路径: $DATASET_PATH"
    log_info "数据集目录是否存在: $(test -d "$DATASET_PATH" && echo "是" || echo "否")"
    log_info "=================="
    
    check_environment
    validate_files
    build_workspace
    load_environment
    prepare_output
    start_mapping
    
    # 等待数据包播放完成
    echo ""
    echo "系统运行中... 等待数据包播放完成"
    
    # 等待数据包播放进程结束
    if [[ -n "$BAG_PID" ]]; then
        log_info "等待数据包播放完成..."
        wait $BAG_PID
        log_success "数据包播放完成"
        
        # 数据包播放完成后，主动调用 cleanup
        log_success "数据包播放完成，系统即将自动清理..."
        sleep 3
        cleanup
    else
        echo "按 Ctrl+C 停止系统..."
        # 等待用户中断
        while true; do
            sleep 1
        done
    fi
    # 建图任务完成的消息会在cleanup中显示
}

# 检查是否为直接运行
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi