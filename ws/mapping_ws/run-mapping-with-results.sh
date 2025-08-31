#!/bin/bash

# ROS2建图任务运行脚本 - 修复NumPy兼容性和路径问题

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

IMAGE_NAME="outdoor-mapping-vm:humble"
DATASET=${1:-"easy"}
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
DISPLAY_MODE=${2:-"headless"}  # headless 或 gui

echo -e "${CYAN}==============================================${NC}"
echo -e "${CYAN}  ROS2建图任务运行器 ${NC}"
echo -e "${CYAN}==============================================${NC}"

# 检查镜像
if ! sudo docker images | grep -q "outdoor-mapping-vm"; then
    echo -e "${RED}错误: 镜像未找到，请先加载镜像${NC}"
    echo "   sudo docker load -i outdoor-mapping-vm_20250821_172011.tar"
    exit 1
fi

# 检查Open3D依赖
echo -e "${CYAN}🔍 检查依赖...${NC}"
sudo docker run --rm $IMAGE_NAME python3 -c "import open3d; print('Open3D版本:', open3d.__version__)" 2>/dev/null || {
    echo -e "${YELLOW}  发现NumPy兼容性问题，正在修复...${NC}"
    
    # 创建修复的容器
    cat > ./fix-open3d-container.sh << 'EOF'
#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /home/lonestarl/ws/mapping_ws/install/setup.bash

# 修复APT源问题
sudo rm -f /etc/apt/sources.list.d/borglab-gtsam-release-4-0-jammy.list || true
sudo rm -f /etc/apt/sources.list.d/gtsam* || true
sudo rm -f /etc/apt/sources.list.d/ros2-latest.list || true
sudo apt-get update -qq --allow-releaseinfo-change || true

# 配置国内pip源
mkdir -p ~/.pip
cat > ~/.pip/pip.conf << 'PIPCONF'
[global]
index-url = https://pypi.tuna.tsinghua.edu.cn/simple
extra-index-url = https://mirrors.aliyun.com/pypi/simple/
[install]
trusted-host = pypi.tuna.tsinghua.edu.cn
               mirrors.aliyun.com
PIPCONF

echo "=== 清理旧版本 ==="
# 清理可能冲突的包
pip3 uninstall -y numpy scipy scikit-learn open3d 2>/dev/null || true

echo "=== 安装兼容版本 ==="
# 安装兼容的NumPy版本（<2.0）
pip3 install --user --upgrade pip setuptools wheel -i https://pypi.tuna.tsinghua.edu.cn/simple
pip3 install --user "numpy<1.25.0" -i https://pypi.tuna.tsinghua.edu.cn/simple

# 安装兼容的scipy和scikit-learn
pip3 install --user "scipy>=1.7.0,<1.11.0" -i https://pypi.tuna.tsinghua.edu.cn/simple
pip3 install --user "scikit-learn>=1.0.0,<1.4.0" -i https://pypi.tuna.tsinghua.edu.cn/simple

# 安装Open3D
pip3 install --user open3d -i https://pypi.tuna.tsinghua.edu.cn/simple

echo "=== 验证安装 ==="
# 验证所有依赖
python3 -c "
import numpy
print('NumPy版本:', numpy.__version__)

import scipy
print('SciPy版本:', scipy.__version__)

import sklearn
print('scikit-learn版本:', sklearn.__version__)

try:
    import open3d
    print('Open3D版本:', open3d.__version__)
    print(' 所有依赖安装成功')
except Exception as e:
    print(' Open3D导入失败:', e)
    exit(1)
"

# 如果仍然失败，尝试最小化安装
echo "=== 备用方案 ==="
python3 -c "import open3d" 2>/dev/null || {
    echo "使用Open3D最小化安装..."
    pip3 uninstall -y open3d 2>/dev/null || true
    pip3 install --user --no-deps open3d==0.16.0 -i https://pypi.tuna.tsinghua.edu.cn/simple
    pip3 install --user "numpy<1.25.0" "scipy<1.11.0" -i https://pypi.tuna.tsinghua.edu.cn/simple
}
EOF
    
    chmod +x ./fix-open3d-container.sh
    sudo docker run --rm -v $(pwd)/fix-open3d-container.sh:/fix-open3d-container.sh $IMAGE_NAME bash /fix-open3d-container.sh
    echo -e "${GREEN} 依赖已修复${NC}"
}

# 创建本地结果目录
echo -e "${CYAN} 创建结果目录...${NC}"
mkdir -p ./mapping-results/$TIMESTAMP/{maps,logs,data}

# 显示可用数据集
echo -e "${CYAN} 可用数据集:${NC}"
sudo docker run --rm $IMAGE_NAME ls maps/

echo -e "${CYAN}s 运行数据集: $DATASET${NC}"
echo -e "${CYAN}显示模式: $DISPLAY_MODE${NC}"
echo -e "${CYAN}结果保存到: ./mapping-results/$TIMESTAMP/${NC}"

# 创建容器内运行脚本内容，通过环境变量传递
cat > ./run-container-mapping-local.sh << EOF
#!/bin/bash
set -e

# 配置国内pip源
mkdir -p ~/.pip
cat > ~/.pip/pip.conf << 'PIPCONF'
[global]
index-url = https://pypi.tuna.tsinghua.edu.cn/simple
extra-index-url = https://mirrors.aliyun.com/pypi/simple/
[install]
trusted-host = pypi.tuna.tsinghua.edu.cn
               mirrors.aliyun.com
PIPCONF

echo '=== 环境检查 ==='
source /opt/ros/humble/setup.bash
source /home/lonestarl/ws/mapping_ws/install/setup.bash

echo '=== 验证依赖 ==='
python3 -c "
import numpy
print('NumPy版本:', numpy.__version__)

import scipy
print('SciPy版本:', scipy.__version__)

try:
    import open3d
    print('Open3D版本:', open3d.__version__)
    print(' 所有依赖兼容')
except Exception as e:
    print(' 依赖问题:', e)
    print('尝试重新安装...')
    import subprocess
    subprocess.run(['pip3', 'install', '--user', '--force-reinstall', 'numpy<1.25.0', 'open3d', '-i', 'https://pypi.tuna.tsinghua.edu.cn/simple'])
"

echo '=== 运行建图 ==='
echo "数据集: $DATASET"
echo "模式: $DISPLAY_MODE"

if [[ "$DISPLAY_MODE" == "headless" ]]; then
    echo '使用无头模式运行...'
    # 修改launch文件禁用rviz2
    sed -i '/rviz2_node/s/^/# /' /home/lonestarl/ws/mapping_ws/src/outdoor_mapping/launch/mapping.launch.py || true
fi

# 运行建图
cd /home/lonestarl/ws/mapping_ws
./run_mapping.sh $DATASET

echo '=== 任务完成 ==='
echo '结果文件:'
ls -la /home/lonestarl/ws/mapping_ws/results/
EOF

chmod +x ./run-container-mapping-local.sh

# 构建Docker运行命令
DOCKER_CMD="sudo docker run -it --rm --name \"mapping-$TIMESTAMP\""

# 添加挂载卷
DOCKER_CMD="$DOCKER_CMD -v \$(pwd)/mapping-results/$TIMESTAMP/maps:/home/lonestarl/ws/mapping_ws/results"
DOCKER_CMD="$DOCKER_CMD -v \$(pwd)/mapping-results/$TIMESTAMP/logs:/home/lonestarl/ws/mapping_ws/logs"
DOCKER_CMD="$DOCKER_CMD -v \$(pwd)/mapping-results/$TIMESTAMP/data:/home/lonestarl/ws/mapping_ws/data"
DOCKER_CMD="$DOCKER_CMD -v \$(pwd)/run-container-mapping-local.sh:/run-container-mapping-local.sh"

# 处理显示模式
if [[ "$DISPLAY_MODE" == "gui" ]]; then
    # GUI模式
    if [[ -n "$DISPLAY" ]]; then
        DOCKER_CMD="$DOCKER_CMD -e DISPLAY=$DISPLAY"
        if [[ "$(uname)" == "Linux" ]]; then
            DOCKER_CMD="$DOCKER_CMD -v /tmp/.X11-unix:/tmp/.X11-unix"
        fi
    else
        echo -e "${YELLOW}  DISPLAY未设置，使用无头模式${NC}"
        DISPLAY_MODE="headless"
    fi
fi

# 添加环境变量避免X11问题
DOCKER_CMD="$DOCKER_CMD -e QT_QPA_PLATFORM=offscreen"
DOCKER_CMD="$DOCKER_CMD -e LIBGL_ALWAYS_SOFTWARE=1"
DOCKER_CMD="$DOCKER_CMD -e PYTHONPATH=/home/lonestarl/.local/lib/python3.10/site-packages"

# 完成命令
DOCKER_CMD="$DOCKER_CMD $IMAGE_NAME"

# 运行容器
echo -e "${YELLOW} 启动建图任务...${NC}"
eval $DOCKER_CMD bash /run-container-mapping-local.sh

echo -e "${GREEN}==============================================${NC}"
echo -e "${GREEN}   建图任务完成!${NC}"
echo -e "${GREEN}==============================================${NC}"

echo -e "${CYAN} 结果文件位置:${NC}"
echo "  主目录: ./mapping-results/$TIMESTAMP/"
echo
echo -e "${CYAN} 文件结构:${NC}"
ls -la ./mapping-results/$TIMESTAMP/

echo -e "${CYAN}  地图文件:${NC}"
ls -la ./mapping-results/$TIMESTAMP/maps/

echo -e "${CYAN} 日志文件:${NC}"
ls -la ./mapping-results/$TIMESTAMP/logs/

echo -e "${CYAN} 使用说明:${NC}"
echo "1. 无头模式运行:"
echo "   ./run-mapping-with-results.sh easy"
echo
echo "2. GUI模式运行(需要X11):"
echo "   ./run-mapping-with-results.sh easy gui"
echo
echo "3. 查看结果:"
echo "   ls -la ./mapping-results/$TIMESTAMP/"