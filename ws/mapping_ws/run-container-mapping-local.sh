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
echo "数据集: easy"
echo "模式: headless"

if [[ "headless" == "headless" ]]; then
    echo '使用无头模式运行...'
    # 修改launch文件禁用rviz2
    sed -i '/rviz2_node/s/^/# /' /home/lonestarl/ws/mapping_ws/src/outdoor_mapping/launch/mapping.launch.py || true
fi

# 运行建图
cd /home/lonestarl/ws/mapping_ws
./run_mapping.sh easy

echo '=== 任务完成 ==='
echo '结果文件:'
ls -la /home/lonestarl/ws/mapping_ws/results/
