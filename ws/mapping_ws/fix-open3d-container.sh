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
