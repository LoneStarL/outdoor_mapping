#!/bin/bash
# 修复依赖问题
source /opt/ros/humble/setup.bash
source /home/lonestarl/ws/mapping_ws/install/setup.bash

# 安装缺失的依赖
echo "安装Open3D..."
sudo apt-get update
sudo apt-get install -y python3-pip
pip3 install open3d

# 验证安装
python3 -c "import open3d; print('Open3D已安装:', open3d.__version__)"
