#!/usr/bin/env python3
# 地图管理模块 处理地图的保存、加载和管理

import os
import time
from datetime import datetime
import open3d as o3d
from std_srvs.srv import Trigger

class MapManager:
    """地图管理类"""
    def __init__(self, output_dir, logger=None):
        self.output_dir = output_dir
        self.logger = logger
        self.last_save_time = time.time()
        
        # 确保输出目录存在
        os.makedirs(self.output_dir, exist_ok=True)
    
    def save_map(self, pointcloud, formats=None, prefix="map"):
        """
        保存地图到文件 
        Args:
            pointcloud: Open3D点云对象
            formats: 保存格式列表，如['pcd']
            prefix: 文件名前缀
        Returns:
            list: 保存的文件路径列表
        """
        if pointcloud is None:
            if self.logger:
                self.logger.warn('No map to save')
            return []
        
        if formats is None:
            formats = ['pcd']
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        saved_files = []
        
        try:
            for fmt in formats:
                filename = f'{prefix}_{timestamp}.{fmt}'
                filepath = os.path.join(self.output_dir, filename)
                
                if fmt == 'pcd':
                    o3d.io.write_point_cloud(filepath, pointcloud)
                elif fmt == 'ply':
                    o3d.io.write_point_cloud(filepath, pointcloud)
                
                saved_files.append(filepath)
            
            if saved_files and self.logger:
                self.logger.info(f'Map saved: {", ".join(saved_files)}')
            
            return saved_files
            
        except Exception as e:
            if self.logger:
                self.logger.error(f'Error saving map: {str(e)}')
            return []
    
    def save_final_map(self, pointcloud):
        """
        保存最终地图
        Args:
            pointcloud: Open3D点云对象
        Returns:
            list: 保存的文件路径列表
        """
        return self.save_map(pointcloud, formats=['pcd', 'ply'], prefix='final_map')
    
    def save_final_map_direct(self, pointcloud):
        """
        直接保存最终地图
        Args:
            pointcloud: Open3D点云对象
        """
        if pointcloud is not None and len(pointcloud.points) > 0:
            saved_files = self.save_final_map(pointcloud)
            if self.logger and saved_files:
                self.logger.info(f'Final map saved: {", ".join(saved_files)}')
            return saved_files
        return []
    
    def periodic_save(self, pointcloud, save_interval):
        """
        定期保存地图
        Args:
            pointcloud: Open3D点云对象
            save_interval: 保存间隔（秒）
        Returns:
            bool: 是否执行了保存
        """
        current_time = time.time()
        if current_time - self.last_save_time >= save_interval:
            self.save_map(pointcloud)
            self.last_save_time = current_time
            return True
        return False
    
    def get_output_directory(self):
        """获取输出目录"""
        return self.output_dir