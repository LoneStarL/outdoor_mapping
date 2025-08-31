#!/usr/bin/env python3
# 点云处理工具模块 提供点云转换、处理和可视化功能

import numpy as np
import open3d as o3d
import struct
import math
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

class PointCloudUtils:
    """点云处理工具类"""
    @staticmethod
    def ros_to_numpy(msg):
        """
        将ROS PointCloud2消息转换为numpy数组
        Args:
            msg: PointCloud2消息
        Returns:
            numpy数组，形状为(N, 3)
        """
        try:
            fmt = 'ffff'  # x, y, z, intensity
            width = msg.width
            height = msg.height
            point_step = msg.point_step
            row_step = msg.row_step
            
            points = []
            for i in range(height):
                for j in range(width):
                    offset = i * row_step + j * point_step
                    if offset + 16 <= len(msg.data):
                        x, y, z, intensity = struct.unpack_from(fmt, msg.data, offset)
                        if not (math.isnan(x) or math.isnan(y) or math.isnan(z)):
                            points.append([x, y, z])
            
            return np.array(points) if points else None
            
        except Exception as e:
            print(f'Error converting point cloud: {str(e)}')
            return None
    
    @staticmethod
    def numpy_to_ros(points, frame_id, timestamp=None):
        """
        将numpy数组转换为ROS PointCloud2消息
        Args:
            points: numpy数组，形状为(N, 3)
            frame_id: 坐标系名称
            timestamp: 时间戳，如果为None则使用当前时间
        Returns:
            PointCloud2消息
        """
        if points is None or len(points) == 0:
            return None
            
        try:
            from rclpy.time import Time
            if timestamp is None:
                timestamp = Time().to_msg()
            
            header = Header()
            header.stamp = timestamp
            header.frame_id = frame_id
            
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            
            point_data = points.astype(np.float32).tobytes()
            
            map_msg = PointCloud2()
            map_msg.header = header
            map_msg.height = 1
            map_msg.width = len(points)
            map_msg.fields = fields
            map_msg.is_bigendian = False
            map_msg.point_step = 12
            map_msg.row_step = map_msg.point_step * len(points)
            map_msg.is_dense = True
            map_msg.data = point_data
            
            return map_msg
            
        except Exception as e:
            print(f'Error creating PointCloud2 message: {str(e)}')
            return None
    
    @staticmethod
    def preprocess_pointcloud(points, voxel_size=0.05):
        """
        预处理点云：下采样和去噪
        Args:
            points: numpy数组，形状为(N, 3)
            voxel_size: 体素下采样大小
        Returns:
            处理后的Open3D点云对象
        """
        if points is None or len(points) == 0:
            return None
            
        # 创建Open3D点云对象
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        
        # 下采样
        pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
        
        # 去除离群点
        pcd, _ = pcd.remove_statistical_outlier(
            nb_neighbors=20, std_ratio=2.0
        )
        
        return pcd
    
    @staticmethod
    def merge_pointclouds(pcd_list, voxel_size=0.05):
        """
        合并多个点云
        Args:
            pcd_list: Open3D点云对象列表
            voxel_size: 合并后的体素下采样大小
        Returns:
            合并后的Open3D点云对象
        """
        if not pcd_list:
            return None
            
        merged = pcd_list[0]
        for pcd in pcd_list[1:]:
            merged += pcd
        
        # 合并后下采样
        merged = merged.voxel_down_sample(voxel_size=voxel_size)
        
        return merged