#!/usr/bin/env python3
# 坐标变换工具模块 提供旋转矩阵与四元数转换等功能

import numpy as np
from geometry_msgs.msg import TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

class TransformUtils:
    """坐标变换工具类"""
    @staticmethod
    def matrix_to_quaternion(matrix):
        """
        将4x4变换矩阵转换为四元数
        Args:
            matrix: 4x4 numpy数组，表示变换矩阵
        Returns:
            tuple: (x, y, z, w) 四元数分量
        """
        rotation_matrix = matrix[:3, :3]
        trace = np.trace(rotation_matrix)
        
        if trace > 0:
            s = 2.0 * np.sqrt(trace + 1.0)
            w = 0.25 * s
            x = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
            y = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
            z = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
        else:
            if rotation_matrix[0, 0] > rotation_matrix[1, 1] and rotation_matrix[0, 0] > rotation_matrix[2, 2]:
                s = 2.0 * np.sqrt(1.0 + rotation_matrix[0, 0] - rotation_matrix[1, 1] - rotation_matrix[2, 2])
                w = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
                x = 0.25 * s
                y = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
                z = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
            elif rotation_matrix[1, 1] > rotation_matrix[2, 2]:
                s = 2.0 * np.sqrt(1.0 + rotation_matrix[1, 1] - rotation_matrix[0, 0] - rotation_matrix[2, 2])
                w = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
                x = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
                y = 0.25 * s
                z = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + rotation_matrix[2, 2] - rotation_matrix[0, 0] - rotation_matrix[1, 1])
                w = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
                x = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
                y = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
                z = 0.25 * s
        
        # 归一化
        norm = np.sqrt(x*x + y*y + z*z + w*w)
        if norm > 0:
            x, y, z, w = x/norm, y/norm, z/norm, w/norm
        
        return x, y, z, w
    
    @staticmethod
    def create_transform_stamped(matrix, parent_frame, child_frame, timestamp=None):
        """
        从变换矩阵创建TransformStamped消息
        Args:
            matrix: 4x4 numpy数组，表示变换矩阵
            parent_frame: 父坐标系名称
            child_frame: 子坐标系名称
            timestamp: 时间戳，如果为None则使用当前时间
        Returns:
            TransformStamped消息
        """
        if timestamp is None:
            from rclpy.time import Time
            timestamp = Time().to_msg()
        
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        
        # 设置平移
        t.transform.translation.x = float(matrix[0, 3])
        t.transform.translation.y = float(matrix[1, 3])
        t.transform.translation.z = float(matrix[2, 3])
        
        # 设置旋转
        x, y, z, w = TransformUtils.matrix_to_quaternion(matrix)
        t.transform.rotation.x = x
        t.transform.rotation.y = y
        t.transform.rotation.z = z
        t.transform.rotation.w = w
        
        return t
    
    @staticmethod
    def create_odometry_msg(matrix, parent_frame, child_frame, timestamp=None):
        """
        从变换矩阵创建Odometry消息
        Args:
            matrix: 4x4 numpy数组，表示变换矩阵
            parent_frame: 父坐标系名称
            child_frame: 子坐标系名称
            timestamp: 时间戳，如果为None则使用当前时间
        Returns:
            Odometry消息
        """
        if timestamp is None:
            from rclpy.time import Time
            timestamp = Time().to_msg()
        
        odom = Odometry()
        odom.header.stamp = timestamp
        odom.header.frame_id = parent_frame
        odom.child_frame_id = child_frame
        
        # 设置位置
        odom.pose.pose.position.x = float(matrix[0, 3])
        odom.pose.pose.position.y = float(matrix[1, 3])
        odom.pose.pose.position.z = float(matrix[2, 3])
        
        # 设置方向
        x, y, z, w = TransformUtils.matrix_to_quaternion(matrix)
        odom.pose.pose.orientation.x = x
        odom.pose.pose.orientation.y = y
        odom.pose.pose.orientation.z = z
        odom.pose.pose.orientation.w = w
        
        return odom