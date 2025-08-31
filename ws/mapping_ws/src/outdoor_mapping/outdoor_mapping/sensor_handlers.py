#!/usr/bin/env python3
# 传感器数据处理模块 处理IMU、GPS、里程计等传感器数据

import time
from collections import deque
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import TwistStamped, QuaternionStamped

class SensorDataHandler:
    """传感器数据处理基类"""
    def __init__(self, max_history=100):
        self.data = deque(maxlen=max_history)
        self.last_time = None
    
    def add_data(self, data):
        """添加数据到历史记录"""
        self.data.append(data)
        self.last_time = time.time()
    
    def get_latest(self):
        """获取最新数据"""
        return self.data[-1] if self.data else None
    
    def get_history(self):
        """获取所有历史数据"""
        return list(self.data)
    
    def is_data_available(self):
        """检查是否有可用数据"""
        return len(self.data) > 0

class IMUHandler(SensorDataHandler):
    """IMU数据处理类"""
    def __init__(self, max_history=100):
        super().__init__(max_history)
    
    def process_imu_message(self, msg):
        """处理IMU消息"""
        data = {
            'time': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'orientation': [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w],
            'angular_velocity': [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            'linear_acceleration': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        }
        self.add_data(data)

class GPSHandler(SensorDataHandler):
    """GPS数据处理类"""
    def __init__(self, max_history=10):
        super().__init__(max_history)
        self.initial_position = None
    
    def process_gps_message(self, msg):
        """处理GPS消息"""
        if self.initial_position is None:
            self.initial_position = [msg.latitude, msg.longitude, msg.altitude]
        
        data = {
            'time': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'altitude': msg.altitude,
            'status': msg.status.status
        }
        self.add_data(data)
    
    def get_initial_position(self):
        """获取初始GPS位置"""
        return self.initial_position

class OdometryHandler(SensorDataHandler):
    """里程计数据处理类"""
    def __init__(self, max_history=100):
        super().__init__(max_history)
    
    def process_velocity_message(self, msg):
        """处理速度消息"""
        data = {
            'time': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'linear_velocity': [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z],
            'angular_velocity': [msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z]
        }
        self.add_data(data)
    
    def process_heading_message(self, msg):
        """处理航向消息"""
        data = {
            'time': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'heading': [msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w]
        }
        self.add_data(data)
    
    def get_initial_guess_from_odometry(self, previous_pose, dt=0.1):
        """
        基于里程计数据计算初始估计
        Args:
            previous_pose: 上一帧的4x4变换矩阵
            dt: 时间间隔
        Returns:
            4x4变换矩阵，表示初始估计
        """
        initial_guess = previous_pose.copy()
        
        if self.is_data_available():
            latest_odom = self.get_latest()
            
            # 使用速度积分
            if 'linear_velocity' in latest_odom:
                vx, vy, vz = latest_odom['linear_velocity']
                initial_guess[0, 3] += vx * dt
                initial_guess[1, 3] += vy * dt
                initial_guess[2, 3] += vz * dt
        
        return initial_guess

class SensorFusion:
    """传感器融合类"""
    def __init__(self, use_imu=True, use_gps=True, use_odom=True):
        self.use_imu = use_imu
        self.use_gps = use_gps
        self.use_odom = use_odom
        
        self.imu_handler = IMUHandler() if use_imu else None
        self.gps_handler = GPSHandler() if use_gps else None
        self.odom_handler = OdometryHandler() if use_odom else None
    
    def process_imu(self, msg):
        """处理IMU数据"""
        if self.imu_handler:
            self.imu_handler.process_imu_message(msg)
    
    def process_gps(self, msg):
        """处理GPS数据"""
        if self.gps_handler:
            self.gps_handler.process_gps_message(msg)
    
    def process_velocity(self, msg):
        """处理速度数据"""
        if self.odom_handler:
            self.odom_handler.process_velocity_message(msg)
    
    def process_heading(self, msg):
        """处理航向数据"""
        if self.odom_handler:
            self.odom_handler.process_heading_message(msg)
    
    def get_initial_guess(self, previous_pose, current_time):
        """
        所有可用传感器数据计算初始估计
        Args:
            previous_pose: 上一帧的4x4变换矩阵
            current_time: 当前时间戳
        Returns:
            4x4变换矩阵，表示初始估计
        """
        # 默认使用上一帧的位姿
        initial_guess = previous_pose.copy()
        
        # 使用里程计数据
        if self.use_odom and self.odom_handler and self.odom_handler.is_data_available():
            initial_guess = self.odom_handler.get_initial_guess_from_odometry(previous_pose)
        
        return initial_guess