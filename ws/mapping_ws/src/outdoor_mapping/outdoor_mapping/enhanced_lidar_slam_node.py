#!/usr/bin/env python3
# 增强版LiDAR SLAM节点 - 支持多传感器融合

import os
import time
from datetime import datetime
import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import PointCloud2, Imu, NavSatFix
from geometry_msgs.msg import TwistStamped, QuaternionStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
import tf2_ros
from tf2_ros import TransformBroadcaster

from .transform_utils import TransformUtils
from .pointcloud_utils import PointCloudUtils
from .sensor_handlers import SensorFusion
from .map_manager import MapManager

class EnhancedLidarSLAMNode(Node):
    def __init__(self):
        super().__init__('enhanced_lidar_slam_node')
        
        # 参数声明
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('lidar_frame', 'livox_frame_front')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('max_correspondence_distance', 0.8)
        self.declare_parameter('max_iteration', 50)
        self.declare_parameter('save_map_interval', 30.0)  
        self.declare_parameter('output_dir', 'maps')

        self.declare_parameter('use_imu', True)
        self.declare_parameter('use_gps', True)
        self.declare_parameter('use_odom', True)
        self.declare_parameter('dataset', 'default') 
        
        # 获取参数
        self.map_frame = self.get_parameter('map_frame').value
        self.lidar_frame = self.get_parameter('lidar_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.voxel_size = self.get_parameter('voxel_size').value
        self.max_correspondence_distance = self.get_parameter('max_correspondence_distance').value
        self.max_iteration = self.get_parameter('max_iteration').value
        self.save_map_interval = self.get_parameter('save_map_interval').value
        self.output_dir = self.get_parameter('output_dir').value
        self.use_imu = self.get_parameter('use_imu').value
        self.use_gps = self.get_parameter('use_gps').value
        self.use_odom = self.get_parameter('use_odom').value
        
        # 创建输出目录
        # 获取ROS2工作空间根目录
        workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
        
        # 获取数据集名称参数
        dataset_name = self.get_parameter('dataset').value
        
        # 构建输出路径
        self.output_dir = os.path.join(workspace_root, 'maps', dataset_name)
        os.makedirs(self.output_dir, exist_ok=True)
        
        self.get_logger().info(f'Output directory: {self.output_dir}')
        self.get_logger().info(f'Map frame: {self.map_frame}')
        self.get_logger().info(f'LiDAR frame: {self.lidar_frame}')
        self.get_logger().info(f'Using IMU: {self.use_imu}')
        self.get_logger().info(f'Using GPS: {self.use_gps}')
        self.get_logger().info(f'Using Odom: {self.use_odom}')
        
        # TF相关
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 核心状态
        self.map_pointcloud = None
        self.current_scan = None
        self.pose = np.eye(4)  # 初始位姿为单位矩阵
        self.previous_pose = np.eye(4)
        
        # 传感器融合
        self.sensor_fusion = SensorFusion(self.use_imu, self.use_gps, self.use_odom)
        
        # 地图管理
        self.map_manager = MapManager(self.output_dir, self.get_logger())
        
        # 退化检测
        self.degeneracy_threshold = 0.1
        self.consecutive_degenerate = 0
        self.use_gps_fallback = False
        
        # 内存管理参数
        self.max_map_points = self.get_parameter('max_map_points').get_parameter_value().integer_value if self.has_parameter('max_map_points') else 1000000
        self.enable_compression = self.get_parameter('enable_compression').get_parameter_value().bool_value if self.has_parameter('enable_compression') else False
        
        # 状态计数器
        self.scan_count = 0
        self.is_initialized = False
        self.initial_gps = None
        self.last_save_time = time.time()  # 初始化最后保存时间
        
        # QoS配置
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 订阅点云数据
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/livox/pointcloud_livox_frame_front',
            self.pointcloud_callback,
            qos_profile
        )
        
        # 订阅IMU数据
        if self.use_imu:
            self.imu_sub = self.create_subscription(
                Imu,
                '/livox/imu_livox_frame_front',
                self.imu_callback,
                qos_profile
            )
        
        # 订阅GPS数据
        if self.use_gps:
            self.gps_sub = self.create_subscription(
                NavSatFix,
                '/fix',
                self.gps_callback,
                qos_profile
            )
        
        # 订阅里程计数据
        if self.use_odom:
            self.vel_sub = self.create_subscription(
                TwistStamped,
                '/vel',
                self.vel_callback,
                qos_profile
            )
            
            self.heading_sub = self.create_subscription(
                QuaternionStamped,
                '/heading',
                self.heading_callback,
                qos_profile
            )
        
        # 发布地图点云
        self.map_publisher = self.create_publisher(
            PointCloud2,
            '/map_pointcloud',
            10
        )
        
        # 发布里程计
        self.odom_publisher = self.create_publisher(
            Odometry,
            '/slam_odom',
            10
        )
        
        # 定期保存地图
        self.save_timer = self.create_timer(
            self.save_map_interval,
            self.save_map_callback
        )
        
        # 保存最终地图
        self.save_final_map_service = self.create_service(
            Trigger,
            '/lidar_slam/save_final_map',
            self.save_final_map_service_callback
        )
        

        
        self.get_logger().info('Enhanced Lidar SLAM Node initialized')
    
    def imu_callback(self, msg):
        """处理IMU数据"""
        self.sensor_fusion.process_imu(msg)

    def gps_callback(self, msg):
        """处理GPS数据"""
        self.sensor_fusion.process_gps(msg)

    def vel_callback(self, msg):
        """处理速度数据"""
        self.sensor_fusion.process_velocity(msg)

    def heading_callback(self, msg):
        """处理航向数据"""
        self.sensor_fusion.process_heading(msg)
    
    def pointcloud_callback(self, msg):
        """处理接收到的点云数据"""
        try:
            current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
            # 将ROS PointCloud2转换为numpy数组
            points = PointCloudUtils.ros_to_numpy(msg)
            if points is None or len(points) == 0:
                self.get_logger().warn('Received empty point cloud')
                return
                
            # 预处理点云
            pcd = PointCloudUtils.preprocess_pointcloud(points, self.voxel_size)
            if pcd is None:
                return
            
            if not self.is_initialized:
                # 第一次扫描，初始化地图
                self.map_pointcloud = pcd
                self.is_initialized = True
                self.get_logger().info('Map initialized with first scan')
            else:
                # 执行ICP配准
                initial_guess = self.sensor_fusion.get_initial_guess(self.previous_pose, current_time)
                success = self.perform_enhanced_icp_registration(pcd, initial_guess)
                
                if not success:
                    self.get_logger().warn('ICP failed, using sensor-based estimation')
                    self.pose = initial_guess
            
            # 发布数据
            self.publish_all_data()
            
            # 定期保存地图
            self.map_manager.periodic_save(self.map_pointcloud, self.save_map_interval)
            
            self.scan_count += 1
            
            if self.scan_count % 10 == 0:
                self.get_logger().info(
                    f'Processed {self.scan_count} scans, '
                    f'Map size: {len(self.map_pointcloud.points)} points, '
                    f'Degenerate: {self.consecutive_degenerate}'
                )
                
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {str(e)}')
    
    def publish_all_data(self):
        """发布所有相关数据"""
        self.publish_map()
        self.publish_transform()
        self.publish_odometry()
    
    def perform_enhanced_icp_registration(self, current_pcd, initial_guess):
        """执行增强ICP配准，包含退化检测"""
        try:
            # 计算初始对应关系
            result = o3d.pipelines.registration.registration_icp(
                current_pcd,
                self.map_pointcloud,
                self.max_correspondence_distance,
                initial_guess,
                o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                o3d.pipelines.registration.ICPConvergenceCriteria(
                    max_iteration=self.max_iteration
                )
            )
            
            # 退化检测：检查匹配质量
            fitness = result.fitness
            inlier_rmse = result.inlier_rmse
            
            # 简单的退化检测
            is_degenerate = fitness < 0.3 or inlier_rmse > 0.5
            
            if is_degenerate:
                self.consecutive_degenerate += 1
                self.get_logger().warn(f'Degenerate case detected: fitness={fitness:.3f}, rmse={inlier_rmse:.3f}')
                
                # 如果连续退化，启用GPS回退
                if self.consecutive_degenerate > 5 and self.use_gps:
                    self.use_gps_fallback = True
                    self.get_logger().info('Enabling GPS fallback mode')
            else:
                self.consecutive_degenerate = 0
                self.pose = result.transformation
                self.previous_pose = self.pose
                
                # 将当前扫描添加到地图
                current_pcd.transform(self.pose)
                self.map_pointcloud += current_pcd
                
                # 再次下采样以减少点数
                self.map_pointcloud = self.map_pointcloud.voxel_down_sample(
                    voxel_size=self.voxel_size
                )
                
                # 限制最大地图点数
                if len(self.map_pointcloud.points) > self.max_map_points:
                    # 如果点数超限，使用更大的体素进行下采样
                    oversample_ratio = len(self.map_pointcloud.points) / self.max_map_points
                    adaptive_voxel = self.voxel_size * (oversample_ratio ** (1/3))
                    self.map_pointcloud = self.map_pointcloud.voxel_down_sample(
                        voxel_size=adaptive_voxel
                    )
                    self.get_logger().warn(
                        f'Map points limit exceeded: {len(self.map_pointcloud.points)} > {self.max_map_points}, '
                        f'adapted voxel size to {adaptive_voxel:.3f}m'
                    )
                
                return True
                
        except Exception as e:
            self.get_logger().error(f'Error in enhanced ICP registration: {str(e)}')
            return False
        
        return False
    

    
    def publish_map(self):
        """发布地图点云"""
        if self.map_pointcloud is None or len(self.map_pointcloud.points) == 0:
            return
        cloud_msg = PointCloudUtils.numpy_to_ros(
            np.asarray(self.map_pointcloud.points),
            self.map_frame,
            self.get_clock().now().to_msg()
        )
        self.map_publisher.publish(cloud_msg)
    
    def publish_transform(self):
        """发布TF变换"""
        try:
            transform = TransformUtils.create_transform_stamped(
                self.pose, self.map_frame, self.lidar_frame, self.get_clock().now().to_msg()
            )
            self.tf_broadcaster.sendTransform(transform)
        except Exception as e:
            self.get_logger().error(f'Error publishing transform: {str(e)}')
    
    def publish_odometry(self):
        """发布SLAM里程计"""
        try:
            odom = TransformUtils.create_odometry_msg(
                self.pose, self.map_frame, self.lidar_frame, self.get_clock().now().to_msg()
            )
            self.odom_publisher.publish(odom)
            # 保存当前位姿作为上一帧
            self.previous_pose = self.pose.copy()
        except Exception as e:
            self.get_logger().error(f'Error publishing odometry: {str(e)}')
    
    def save_map_callback(self):
        """定期保存地图"""
        if self.map_pointcloud is None:
            return
            
        try:
            current_time = time.time()
            if current_time - self.last_save_time >= self.save_map_interval:
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                
                saved_files = []
                # 保存PCD格式
                fmt = 'pcd'
                filename = f'map_{timestamp}.{fmt}'
                filepath = os.path.join(self.output_dir, filename)
                
                o3d.io.write_point_cloud(filepath, self.map_pointcloud)
                saved_files.append(filepath)
                
                if saved_files:
                    self.get_logger().info(f'Map saved: {", ".join(saved_files)}')
                
                self.last_save_time = current_time
                
        except Exception as e:
            self.get_logger().error(f'Error saving map: {str(e)}')
    
    def save_final_map_service_callback(self, request, response):
        """服务回调：保存最终地图"""
        try:
            saved_files = self.map_manager.save_final_map_direct(self.map_pointcloud)
            if saved_files:
                response.success = True
                response.message = f'Final map saved: {", ".join(saved_files)}'
            else:
                response.success = False
                response.message = 'No map data to save'
        except Exception as e:
            response.success = False
            response.message = f'Error saving final map: {str(e)}'
        return response

    def destroy_node(self):
        """节点销毁时自动保存最终地图"""
        try:
            self.get_logger().info('Node shutting down, saving final map...')
            self.save_final_map()
            self.get_logger().info('Final map saved on shutdown')
        except Exception as e:
            self.get_logger().error(f'Failed to save final map on shutdown: {str(e)}')
        super().destroy_node()
    
    def save_final_map(self):
        """保存最终地图"""
        self.map_manager.save_final_map_direct(self.map_pointcloud)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        slam_node = EnhancedLidarSLAMNode()
        
        try:
            rclpy.spin(slam_node)
        except KeyboardInterrupt:
            slam_node.get_logger().info('Keyboard interrupt received, saving final map...')
            slam_node.save_final_map()
        finally:
            slam_node.destroy_node()
            
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()