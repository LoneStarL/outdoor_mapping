#!/usr/bin/env python3
# LiDAR SLAM节点

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
        self.degeneracy_threshold = 0.05  # 降低阈值，减少数据丢弃
        self.consecutive_degenerate = 0
        self.use_gps_fallback = False
        self.consecutive_degenerate_limit = 10  # 增加容忍次数
        
        # 内存管理参数
        self.max_map_points = self.get_parameter('max_map_points').get_parameter_value().integer_value if self.has_parameter('max_map_points') else 50000000
        self.enable_compression = self.get_parameter('enable_compression').get_parameter_value().bool_value if self.has_parameter('enable_compression') else False
        
        # 状态计数器
        self.scan_count = 0
        self.is_initialized = False
        self.initial_gps = None
        self.last_save_time = time.time()  # 初始化最后保存时间
        
        # QoS配置
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50
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
        """处理点云数据"""
        try:
            # 获取当前时间
            current_time = self.get_clock().now()
            
            # 转换点云
            points = self.pointcloud2_to_array(msg)
            if len(points) < 100:
                self.get_logger().warn(f"点云点数过少: {len(points)}")
                return
            
            # 预处理
            voxel_size = 0.2  
            points = self.voxel_downsample(points, voxel_size)
            
            # 保留所有数据
            if len(points) > 0:
                # 直接添加到地图
                self.map_points.extend(points.tolist())
                
                # 每帧都保存
                if len(self.map_points) > 1000:
                    self.save_map()
                    
                # 全局优化
                if len(self.map_points) % 50000 == 0:
                    self.perform_global_optimization()
                    
                # 实时进度报告
                estimated_length = len(self.map_points) / 15000  # 粗略估算
                self.get_logger().info(
                    f"长轨迹建图进行中: "
                    f"点数={len(self.map_points):,}, "
                    f"估算距离={estimated_length:.0f}/650米, "
                    f"进度={min(estimated_length/650*100, 100):.1f}%"
                )
                
        except Exception as e:
            self.get_logger().error(f"点云处理错误: {e}")
            
            if len(self.map_points) > 0:
                self.save_map()

    def perform_enhanced_icp_registration(self, source, target):
        """ICP配准"""
        try:
            # 参数配置
            icp_criteria = o3d.pipelines.registration.ICPConvergenceCriteria()
            icp_criteria.max_iteration = 500
            
            # 最大匹配距离
            max_correspondence_distance = 500.0
            
            # 初始变换 - 使用GPS约束
            initial_transform = self.get_gps_correction()
            
            # 执行ICP
            result = o3d.pipelines.registration.registration_icp(
                source, target, max_correspondence_distance,
                initial_transform,
                o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                icp_criteria
            )
            
            return result.transformation, True, 0.0, 0.0
            
        except Exception as e:
            self.get_logger().warn(f"ICP失败，使用单位矩阵: {e}")
            return np.eye(4), True, 0.0, 0.0

    def get_gps_correction(self):
        """获取GPS约束"""
        try:
            if hasattr(self, 'gps_data') and self.gps_data:
                # 使用GPS数据提供初始估计
                latest_gps = self.gps_data[-1]
                
                # 计算GPS偏移
                if len(self.gps_data) > 1:
                    prev_gps = self.gps_data[-2]
                    dx = latest_gps['x'] - prev_gps['x']
                    dy = latest_gps['y'] - prev_gps['y']
                    dz = latest_gps['z'] - prev_gps['z']
                    
                    # 创建变换矩阵
                    transform = np.eye(4)
                    transform[0, 3] = dx
                    transform[1, 3] = dy
                    transform[2, 3] = dz
                    
                    return transform
                    
        except Exception as e:
            self.get_logger().debug(f"GPS约束错误: {e}")
            
        return np.eye(4)

    def save_map(self):
        """保存地图"""
        try:
            if len(self.map_points) == 0:
                return
                
            # 创建点云
            pcd = o3d.geometry.PointCloud()
            points_array = np.array(self.map_points)
            pcd.points = o3d.utility.Vector3dVector(points_array)
            
            # 最小滤波
            pcd = pcd.voxel_down_sample(voxel_size=0.1)
            
            # 保存PCD和PLY
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_dir = os.path.join(self.output_dir, "degenerative_hard")
            os.makedirs(output_dir, exist_ok=True)
            
            pcd_path = os.path.join(output_dir, f"final_map_{timestamp}.pcd")
            ply_path = os.path.join(output_dir, f"final_map_{timestamp}.ply")
            
            o3d.io.write_point_cloud(pcd_path, pcd)
            o3d.io.write_point_cloud(ply_path, pcd)
            
            # 保存中间地图用于监控
            if len(self.map_points) % 100000 == 0:
                mid_pcd_path = os.path.join(output_dir, f"map_{timestamp}.pcd")
                o3d.io.write_point_cloud(mid_pcd_path, pcd)
            
            self.get_logger().info(
                f"地图保存完成: {len(points_array):,}点, "
                f"文件大小={os.path.getsize(pcd_path)/1024/1024:.1f}MB"
            )
            
        except Exception as e:
            self.get_logger().error(f"地图保存错误: {e}")

    def perform_global_optimization(self):
        """执行全局优化"""
        try:
            if len(self.map_points) < 1000:
                return
                
            # 创建点云
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(np.array(self.map_points))
            
            # 执行全局优化
            pcd = pcd.voxel_down_sample(voxel_size=0.2)
            pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
            
            self.map_points = np.asarray(pcd.points).tolist()
            
            self.get_logger().info(
                f"全局优化完成: 优化后点数={len(self.map_points):,}"
            )
            
        except Exception as e:
            self.get_logger().error(f"全局优化错误: {e}")
    
    def pointcloud_callback(self, msg):
        """处理点云数据"""
        try:
            current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
            # 将ROS PointCloud2转换为numpy数组
            points = PointCloudUtils.ros_to_numpy(msg)
            if points is None or len(points) == 0:
                self.get_logger().warn('收到空点云')
                return
                
            # 预处理点云
            pcd = PointCloudUtils.preprocess_pointcloud(points, self.voxel_size * 1.5)
            if pcd is None:
                return
            
            if not self.is_initialized:
                # 第一次扫描，初始化地图
                self.map_pointcloud = pcd
                self.is_initialized = True
                self.get_logger().info(f'地图初始化完成，点数: {len(self.map_pointcloud.points):,}')
            else:
                # 获取初始估计
                initial_guess = self.sensor_fusion.get_initial_guess(self.previous_pose, current_time)
                
                # 执行配准
                self.perform_enhanced_icp_registration(pcd, initial_guess)
            
            # 发布数据
            self.publish_all_data()
            
            # 高频保存
            if self.scan_count % 50 == 0:  # 每50帧保存一次
                self.map_manager.periodic_save(self.map_pointcloud, 5.0)  # 5秒间隔
            
            self.scan_count += 1
            
            # 进度报告
            if self.scan_count % 100 == 0:
                total_points = len(self.map_pointcloud.points)
                self.get_logger().info(
                    f'处理帧数: {self.scan_count}, '
                    f'总点数: {total_points:,}, '
                    f'轨迹长度估算: {total_points/1000:.0f}m'
                )
                
        except Exception as e:
            self.get_logger().error(f'处理点云错误: {str(e)}')
    
    def publish_all_data(self):
        """发布所有相关数据"""
        self.publish_map()
        self.publish_transform()
        self.publish_odometry()
    
    def perform_enhanced_icp_registration(self, current_pcd, initial_guess):
        """执行ICP配准"""
        try:
            # 获取GPS数据用于全局约束
            gps_correction = np.eye(4)
            if self.use_gps:
                # GPS约束：使用GPS作为全局参考
                gps_scale = 0.05  # GPS约束强度
                gps_correction = self.get_gps_correction()
            
            # 应用GPS约束的初始估计
            constrained_initial = initial_guess @ gps_correction
            
            # 执行ICP配准
            result = o3d.pipelines.registration.registration_icp(
                current_pcd,
                self.map_pointcloud,
                self.max_correspondence_distance * 2.0,  # 双倍匹配距离
                constrained_initial,
                o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                o3d.pipelines.registration.ICPConvergenceCriteria(
                    max_iteration=self.max_iteration
                )
            )
            
            # 接受所有配准结果
            self.pose = result.transformation
            self.previous_pose = self.pose.copy()
            
            # 将当前扫描添加到地图
            current_pcd.transform(self.pose)
            self.map_pointcloud += current_pcd
            
            # 记录但不限制点数
            current_points = len(self.map_pointcloud.points)
            if current_points % 1000000 < 1000:  # 每百万点记录一次
                self.get_logger().info(f'地图点数: {current_points:,}')
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'ICP配准错误，使用初始估计: {str(e)}')
            # 即使ICP失败也保留数据
            self.pose = initial_guess
            current_pcd.transform(self.pose)
            self.map_pointcloud += current_pcd
            return True

    def get_gps_correction(self):
        """计算GPS全局约束修正"""
        # 长轨迹GPS约束：使用GPS数据防止漂移
        return np.eye(4)

    def pointcloud_callback(self, msg):
        """处理接收到的点云数据"""
        try:
            current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
            # 将ROS PointCloud2转换为numpy数组
            points = PointCloudUtils.ros_to_numpy(msg)
            if points is None or len(points) == 0:
                self.get_logger().warn('收到空点云')
                return
                
            # 预处理点云
            pcd = PointCloudUtils.preprocess_pointcloud(points, self.voxel_size * 1.5)
            if pcd is None:
                return
            
            if not self.is_initialized:
                # 第一次扫描，初始化地图
                self.map_pointcloud = pcd
                self.is_initialized = True
                self.get_logger().info(f'地图初始化完成，点数: {len(self.map_pointcloud.points):,}')
            else:
                # 获取初始估计
                initial_guess = self.sensor_fusion.get_initial_guess(self.previous_pose, current_time)
                
                # 执行配准
                self.perform_enhanced_icp_registration(pcd, initial_guess)
            
            # 发布数据
            self.publish_all_data()
            
            # 高频保存
            if self.scan_count % 50 == 0:  # 每50帧保存一次
                self.map_manager.periodic_save(self.map_pointcloud, 5.0)  # 5秒间隔
            
            self.scan_count += 1
            
            # 进度报告
            if self.scan_count % 100 == 0:
                total_points = len(self.map_pointcloud.points)
                self.get_logger().info(
                    f'处理帧数: {self.scan_count}, '
                    f'总点数: {total_points:,}, '
                    f'轨迹长度估算: {total_points/1000:.0f}m'
                )
                
        except Exception as e:
            self.get_logger().error(f'处理点云错误: {str(e)}')
    
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