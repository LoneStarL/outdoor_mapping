import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 定义参数
    dataset_arg = DeclareLaunchArgument(
        'dataset',
        default_value='degenerative_hard',
        description='选择数据包: 10m_easy, 300m_medium, degenerative_hard'
    )
    
    save_format_arg = DeclareLaunchArgument(
        'save_format',
        default_value='pcd',
        description='地图保存格式: pcd, ply, obj, 或组合格式如 pcd,ply'
    )
    
    save_interval_arg = DeclareLaunchArgument(
        'save_interval',
        default_value='30.0',
        description='自动保存间隔（秒）'
    )
    
    def create_nodes(context):
        # 使用ROS2包路径推导工作空间根目录
        package_share_dir = get_package_share_directory('outdoor_mapping')
        
        # 工作空间根目录
        if 'install' in package_share_dir:
            # 安装路径: .../ws/install/outdoor_mapping/share/outdoor_mapping
            workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(package_share_dir))))
        else:
            # 源代码路径: .../ws/src/outdoor_mapping
            workspace_root = os.path.dirname(os.path.dirname(package_share_dir))
        
        # 使用源代码路径
        src_package_dir = os.path.join(workspace_root, 'src', 'outdoor_mapping')
        
        print(f'[INFO] [launch.user]: 工作空间根目录: {workspace_root}')
        print(f'[INFO] [launch.user]: 源代码包路径: {src_package_dir}')
        
        # 获取参数值
        dataset = context.launch_configurations['dataset']
        save_format = context.launch_configurations.get('save_format', 'pcd')
        save_interval = float(context.launch_configurations.get('save_interval', '30.0'))
        
        # 使用源代码路径构建文件路径
        config_file = os.path.join(src_package_dir, 'config', f'{dataset}_mapping.yaml')
        rviz_file = os.path.join(src_package_dir, 'rviz', f'{dataset}_mapping.rviz')
        
        # 构建输出目录路径
        output_dir = os.path.join(workspace_root, 'maps', dataset)
        
        # 确保输出目录存在
        os.makedirs(output_dir, exist_ok=True)
        
        # 调试信息输出
        print('[INFO] [launch]: Default logging verbosity is set to INFO')
        print('[INFO] [launch.user]: === 启动调试信息 ===')
        print(f'[INFO] [launch.user]: 工作空间根目录: {workspace_root}')
        print(f'[INFO] [launch.user]: 源代码包路径: {src_package_dir}')
        print(f'[INFO] [launch.user]: 数据集参数: {dataset}')
        print(f'[INFO] [launch.user]: 保存格式: {save_format}')
        print(f'[INFO] [launch.user]: 保存间隔: {save_interval}秒')
        print(f'[INFO] [launch.user]: 配置文件路径: {config_file}')
        print(f'[INFO] [launch.user]: RViz配置文件路径: {rviz_file}')
        print(f'[INFO] [launch.user]: 输出目录: {output_dir}')
        
        # 检查文件是否存在
        if not os.path.isfile(config_file):
            print(f'[WARNING] [launch.user]: 配置文件不存在: {config_file}')
        else:
            print(f'[INFO] [launch.user]: 配置文件存在: {config_file}')
            
        if not os.path.isfile(rviz_file):
            print(f'[WARNING] [launch.user]: RViz配置文件不存在: {rviz_file}')
        else:
            print(f'[INFO] [launch.user]: RViz配置文件存在: {rviz_file}')
        
        return [
            # 增强LiDAR SLAM节点
            Node(
                package='outdoor_mapping',
                executable='enhanced_lidar_slam_node',
                name='enhanced_lidar_slam_node',
                output='screen',
                parameters=[
                    config_file,
                    {
                        'output_dir': output_dir,
                        'save_format': save_format,
                        'save_map_interval': save_interval,
                        'dataset': dataset
                    }
                ],
                remappings=[
                    ('/livox/pointcloud_livox_frame_front', '/livox/pointcloud_livox_frame_front'),
                    ('/imu', '/livox/imu_livox_frame_front'),
                    ('/vel', '/vel'),
                    ('/fix', '/fix'),
                    ('/heading', '/heading'),
                ]
            ),
            
            # RViz2可视化
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='log', 
                arguments=['-d', rviz_file]
            )
        ]
    
    return LaunchDescription([
        SetEnvironmentVariable(name='RCUTILS_LOGGING_SEVERITY_THRESHOLD', value='WARN'),
        SetEnvironmentVariable(name='RCUTILS_CONSOLE_OUTPUT_FORMAT', value='[{severity}] {message}'),
        
        dataset_arg,
        save_format_arg,
        save_interval_arg,
        OpaqueFunction(function=create_nodes)
    ])