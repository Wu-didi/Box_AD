from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'Box_AD'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 包含 launch 文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wudi',
    maintainer_email='164662525@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                    'publish_imu = Box_AD.imu_node:main',   # 发布imu数据
                    'imu_gps_logger = Box_AD.imu_gps_logger:main',  # 记录imu和gps数据
                    'png_map_publisher = Box_AD.map.png_map_publisher:main',     # 发布地图
                    'astar_planner_node = Box_AD.planner.global_planner_astar_node:main',  # 全局规划
                    'local_path_planner_node = Box_AD.planner.local_path_planner_node:main', # 局部路径规划(梯形绕行)
                    'local_planner_astar = Box_AD.planner.local_planner_astar_node:main',  # 局部路径规划(A*)
                    'local_path_to_mpc = Box_AD.planner.local_path_to_mpc_node:main',  # Path转MPC格式
                    'local_path_to_follow = Box_AD.planner.local_path_to_follow_node:main',  # Path转follow格式(经纬度)
                    'fast_lidar_obstacle_detector = Box_AD.perception.fast_lidar_obstacle_detector_node:main', # 激光雷达障碍物检测
                    'fast_obstacle_temporal_filter_node = Box_AD.perception.fast_obstacle_temporal_filter_node:main', # 没有用到
                    'path_visualizer = Box_AD.visualizer.path_visualizer_node:main',   # 轨迹可视化
                    'gps_imu_to_utm_pose = Box_AD.gps_imu_to_utm_pose_node:main',  # 这个也是要必须启动的
                    'ego_visualizer = Box_AD.visualizer.ego_visualizer_node:main',            # 自车可视化
                    'fast_obstacle_rviz_in_map = Box_AD.perception.fast_obstacle_visualizer_node_in_map:main', # 激光雷达障碍物检测可视化
                    # follow_traj_wd 循迹相关节点
                    'can_node = Box_AD.follow_traj_wd.can_node:main',  # CAN通信节点
                    'follow_node = Box_AD.follow_traj_wd.follow_node:main',  # 几何循迹节点(高速)
                    'mpc_node = Box_AD.follow_traj_wd.mpc_node_v2:main',  # MPC循迹节点(低速)
                    'auto_drive_panel = Box_AD.follow_traj_wd.auto_drive_panel_node:main',  # 智驾控制面板
        ],
    },
)
