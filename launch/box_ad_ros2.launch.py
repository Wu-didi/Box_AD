#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Box_AD 完整自动驾驶栈 Launch 文件 - ROS2 Hybrid A* 版本

启动命令:
  ros2 launch Box_AD box_ad_ros2.launch.py

带参数启动:
  ros2 launch Box_AD box_ad_ros2.launch.py map_path:=/path/to/map.png max_speed:=3.0

只启动部分模块:
  ros2 launch Box_AD box_ad_ros2.launch.py use_imu:=false use_perception:=false
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # ==================== 参数声明 ====================

    # 功能开关
    declare_use_imu = DeclareLaunchArgument(
        'use_imu', default_value='true',
        description='是否启动 IMU/GPS 驱动 (CAN总线)'
    )
    declare_use_perception = DeclareLaunchArgument(
        'use_perception', default_value='true',
        description='是否启动障碍物检测'
    )
    declare_use_planner = DeclareLaunchArgument(
        'use_planner', default_value='true',
        description='是否启动规划模块'
    )
    declare_use_viz = DeclareLaunchArgument(
        'use_viz', default_value='true',
        description='是否启动可视化模块'
    )

    # 地图参数
    declare_map_path = DeclareLaunchArgument(
        'map_path', default_value='./maps/map_2d.png',
        description='PNG地图文件路径'
    )
    declare_map_yaml_path = DeclareLaunchArgument(
        'map_yaml_path', default_value='./maps/map.yaml',
        description='地图YAML配置文件路径'
    )
    declare_map_resolution = DeclareLaunchArgument(
        'map_resolution', default_value='0.1',
        description='地图分辨率 (米/像素)'
    )
    declare_map_origin_x = DeclareLaunchArgument(
        'map_origin_x', default_value='0.0',
        description='地图原点X (米)'
    )
    declare_map_origin_y = DeclareLaunchArgument(
        'map_origin_y', default_value='0.0',
        description='地图原点Y (米)'
    )

    # 规划参数
    declare_robot_radius = DeclareLaunchArgument(
        'robot_radius', default_value='1.2',
        description='机器人半径 (米)，用于碰撞检测'
    )
    declare_max_speed = DeclareLaunchArgument(
        'max_speed', default_value='3.0',
        description='最大线速度 (米/秒)'
    )
    declare_lookahead_distance = DeclareLaunchArgument(
        'lookahead_distance', default_value='10.0',
        description='前视距离 (米)'
    )
    declare_min_turning_radius = DeclareLaunchArgument(
        'min_turning_radius', default_value='3.0',
        description='最小转弯半径 (米)'
    )

    # Hybrid A* 参数文件路径
    declare_hybrid_astar_params = DeclareLaunchArgument(
        'hybrid_astar_params',
        default_value='/home/nvidia/vcii/wudi/Box_AD/ROS2_Hybrid_A_Star/config/params.yaml',
        description='Hybrid A* 参数文件路径'
    )

    # ==================== 节点定义 ====================

    # 1. IMU/GPS 驱动
    imu_node = Node(
        package='Box_AD',
        executable='publish_imu',
        name='publish_imu',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_imu'))
    )

    # 2. GPS/IMU 定位
    gps_imu_pose_node = Node(
        package='Box_AD',
        executable='gps_imu_to_utm_pose',
        name='gps_imu_to_utm_pose',
        output='screen',
        parameters=[{
            'lat0': 31.89131250,  # 和 map.yaml 中的参考点一致
            'lon0': 118.81631310,
        }],
        condition=IfCondition(LaunchConfiguration('use_imu'))
    )

    # 3. 地图发布
    map_publisher_node = Node(
        package='Box_AD',
        executable='png_map_publisher',
        name='png_map_publisher',
        output='screen',
        parameters=[{
            'image_path': LaunchConfiguration('map_path'),
            'resolution': LaunchConfiguration('map_resolution'),
            'origin_x': LaunchConfiguration('map_origin_x'),
            'origin_y': LaunchConfiguration('map_origin_y'),
        }]
    )

    # 4. 障碍物检测
    obstacle_detector_node = Node(
        package='Box_AD',
        executable='fast_lidar_obstacle_detector',
        name='fast_lidar_obstacle_detector',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_perception'))
    )

    # ========== 5. 全局规划 - ROS2 Hybrid A* (替换原来的Python A*) ==========
    global_planner_node = Node(
        package='ros2_hybrid_a_star',
        executable='hybrid_a_star_node',
        name='hybrid_a_star_planner',
        output='screen',
        arguments=['--ros-args', '--params-file', LaunchConfiguration('hybrid_astar_params')],
        condition=IfCondition(LaunchConfiguration('use_planner'))
    )

    # 6. 局部规划 (A* 避障)
    local_planner_node = Node(
        package='Box_AD',
        executable='local_planner_astar',
        name='local_planner_astar_node',
        output='screen',
        parameters=[{
            # 车辆尺寸（用于计算安全半径）
            'vehicle_length': 3.5,
            'vehicle_width': 1.5,
            'safety_margin': 0.05,
            'obstacle_inflation': 0.03,
            # 障碍物处理
            'min_obstacle_radius': 0.3,
            'obstacle_cluster_dist': 1.5,
            'max_obstacle_distance': 50.0,
            # 规划参数
            'lookahead_distance': LaunchConfiguration('lookahead_distance'),
            'local_map_size': 60.0,
            'replan_frequency': 5.0,
            'path_sample_step': 0.3,
            'enable_smoothing': True,
            'min_turning_radius': LaunchConfiguration('min_turning_radius'),
            'max_speed': LaunchConfiguration('max_speed'),
            'map_yaml_path': LaunchConfiguration('map_yaml_path'),
        }],
        condition=IfCondition(LaunchConfiguration('use_planner'))
    )

    # 6.5 Path转MPC格式 (连接local_planner和MPC)
    path_to_mpc_node = Node(
        package='Box_AD',
        executable='local_path_to_mpc',
        name='local_path_to_mpc',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_planner'))
    )

    # 7. 自车可视化
    ego_viz_node = Node(
        package='Box_AD',
        executable='ego_visualizer',
        name='ego_visualizer',
        output='screen',
        parameters=[{
            'map_yaml_path': LaunchConfiguration('map_yaml_path'),
        }],
        condition=IfCondition(LaunchConfiguration('use_viz'))
    )

    # 8. 路径可视化
    path_viz_node = Node(
        package='Box_AD',
        executable='path_visualizer',
        name='path_visualizer',
        output='screen',
        parameters=[{
            'map_yaml_path': LaunchConfiguration('map_yaml_path'),
        }],
        condition=IfCondition(LaunchConfiguration('use_viz'))
    )

    # 9. 障碍物可视化
    obstacle_viz_node = Node(
        package='Box_AD',
        executable='fast_obstacle_rviz_in_map',
        name='fast_obstacle_rviz_in_map',
        output='screen',
        parameters=[{
            'map_yaml_path': LaunchConfiguration('map_yaml_path'),
        }],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('use_viz'), "' == 'true' and '",
                LaunchConfiguration('use_perception'), "' == 'true'"
            ])
        )
    )

    # ==================== 启动日志 ====================

    startup_log = LogInfo(msg='Box_AD 自动驾驶栈启动中 (使用 ROS2 Hybrid A* 全局规划)...')

    # ==================== 返回 LaunchDescription ====================

    return LaunchDescription([
        # 参数声明
        declare_use_imu,
        declare_use_perception,
        declare_use_planner,
        declare_use_viz,
        declare_map_path,
        declare_map_yaml_path,
        declare_map_resolution,
        declare_map_origin_x,
        declare_map_origin_y,
        declare_robot_radius,
        declare_max_speed,
        declare_lookahead_distance,
        declare_min_turning_radius,
        declare_hybrid_astar_params,  # 新增：Hybrid A* 参数文件

        # 启动日志
        startup_log,

        # 节点
        imu_node,
        gps_imu_pose_node,
        map_publisher_node,
        obstacle_detector_node,
        global_planner_node,  # 已替换为 Hybrid A*
        local_planner_node,
        path_to_mpc_node,
        ego_viz_node,
        path_viz_node,
        obstacle_viz_node,
    ])
