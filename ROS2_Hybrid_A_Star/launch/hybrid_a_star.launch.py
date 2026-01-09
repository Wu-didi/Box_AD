#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS2 Hybrid A* Launch File - 完整版

启动命令:
  ros2 launch ros2_hybrid_a_star hybrid_a_star.launch.py

带参数启动:
  ros2 launch ros2_hybrid_a_star hybrid_a_star.launch.py map_yaml_path:=/path/to/map.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 车辆参数：车长4.53m, 车宽1.9m, 轴距2.85m, 最大转向角60度
    declare_vehicle_length = DeclareLaunchArgument(
        'vehicle_length', default_value='4.53',
        description='车辆长度 (米)'
    )
    declare_vehicle_width = DeclareLaunchArgument(
        'vehicle_width', default_value='1.9',
        description='车辆宽度 (米)'
    )
    declare_rear_axle_dist = DeclareLaunchArgument(
        'rear_axle_dist', default_value='1.3',
        description='后轴到车尾的距离 (米)'
    )
    declare_wheelbase = DeclareLaunchArgument(
        'wheelbase', default_value='2.85',
        description='轴距 (米)'
    )
    declare_max_steer_angle = DeclareLaunchArgument(
        'max_steer_angle', default_value='60.0',
        description='最大转向角 (度)'
    )
    declare_segment_length = DeclareLaunchArgument(
        'segment_length', default_value='1.6',
        description='每次扩展的步长 (米)'
    )
    declare_segment_length_discrete_num = DeclareLaunchArgument(
        'segment_length_discrete_num', default_value='8',
        description='步长离散数量'
    )
    declare_steering_angle_discrete_num = DeclareLaunchArgument(
        'steering_angle_discrete_num', default_value='1',
        description='转向角离散数量'
    )
    declare_steering_penalty = DeclareLaunchArgument(
        'steering_penalty', default_value='1.05',
        description='转向惩罚系数'
    )
    declare_reversing_penalty = DeclareLaunchArgument(
        'reversing_penalty', default_value='2.0',
        description='倒车惩罚系数'
    )
    declare_steering_change_penalty = DeclareLaunchArgument(
        'steering_change_penalty', default_value='1.5',
        description='转向改变惩罚系数'
    )
    declare_shot_distance = DeclareLaunchArgument(
        'shot_distance', default_value='10.0',
        description='解析扩展触发距离 (米)'
    )
    declare_state_grid_resolution = DeclareLaunchArgument(
        'state_grid_resolution', default_value='1.0',
        description='状态网格分辨率 (米)'
    )
    declare_map_grid_resolution = DeclareLaunchArgument(
        'map_grid_resolution', default_value='0.1',
        description='地图网格分辨率 (米)'
    )

    # UTM 相关参数
    declare_map_yaml_path = DeclareLaunchArgument(
        'map_yaml_path', default_value='./maps/map.yaml',
        description='map.yaml文件路径（用于解析UTM信息）'
    )
    declare_utm_zone = DeclareLaunchArgument(
        'utm_zone', default_value='50',
        description='UTM区域编号'
    )
    declare_utm_zone_letter = DeclareLaunchArgument(
        'utm_zone_letter', default_value='N',
        description='UTM区域字母',
        choices=['N', 'S']  # 限制为有效值
    )

    # Hybrid A* 节点
    hybrid_a_star_node = Node(
        package='ros2_hybrid_a_star',
        executable='hybrid_a_star_node',
        name='hybrid_a_star_node',
        output='screen',
        parameters=[{
            'vehicle_length': LaunchConfiguration('vehicle_length'),
            'vehicle_width': LaunchConfiguration('vehicle_width'),
            'rear_axle_dist': LaunchConfiguration('rear_axle_dist'),
            'wheelbase': LaunchConfiguration('wheelbase'),
            'max_steer_angle': LaunchConfiguration('max_steer_angle'),
            'segment_length': LaunchConfiguration('segment_length'),
            'segment_length_discrete_num': LaunchConfiguration('segment_length_discrete_num'),
            'steering_angle_discrete_num': LaunchConfiguration('steering_angle_discrete_num'),
            'steering_penalty': LaunchConfiguration('steering_penalty'),
            'reversing_penalty': LaunchConfiguration('reversing_penalty'),
            'steering_change_penalty': LaunchConfiguration('steering_change_penalty'),
            'shot_distance': LaunchConfiguration('shot_distance'),
            'state_grid_resolution': LaunchConfiguration('state_grid_resolution'),
            'map_grid_resolution': LaunchConfiguration('map_grid_resolution'),
            'map_yaml_path': LaunchConfiguration('map_yaml_path'),
            'utm_zone': LaunchConfiguration('utm_zone'),
            'utm_zone_letter': ParameterValue(LaunchConfiguration('utm_zone_letter'), value_type=str),
        }]
    )

    return LaunchDescription([
        # 车辆参数
        declare_vehicle_length,
        declare_vehicle_width,
        declare_rear_axle_dist,
        declare_wheelbase,
        declare_max_steer_angle,

        # 算法参数
        declare_segment_length,
        declare_segment_length_discrete_num,
        declare_steering_angle_discrete_num,
        declare_steering_penalty,
        declare_reversing_penalty,
        declare_steering_change_penalty,
        declare_shot_distance,
        declare_state_grid_resolution,
        declare_map_grid_resolution,

        # UTM参数
        declare_map_yaml_path,
        declare_utm_zone,
        declare_utm_zone_letter,

        # 节点
        hybrid_a_star_node,
    ])
