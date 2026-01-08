#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
local_path_to_follow_node.py

将 local_path_planner 发布的 nav_msgs/Path (UTM坐标) 转换为
follow_node 需要的 geometry_msgs/PoseArray (经纬度坐标)

订阅:
  - /local_path (nav_msgs/Path) - UTM坐标的局部规划轨迹

发布:
  - /trajectory (geometry_msgs/PoseArray) - 经纬度坐标
    - position.x: 经度 (longitude)
    - position.y: 纬度 (latitude)
    - orientation.z, orientation.w: 航向角的四元数
"""

import math
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseArray, Pose

try:
    from pyproj import Proj
    HAS_PYPROJ = True
except ImportError:
    HAS_PYPROJ = False


class LocalPathToFollowNode(Node):
    def __init__(self):
        super().__init__('local_path_to_follow_node')

        if not HAS_PYPROJ:
            self.get_logger().error('pyproj not installed! Run: pip install pyproj')
            raise RuntimeError('pyproj not installed')

        # 声明参数：投影中央经线和原点纬度
        self.declare_parameter('central_longitude', 118.8170043)
        self.declare_parameter('central_latitude', 31.8926311)

        central_lon = self.get_parameter('central_longitude').value
        central_lat = self.get_parameter('central_latitude').value

        # 创建投影对象 (与原系统一致的TM投影)
        proj_string = f'+proj=tmerc +lon_0={central_lon} +lat_0={central_lat} +ellps=WGS84'
        self.proj = Proj(proj_string)

        self.get_logger().info(f'Projection: {proj_string}')

        # 订阅局部路径
        self.path_sub = self.create_subscription(
            Path,
            '/local_path',
            self.path_callback,
            10
        )

        # 发布给follow_node的轨迹
        self.trajectory_pub = self.create_publisher(
            PoseArray,
            '/trajectory',
            10
        )

        self.get_logger().info('LocalPathToFollowNode started. /local_path (UTM) -> /trajectory (lat/lon)')

    def path_callback(self, msg: Path):
        """将UTM坐标的Path转换为经纬度的PoseArray"""
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'map'

        for pose_stamped in msg.poses:
            # UTM坐标
            x_utm = pose_stamped.pose.position.x
            y_utm = pose_stamped.pose.position.y

            # UTM -> 经纬度 (inverse=True)
            lon, lat = self.proj(x_utm, y_utm, inverse=True)

            # 保留原始航向
            qz = pose_stamped.pose.orientation.z
            qw = pose_stamped.pose.orientation.w

            # 构建Pose
            pose = Pose()
            pose.position.x = lon  # 经度
            pose.position.y = lat  # 纬度
            pose.position.z = 0.0

            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = qz
            pose.orientation.w = qw

            pose_array.poses.append(pose)

        self.trajectory_pub.publish(pose_array)


def main(args=None):
    rclpy.init(args=args)
    node = LocalPathToFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
