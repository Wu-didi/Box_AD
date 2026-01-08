#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
local_path_to_mpc_node.py

将 local_path_planner 发布的 nav_msgs/Path 转换为 MPC 需要的 geometry_msgs/PoseArray
并计算每个点的曲率(curvature)存入 orientation.x

订阅:
  - /local_path (nav_msgs/Path) - 局部规划轨迹

发布:
  - /mpc_trajectory (geometry_msgs/PoseArray) - MPC所需格式
    - position.x, position.y: 位置
    - orientation.x: 曲率 ck
    - orientation.z, orientation.w: 航向角的四元数 (qz, qw)
"""

import math
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseArray, Pose


class LocalPathToMpcNode(Node):
    def __init__(self):
        super().__init__('local_path_to_mpc_node')

        # 订阅局部路径
        self.path_sub = self.create_subscription(
            Path,
            '/local_path',
            self.path_callback,
            10
        )

        # 发布MPC轨迹
        self.mpc_pub = self.create_publisher(
            PoseArray,
            '/mpc_trajectory',
            10
        )

        self.get_logger().info('LocalPathToMpcNode started. /local_path -> /mpc_trajectory')

    def path_callback(self, msg: Path):
        """将Path转换为PoseArray并计算曲率"""
        if len(msg.poses) < 2:
            return

        # 提取点坐标
        points = []
        for pose_stamped in msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            # 从四元数提取yaw
            qz = pose_stamped.pose.orientation.z
            qw = pose_stamped.pose.orientation.w
            yaw = 2.0 * math.atan2(qz, qw)
            points.append((x, y, yaw, qz, qw))

        # 计算曲率
        curvatures = self.compute_curvatures(points)

        # 构建PoseArray
        pose_array = PoseArray()
        pose_array.header = msg.header

        for i, (x, y, yaw, qz, qw) in enumerate(points):
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0.0

            # 曲率存入orientation.x
            pose.orientation.x = curvatures[i]
            pose.orientation.y = 0.0
            pose.orientation.z = qz
            pose.orientation.w = qw

            pose_array.poses.append(pose)

        self.mpc_pub.publish(pose_array)

    def compute_curvatures(self, points):
        """
        计算每个点的曲率
        使用三点法: curvature = 2 * sin(angle) / distance

        Args:
            points: [(x, y, yaw, qz, qw), ...]

        Returns:
            curvatures: [float, ...]
        """
        n = len(points)
        curvatures = [0.0] * n

        if n < 3:
            return curvatures

        for i in range(1, n - 1):
            x0, y0 = points[i-1][0], points[i-1][1]
            x1, y1 = points[i][0], points[i][1]
            x2, y2 = points[i+1][0], points[i+1][1]

            # 向量
            dx1 = x1 - x0
            dy1 = y1 - y0
            dx2 = x2 - x1
            dy2 = y2 - y1

            # 长度
            len1 = math.sqrt(dx1*dx1 + dy1*dy1)
            len2 = math.sqrt(dx2*dx2 + dy2*dy2)

            if len1 < 1e-6 or len2 < 1e-6:
                curvatures[i] = 0.0
                continue

            # 叉积 (用于计算sin)
            cross = dx1 * dy2 - dy1 * dx2

            # 曲率 = 2 * sin(theta) / chord_length
            # sin(theta) = cross / (len1 * len2)
            # chord_length ≈ (len1 + len2) / 2
            chord = (len1 + len2) / 2.0
            sin_theta = cross / (len1 * len2)

            curvatures[i] = 2.0 * sin_theta / chord

        # 端点曲率用邻近点的值
        curvatures[0] = curvatures[1] if n > 1 else 0.0
        curvatures[-1] = curvatures[-2] if n > 1 else 0.0

        return curvatures


def main(args=None):
    rclpy.init(args=args)
    node = LocalPathToMpcNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
