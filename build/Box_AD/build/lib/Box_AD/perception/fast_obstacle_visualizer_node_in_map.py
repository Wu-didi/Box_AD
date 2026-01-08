#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
fast_obstacle_visualizer_node.py

功能：
- 从 map.yaml 解析 PNG 左下角 UTM: (E_png, N_png)
- 订阅 /map (OccupancyGrid)，获取 map 原点 (origin_x, origin_y)
- 计算 map 坐标原点对应的 UTM:
      utm_origin_e = E_png - origin_x
      utm_origin_n = N_png - origin_y
- 订阅 /pose_utm (PoseStamped，frame_id="utm"，position.x=E，position.y=N)
- 订阅 /fast_obstacles (std_msgs/String, JSON)
    {
      "stamp": {...},
      "frame_id": "...",
      "has_obstacle": true/false,
      "obstacles": [
        {"x":..., "y":..., "z":..., "count": N},
        ...
      ]
    }
- 将自车坐标系下的障碍物点变换到 map 坐标系，并用 Marker(POINTS) 在 map 上可视化

RViz 使用：
- Fixed Frame 选 "map"
- 添加一个 Marker 显示，topic 选 "/fast_obstacle_markers"
"""

import os
import re
import json
import math
import time
from typing import Optional, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion


def yaw_from_quaternion(q: Quaternion) -> float:
    """从四元数提取 Z 轴 yaw（标准 ROS 公式）"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class FastObstacleVisualizerNode(Node):
    def __init__(self):
        super().__init__('fast_obstacle_visualizer_node')

        # ------------ 参数 ------------
        self.declare_parameter('map_yaml_path', './maps/map.yaml')
        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('pose_utm_topic', 'pose_utm')
        self.declare_parameter('fast_obstacle_topic', 'fast_obstacles')

        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('utm_frame_id', 'utm')

        # 可视化参数
        self.declare_parameter('point_size', 0.4)   # 障碍物点显示大小
        self.declare_parameter('color_r', 1.0)
        self.declare_parameter('color_g', 0.0)
        self.declare_parameter('color_b', 0.0)
        self.declare_parameter('color_a', 0.9)

        # 发布频率控制
        self.declare_parameter('publish_rate', 2.0)  # 最大发布频率 Hz（降低以减轻RViz压力）

        self.map_yaml_path = self.get_parameter('map_yaml_path').get_parameter_value().string_value
        self.map_topic = self.get_parameter('map_topic').value
        self.pose_utm_topic = self.get_parameter('pose_utm_topic').value
        self.fast_obstacle_topic = self.get_parameter('fast_obstacle_topic').value

        self.map_frame_id = self.get_parameter('map_frame_id').value
        self.utm_frame_id = self.get_parameter('utm_frame_id').value

        self.point_size = float(self.get_parameter('point_size').value)
        self.color_r = float(self.get_parameter('color_r').value)
        self.color_g = float(self.get_parameter('color_g').value)
        self.color_b = float(self.get_parameter('color_b').value)
        self.color_a = float(self.get_parameter('color_a').value)

        # 发布频率控制
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.min_publish_interval = 1.0 / self.publish_rate if self.publish_rate > 0 else 0.0
        self.last_publish_time = 0.0

        # ------------ 从 map.yaml 解析 PNG origin UTM ------------
        self.png_origin_e: Optional[float] = None
        self.png_origin_n: Optional[float] = None
        self.load_png_origin_utm(self.map_yaml_path)

        # ------------ /map 原点 & utm_origin ------------
        self.map_origin_x: Optional[float] = None
        self.map_origin_y: Optional[float] = None
        self.utm_origin_e: Optional[float] = None
        self.utm_origin_n: Optional[float] = None
        self._utm_warned = False

        # ------------ 当前车辆 UTM 位姿 ------------
        self.have_pose = False
        self.car_e = 0.0
        self.car_n = 0.0
        self.car_yaw = 0.0

        # ------------ 订阅 /map (使用 TRANSIENT_LOCAL 匹配 map publisher) ------------
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            map_qos
        )

        # ------------ 订阅 /pose_utm ------------
        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.pose_utm_topic,
            self.pose_utm_callback,
            10
        )

        # ------------ 订阅 /fast_obstacles ------------
        self.fast_obs_sub = self.create_subscription(
            String,
            self.fast_obstacle_topic,
            self.fast_obstacles_callback,
            10
        )

        # ------------ 发布 Marker ------------
        self.marker_pub = self.create_publisher(
            Marker,
            'fast_obstacle_markers_map',
            10
        )

        self.get_logger().info(
            f"FastObstacleVisualizerNode 启动：\n"
            f"  map_yaml_path        = {self.map_yaml_path}\n"
            f"  map_topic            = {self.map_topic}\n"
            f"  pose_utm_topic       = {self.pose_utm_topic}\n"
            f"  fast_obstacle_topic  = {self.fast_obstacle_topic}\n"
            f"  输出 Marker: /fast_obstacle_markers, frame_id={self.map_frame_id}"
        )

    # ================== 解析 map.yaml ==================

    def load_png_origin_utm(self, yaml_path: str):
        """
        从 map.yaml 中解析:
            # PNG origin UTM: E=..., N=...
        得到 PNG 左下角对应的 UTM 坐标 (png_origin_e, png_origin_n)。
        """
        if not os.path.exists(yaml_path):
            self.get_logger().error(f"map_yaml_path 不存在: {yaml_path}")
            return

        try:
            with open(yaml_path, 'r') as f:
                for line in f:
                    line = line.strip()
                    if line.startswith("# PNG origin UTM"):
                        m = re.search(r"E\s*=\s*([0-9.+\-eE]+).*N\s*=\s*([0-9.+\-eE]+)", line)
                        if m:
                            self.png_origin_e = float(m.group(1))
                            self.png_origin_n = float(m.group(2))
                            self.get_logger().debug(
                                f"从 map.yaml 解析 PNG origin UTM: "
                                f"E={self.png_origin_e:.3f}, N={self.png_origin_n:.3f}"
                            )
                            return
            self.get_logger().warn(
                "在 map.yaml 中未找到 '# PNG origin UTM: E=..., N=...' 注释，"
                "将无法计算 utm_origin_e/n，UTM -> map 转换会不准确。"
            )
        except Exception as e:
            self.get_logger().error(f"解析 map.yaml 失败: {e}")

    # ================== /map 回调：计算 utm_origin ==================

    def map_callback(self, msg: OccupancyGrid):
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y

        self.get_logger().debug(
            f"收到 /map: origin=({self.map_origin_x:.3f}, {self.map_origin_y:.3f}), "
            f"res={msg.info.resolution:.3f}, size=({msg.info.width}x{msg.info.height})"
        )

        if self.png_origin_e is not None and self.png_origin_n is not None:
            self.utm_origin_e = self.png_origin_e - self.map_origin_x
            self.utm_origin_n = self.png_origin_n - self.map_origin_y
            self.get_logger().debug(
                f"计算得到 utm_origin: E0={self.utm_origin_e:.3f}, N0={self.utm_origin_n:.3f} "
                f"(map (0,0) 对应的 UTM 坐标)"
            )
        else:
            self.get_logger().warn(
                "尚未解析到 PNG origin UTM，无法计算 utm_origin_e/n。"
            )

    # ================== UTM -> map ==================

    def utm_to_map_xy(self, E: float, N: float) -> Tuple[float, float]:
        """
        UTM (E, N) -> map (x, y)
        用：
            E = utm_origin_e + x_map
            N = utm_origin_n + y_map
        """
        if self.utm_origin_e is None or self.utm_origin_n is None:
            if not self._utm_warned:
                self.get_logger().warn(
                    "utm_origin_e/n 尚未就绪，暂时无法准确将 UTM 坐标转换到 map；"
                    "请确认 /map 已发布且 map.yaml 中含有 PNG origin UTM 注释。"
                )
                self._utm_warned = True
            # 临时直接返回 UTM 值，避免节点崩溃
            return E, N

        x = E - self.utm_origin_e
        y = N - self.utm_origin_n
        return x, y

    # ================== /pose_utm 回调 ==================

    def pose_utm_callback(self, msg: PoseStamped):
        """
        记录当前车辆在 UTM 下的位置 (E,N) 和 yaw。
        """
        self.car_e = msg.pose.position.x
        self.car_n = msg.pose.position.y
        self.car_yaw = yaw_from_quaternion(msg.pose.orientation)
        self.have_pose = True

    # ================== /fast_obstacles 回调 ==================

    def fast_obstacles_callback(self, msg: String):
        """
        将 fast_obstacles 中的自车系障碍物点，转换到 map 坐标系并发布 Marker。
        如果没有 pose_utm，则直接在雷达坐标系下显示。
        """
        # 节流：限制发布频率
        current_time = time.time()
        if current_time - self.last_publish_time < self.min_publish_interval:
            return
        self.last_publish_time = current_time

        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"解析 fast_obstacles JSON 失败: {e}")
            return

        if not data.get("has_obstacle", False):
            # 清空显示
            marker = self.make_empty_marker()
            self.marker_pub.publish(marker)
            return

        obs_list = data.get("obstacles", [])
        if not obs_list:
            marker = self.make_empty_marker()
            self.marker_pub.publish(marker)
            return

        points_map: List[Point] = []

        # 如果没有 pose_utm，直接在雷达坐标系下显示
        if not self.have_pose:
            frame_id = data.get("frame_id", "rslidar")
            for ob in obs_list:
                try:
                    p = Point()
                    p.x = float(ob.get("x", 0.0))
                    p.y = float(ob.get("y", 0.0))
                    p.z = float(ob.get("z", 0.0))
                    points_map.append(p)
                except Exception:
                    continue
            marker = self.make_points_marker(points_map, frame_id=frame_id)
            self.marker_pub.publish(marker)
            return

        # 有 pose_utm，转换到 map 坐标系
        E0 = self.car_e
        N0 = self.car_n
        yaw = self.car_yaw
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        for ob in obs_list:
            try:
                x_rel = float(ob.get("x", 0.0))  # 车体坐标系：前方为 +x
                y_rel = float(ob.get("y", 0.0))  # 左侧为 +y
            except Exception:
                continue

            # 车体 -> UTM（平移+旋转）
            E = E0 + x_rel * cos_yaw - y_rel * sin_yaw
            N = N0 + x_rel * sin_yaw + y_rel * cos_yaw

            # UTM -> map
            x_map, y_map = self.utm_to_map_xy(E, N)

            p = Point()
            p.x = float(x_map)
            p.y = float(y_map)
            p.z = 0.0
            points_map.append(p)

        marker = self.make_points_marker(points_map)
        self.marker_pub.publish(marker)

    # ================== 生成 Marker ==================

    def make_empty_marker(self) -> Marker:
        m = Marker()
        m.header.frame_id = self.map_frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "fast_obstacles"
        m.id = 0
        m.type = Marker.POINTS
        m.action = Marker.ADD

        # 障碍物可能消失，设置 1 秒过期
        m.lifetime.sec = 1
        m.lifetime.nanosec = 0

        m.scale.x = self.point_size
        m.scale.y = self.point_size

        m.color.r = self.color_r
        m.color.g = self.color_g
        m.color.b = self.color_b
        m.color.a = self.color_a

        m.points = []
        return m

    def make_points_marker(self, points: List[Point], frame_id: str = None) -> Marker:
        m = Marker()
        m.header.frame_id = frame_id if frame_id else self.map_frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "fast_obstacles"
        m.id = 0
        m.type = Marker.POINTS
        m.action = Marker.ADD

        # 障碍物可能消失，设置 1 秒过期
        m.lifetime.sec = 1
        m.lifetime.nanosec = 0

        m.scale.x = self.point_size
        m.scale.y = self.point_size

        m.color.r = self.color_r
        m.color.g = self.color_g
        m.color.b = self.color_b
        m.color.a = self.color_a

        m.points = points
        return m


def main(args=None):
    rclpy.init(args=args)
    node = FastObstacleVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
