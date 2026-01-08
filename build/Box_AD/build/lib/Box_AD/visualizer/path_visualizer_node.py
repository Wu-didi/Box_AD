#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
path_visualizer_node.py

功能：
- 读取 map.yaml，解析 PNG 左下角的 UTM 坐标 (png_origin_e, png_origin_n)
- 订阅 /map (OccupancyGrid)，得到 map 原点 (origin_x, origin_y)
- 计算 map 坐标系原点对应的 UTM

- 订阅以下路径:
    * /global_path_utm    (nav_msgs/Path, frame_id="utm") - 原始全局路径
    * /local_path         (nav_msgs/Path, frame_id="utm") - 调整后的局部路径

- 在 map 坐标系下重新发布 Path：
    * /global_path_in_map  (原始全局路径)
    * /local_path_in_map   (调整后的局部路径)

RViz 中 Fixed Frame 设为 map，订阅上述 2 个话题即可。
"""

import os
import re
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped


class PathVisualizerNode(Node):
    def __init__(self):
        super().__init__('path_visualizer_node')

        # -------- 参数 --------
        self.declare_parameter('map_yaml_path', './maps/map.yaml')
        self.declare_parameter('map_topic', 'map')

        self.declare_parameter('global_path_utm_topic', 'global_path_utm')
        self.declare_parameter('local_path_topic', 'local_path')

        self.declare_parameter('global_path_in_map_topic', 'global_path_in_map')
        self.declare_parameter('local_path_in_map_topic', 'local_path_in_map')

        self.declare_parameter('map_frame_id', 'map')

        # 发布频率控制
        self.declare_parameter('publish_rate', 5.0)

        self.map_yaml_path = self.get_parameter('map_yaml_path').get_parameter_value().string_value
        self.map_topic = self.get_parameter('map_topic').value

        self.global_path_utm_topic = self.get_parameter('global_path_utm_topic').value
        self.local_path_topic = self.get_parameter('local_path_topic').value

        self.global_path_in_map_topic = self.get_parameter('global_path_in_map_topic').value
        self.local_path_in_map_topic = self.get_parameter('local_path_in_map_topic').value

        self.map_frame_id = self.get_parameter('map_frame_id').value

        # 发布频率控制
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.min_publish_interval = 1.0 / self.publish_rate if self.publish_rate > 0 else 0.0
        self.last_local_path_time = 0.0

        # -------- 从 map.yaml 解析 PNG origin UTM --------
        self.png_origin_e: Optional[float] = None
        self.png_origin_n: Optional[float] = None
        self.load_png_origin_utm(self.map_yaml_path)

        # -------- /map 原点 & utm_origin --------
        self.map_origin_x: Optional[float] = None
        self.map_origin_y: Optional[float] = None
        self.utm_origin_e: Optional[float] = None
        self.utm_origin_n: Optional[float] = None
        self.have_map_info: bool = False

        # utm_origin 没就绪时仅 warn 一次
        self.warned_no_utm_origin = False

        # -------- 订阅 /map (使用 TRANSIENT_LOCAL 匹配 map publisher) --------
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

        # -------- 订阅路径 --------
        global_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.sub_global_path_utm = self.create_subscription(
            Path,
            self.global_path_utm_topic,
            self.global_path_utm_callback,
            global_qos
        )
        self.sub_local_path = self.create_subscription(
            Path,
            self.local_path_topic,
            self.local_path_callback,
            10
        )

        # -------- 发布 Path（map 下）--------
        self.pub_global_path_in_map = self.create_publisher(
            Path,
            self.global_path_in_map_topic,
            10
        )
        self.pub_local_path_in_map = self.create_publisher(
            Path,
            self.local_path_in_map_topic,
            10
        )

        self.get_logger().info(
            f"PathVisualizerNode 启动：\n"
            f"  map_yaml_path = {self.map_yaml_path}\n"
            f"  订阅: {self.global_path_utm_topic}, {self.local_path_topic}\n"
            f"  发布: {self.global_path_in_map_topic}, {self.local_path_in_map_topic}"
        )

    # =============== 解析 map.yaml 中的 PNG origin UTM ===============

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
                        m = re.search(r"E\s*=\s*([0-9.+\\-eE]+).*N\s*=\s*([0-9.+\\-eE]+)", line)
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
                "将无法计算 utm_origin_e/n，UTM 路径转换会不准确。"
            )
        except Exception as e:
            self.get_logger().error(f"解析 map.yaml 失败: {e}")

    # =============== /map 回调：计算 utm_origin ===============

    def map_callback(self, msg: OccupancyGrid):
        """
        OccupancyGrid.info.origin.position 给出 map 原点在世界坐标中的位置。
        在你的建图流程中，origin_x, origin_y 是 PNG 左下角在 map 坐标系下的位置。

        已知:
            PNG 左下角 UTM: (png_origin_e, png_origin_n)
            PNG 左下角 map: (origin_x, origin_y)
        又有:
            E_png = utm_origin_e + origin_x
            N_png = utm_origin_n + origin_y
        => utm_origin_e = E_png - origin_x
           utm_origin_n = N_png - origin_y
        """
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.have_map_info = True

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

    # =============== 坐标变换：UTM -> map ===============

    def utm_to_map_xy(self, E: float, N: float):
        """
        UTM (E, N) -> map (x, y)
        用:
            E = utm_origin_e + x_map
            N = utm_origin_n + y_map
        """
        if self.utm_origin_e is None or self.utm_origin_n is None:
            # 没有 origin 就直接返回 UTM 值（防崩溃；位置可能不准）
            if not self.warned_no_utm_origin:
                self.get_logger().warn(
                    "utm_origin_e/n 尚未就绪，UTM -> map 转换不准确，将直接使用 (E,N) 作为 (x,y)。"
                )
                self.warned_no_utm_origin = True
            return E, N

        x = E - self.utm_origin_e
        y = N - self.utm_origin_n
        return x, y

    # =============== Path 转换工具 ===============

    def convert_utm_path_to_map(self, path_utm: Path) -> Optional[Path]:
        """
        输入: frame_id='utm' 的 Path
        输出: frame_id='map' 的 Path（坐标已转换）
        """
        if not path_utm.poses:
            return None

        path_map = Path()
        path_map.header.stamp = path_utm.header.stamp
        path_map.header.frame_id = self.map_frame_id

        for ps in path_utm.poses:
            E = ps.pose.position.x
            N = ps.pose.position.y
            x, y = self.utm_to_map_xy(E, N)

            new_ps = PoseStamped()
            new_ps.header = path_map.header
            new_ps.pose = ps.pose
            new_ps.pose.position.x = float(x)
            new_ps.pose.position.y = float(y)
            # z、orientation 沿用原来的（假设 UTM / map 方向一致）
            path_map.poses.append(new_ps)

        return path_map

    # =============== 各路径回调 ===============

    def global_path_utm_callback(self, msg: Path):
        """
        /global_path_utm：UTM 坐标，转换到 map 再发布。
        """
        path_map = self.convert_utm_path_to_map(msg)
        if path_map is None:
            return
        self.pub_global_path_in_map.publish(path_map)

    def local_path_callback(self, msg: Path):
        """
        /local_path：UTM 坐标，转换到 map 再发布。
        添加节流以避免 rviz2 卡顿。
        """
        current_time = time.time()
        if current_time - self.last_local_path_time < self.min_publish_interval:
            return
        self.last_local_path_time = current_time

        path_map = self.convert_utm_path_to_map(msg)
        if path_map is None:
            return
        self.pub_local_path_in_map.publish(path_map)


def main(args=None):
    rclpy.init(args=args)
    node = PathVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
