#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ego_visualizer_node.py

功能：
- 从 map.yaml 解析 PNG 左下角 UTM: (E_png, N_png)
- 订阅 /map (OccupancyGrid)，获取 map 原点 (origin_x, origin_y)
- 计算 map 坐标原点对应的 UTM:
      utm_origin_e = E_png - origin_x
      utm_origin_n = N_png - origin_y
- 订阅 /pose_utm (PoseStamped，frame_id="utm"，position.x=E，position.y=N)
- 在 map 坐标系下发布一个 Marker 表示自车位置和朝向：
    * 默认用 3D 车模 (MESH_RESOURCE)
    * 也可以通过参数改回 CUBE

RViz 使用：
- Fixed Frame 选 "map"
- 添加一个 Marker 显示，topic 选 "/ego_marker"
"""

import os
import re
import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Quaternion
from visualization_msgs.msg import Marker


# ===== 工具函数：rpy -> quat, quat 相乘 =====

def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """roll, pitch, yaw -> 四元数 (ROS 顺序 x,y,z,w)"""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    q = Quaternion()
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    q.w = cr * cp * cy + sr * sp * sy
    return q


def quat_multiply(q1: Quaternion, q2: Quaternion) -> Quaternion:
    """
    四元数乘法：q = q1 * q2
    表示先做 q2 旋转，再做 q1 旋转（和 URDF 里 visual 的 origin 叠加一致）。
    """
    w1, x1, y1, z1 = q1.w, q1.x, q1.y, q1.z
    w2, x2, y2, z2 = q2.w, q2.x, q2.y, q2.z

    q = Quaternion()
    q.w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    q.x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    q.y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    q.z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return q


class EgoVisualizerNode(Node):
    def __init__(self):
        super().__init__('ego_visualizer_node')

        # ------------ 参数 ------------
        self.declare_parameter('map_yaml_path', './maps/map.yaml')
        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('pose_utm_topic', 'pose_utm')

        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('utm_frame_id', 'utm')

        # 车辆尺寸（作为 CUBE 尺寸，或者 mesh 的缩放）
        self.declare_parameter('car_length', 2.0)
        self.declare_parameter('car_width', 1.0)
        self.declare_parameter('car_height', 0.75)

        # CUBE 颜色
        self.declare_parameter('color_r', 0.0)
        self.declare_parameter('color_g', 1.0)
        self.declare_parameter('color_b', 1.0)
        self.declare_parameter('color_a', 0.9)

        # ===== 使用 mesh 模型 =====
        self.declare_parameter('use_mesh', False)
        self.declare_parameter(
            'mesh_resource',
            './car_model/CAR_original.dae'
        )
        self.declare_parameter('mesh_scale_x', 1.0)
        self.declare_parameter('mesh_scale_y', 1.0)
        self.declare_parameter('mesh_scale_z', 1.0)

        # ===== 新增：mesh 固定姿态修正（照抄 Autoware 的 default.urdf）=====
        # <origin xyz="0 0 0.0" rpy="1.57 0 4.71" />
        self.declare_parameter('mesh_roll', 1.57)   # 约 +90°
        self.declare_parameter('mesh_pitch', 0.0)
        self.declare_parameter('mesh_yaw', 4.71)    # 约 270° = -90°

        # ===== 可视化发布频率控制 =====
        self.declare_parameter('publish_rate', 5.0)  # 最大发布频率 Hz

        self.map_yaml_path = self.get_parameter('map_yaml_path').get_parameter_value().string_value
        self.map_topic = self.get_parameter('map_topic').value
        self.pose_utm_topic = self.get_parameter('pose_utm_topic').value

        self.map_frame_id = self.get_parameter('map_frame_id').value
        self.utm_frame_id = self.get_parameter('utm_frame_id').value

        self.car_length = float(self.get_parameter('car_length').value)
        self.car_width = float(self.get_parameter('car_width').value)
        self.car_height = float(self.get_parameter('car_height').value)

        self.color_r = float(self.get_parameter('color_r').value)
        self.color_g = float(self.get_parameter('color_g').value)
        self.color_b = float(self.get_parameter('color_b').value)
        self.color_a = float(self.get_parameter('color_a').value)

        self.use_mesh = bool(self.get_parameter('use_mesh').value)
        self.mesh_resource = self.get_parameter('mesh_resource').value
        self.mesh_scale_x = float(self.get_parameter('mesh_scale_x').value)
        self.mesh_scale_y = float(self.get_parameter('mesh_scale_y').value)
        self.mesh_scale_z = float(self.get_parameter('mesh_scale_z').value)

        self.mesh_roll = float(self.get_parameter('mesh_roll').value)
        self.mesh_pitch = float(self.get_parameter('mesh_pitch').value)
        self.mesh_yaw = float(self.get_parameter('mesh_yaw').value)

        # 预先算好 mesh 固定姿态的四元数
        self.mesh_q_fix = rpy_to_quaternion(self.mesh_roll,
                                            self.mesh_pitch,
                                            self.mesh_yaw)

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

        self._utm_warned = False  # 防止日志刷屏

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

        # ------------ 发布 Marker ------------
        self.marker_pub = self.create_publisher(
            Marker,
            'ego_marker',
            10
        )

        self.get_logger().info(
            f"EgoVisualizerNode 启动：\n"
            f"  map_yaml_path   = {self.map_yaml_path}\n"
            f"  map_topic       = {self.map_topic}\n"
            f"  pose_utm_topic  = {self.pose_utm_topic}\n"
            f"  use_mesh        = {self.use_mesh}\n"
            f"  mesh_resource   = {self.mesh_resource}\n"
            f"  mesh_rpy        = ({self.mesh_roll:.2f}, "
            f"{self.mesh_pitch:.2f}, {self.mesh_yaw:.2f})\n"
            f"  输出 Marker: /ego_marker, frame_id={self.map_frame_id}"
        )

    # ================== 解析 map.yaml ==================

    def load_png_origin_utm(self, yaml_path: str):
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
                "将无法计算 utm_origin_e/n，自车位置可视化会不准确。"
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

    def utm_to_map_xy(self, E: float, N: float):
        if self.utm_origin_e is None or self.utm_origin_n is None:
            if not self._utm_warned:
                self.get_logger().warn(
                    "utm_origin_e/n 尚未就绪，暂时无法准确将 UTM 坐标转换到 map；"
                    "请确认 /map 已发布且 map.yaml 中含有 PNG origin UTM 注释。"
                )
                self._utm_warned = True
            return E, N

        x = E - self.utm_origin_e
        y = N - self.utm_origin_n
        return x, y

    # ================== 生成车辆 Marker ==================

    def make_ego_marker(self, x_map: float, y_map: float, base_orientation: Quaternion) -> Marker:
        """
        在 map 坐标系下生成一个代表自车的 Marker。
        base_orientation 是 /pose_utm 的四元数（只含 yaw）。
        仅当使用 mesh 时，最终姿态 = base_orientation * mesh_q_fix
        """
        m = Marker()
        m.header.frame_id = self.map_frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "ego_vehicle"
        m.id = 0

        if self.use_mesh:
            # 叠加 mesh 固定姿态偏移
            ori = quat_multiply(base_orientation, self.mesh_q_fix)
            m.type = Marker.MESH_RESOURCE
            m.mesh_resource = self.mesh_resource
            m.mesh_use_embedded_materials = True

            m.scale.x = self.mesh_scale_x
            m.scale.y = self.mesh_scale_y
            m.scale.z = self.mesh_scale_z

            m.color.a = self.color_a  # 只控制透明度
            m.pose.orientation = ori
        else:
            # CUBE 直接使用原始朝向，不需要姿态修正
            m.type = Marker.CUBE
            m.scale.x = self.car_length
            m.scale.y = self.car_width
            m.scale.z = self.car_height
            m.color.r = self.color_r
            m.color.g = self.color_g
            m.color.b = self.color_b
            m.color.a = self.color_a
            m.pose.orientation = base_orientation

        m.action = Marker.ADD

        # 自车位置使用固定ID覆盖，不设置过期（永不过期）
        m.lifetime.sec = 0
        m.lifetime.nanosec = 0  # 永不过期，靠固定ID覆盖更新

        m.pose.position.x = float(x_map)
        m.pose.position.y = float(y_map)
        m.pose.position.z = self.car_height * 0.5

        return m

    # ================== /pose_utm 回调 ==================

    def pose_utm_callback(self, msg: PoseStamped):
        # 节流：限制发布频率
        current_time = time.time()
        if current_time - self.last_publish_time < self.min_publish_interval:
            return
        self.last_publish_time = current_time

        E = msg.pose.position.x
        N = msg.pose.position.y

        x_map, y_map = self.utm_to_map_xy(E, N)
        marker = self.make_ego_marker(x_map, y_map, msg.pose.orientation)
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = EgoVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
