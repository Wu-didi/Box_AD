#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose


class PngMapPublisher(Node):
    def __init__(self):
        super().__init__('png_map_publisher')

        # 参数
        self.declare_parameter('image_path', './maps/map_2d.png')
        self.declare_parameter('resolution', 0.1)
        self.declare_parameter('origin_x', 0.0)
        self.declare_parameter('origin_y', 0.0)
        self.declare_parameter('frame_id', 'map')

        image_path = self.get_parameter('image_path').value
        self.resolution = float(self.get_parameter('resolution').value)
        self.origin_x = float(self.get_parameter('origin_x').value)
        self.origin_y = float(self.get_parameter('origin_y').value)
        self.frame_id = self.get_parameter('frame_id').value

        if not os.path.exists(image_path):
            raise FileNotFoundError(f"image_path not found: {image_path}")

        # 读取 PNG
        img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise RuntimeError(f"无法读取 PNG: {image_path}")

        # 关键一步：把图像上下翻转回“以左下角为原点”
        # 因为你在生成 PNG 时，是按图像坐标 (0,0)=左上角 写的
        img = cv2.flip(img, 0)

        self.height, self.width = img.shape
        self.get_logger().info(f"加载 PNG 地图: {image_path}, size = {self.width} x {self.height}")

        # 转成 occupancy grid：白(255) free=0，黑(0) occupied=100，其它=unknown(-1)
        self.occ_grid = np.full((self.height, self.width), -1, dtype=np.int8)
        self.occ_grid[img > 250] = 0      # free
        self.occ_grid[img < 5] = 100      # occupied

        # 使用 TRANSIENT_LOCAL QoS，新订阅者可以收到最后一次发布的 map
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', map_qos)
        self.first_pub = True

        # 静态地图不需要频繁发布，30秒发布一次即可
        self.timer = self.create_timer(30.0, self.publish_map)

        # 启动时立即发布一次
        self.publish_map()

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        info = MapMetaData()
        info.map_load_time = msg.header.stamp
        info.resolution = self.resolution
        info.width = self.width
        info.height = self.height

        origin = Pose()
        origin.position.x = self.origin_x
        origin.position.y = self.origin_y
        origin.position.z = 0.0
        origin.orientation.w = 1.0
        info.origin = origin

        msg.info = info
        msg.data = self.occ_grid.reshape(-1).tolist()

        self.map_pub.publish(msg)

        if self.first_pub:
            self.first_pub = False
            self.get_logger().info(
                f"首次发布 /map: size=({self.width}x{self.height}), "
                f"res={self.resolution}, origin=({self.origin_x:.2f}, {self.origin_y:.2f})"
            )


def main(args=None):
    rclpy.init(args=args)
    node = PngMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
