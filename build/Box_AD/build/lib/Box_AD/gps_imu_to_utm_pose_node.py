#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

import pyproj


def make_utm_transformers(lat0, lon0):
    """
    根据参考点经纬度选择 UTM 分区，创建正向 / 反向 Transformer。
    正向: WGS84(lat,lon) -> UTM(E,N)
    反向: UTM(E,N) -> WGS84(lat,lon)
    """
    zone = int((lon0 + 180.0) // 6) + 1  # 1..60
    print(f"[UTM] 参考点经度 lon0={lon0:.6f}，使用 UTM zone {zone}")
    if lat0 >= 0:
        epsg_code = 32600 + zone  # 北半球
    else:
        epsg_code = 32700 + zone  # 南半球

    # 正向：经纬度 -> UTM
    fwd = pyproj.Transformer.from_crs(
        "EPSG:4326",         # WGS84
        f"EPSG:{epsg_code}", # UTM
        always_xy=True       # (lon, lat)
    )
    # 反向：UTM -> 经纬度
    inv = pyproj.Transformer.from_crs(
        f"EPSG:{epsg_code}",
        "EPSG:4326",
        always_xy=True
    )
    return fwd, inv, zone, epsg_code


def yaw_from_quaternion(x, y, z, w):
    """从四元数提取 Z 轴 yaw（标准 ROS 公式）"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class GpsImuToUtmPoseNode(Node):
    def __init__(self):
        super().__init__('gps_imu_to_utm_pose')

        # -------- 参数：是否指定参考经纬度 (lat0,lon0) --------
        # 如果你在建图/生成 map.yaml 时有一个固定的 lat0,lon0，可以在 launch 里传进来
        self.declare_parameter('lat0', 0.0)
        self.declare_parameter('lon0', 0.0)

        # 可选：IMU 安装与车头之间的偏差（单位：度），正值表示“IMU heading 比车头多转这么多度”
        # 例如 IMU 物理上顺时针偏了 5°，则可以设置 yaw_bias_deg = -5.0 做补偿
        self.declare_parameter('yaw_bias_deg', 0.0)

        self.lat0 = float(self.get_parameter('lat0').value)
        self.lon0 = float(self.get_parameter('lon0').value)
        self.yaw_bias_deg = float(self.get_parameter('yaw_bias_deg').value)

        self.fwd_utm = None  # WGS84 -> UTM
        self.inv_utm = None  # UTM   -> WGS84
        self.utm_zone = None
        self.utm_epsg = None

        # 如果用户提供了 lat0,lon0，就直接初始化 transformer
        if abs(self.lat0) > 1e-6 or abs(self.lon0) > 1e-6:
            self.fwd_utm, self.inv_utm, self.utm_zone, self.utm_epsg = \
                make_utm_transformers(self.lat0, self.lon0)
            self.get_logger().info(
                f"已根据参数 lat0={self.lat0:.6f}, lon0={self.lon0:.6f} "
                f"初始化 UTM Transformer (EPSG:{self.utm_epsg})")
        else:
            self.get_logger().warn(
                "未指定 lat0/lon0，将在第一次收到 GPS fix 时自动选择 UTM zone。")

        # -------- 订阅 IMU & GPS --------
        self.sub_imu = self.create_subscription(
            Imu, 'IMU', self.imu_callback, 10)
        self.sub_fix = self.create_subscription(
            NavSatFix, 'gps/fix', self.fix_callback, 10)

        # -------- 发布 UTM Pose --------
        self.pub_pose = self.create_publisher(
            PoseStamped, 'pose_utm', 10)

        # 缓存最近一次 yaw（这里的 yaw 先理解为“IMU heading 的弧度”）和 经纬度
        self.last_yaw = 0.0
        self.have_yaw = False

        self.last_lat = None
        self.last_lon = None
        self.last_alt = None

        self.get_logger().info(
            "GpsImuToUtmPoseNode 启动: 订阅 IMU='IMU', GPS='gps/fix', 发布 'pose_utm' (frame_id='utm')"
        )

    # ---------------- IMU 回调：更新姿态 yaw ----------------
    def imu_callback(self, msg: Imu):
        """
        这里假设 IMU 节点发布的 orientation 是用“heading(0°=北, 东为正)”转出来的四元数，
        即：
            yaw_deg = heading_deg
            q = euler_to_quaternion(roll, pitch, yaw_deg * pi/180)
        那我们从四元数反算的 yaw，其实就是 heading_rad。
        """
        q = msg.orientation
        heading_rad = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.last_yaw = heading_rad   # 暂存为 heading_rad
        self.have_yaw = True

        # 如果已经有经纬度，也可以顺手发一次 pose
        if self.last_lat is not None and self.last_lon is not None:
            self.publish_pose()

    # ---------------- GPS 回调：更新经纬度 ----------------
    def fix_callback(self, msg: NavSatFix):
        if msg.status.status < 0:
            # 无效定位就不处理
            return

        self.last_lat = msg.latitude
        self.last_lon = msg.longitude
        self.last_alt = msg.altitude

        # 如果还没初始化 UTM transformer，这里用第一次有 fix 的点做参考
        if self.fwd_utm is None:
            self.fwd_utm, self.inv_utm, self.utm_zone, self.utm_epsg = \
                make_utm_transformers(self.last_lat, self.last_lon)
            self.get_logger().info(
                f"根据首次 GPS fix 初始化 UTM Transformer: "
                f"lat={self.last_lat:.6f}, lon={self.last_lon:.6f}, EPSG:{self.utm_epsg}"
            )

        if self.have_yaw and self.fwd_utm is not None:
            self.publish_pose()

    # ---------------- 发布 UTM 位姿 ----------------
    def publish_pose(self):
        if self.fwd_utm is None:
            return
        if self.last_lat is None or self.last_lon is None:
            return

        # 经纬度 -> UTM
        try:
            # 注意 always_xy=True: fwd.transform(lon, lat)
            e, n = self.fwd_utm.transform(self.last_lon, self.last_lat)
        except Exception as e_err:
            self.get_logger().error(f"UTM 转换失败: {e_err}")
            return

        # self.last_yaw 此时是“heading_rad”，即 0°=北，东为正（顺时针）
        heading_deg = math.degrees(self.last_yaw)

        # 转成 ROS ENU yaw：0° = 东，逆时针为正
        # yaw_deg = 90 - heading_deg (+ 安装偏差)
        yaw_deg = 90.0 - heading_deg + self.yaw_bias_deg
        yaw = math.radians(yaw_deg)

        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'utm'

        pose_msg.pose.position.x = float(e)
        pose_msg.pose.position.y = float(n)
        pose_msg.pose.position.z = float(self.last_alt if self.last_alt is not None else 0.0)

        half = 0.5 * yaw
        pose_msg.pose.orientation.w = math.cos(half)
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = math.sin(half)

        self.pub_pose.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GpsImuToUtmPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
            pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
