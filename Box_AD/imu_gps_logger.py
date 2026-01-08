#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import csv

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Imu, NavSatFix


def to_ns(stamp):
    # stamp 是 builtin_interfaces/Time
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class ImuGpsLogger(Node):
    def __init__(self):
        super().__init__('imu_gps_logger')

        imu_topic = 'IMU'
        gps_topic = 'gps/fix'

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        os.makedirs('logs', exist_ok=True)

        # IMU CSV
        self.imu_csv_path = os.path.join('logs', 'imu.csv')
        imu_exists = os.path.exists(self.imu_csv_path)
        self.imu_csv = open(self.imu_csv_path, 'a', newline='')
        self.imu_writer = csv.writer(self.imu_csv)
        if not imu_exists:
            # 多写一列 timestamp_ns，和你的 pcd 对应
            self.imu_writer.writerow([
                'timestamp_ns', 'stamp_sec', 'stamp_nanosec',
                'ori_x', 'ori_y', 'ori_z', 'ori_w',
                'ang_x', 'ang_y', 'ang_z',
                'lin_x', 'lin_y', 'lin_z'
            ])

        # GPS CSV
        self.gps_csv_path = os.path.join('logs', 'gps.csv')
        gps_exists = os.path.exists(self.gps_csv_path)
        self.gps_csv = open(self.gps_csv_path, 'a', newline='')
        self.gps_writer = csv.writer(self.gps_csv)
        if not gps_exists:
            self.gps_writer.writerow([
                'timestamp_ns', 'stamp_sec', 'stamp_nanosec',
                'latitude', 'longitude', 'altitude'
            ])

        self.create_subscription(Imu, imu_topic, self.imu_callback, sensor_qos)
        self.create_subscription(NavSatFix, gps_topic, self.gps_callback, sensor_qos)

        self.get_logger().info('IMU/GPS logger started')

    def imu_callback(self, msg: Imu):
        ts_ns = to_ns(msg.header.stamp)
        self.imu_writer.writerow([
            ts_ns,
            msg.header.stamp.sec,
            msg.header.stamp.nanosec,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ])
        self.imu_csv.flush()

    def gps_callback(self, msg: NavSatFix):
        ts_ns = to_ns(msg.header.stamp)
        self.gps_writer.writerow([
            ts_ns,
            msg.header.stamp.sec,
            msg.header.stamp.nanosec,
            msg.latitude,
            msg.longitude,
            msg.altitude,
        ])
        self.gps_csv.flush()

    def destroy_node(self):
        try:
            self.imu_csv.close()
        except Exception:
            pass
        try:
            self.gps_csv.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuGpsLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
