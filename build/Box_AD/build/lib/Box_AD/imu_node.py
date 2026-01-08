#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import threading
import can
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header


def euler_to_quaternion(roll, pitch, yaw):
    """
    欧拉角转四元数
    roll  : 绕X
    pitch : 绕Y
    yaw   : 绕Z
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return qx, qy, qz, qw


class CanImuNode(Node):
    def __init__(self):
        super().__init__('can_imu_node')

        # 1) 发布 IMU
        self.pub_imu = self.create_publisher(Imu, 'IMU', 10)
        # 2) 发布经纬度
        self.pub_fix = self.create_publisher(NavSatFix, 'gps/fix', 10)

        # 打开 CAN（你的设备在 can0 上）
        self.bus_ins = can.interface.Bus(channel='can0', bustype='socketcan')

        # ---------- 缓存 IMU 数据 ----------
        self.acc_x = 0.0
        self.acc_y = 0.0
        self.acc_z = 9.8
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # ---------- 缓存 GNSS 数据 ----------
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0     # 用 0x503 填

        # 启动接收线程
        self.recv_thread = threading.Thread(target=self.recv_can_loop, daemon=True)
        self.recv_thread.start()

        # 定时发 IMU，50Hz
        self.timer = self.create_timer(0.02, self.publish_imu)

    # =========================================================
    #  CAN 接收并解析
    # =========================================================
    def recv_can_loop(self):
        while rclpy.ok():
            msg = self.bus_ins.recv(timeout=1.0)
            if msg is None:
                continue

            can_id = msg.arbitration_id
            data = msg.data

            # ------------ 0x500 加速度 ------------
            if can_id == 0x500:
                # Motorola 大端，每 2 字节一个
                raw_acc_x = (data[0] << 8) | data[1]
                raw_acc_y = (data[2] << 8) | data[3]
                raw_acc_z = (data[4] << 8) | data[5]

                # 文档 factor = 0.0001220... , offset = -4, 单位 g
                factor = 0.0001220703125
                offset = -4.0

                ax_g = raw_acc_x * factor + offset
                ay_g = raw_acc_y * factor + offset
                az_g = raw_acc_z * factor + offset

                # g -> m/s^2
                self.acc_x = ax_g * 9.8
                self.acc_y = ay_g * 9.8
                self.acc_z = az_g * 9.8

                # 文档 Z 向下，ROS Z 向上，翻一下
                self.acc_z = -self.acc_z

            # ------------ 0x501 角速度 ------------
            elif can_id == 0x501:
                raw_gx = (data[0] << 8) | data[1]
                raw_gy = (data[2] << 8) | data[3]
                raw_gz = (data[4] << 8) | data[5]

                # 文档：factor 0.0076293，offset -250，单位 deg/s
                factor = 0.0076293
                offset = -250.0

                gx_deg = raw_gx * factor + offset
                gy_deg = raw_gy * factor + offset
                gz_deg = raw_gz * factor + offset

                # deg/s -> rad/s
                self.gyro_x = math.radians(gx_deg)
                self.gyro_y = math.radians(gy_deg)
                self.gyro_z = math.radians(gz_deg)

                # 同上，Z 轴翻向上
                self.gyro_z = -self.gyro_z

            # ------------ 0x502 姿态角 ------------
            elif can_id == 0x502:
                # 顺序：Pitch(0-1), Roll(2-3), Heading(4-5)
                raw_pitch = (data[0] << 8) | data[1]
                raw_roll = (data[2] << 8) | data[3]
                raw_heading = (data[4] << 8) | data[5]

                factor = 0.010986
                offset = -360.0

                pitch_deg = raw_pitch * factor + offset
                roll_deg = raw_roll * factor + offset
                yaw_deg = raw_heading * factor + offset  # 0° 北，东为正

                self.pitch = math.radians(pitch_deg)
                self.roll = math.radians(roll_deg)
                self.yaw = math.radians(yaw_deg)

            # ------------ 0x504 经纬度 ------------
            elif can_id == 0x504:
                # 你之前的写法：前4字节纬度，后4字节经度，1e-7 deg，-180
                lat_raw = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]
                lon_raw = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7]

                self.lat = lat_raw * 0.0000001 - 180.0
                self.lon = lon_raw * 0.0000001 - 180.0

                # 有经纬度就发一次
                self.publish_fix()

            # ------------ 0x503 高度 + GPS时间 ------------
            elif can_id == 0x503:
                # 说明书 StartBit=24，但常见设备这里就是前4字节是高度
                # 我们保持和 0x504 一样的取法：从 data[0] 开始取 32bit 大端
                height_raw = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]
                # factor 0.001, offset -10000, 单位 m
                height_m = height_raw * 0.001 - 10000.0
                self.alt = height_m

                # 也可以发一次，保证高度更新
                self.publish_fix()

            # 其他 CAN ID 不处理

    # =========================================================
    #  发布 IMU
    # =========================================================
    def publish_imu(self):
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # 姿态
        qx, qy, qz, qw = euler_to_quaternion(self.roll, self.pitch, self.yaw)
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        # 如果你不确定姿态质量，可以打开下面这一行
        # msg.orientation_covariance[0] = -1.0

        # 角速度
        msg.angular_velocity.x = self.gyro_x
        msg.angular_velocity.y = self.gyro_y
        msg.angular_velocity.z = self.gyro_z
        msg.angular_velocity_covariance[0] = 0.01
        msg.angular_velocity_covariance[4] = 0.01
        msg.angular_velocity_covariance[8] = 0.01

        # 线加速度
        msg.linear_acceleration.x = self.acc_x
        msg.linear_acceleration.y = self.acc_y
        msg.linear_acceleration.z = self.acc_z
        msg.linear_acceleration_covariance[0] = 0.1
        msg.linear_acceleration_covariance[4] = 0.1
        msg.linear_acceleration_covariance[8] = 0.1

        self.pub_imu.publish(msg)

    # =========================================================
    #  发布 NavSatFix
    # =========================================================
    def publish_fix(self):
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = 'gps_link'   # 你也可以改成 base_link，看你的TF

        # 状态
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS

        # 经纬度 + 高度
        fix.latitude = self.lat
        fix.longitude = self.lon
        fix.altitude = self.alt
        print(self.lat,self.lon,self.alt)

        # 没有协方差就先标 unknown
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.pub_fix.publish(fix)


def main(args=None):
    rclpy.init(args=args)
    node = CanImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
