import can
import logging
from math import  cos, sin, asin, sqrt, degrees, atan2
from pyproj import CRS, Transformer
import math
import numpy as np

def normalize_angle(angle):
    normalized_angle = angle % 360
    if normalized_angle < 0:
        normalized_angle += 360
    return normalized_angle


wgs84 = CRS("EPSG:4326")
utm_zone_number = 49  # 根据实际情况选择合适的 UTM 区域
utm_crs = CRS(f"EPSG:{32600 + utm_zone_number}")  # 例如，UTM Zone 50N 对应 EPSG:32650
projector_to_utm = Transformer.from_crs(wgs84, utm_crs, always_xy=True)
projector_to_wgs84 = Transformer.from_crs(utm_crs, wgs84, always_xy=True)
def latlon_to_utm(lon, lat):
    """将经纬度转换为 UTM 坐标"""
    x, y = projector_to_utm.transform(lon, lat)
    return x, y

def smooth_yaw_iter(previous_yaw, new_yaw):
    """
    Smooth the yaw angle based on the previous yaw to ensure continuity.

    :param previous_yaw: (float) Previous yaw angle in radians
    :param new_yaw: (float) New yaw angle in radians (not yet normalized)
    :return: (float) Smoothed and normalized yaw angle in radians within (-pi, pi]
    """
    dyaw = new_yaw - previous_yaw

    # 调整 dyaw，使其在 [-pi, pi] 范围内
    dyaw = (dyaw + np.pi) % (2.0 * np.pi) - np.pi

    # 平滑后的 yaw
    smoothed_yaw = previous_yaw + dyaw

    return smoothed_yaw




class Can_use:
    def __init__(self):
        self.bus_ins = can.interface.Bus(channel='can0', bustype='socketcan')
        self.ego_lon = 31.8925019
        self.ego_lat = 118.8171577
        self.ego_yaw_deg = 90
        self.ego_yaw = math.radians(self.ego_yaw_deg)
        self.ego_yaw_rad = 0
        self.ego_v = 3
        self.ego_a = 0
        self.eps_mode = 2
        self.auto_driver_allowed = False
        self.ego_x = 0
        self.ego_y = 0
        self.flag = False

    def read_ins_info(self):
        """获取惯导的主车信息"""
        message_ins = self.bus_ins.recv()
        # print("in read ins info")
        # if message_ins is None:
        #     self.flag = False
        # else:
        #     self.flag = True\\\
        self.flag = False
        if message_ins is not None and message_ins.arbitration_id == 0x504:
            self.flag = True
            # 直接获取数据字节
            can_data = message_ins.data
            # 解析前4个字节为纬度
            INS_Latitude = (can_data[0] << 24) | (can_data[1] << 16) | (can_data[2] << 8) | can_data[3]
            # 解析后4个字节为经度
            INS_Longitude = (can_data[4] << 24) | (can_data[5] << 16) | (can_data[6] << 8) | can_data[7]
            INS_Latitude = INS_Latitude * 0.0000001 - 180
            INS_Longitude = INS_Longitude * 0.0000001 - 180
            # print(f"INS_Latitude:{INS_Latitude},INS_Longitude:{INS_Longitude}")


            self.ego_lon = INS_Longitude
            self.ego_lat = INS_Latitude
            ego_x, ego_y = latlon_to_utm(INS_Longitude, INS_Latitude)
            
            self.ego_x = ego_x
            self.ego_y = ego_y
            # print(f"ego_x:{ego_x},ego_y:{ego_y}")


        if message_ins is not None and message_ins.arbitration_id == 0x502:
            # self.ego_yaw = angle
            Angle_data = message_ins.data
            HeadingAngle =  (Angle_data[4] << 8) | Angle_data[5]
            HeadingAngle = HeadingAngle * 0.010986 - 360
            self.ego_yaw_deg = HeadingAngle 
            
            # 将航向角从 INS 坐标系转换为 UTM 坐标系
            # INS: 0° 正北，东为正
            # UTM: 0° 正东，北为正
            # 转换公式：UTM_yaw = 90 - INS_yaw
            utm_yaw_deg = 90 - HeadingAngle
            # print("utm yaw deg: ",utm_yaw_deg)
            utm_yaw_deg = normalize_angle(utm_yaw_deg)               
            
            utm_yaw_rad = math.radians(utm_yaw_deg)
            
            # 平滑航向角
            # smoothed_yaw = smooth_yaw_iter(self.previous_yaw, utm_yaw_rad)
            smoothed_yaw_rad = utm_yaw_rad
            self.previous_yaw_rad = smoothed_yaw_rad
            self.ego_yaw_rad = smoothed_yaw_rad
