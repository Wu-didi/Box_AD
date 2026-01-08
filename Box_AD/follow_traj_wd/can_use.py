import can
import logging
from math import  cos, sin, asin, sqrt, degrees, atan2
from pyproj import CRS, Transformer
import math

def normalize_angle(angle):
    normalized_angle = angle % 360
    if normalized_angle < 0:
        normalized_angle += 360
    return normalized_angle


wgs84 = CRS("EPSG:4326")
utm_zone_number = 50  # 根据实际情况选择合适的 UTM 区域
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
        self.bus_vcu = can.interface.Bus(channel='can1', bustype='socketcan')
        self.ego_lon = 31.8925019
        self.ego_lat = 118.8171577
        self.ego_yaw_deg = 90
        self.ego_yaw = math.radians(self.ego_yaw_deg)
        self.ego_v = 3
        self.ego_a = 0
        self.eps_mode = 2
        self.auto_driver_allowed = False

    def read_ins_info(self):
        """获取惯导的主车信息"""
        print("read ins info")
        message_ins = self.bus_ins.recv()
        message_vcu = self.bus_vcu.recv()
        if message_ins is not None and message_ins.arbitration_id == 0x504:
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
            # logging.info(f"ego_x:{ego_x},ego_y:{ego_y}")

        if message_ins is not None and message_ins.arbitration_id == 0x505:
            speed_data = message_ins.data
            # 北向速度
            INS_NorthSpd =  (speed_data[0] << 8) | speed_data[1]
            INS_NorthSpd =   INS_NorthSpd * 0.0030517 - 100    # m/s
            INS_NorthSpd *= 3.6
            # 东向速度
            INS_EastSpd =  (speed_data[2] << 8) | speed_data[3]
            INS_EastSpd =   INS_EastSpd * 0.0030517 - 100    # m/s
            INS_EastSpd *= 3.6
            # 地向速度
            INS_ToGroundSpd =  (speed_data[4] << 8) | speed_data[5]
            INS_ToGroundSpd =   INS_ToGroundSpd * 0.0030517 - 100    # m/s
            INS_ToGroundSpd *= 3.6
                    
            speed =  sqrt(INS_EastSpd**2 + INS_NorthSpd**2 + INS_ToGroundSpd**2)
                    
            self.ego_v = speed 
            # self.ego_v = 2.7

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
            
            
        if message_ins is not None and message_ins.arbitration_id == 0x500:
            acc_data = message_ins.data
            # 北向速度
            ACC_X =  (acc_data[0] << 8) | acc_data[1]
            ACC_X =   (ACC_X * 0.0001220703125 - 4) * 9.8   # g
            self.ego_a = ACC_X
        
        if message_vcu is not None and message_vcu.arbitration_id == 0x15C:
            allow_value = message_vcu.data[2] & 0x01
            self.auto_driver_allowed = (allow_value == 1)

        if message_vcu is not None and message_vcu.arbitration_id == 0x124:
            eps_mode = (message_vcu.data[6] >> 4) & 0x03
            self.eps_mode = eps_mode
        print("finish read ins info")

    def publish_planner_action(self, action, id, action_type, mod, enable):
        """将规划动作发布到CAN"""

        if action_type == "angle":    
            # 数据缩放和转换
            data1 = int((action - (-738)) / 0.1)  # 确保data1根据传入angle正确计算
            data1_high = (data1 >> 8) & 0xFF    # data1的高8位
            data1_low = data1 & 0xFF            # data1的低8位

            data2 = int(mod) & 0x03             # data2缩放到2位范围，0-3
            data3 = int(250 / 10) & 0xFF     # data3缩放到8位范围，0-255, angle_spd=100
            data4 = int(enable) & 0x01          # data4缩放到1位范围，0或1
                
            # 构建发送数据，确保8字节长度
            data = [data1_high, data1_low, data2, data3, data4, 0, 0, 0]

            msg = can.Message(arbitration_id=id, data=data, is_extended_id=False)
            self.bus_vcu.send(msg)
        
        if action_type == "acc":
            auto_drive_cmd_bits = mod & 0x07  # 取最低3位
            # Auto speed cmd（位3-7）
            # 首先对速度进行缩放和偏移
            # 期望速度 单位km/h
            desired_speed = action[0] 
            # desired_speed = 3
            speed_scaled = int(desired_speed) & 0x1F  # 取5位（位3-7）
            # 组合BYTE0
            byte0 = (speed_scaled << 3) | auto_drive_cmd_bits

            # BYTE1-BYTE2（需求方向盘转角）
            # 需要根据具体缩放因子和偏移量进行计算，假设缩放因子为0.1，偏移量为0
            # action[1] = 396
            logging.info(f"final turn angle:{action[1]}")
            angle_scaled = int((action[1] - (-500)) / 0.1) & 0xFFFF  # 16位
            byte1 = (angle_scaled >> 8) & 0xFF  # 高8位
            byte2 = angle_scaled & 0xFF         # 低8位

            # BYTE3（需求制动减速度）
            # 进行缩放和偏移
            acc  =  action[2]
            # acc = 0
            acc_scaled = int((acc - (-4)) / 1) & 0xFF  # 假设缩放因子1，偏移量-4

            # 构建发送数据，剩余字节填充0
            data_666 = [byte0, byte1, byte2, acc_scaled, 0, 0, 0, 0]
            
            msg = can.Message(arbitration_id=id, data=data_666, is_extended_id=False)
            # 发送CAN消息
            self.bus_vcu.send(msg)
            # time.sleep(0.01)