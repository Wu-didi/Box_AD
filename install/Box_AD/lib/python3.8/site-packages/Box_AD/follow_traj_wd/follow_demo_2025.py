'''two path select'''
import can
from math import radians, cos, sin, asin, sqrt, degrees, atan2
import matplotlib.pyplot as plt
import numpy as np
import threading
import time
import math
# import cv2
import csv
from pyproj import Proj
import matplotlib.pyplot as plt
# from perception.yolov8_detect import CameraObjectDetector
from can_use import Can_use
from utils import ISGSpeedFilter

import pyproj
import time 



# 车辆参数
VEHICLE_WIDTH = 1.9   # m
VEHICLE_LENGTH = 4.5  # m
WHEEL_FACTOR = 7.2
manual_triggered = False
stop_record = False
mod_666 = 0
mod_AE = 0

lonlat2xy_old = Proj('+proj=tmerc +lon_0=118.8170043 +lat_0=31.8926311 +ellps=WGS84')
def read_csv(csv_file_path):
    traj_data = []
    x = []
    y = []
    # 打开CSV文件并读取内容  
    with open(csv_file_path, mode='r', newline='') as file:  
        csv_reader = csv.reader(file)  
        
        # 跳过标题行（如果有的话）  
        headers = next(csv_reader, None)  # 这行代码会读取第一行，如果第一行是标题则跳过  
        
        # 读取每一行数据并添加到列表中  
        for row in csv_reader:  
            # 将每一行的数据转换为整数或浮点数（根据具体情况选择）  
            # 假设x坐标、y坐标、航向角和速度都是浮点数  
            x_coord = float(row[0])  
            y_coord = float(row[1])
            x_coord_utm, y_coord_utm = lonlat2xy_old(x_coord, y_coord, inverse=False)  
            heading = float(row[2])  
            x.append(x_coord_utm)
            y.append(y_coord_utm)
            # 将这些信息存储为一个列表，并添加到data_list中  
            data_row = [x_coord, y_coord, heading]  
            traj_data.append(data_row)
    plt.scatter(x , y)
    plt.scatter([x[-1]], [y[-1]], color="red") # end
    plt.scatter(12.27, -3.27, color="brown")
    plt.scatter([x[0]], [y[0]], color="black") # start
    plt.title('reference_trajectory_utm')  
    plt.xlabel('longitudinal')  
    plt.ylabel('latitudinal')
    plt.savefig('ref_traj_utm.png')
    return traj_data


# 计算轨迹点与障碍物的距离
def calculate_distance(p1, p2):
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

class VehicleTrajectoryFollower:
    def __init__(self, target_index=0):
        """
        初始化，读取主轨迹和备选轨迹点
        :param main_trajectory_csv: 包含主轨迹点的CSV文件路径
        :param alternate_trajectory_csv: 包含备选轨迹点的CSV文件路径
        """
        self.current_trajectory = None
        self.is_using_alternate = False  # 标志当前是否在使用备选轨迹
        self.main_closest_index = 0
        self.alternate_closest_index = 0
        self.wheelbase = 3.5
        self.offset_target_index = 3
        self.target_index = 0
        self.should_stop = False  # 增加停车标志位
        self.obstacle_detected = False  # 标记是否检测到障碍物
        self.previous_turn_angle = 0
        self.far_previous_turn_angle = 0
        self.max_turn_rate = 6  # 限制每次转向角的最大变化速率（度）
        self.far_index = 10  # 远处的目标点，用于控制速度
        self.control_speed_the = 30 #用於判斷遠處目標點和當前head的差值是否超過該值，然後進行速度的處理
    
    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        dLon = math.radians(lon2 - lon1)
        lat1 = math.radians(lat1)
        lat2 = math.radians(lat2)
        x = math.sin(dLon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(dLon))
        initial_bearing = math.atan2(x, y)
        initial_bearing = math.degrees(initial_bearing)
        compass_bearing = (initial_bearing + 360) % 360
        return compass_bearing

    def calculate_distance(self, lat1, lon1, lat2, lon2):
        R = 6371000  # 地球半径，单位为米
        dLat = math.radians(lat2 - lat1)
        dLon = math.radians(lon2 - lon1)
        a = math.sin(dLat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dLon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c
        return distance
    
    def find_closest_point_index_bank(self, current_lat, current_lon, trajectory=None):
        """
        找到距离当前车辆位置最近的轨迹点索引
        :param current_lat: 车辆当前纬度
        :param current_lon: 车辆当前经度
        :param trajectory: 要查找的轨迹，默认使用当前轨迹
        :return: 距离最近的轨迹点索引
        """
        if trajectory is None:
            trajectory = self.current_trajectory
        closest_index = 0
        min_distance = float('inf')
        if self.closest_index == 0:
            max_bound = len(trajectory)-1
        else:
            max_bound = 200
        for i, (lon, lat, _) in enumerate(trajectory[self.closest_index:self.closest_index+max_bound]):  # 经度在前，纬度在后
            distance = self.calculate_distance(current_lat, current_lon, lat, lon)
            if distance < min_distance:
                min_distance = distance
                closest_index = i
        return closest_index+self.closest_index

    
    def find_closest_point_index(self, current_lat, current_lon, trajectory=None,is_main=True):
        """
        找到距离当前车辆位置最近的轨迹点索引
        :param current_lat: 车辆当前纬度
        :param current_lon: 车辆当前经度
        :param trajectory: 要查找的轨迹，默认使用当前轨迹
        :return: 距离最近的轨迹点索引
        """
        if trajectory is None:
            trajectory = self.current_trajectory
        closest_index_temp = 0
        min_distance = float('inf')
        
        if is_main:
            closest_index = self.main_closest_index
        else:
            closest_index = self.alternate_closest_index
        
        if closest_index == 0:
            max_bound = len(trajectory)-1
        else:
            max_bound = 200
        # print(max(closest_index,0),min(closest_index+max_bound, len(trajectory)-1))
        for i, (lon, lat, _) in enumerate(trajectory[closest_index:max(closest_index+max_bound,len(trajectory))]):  # 经度在前，纬度在后
            distance = self.calculate_distance(current_lat, current_lon, lat, lon)
            if distance < min_distance:
                min_distance = distance
                closest_index_temp = i
                
        return closest_index_temp+closest_index
    
    
    def adjust_position_to_front_axle(self, rear_lat, rear_lon, heading):
        """
        根据后轴中心的经纬度和heading计算前轴的经纬度
        :param rear_lat: 后轴的纬度
        :param rear_lon: 后轴的经度
        :param heading: 车辆的航向角，相对于正北方向
        :return: 前轴的经纬度 (lat, lon)
        """
        # 先将heading转换为弧度
        heading_rad = math.radians(heading)

        # 计算纬度上的变化，假设1度纬度大约为111,320米
        delta_lat = (self.wheelbase / 6371000) * math.cos(heading_rad)

        # 计算经度上的变化，假设经度的变化随着纬度而变化，纬度越高，1度经度的实际距离越小
        delta_lon = (self.wheelbase / 6371000) * math.sin(heading_rad) / math.cos(math.radians(rear_lat))

        # 计算前轴的经纬度
        front_lat = rear_lat + math.degrees(delta_lat)
        front_lon = rear_lon + math.degrees(delta_lon)

        return front_lat, front_lon
    
    # 用于平滑转角
    def smooth_turn_angle(self, turn_angle):
        # 限制转向角的最大变化速率
        angle_difference = turn_angle - self.previous_turn_angle
        if angle_difference > self.max_turn_rate:
            update_turn_angle = self.previous_turn_angle + 4
        elif angle_difference < -self.max_turn_rate:
            update_turn_angle = self.previous_turn_angle - 4
        else:
            update_turn_angle = turn_angle
        # 更新上一次的转向角
        self.previous_turn_angle = update_turn_angle
        return update_turn_angle

    def calculate_turn_angle(self, current_position, current_heading, current_speed, offset_target_index = None):
        if  self.current_trajectory == None:
            return 'no_current_trajectory'
        
        if offset_target_index is not None:
            target_index_obstacle = offset_target_index
        else:
            target_index_obstacle = self.offset_target_index
        if current_speed >= 20:
            target_index_obstacle = target_index_obstacle
        else: 
            target_index_obstacle = 3
            
        # print("==============", target_index_obstacle)
        current_lat, current_lon, _ = current_position
        # 根据后轴的位置和heading调整得到前轴的位置
        front_lat, front_lon = self.adjust_position_to_front_axle(current_lat, current_lon, current_heading)
        
        # 找到距离最近的点的索引
        self.closest_index = self.find_closest_point_index(front_lat, front_lon)
        print("self closest index",self.closest_index)
        target_index = min(self.closest_index + target_index_obstacle, len(self.current_trajectory) - 1)  # 防止超出范围
        self.target_index = target_index
        print("self.target idex:  ",self.target_index)
        next_lon, next_lat, _ = self.current_trajectory[target_index]  # 注意经纬度顺序
        
        # 计算目标点相对当前位置的方位角
        desired_heading = self.calculate_bearing(current_lat, current_lon, next_lat, next_lon)
        # 计算转向角
        turn_angle = (desired_heading - current_heading + 360) % 360
        if turn_angle > 180:
            turn_angle -= 360        
        # 映射到方向盘转角
        if turn_angle * WHEEL_FACTOR > 460:
            turn_angle = 460
        elif turn_angle * WHEEL_FACTOR < -460:
            turn_angle = -460
        else:
            turn_angle = turn_angle * WHEEL_FACTOR    
        
        # turn_angle = self.smooth_turn_angle(turn_angle)
        
        return turn_angle

    # 计算期望速度和加速度
    def calculate_speedAndacc(self, turn_angle, current_position, current_speed, is_obstacle = False, points_num_threshold=20,
                              high_speed = 30,
                              low_speed = 5):
        if current_speed < 1:
            speed = high_speed
            acc = 0
            return speed, acc
        
        if abs(turn_angle) >= 50:
            if current_speed >= 15:
                speed = low_speed
                acc = -1
            else:
                speed = low_speed
                acc = 0
            return speed, acc 
        
        current_lat, current_lon, current_heading = current_position
        next_lon, next_lat,  next_heading = self.current_trajectory[min(self.closest_index + self.far_index, len(self.current_trajectory) - 1)]  # 注意经纬度顺序
        print(f"current_heading: {current_heading}, next_heading: {next_heading}")
        
        if current_heading <= 0:
            current_heading += 360
        diff_heading = abs(current_heading-next_heading)
        print(f"current_heading: {current_heading}, next_heading: {next_heading}, diff_heading: {diff_heading}")
        # 计算目标点相对当前位置的方位角
        far_desired_heading = self.calculate_bearing(current_lat, current_lon, next_lat, next_lon)

        # 计算转向角
        far_turn_angle = (far_desired_heading - current_heading + 360) % 360
        if far_turn_angle > 180:
            far_turn_angle -= 360        
        # 映射到方向盘转角
        if far_turn_angle * WHEEL_FACTOR > 460:
            far_turn_angle = 460
        elif far_turn_angle * WHEEL_FACTOR < -460:
            far_turn_angle = -460
        else:
            far_turn_angle = far_turn_angle * WHEEL_FACTOR    
        print("====== far_turn_angle: ",far_turn_angle)
        if abs(far_turn_angle) >= 30 or diff_heading >=120:
            print("========================将会发生转弯，减速！减速！减速！=============================")
            if current_speed >= high_speed-5:
                speed = high_speed-10
                acc = -2
            elif current_speed >= low_speed+5:
                speed = low_speed
                acc = -1
            else:
                speed = low_speed
                acc = 0
        else:
            speed = high_speed
            acc = 0
        
        if is_obstacle:
            print("find obstacle reduce speed")
            if current_speed >= low_speed+5:
                speed = low_speed
                acc = -3
            else:
                speed = low_speed
                acc = 0
        return speed, acc    
