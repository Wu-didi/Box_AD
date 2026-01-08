import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
import time
import sys
sys.path.append('/home/nvidia/vcii/wudi/Box_AD/Box_AD/follow_traj_wd')
from follow_demo_2025 import VehicleTrajectoryFollower
from utils import ISGSpeedFilter
from can_use import Can_use
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
import math
import csv  
from pyproj import Proj

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
    return traj_data  


class FollowNode(Node):
    def __init__(self):
        super().__init__('Follow_node')
        self.publisher_ = self.create_publisher(
            Float32MultiArray, 
            'my_planner_action', 
            1
        )
        self.vs_subscription = self.create_subscription(
            Float32MultiArray,
            'vehicle_state',
            self.vs_callback,
            1
        )
        self.eps_subscription = self.create_subscription(
            Int32,
            'eps_mode',
            self.eps_callback,
            1
        )
        
        # 订阅到轨迹
        self.trajectory = self.create_subscription(
            PoseArray,
            'trajectory',
            self.trajectory_callback,
            10
        )
        
         # 订阅到轨迹
        self.reduce_speed = self.create_subscription(
            String,
            'obstacle_reduce_speed',
            self.obstacle_reduce_speed_callback,
            10
        )
        
        # 用于存储转换后的轨迹点数据 [[x, y, heading], ...]
        self.traj_data = []
        
        self.manual_triggered = True
        self.mode_AE = 1
        self.mode_666 = 1
        self.eps_subscription
        self.vs_subscription  # prevent unused variable warning
        self.obstacle_reduce_speed = False
        


        self.follower = VehicleTrajectoryFollower()
        self.filter = ISGSpeedFilter()
        self.latest_eps_mode = None
        self.can_use = Can_use()
        
    def eps_callback(self, msg):
        self.latest_eps_mode = msg.data
        
    def vs_callback(self, msg):
        if self.latest_eps_mode is None:
            self.get_logger().warn("尚未接收到eps_mode，跳过一次控制")
            return
        # self.get_logger().info(f"[vs_callback] Received state: {msg.data}")
        # self.get_logger().info(f"[vs_callback] EPS mode: {self.latest_eps_mode}")
        eps_mode = self.latest_eps_mode
        start = time.time()
        for i in range(20):
            self.can_use.read_ins_info()
        # ego_lat = msg.data[0]
        # ego_lon = msg.data[1]
        # ego_yaw = msg.data[2]
        # ego_v = msg.data[3]
        ego_lat = self.can_use.ego_lat
        ego_lon = self.can_use.ego_lon
        ego_yaw = self.can_use.ego_yaw_deg
        ego_v   = self.can_use.ego_v
        if eps_mode != 3 and self.manual_triggered:
            self.mode_AE = 1
            self.mode_666 = 1
        if eps_mode ==3:
            self.mode_AE = 3
            self.mode_666 = 0
            self.manual_triggered = False
        if self.mode_AE == 1 and self.mode_666 == 1:
            if ego_lon is not None and ego_lat is not None:
                turn_angle = self.follower.calculate_turn_angle(
                    (ego_lat, ego_lon, ego_yaw), ego_yaw, ego_v)
                if turn_angle == "no_current_trajectory":
                    self.get_logger().info(f"没有轨迹 or 轨迹不可通行，需要停车")
                    # 需要紧急停车
                    self.frame = [float(0),    # 分别是期望速度、角度、减速度等级
                                float(0), 
                                float(-3)]
                else:
                    # filtered_angle = self.filter.update_speed(turn_angle)
                    filtered_angle = turn_angle
                    desired_speed, desired_acc = self.follower.calculate_speedAndacc(
                            turn_angle, (ego_lat, ego_lon, ego_yaw), ego_v, is_obstacle = self.obstacle_reduce_speed)
                    # logging.info(f'trun angle: {turn_angle}, filter angle: {filtered_angle}')
                    self.frame = [float(desired_speed), 
                                float(filtered_angle), 
                                float(desired_acc)]
                    
                    
                planner_frame = Float32MultiArray()
                planner_frame.data = self.frame
                # self.get_logger().info(f"[vs_callback] Send frame: {planner_frame.data}")
                self.publisher_.publish(planner_frame)
        elapsed_time = time.time() - start
        # self.get_logger().info(f"calc time:{elapsed_time:.6f} [sec]")

    def trajectory_callback(self, msg: PoseArray):
        """
        将订阅到的 PoseArray 转换为和 CSV 中相同的格式：
        [[x_utm, y_utm, heading], [x_utm, y_utm, heading], ...]
        """
        traj_data = []
        # 为空，表示没有轨迹或者轨迹不可用，需要停止
        if len(msg.poses) == 0:
            print("len of msg.poses: " , len(msg.poses))
            self.follower.current_trajectory = None
        else:
            for pose in msg.poses:
                x = pose.position.x
                y = pose.position.y
                
                # 提取四元数 (只关心 z, w，假设 x=y=0)
                qz = pose.orientation.z
                qw = pose.orientation.w
                
                # 反向获取 yaw（弧度）
                yaw_rad = 2.0 * math.atan2(qz, qw)
                # 将弧度转为度数
                heading_deg = math.degrees(yaw_rad)
                # 归一化到 [0, 360)
                heading_deg = (heading_deg + 360) % 360
                traj_data.append([x, y, heading_deg])
                
            # 更新最新的轨迹
            self.follower.current_trajectory = traj_data

    def obstacle_reduce_speed_callback(self, msg):
        data = str(msg.data)
        if data == 'Yes':
            self.obstacle_reduce_speed = True
        else:
            self.obstacle_reduce_speed = False
        self.get_logger().info(f"是否因为障碍物减速：{data}")
    

    


def main(args=None):
    rclpy.init(args=args)
    follow_node = FollowNode()
    rclpy.spin(follow_node)
    FollowNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()