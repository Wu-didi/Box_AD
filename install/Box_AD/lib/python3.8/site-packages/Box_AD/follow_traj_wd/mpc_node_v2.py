import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
import time
import sys
import math
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String

# 添加模块所在路径
sys.path.append('/home/nvidia/vcii/wudi/Box_AD/Box_AD/follow_traj_wd')

# from mpc_follower import MPCfollower, State
from follow_demo_mpc_bank import VehicleTrajectoryFollower
from utils import ISGSpeedFilter
from can_use import  Can_use
# import logging

class FollowNode(Node):
    def __init__(self, main_trajectory_csv):
        super().__init__('Follow_node')
        # 创建发布者
        self.publisher_ = self.create_publisher(Float32MultiArray,
                                                'mpc_planner_action', 
                                                1)
        
        self.vs_subscription = self.create_subscription(
            Float32MultiArray,
            'vehicle_state',
            self.compute_and_publish,
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
            'mpc_trajectory',
            self.trajectory_mpc_callback,
            10
        )
        
         # 订阅到轨迹
        self.reduce_speed = self.create_subscription(
            String,
            'obstacle_reduce_speed',
            self.obstacle_reduce_speed_callback,
            10
        )
        
        # 初始化计算相关对象
        self.follower = VehicleTrajectoryFollower()
        self.filter = ISGSpeedFilter()
        self.can_use = Can_use()
        self.obstacle_reduce_speed = False
        
    def eps_callback(self, msg):
        self.latest_eps_mode = msg.data

    def compute_and_publish(self,msg):
        """
        手动调用的计算与发布函数：
        1. 读取当前车辆状态（通过 can_use 对象）
        2. 调用 MPCfollower 计算控制指令
        3. 对控制指令进行滤波
        4. 发布最新计算得到的控制指令
        """
        start_time = time.time()
        
        # 模拟连续读取 INS 数据（读取 20 次数据）
        for _ in range(20):
            self.can_use.read_ins_info()
        ego_lat = self.can_use.ego_lat
        ego_lon = self.can_use.ego_lon
        ego_yaw = self.can_use.ego_yaw_deg
        ego_v   = self.can_use.ego_v

        new_frame = [0,0,0]
        # 调用 MPCfollower 计算控制指令
        if self.can_use.ego_x is not None and self.can_use.ego_y is not None and (self.follower.cx is not None):     
            turn_angle = self.follower.calculate_turn_angle((self.can_use.ego_x, self.can_use.ego_y, self.can_use.ego_yaw_rad), self.can_use.ego_yaw_rad, self.can_use.ego_v)
            # turn_angle = 0
            
            # 因为内部有计算参考轨迹的时候，会计算目标点，这里就不再计算了 
            self.follower.update_target_index((self.can_use.ego_x, self.can_use.ego_y), self.can_use.ego_yaw_rad, self.can_use.ego_v)
            
            # print("turn_angle=", turn_angle)
            filtered_angle = self.filter.update_speed(turn_angle)
            desired_speed, desired_acc = self.follower.calculate_speedAndacc(
                            turn_angle, (ego_lat, ego_lon, ego_yaw), ego_v, is_obstacle = self.obstacle_reduce_speed)
            # print("===========",turn_angle)
            # print("time cost: ",time.time()-t0)
            new_frame = [float(desired_speed),
                         float(filtered_angle), 
                         float(desired_acc)]     
            # new_frame = [float(5),
            #              float(0), 
            #              float(0)] 
        else:
            self.get_logger().info(f"self.follower.cx is None")
            new_frame = [0.0, 0.0, -3.0]     
        
        # 构造发布消息（示例中使用固定加速度5.0，实际可根据需求调整）
        planner_frame = Float32MultiArray()
        planner_frame.data = new_frame
        
        # 发布消息
        self.get_logger().info(f"Publishing frame: {planner_frame.data}")
        self.publisher_.publish(planner_frame)
        
        elapsed_time = time.time() - start_time
        self.get_logger().info(f"Calculation time: {elapsed_time:.6f} sec")


    def trajectory_mpc_callback(self, msg: PoseArray):
        """
        将订阅到的 PoseArray 转换为和 CSV 中相同的格式：
        [x_utm, y_utm, yaw_rad, ck]

        注意：此用法要求发布端在 Pose.orientation.x 中放置了 ck，
            orientation.z / orientation.w 用于 Z 轴 yaw。
        """
        ref_x = []
        ref_y = []
        ref_yaw = []
        ref_k = []
        self.get_logger().info(f"subscribe mpc trajectory")
        # 若消息为空，则认为无轨迹或轨迹不可用
        if len(msg.poses) == 0:
            print("len of msg.poses:", len(msg.poses))
            self.follower.current_trajectory = None
        else:
            for pose in msg.poses:
                x = pose.position.x
                y = pose.position.y

                # 在发布时把曲率 ck 塞进 orientation.x (非标准用法)
                ck = pose.orientation.x

                # 从 orientation.z, orientation.w 解出 yaw（弧度）
                qz = pose.orientation.z
                qw = pose.orientation.w
                yaw_rad = 2.0 * math.atan2(qz, qw)
                # 此处 yaw_rad 范围在 (-pi, pi)，如果你想让它一直正向增长，
                # 可以再做一次归一化或转换，但通常原样就够用了。

                # 将结果存入列表： [x, y, yaw_rad, ck]
                ref_x.append(x)
                ref_y.append(y)
                ref_yaw.append(yaw_rad)
                ref_k.append(ck)

            # 将解析后的轨迹保存到你的 follower 中
            
            self.follower.cx = ref_x
            self.follower.cy = ref_y
            self.follower.cyaw = ref_yaw
            self.follower.ck = ref_k
    
    def obstacle_reduce_speed_callback(self, msg):
        data = str(msg.data)
        if data == 'Yes':
            self.obstacle_reduce_speed = True
        else:
            self.obstacle_reduce_speed = False
        self.get_logger().info(f"是否因为障碍物减速：{data}")
            

def main(args=None):
    rclpy.init(args=args)
    main_trajectory_csv = '/home/nvidia/vcii/follow_trajectory/collect_trajectory/processed_campus_0411_with_yaw_ck.csv'
    follow_node = FollowNode(main_trajectory_csv)
    rclpy.spin(follow_node)
    FollowNode.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
