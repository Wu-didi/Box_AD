import sys
sys.path.append('/home/nvidia/vcii/wudi/Box_AD/Box_AD/follow_traj_wd')
import rclpy
from rclpy.node import Node
from can_use import Can_use
from utils import ISGSpeedFilter
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32

class CanNode(Node):
    def __init__(self):
        super().__init__('can_node')
        self.speed_filter = ISGSpeedFilter()
        self.mod_AE = 1
        self.mod_666 = 1
        self.manual_triggered = False
        self.can_use = Can_use()  # 初始化 CanUse 类
        self.vs_publisher = self.create_publisher(
            Float32MultiArray,
            'vehicle_state',
            10
        )
        self.eps_publisher = self.create_publisher(
            Int32,
            'eps_mode',
            10
        )
        self.my_subscription = self.create_subscription(
            Float32MultiArray,
            'my_planner_action',
            self.my_planner_callback,
            1
        )

        self.mpc_subscription = self.create_subscription(
            Float32MultiArray,
            'mpc_planner_action',
            self.mpc_planner_callback,
            1
        )

        self.mpc_subscription  # prevent unused variable warning
        self.my_subscription
        self.my_latest_msg = None
        self.mpc_latest_msg = None
        self.latest_msg = None
        self.new_frame = [0, 0, 0]
        print("================init====================-")
        self.timer = self.create_timer(0.001, self.timer_callback)
        self.action_timer = self.create_timer(0.001, self.publish_frame)

    def timer_callback(self):
        print("here")
        self.can_use.read_ins_info()
        print("here")
        vs_msg = Float32MultiArray()
        eps_msg = Int32()
        eps_msg.data = int(self.can_use.eps_mode)
        vs_msg.data = [float(self.can_use.ego_lat), 
                    float(self.can_use.ego_lon), 
                    float(self.can_use.ego_yaw_deg),
                    float(self.can_use.ego_v)]
        self.vs_publisher.publish(vs_msg)
        self.eps_publisher.publish(eps_msg)
    
    def my_planner_callback(self, msg):
        self.my_latest_msg = msg
        
    def mpc_planner_callback(self, msg):
        self.mpc_latest_msg = msg   
    
    def publish_frame(self):
        if (self.mpc_latest_msg is None) and (self.my_latest_msg is None):
            self.get_logger().info("未接收到mpc和my method计算结果")
            
        elif self.my_latest_msg is None:
            self.get_logger().info("未接收到my method计算结果")
            self.latest_msg = self.mpc_latest_msg
        elif self.mpc_latest_msg is None:
            self.get_logger().info("未接收到mpc计算结果")
            self.latest_msg = self.my_latest_msg
        # elif (self.mpc_latest_msg is not None) and (self.my_latest_msg is not None):
        else:
            for _ in range(1):
                self.can_use.read_ins_info()
                
            if self.can_use.ego_v >= 10:
                self.latest_msg = self.my_latest_msg
                # self.get_logger().info("使用my method 计算结果")
            else:
                self.latest_msg = self.mpc_latest_msg
                # self.get_logger().info("使用   mpc  计算结果")

            
        if self.latest_msg is not None:
            data = self.latest_msg.data
            # self.get_logger().info(f"[publish_frame] Send frame: {data}")
            if len(data) >= 3:
                data[1] = self.speed_filter.update_speed(data[1])
                # self.new_frame = [data[0], data[1], data[2]]
                self.new_frame = [3, data[1], data[2]]
            else:
                self.get_logger().warn("planner_action 消息格式错误，期望至少包含3个元素")
        else:
            self.get_logger().warn("未接收到 planner_action 消息")
        # self.new_frame = [5,100,0]
        # 发送到 CAN 总线
        self.get_logger().info(str(self.new_frame))
        self.can_use.publish_planner_action(
            action=self.new_frame,
            id=0x666,
            action_type="acc",
            mod=1,
            enable=1
        )


def main(args=None):
    rclpy.init(args=args)
    can_node = CanNode()
    rclpy.spin(can_node)
    can_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()