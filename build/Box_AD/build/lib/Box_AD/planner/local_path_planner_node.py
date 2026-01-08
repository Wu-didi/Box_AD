#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
局部路径规划器 - 基于全局路径修改的避障方案

订阅:
- /global_path_utm (nav_msgs/Path)        # 全局规划路径 (UTM坐标)
- /pose_utm       (geometry_msgs/PoseStamped)  # 自车位姿
- /fast_obstacles (std_msgs/String, JSON)      # 障碍物检测

发布:
- /local_path     (nav_msgs/Path)          # 避障后的局部路径
- /trajectory     (PoseArray, 经纬度格式)  # 给 follow_node 使用
- /cmd_vel        (geometry_msgs/Twist)    # 速度指令（可选）

算法:
- 检测到障碍物时，直接修改全局路径生成绕行路径
- 支持多障碍物处理和区间合并
- 障碍物前5m开始绕行，障碍物后10m回归
"""

import math
import json
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import utm
from geometry_msgs.msg import Pose, PoseStamped, Twist, PoseArray
from nav_msgs.msg import Path
from std_msgs.msg import String

# 导入轨迹平滑模块
try:
    from Box_AD.planner.trajectory_smoother import TrajectorySmoother, VehicleParams
    SMOOTHER_AVAILABLE = True
except ImportError:
    SMOOTHER_AVAILABLE = False


def normalize_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi)."""
    while angle >= math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_pose(pose: Pose) -> float:
    """从 Pose 提取 yaw 角"""
    qz = pose.orientation.z
    qw = pose.orientation.w
    return 2.0 * math.atan2(qz, qw)


class LocalPathPlannerNode(Node):
    def __init__(self):
        super().__init__('local_path_planner_node')

        # --------- 参数声明 ---------
        self.declare_parameter('global_path_topic', '/global_path_utm')
        self.declare_parameter('pose_topic', '/pose_utm')
        self.declare_parameter('obstacle_topic', '/fast_obstacles')
        self.declare_parameter('local_path_topic', '/local_path')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.declare_parameter('start_offset', 5.0)      # 障碍物前多少米开始绕
        self.declare_parameter('end_offset', 10.0)       # 障碍物后多少米回归
        self.declare_parameter('vehicle_width', 1.0)     # 车宽的一半
        self.declare_parameter('safe_margin', 1.0)       # 额外安全距离
        self.declare_parameter('avoidance_direction', 1.0)  # 默认绕行方向: 1.0=左, -1.0=右
        self.declare_parameter('path_step', 0.5)         # 路径点间隔

        # 轨迹平滑参数
        self.declare_parameter('enable_smoothing', True)
        self.declare_parameter('min_turning_radius', 3.0)

        self.declare_parameter('control_frequency', 10.0)
        self.declare_parameter('max_speed', 7.0)

        # UTM zone 参数
        self.declare_parameter('utm_zone', 50)
        self.declare_parameter('utm_zone_letter', 'N')

        # --------- 读取参数 ---------
        self.global_path_topic = self.get_parameter('global_path_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.obstacle_topic = self.get_parameter('obstacle_topic').value
        self.local_path_topic = self.get_parameter('local_path_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.start_offset = float(self.get_parameter('start_offset').value)
        self.end_offset = float(self.get_parameter('end_offset').value)
        self.vehicle_width = float(self.get_parameter('vehicle_width').value)
        self.safe_margin = float(self.get_parameter('safe_margin').value)
        self.avoidance_direction = float(self.get_parameter('avoidance_direction').value)
        self.path_step = float(self.get_parameter('path_step').value)

        # 轨迹平滑参数
        self.enable_smoothing = self.get_parameter('enable_smoothing').value
        self.min_turning_radius = float(self.get_parameter('min_turning_radius').value)

        self.control_frequency = max(1.0, float(self.get_parameter('control_frequency').value))
        self.max_speed = float(self.get_parameter('max_speed').value)

        self.utm_zone = int(self.get_parameter('utm_zone').value)
        self.utm_zone_letter = self.get_parameter('utm_zone_letter').get_parameter_value().string_value

        self.control_period = 1.0 / self.control_frequency

        # --------- 状态 ---------
        self.global_path_msg: Optional[Path] = None
        self.global_path_points: List[Tuple[float, float]] = []  # [(x, y), ...]
        self.global_frame_id: str = 'utm'

        self.current_pose: Optional[Pose] = None
        self.pose_frame_id: str = 'utm'

        # 障碍物列表 [(x, y, radius), ...] 在 UTM 坐标系下
        self.obstacles: List[Tuple[float, float, float]] = []

        # 缓存上一次有效的轨迹，确保持续发布
        self.last_valid_path: List[Tuple[float, float, float]] = []

        # 初始化轨迹平滑器
        if SMOOTHER_AVAILABLE and self.enable_smoothing:
            vehicle_params = VehicleParams(
                min_turning_radius=self.min_turning_radius,
                max_speed=self.max_speed
            )
            self.smoother = TrajectorySmoother(vehicle_params)
            self.get_logger().info(f"轨迹平滑已启用: min_turning_radius={self.min_turning_radius}m")
        else:
            self.smoother = None
            if self.enable_smoothing and not SMOOTHER_AVAILABLE:
                self.get_logger().warn("轨迹平滑模块不可用，将使用原始路径")

        # --------- 订阅 /global_path_utm ---------
        global_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.global_path_sub = self.create_subscription(
            Path,
            self.global_path_topic,
            self.global_path_callback,
            global_qos
        )

        # --------- 订阅 /pose_utm ---------
        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            10
        )

        # --------- 订阅 /fast_obstacles ---------
        self.obstacle_sub = self.create_subscription(
            String,
            self.obstacle_topic,
            self.obstacle_callback,
            10
        )

        # --------- 发布 ---------
        self.local_path_pub = self.create_publisher(Path, self.local_path_topic, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.trajectory_pub = self.create_publisher(PoseArray, 'trajectory', 10)

        # --------- 控制循环 ---------
        self.timer = self.create_timer(self.control_period, self.control_loop)

        self.get_logger().info(
            f"LocalPathPlanner 启动: "
            f"start_offset={self.start_offset}m, end_offset={self.end_offset}m, "
            f"avoidance_direction={'左' if self.avoidance_direction > 0 else '右'}"
        )

    # ================== 订阅回调 ==================

    def global_path_callback(self, msg: Path):
        self.global_path_msg = msg
        self.global_frame_id = msg.header.frame_id or 'utm'

        # 提取路径点
        self.global_path_points = []
        for ps in msg.poses:
            x = ps.pose.position.x
            y = ps.pose.position.y
            self.global_path_points.append((x, y))

        self.get_logger().info(f"收到全局路径: {len(self.global_path_points)} 点")

    def pose_callback(self, msg: PoseStamped):
        self.current_pose = msg.pose
        self.pose_frame_id = msg.header.frame_id or 'utm'

    def obstacle_callback(self, msg: String):
        """解析障碍物消息，转换到 UTM 坐标系"""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        obstacles_rel = data.get("obstacles", [])
        if not obstacles_rel or self.current_pose is None:
            self.obstacles = []
            return

        # 将相对坐标转换为 UTM 坐标
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        robot_yaw = yaw_from_pose(self.current_pose)

        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)

        self.obstacles = []
        for obs in obstacles_rel:
            # 相对坐标 (前方为x正，左方为y正)
            rel_x = obs.get("x", 0.0)
            rel_y = obs.get("y", 0.0)
            radius = obs.get("radius", 0.5)

            # 转换到 UTM
            utm_x = robot_x + rel_x * cos_yaw - rel_y * sin_yaw
            utm_y = robot_y + rel_x * sin_yaw + rel_y * cos_yaw

            self.obstacles.append((utm_x, utm_y, radius))

        # if self.obstacles:
        #     self.get_logger().info(f"检测到 {len(self.obstacles)} 个障碍物")

    # ================== 主控制循环 ==================

    def control_loop(self):
        # 检查是否有全局路径
        if self.global_path_msg is None or not self.global_path_points:
            # 没有全局路径，但如果有缓存的轨迹，继续发布
            if self.last_valid_path:
                self.publish_paths(self.last_valid_path)
            return

        # 检查是否有位姿
        if self.current_pose is None:
            # 没有位姿，但如果有缓存的轨迹，继续发布
            if self.last_valid_path:
                self.publish_paths(self.last_valid_path)
            return

        # 生成避障路径
        modified_path = self.modify_path_for_obstacles()

        # 缓存有效轨迹
        if modified_path:
            self.last_valid_path = modified_path

        # 发布路径
        self.publish_paths(modified_path)

    # ================== 核心避障逻辑 ==================

    def modify_path_for_obstacles(self) -> List[Tuple[float, float, float]]:
        """
        修改全局路径以绕过障碍物
        返回: [(x, y, yaw), ...]
        """
        global_path = self.global_path_points

        if not self.obstacles:
            # 没有障碍物，返回原路径
            return self.path_with_yaw(global_path)

        # 1. 计算每个障碍物的影响区间，保存完整障碍物信息
        intervals = []
        for obs_x, obs_y, obs_radius in self.obstacles:
            # 检查障碍物是否影响路径
            min_dist = float('inf')
            for px, py in global_path:
                dist = math.hypot(px - obs_x, py - obs_y)
                min_dist = min(min_dist, dist)

            safe_dist = obs_radius + self.vehicle_width + self.safe_margin
            if min_dist > safe_dist + 2.0:
                continue  # 障碍物离路径太远

            start_x = obs_x - self.start_offset - obs_radius
            end_x = obs_x + self.end_offset

            # 保存障碍物信息列表，用于后续合并
            intervals.append({
                'start_x': start_x,
                'end_x': end_x,
                'obstacles': [(obs_x, obs_y, obs_radius)]
            })

        if not intervals:
            return self.path_with_yaw(global_path)

        # 2. 按起点排序
        intervals.sort(key=lambda x: x['start_x'])

        # 3. 合并重叠区间，保留所有障碍物信息
        merged = []
        for interval in intervals:
            if merged and interval['start_x'] <= merged[-1]['end_x']:
                # 重叠，合并区间并合并障碍物列表
                merged[-1]['end_x'] = max(merged[-1]['end_x'], interval['end_x'])
                merged[-1]['obstacles'].extend(interval['obstacles'])
            else:
                merged.append(interval)

        # 4. 生成修改后的路径
        full_path = []
        path_idx = 0

        for interval in merged:
            start_x = interval['start_x']
            end_x = interval['end_x']
            obstacles_in_interval = interval['obstacles']

            # 添加起点之前的原始路径
            while path_idx < len(global_path) and global_path[path_idx][0] < start_x:
                px, py = global_path[path_idx]
                yaw = self.compute_yaw(global_path, path_idx)
                full_path.append((px, py, yaw))
                path_idx += 1

            if path_idx >= len(global_path):
                break

            start_pt = global_path[path_idx]

            # 找终点索引
            end_idx = path_idx
            while end_idx < len(global_path) and global_path[end_idx][0] < end_x:
                end_idx += 1
            if end_idx >= len(global_path):
                end_idx = len(global_path) - 1

            end_pt = global_path[end_idx]

            # 计算最优绕行方向和偏移量（考虑所有障碍物）
            lateral, center_x = self.compute_optimal_bypass(
                start_pt[1], obstacles_in_interval
            )

            # 生成绕行路径
            bypass = self.generate_bypass_path_multi(
                start_pt[0], start_pt[1],
                end_pt[0], end_pt[1],
                obstacles_in_interval,
                lateral, center_x
            )
            full_path.extend(bypass)

            path_idx = end_idx + 1

        # 添加剩余路径
        while path_idx < len(global_path):
            px, py = global_path[path_idx]
            yaw = self.compute_yaw(global_path, path_idx)
            full_path.append((px, py, yaw))
            path_idx += 1

        # 应用轨迹平滑
        if self.smoother is not None and len(full_path) > 2:
            full_path = self.smooth_trajectory(full_path)

        return full_path

    def smooth_trajectory(
        self,
        path: List[Tuple[float, float, float]]
    ) -> List[Tuple[float, float, float]]:
        """
        使用车辆动力学约束平滑轨迹
        """
        # 提取 (x, y) 坐标
        xy_path = [(p[0], p[1]) for p in path]

        # 使用平滑器
        try:
            trajectory = self.smoother.smooth(xy_path, sample_step=self.path_step)
            # 转换回 (x, y, yaw) 格式
            return [(p.x, p.y, p.yaw) for p in trajectory]
        except Exception as e:
            self.get_logger().warn(f"轨迹平滑失败: {e}")
            return path

    def compute_optimal_bypass(
        self,
        path_y: float,
        obstacles: List[Tuple[float, float, float]]
    ) -> Tuple[float, float]:
        """
        计算最优绕行方向和偏移量，确保绕过所有障碍物

        Args:
            path_y: 路径的 y 坐标
            obstacles: 障碍物列表 [(x, y, radius), ...]

        Returns:
            (lateral_offset, center_x): 横向偏移量和中心x坐标
        """
        if not obstacles:
            return 0.0, 0.0

        # 计算向上和向下绕行所需的偏移量
        max_offset_up = 0.0    # 向上绕行（+y方向）所需偏移
        max_offset_down = 0.0  # 向下绕行（-y方向）所需偏移

        center_x = sum(obs[0] for obs in obstacles) / len(obstacles)

        for obs_x, obs_y, obs_radius in obstacles:
            safe_dist = obs_radius + self.vehicle_width + self.safe_margin
            diff = obs_y - path_y  # 障碍物相对路径的y偏移

            if diff >= 0:
                # 障碍物在路径上方或路径上
                # 向上绕需要绕过障碍物顶部
                offset_up = diff + safe_dist
                # 向下绕需要绕过障碍物底部（如果障碍物在路径上）
                offset_down = safe_dist - diff if diff < safe_dist else 0.0
            else:
                # 障碍物在路径下方
                # 向下绕需要绕过障碍物底部
                offset_down = abs(diff) + safe_dist
                # 向上绕需要绕过障碍物顶部（如果障碍物在路径上）
                offset_up = safe_dist - abs(diff) if abs(diff) < safe_dist else 0.0

            max_offset_up = max(max_offset_up, offset_up)
            max_offset_down = max(max_offset_down, offset_down)

        # 选择偏移量较小的方向，如果相等则使用默认方向
        if max_offset_up <= max_offset_down:
            lateral = max_offset_up if max_offset_up > 0 else self.safe_margin + self.vehicle_width
            if self.avoidance_direction < 0 and max_offset_down < max_offset_up * 1.5:
                lateral = -max_offset_down
        else:
            lateral = -max_offset_down
            if self.avoidance_direction > 0 and max_offset_up < max_offset_down * 1.5:
                lateral = max_offset_up

        return lateral, center_x

    def generate_bypass_path_multi(
        self,
        start_x: float, start_y: float,
        end_x: float, end_y: float,
        obstacles: List[Tuple[float, float, float]],
        lateral_offset: float,
        center_x: float
    ) -> List[Tuple[float, float, float]]:
        """
        生成绕行路径段（多障碍物版本）
        梯形绕行：斜向偏移 -> 平行绕过 -> 斜向回归
        """
        path = []
        step = self.path_step

        bypass_y = start_y + lateral_offset

        # 找到最前和最后的障碍物
        min_obs_x = min(obs[0] - obs[2] for obs in obstacles)
        max_obs_x = max(obs[0] + obs[2] for obs in obstacles)

        # 偏移段结束位置：最前障碍物前2m
        offset_end_x = min_obs_x - 2.0
        # 平行段结束位置：最后障碍物后2m
        parallel_end_x = max_obs_x + 2.0

        # 确保位置合理
        offset_end_x = max(start_x + 1.0, min(offset_end_x, end_x - 2.0))
        parallel_end_x = max(offset_end_x + 1.0, min(parallel_end_x, end_x - 1.0))

        # 1. 偏移段
        dist1 = math.hypot(offset_end_x - start_x, bypass_y - start_y)
        num1 = max(int(dist1 / step), 2)
        for i in range(num1 + 1):
            t = i / num1
            x = start_x + t * (offset_end_x - start_x)
            y = start_y + t * (bypass_y - start_y)
            if i < num1:
                nx = start_x + (i + 1) / num1 * (offset_end_x - start_x)
                ny = start_y + (i + 1) / num1 * (bypass_y - start_y)
                yaw = math.atan2(ny - y, nx - x)
            else:
                yaw = 0.0
            path.append((x, y, yaw))

        # 2. 平行段
        dist2 = parallel_end_x - offset_end_x
        num2 = max(int(dist2 / step), 2)
        for i in range(1, num2 + 1):
            t = i / num2
            x = offset_end_x + t * (parallel_end_x - offset_end_x)
            y = bypass_y
            yaw = 0.0
            path.append((x, y, yaw))

        # 3. 回归段
        dist3 = math.hypot(end_x - parallel_end_x, end_y - bypass_y)
        num3 = max(int(dist3 / step), 2)
        for i in range(1, num3 + 1):
            t = i / num3
            x = parallel_end_x + t * (end_x - parallel_end_x)
            y = bypass_y + t * (end_y - bypass_y)
            if i < num3:
                nx = parallel_end_x + (i + 1) / num3 * (end_x - parallel_end_x)
                ny = bypass_y + (i + 1) / num3 * (end_y - bypass_y)
                yaw = math.atan2(ny - y, nx - x)
            else:
                yaw = 0.0
            path.append((x, y, yaw))

        return path

    def generate_bypass_path(
        self,
        start_x: float, start_y: float,
        end_x: float, end_y: float,
        obstacle_x: float, obstacle_y: float,
        lateral_offset: float
    ) -> List[Tuple[float, float, float]]:
        """
        生成绕行路径段
        梯形绕行：斜向偏移 -> 平行绕过 -> 斜向回归
        """
        path = []
        step = self.path_step

        bypass_y = start_y + lateral_offset

        # 偏移段结束位置：障碍物前2m
        offset_end_x = obstacle_x - 2.0
        # 平行段结束位置：障碍物后2m
        parallel_end_x = obstacle_x + 2.0

        # 确保位置合理
        offset_end_x = max(start_x + 1.0, min(offset_end_x, end_x - 2.0))
        parallel_end_x = max(offset_end_x + 1.0, min(parallel_end_x, end_x - 1.0))

        # 1. 偏移段
        dist1 = math.hypot(offset_end_x - start_x, bypass_y - start_y)
        num1 = max(int(dist1 / step), 2)
        for i in range(num1 + 1):
            t = i / num1
            x = start_x + t * (offset_end_x - start_x)
            y = start_y + t * (bypass_y - start_y)
            if i < num1:
                nx = start_x + (i + 1) / num1 * (offset_end_x - start_x)
                ny = start_y + (i + 1) / num1 * (bypass_y - start_y)
                yaw = math.atan2(ny - y, nx - x)
            else:
                yaw = 0.0
            path.append((x, y, yaw))

        # 2. 平行段
        dist2 = parallel_end_x - offset_end_x
        num2 = max(int(dist2 / step), 2)
        for i in range(1, num2 + 1):
            t = i / num2
            x = offset_end_x + t * (parallel_end_x - offset_end_x)
            y = bypass_y
            yaw = 0.0
            path.append((x, y, yaw))

        # 3. 回归段
        dist3 = math.hypot(end_x - parallel_end_x, end_y - bypass_y)
        num3 = max(int(dist3 / step), 2)
        for i in range(1, num3 + 1):
            t = i / num3
            x = parallel_end_x + t * (end_x - parallel_end_x)
            y = bypass_y + t * (end_y - bypass_y)
            if i < num3:
                nx = parallel_end_x + (i + 1) / num3 * (end_x - parallel_end_x)
                ny = bypass_y + (i + 1) / num3 * (end_y - bypass_y)
                yaw = math.atan2(ny - y, nx - x)
            else:
                yaw = 0.0
            path.append((x, y, yaw))

        return path

    # ================== 辅助函数 ==================

    def get_path_y_at_x(self, x: float) -> Optional[float]:
        """获取路径在指定 x 位置的 y 值"""
        for px, py in self.global_path_points:
            if abs(px - x) < 1.0:
                return py
        return None

    def path_with_yaw(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float, float]]:
        """给路径添加航向角"""
        result = []
        for i, (px, py) in enumerate(path):
            yaw = self.compute_yaw(path, i)
            result.append((px, py, yaw))
        return result

    def compute_yaw(self, path: List[Tuple[float, float]], idx: int) -> float:
        """计算路径点的航向角"""
        if idx < len(path) - 1:
            px, py = path[idx]
            nx, ny = path[idx + 1]
            return math.atan2(ny - py, nx - px)
        elif idx > 0:
            px, py = path[idx - 1]
            nx, ny = path[idx]
            return math.atan2(ny - py, nx - px)
        return 0.0

    # ================== 发布函数 ==================

    def publish_paths(self, path: List[Tuple[float, float, float]]):
        """发布 /local_path 和 /trajectory"""
        now = self.get_clock().now().to_msg()

        # 发布 nav_msgs/Path
        path_msg = Path()
        path_msg.header.frame_id = self.global_frame_id
        path_msg.header.stamp = now

        for (x, y, yaw) in path:
            ps = PoseStamped()
            ps.header.frame_id = self.global_frame_id
            ps.header.stamp = now
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = 0.0
            ps.pose.orientation.z = math.sin(yaw * 0.5)
            ps.pose.orientation.w = math.cos(yaw * 0.5)
            path_msg.poses.append(ps)

        self.local_path_pub.publish(path_msg)

        # 发布 PoseArray (经纬度格式，给 follow_node)
        trajectory_msg = PoseArray()
        trajectory_msg.header.stamp = now
        trajectory_msg.header.frame_id = 'map'

        for (x, y, yaw) in path:
            try:
                lat, lon = utm.to_latlon(x, y, self.utm_zone, self.utm_zone_letter)
            except Exception:
                continue

            pose = Pose()
            pose.position.x = lon
            pose.position.y = lat
            pose.position.z = 0.0
            pose.orientation.z = math.sin(yaw * 0.5)
            pose.orientation.w = math.cos(yaw * 0.5)
            trajectory_msg.poses.append(pose)

        self.trajectory_pub.publish(trajectory_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocalPathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
