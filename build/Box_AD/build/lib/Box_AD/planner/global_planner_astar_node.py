#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import heapq
import os
import re
import time

import cv2
import numpy as np
import utm

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose


class AstarPlannerNode(Node):
    """
    功能：
    - 订阅静态 /map（OccupancyGrid）
    - 订阅 /initialpose（起点） 和 /goal_pose（终点）
    - 只要起点和终点都已设置，每次收到新的起点或终点，都会：
        * 使用膨胀后的栅格地图做 A* 规划
        * 对路径做 line-of-sight 简化
        * 发布两条路径：
            - /global_path       : map 坐标系下的路径（方便 RViz 看）
            - /global_path_utm   : UTM 坐标系下的路径（给 MPC / PP 用）
    """

    def __init__(self):
        super().__init__('astar_planner_node')

        # 机器人"半径"配置（单位米）：车宽的一半 + 安全距离
        # 车宽1.9m/2 + 安全距离0.5m = 1.45m
        self.declare_parameter('robot_radius', 1.45)
        self.robot_radius = float(self.get_parameter('robot_radius').value)

        # 车辆动力学相关配置 (Hybrid A* 参数)
        # 车辆参数：车长4.53m, 车宽1.9m, 轴距2.85m, 最大转向角60度
        self.declare_parameter('wheelbase', 2.85)          # 轴距(m)
        self.declare_parameter('min_turning_radius', 1.65) # 最小转弯半径(m) = 轴距/tan(60°)
        self.declare_parameter('max_steer_angle', 1.047)   # 最大转向角(rad), 60度
        self.declare_parameter('motion_step', 2.5)         # 每步前进距离(m), 增大加速搜索
        self.declare_parameter('num_steer_angles', 5)      # 离散转向角数量
        self.declare_parameter('theta_resolution', 0.35)   # 朝向角分辨率(rad), 约20度
        self.declare_parameter('path_sample_step', 0.5)    # 最终路径采样步长(m)
        self.declare_parameter('steer_penalty', 0.1)       # 转向惩罚系数
        self.declare_parameter('steer_change_penalty', 0.3)  # 转向变化惩罚
        self.declare_parameter('goal_xy_threshold', 3.0)   # 目标位置阈值(m)
        self.declare_parameter('goal_theta_threshold', 0.52)  # 目标朝向阈值(rad), 约30度

        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.min_turning_radius = float(self.get_parameter('min_turning_radius').value)
        self.max_steer_angle = float(self.get_parameter('max_steer_angle').value)
        self.motion_step = float(self.get_parameter('motion_step').value)
        self.num_steer_angles = int(self.get_parameter('num_steer_angles').value)
        self.theta_resolution = float(self.get_parameter('theta_resolution').value)
        self.path_sample_step = float(self.get_parameter('path_sample_step').value)
        self.steer_penalty = float(self.get_parameter('steer_penalty').value)
        self.steer_change_penalty = float(self.get_parameter('steer_change_penalty').value)
        self.goal_xy_threshold = float(self.get_parameter('goal_xy_threshold').value)
        self.goal_theta_threshold = float(self.get_parameter('goal_theta_threshold').value)

        # 计算离散转向角列表
        self.steer_angles = np.linspace(-self.max_steer_angle, self.max_steer_angle,
                                         self.num_steer_angles).tolist()

        # === 新增：读取 map.yaml，用于构造 map -> UTM 的线性变换 ===
        #   map_yaml_path 需要在 launch 里传进来，例如：
        #   <param name="map_yaml_path" value="/home/.../map.yaml"/>
        self.declare_parameter('map_yaml_path', './maps/map.yaml')
        self.map_yaml_path = self.get_parameter('map_yaml_path').get_parameter_value().string_value

        # UTM zone 参数（用于 UTM -> 经纬度转换）
        self.declare_parameter('utm_zone', 50)
        self.declare_parameter('utm_zone_letter', 'N')
        self.utm_zone = int(self.get_parameter('utm_zone').value)
        self.utm_zone_letter = self.get_parameter('utm_zone_letter').get_parameter_value().string_value

        # PNG 左下角对应的 UTM 坐标（从 map.yaml 注释中解析）
        self.png_origin_e = None  # E_png
        self.png_origin_n = None  # N_png

        # UTM 原点（建图时 lat0,lon0 对应的 UTM）
        # map 坐标 (x_map, y_map) -> UTM: E = utm_origin_e + x_map, N = utm_origin_n + y_map
        self.utm_origin_e = None
        self.utm_origin_n = None

        if self.map_yaml_path:
            self.load_png_origin_utm(self.map_yaml_path)
        else:
            self.get_logger().warn(
                "未设置 map_yaml_path 参数，将无法发布 UTM 坐标的路径 (/global_path_utm)"
            )

        # 地图数据
        self.map_received = False
        self.grid = None          # numpy int8: -1,0,100
        self.width = 0
        self.height = 0
        self.resolution = 0.1
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.frame_id = 'map'

        # 当前起点 / 终点（可以多次更新）
        self.start_pose = None
        self.goal_pose = None
        self.start_yaw = None  # 起点朝向
        self.goal_yaw = None   # 终点朝向

        # ---- 订阅 /map ----
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10
        )

        # ---- 订阅起点&终点 ----
        self.initpose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            'initialpose',
            self.initialpose_callback,
            10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_callback,
            10
        )

        # ---- 发布路径（使用 TRANSIENT_LOCAL，方便之后打开 RViz 也能看到最后一条）----
        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        # map 坐标系的全局路径（主要给 RViz 看）
        self.path_pub = self.create_publisher(Path, 'global_path', path_qos)
        # UTM 坐标系的全局路径（给 MPC / PP 使用）
        self.path_pub_utm = self.create_publisher(Path, 'global_path_utm', path_qos)
        # 经纬度格式的全局轨迹（未避障的原始路径）
        self.trajectory_pub = self.create_publisher(PoseArray, 'global_trajectory', 10)

        self.get_logger().info(
            "AstarPlannerNode 启动，等待 /map、/initialpose、/goal_pose ..."
        )

    # ========== 解析 map.yaml 中的 PNG origin UTM ==========

    def load_png_origin_utm(self, yaml_path: str):
        """
        从 map.yaml 中解析
            # PNG origin UTM: E=..., N=...
        这行注释，得到 PNG 左下角对应的 UTM 坐标。
        """
        if not os.path.exists(yaml_path):
            self.get_logger().error(f"map_yaml_path 不存在: {yaml_path}")
            return

        try:
            with open(yaml_path, 'r') as f:
                for line in f:
                    line = line.strip()
                    if line.startswith("# PNG origin UTM"):
                        m = re.search(r"E\s*=\s*([0-9.+\-eE]+).*N\s*=\s*([0-9.+\-eE]+)", line)
                        if m:
                            self.png_origin_e = float(m.group(1))
                            self.png_origin_n = float(m.group(2))
                            self.get_logger().info(
                                f"从 map.yaml 解析 PNG origin UTM: "
                                f"E={self.png_origin_e:.3f}, N={self.png_origin_n:.3f}"
                            )
                            return
            self.get_logger().warn(
                "在 map.yaml 中未找到 '# PNG origin UTM: E=..., N=...' 注释，"
                "将无法发布 UTM 路径。"
            )
        except Exception as e:
            self.get_logger().error(f"解析 map.yaml 失败: {e}")

    # ========== 话题回调 ==========

    def map_callback(self, msg: OccupancyGrid):
        # 原始 OccupancyGrid
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.frame_id = msg.header.frame_id

        data = np.array(msg.data, dtype=np.int8)
        grid = data.reshape((self.height, self.width))

        # ---- 障碍膨胀 ----
        obst = (grid == 100).astype(np.uint8)

        inflation_cells = int(self.robot_radius / self.resolution)
        inflation_cells = max(inflation_cells, 1)
        kernel_size = 2 * inflation_cells + 1
        kernel = np.ones((kernel_size, kernel_size), np.uint8)

        inflated_obst = cv2.dilate(obst, kernel)
        inflated_grid = grid.copy()
        inflated_grid[inflated_obst == 1] = 100

        self.grid = inflated_grid
        self.map_received = True

        self.get_logger().debug(
            f"收到 /map 并完成障碍膨胀: size=({self.width}x{self.height}), "
            f"res={self.resolution:.3f}, origin=({self.origin_x:.2f},{self.origin_y:.2f}), "
            f"robot_radius={self.robot_radius:.2f}m, inflation_cells={inflation_cells}"
        )

        # === 一旦有了地图的 origin_xy，并且之前解析出了 PNG origin UTM，就可以算 UTM 原点 ===
        if self.png_origin_e is not None and self.png_origin_n is not None:
            # E_png = E0 + origin_x  =>  E0 = E_png - origin_x
            self.utm_origin_e = self.png_origin_e - self.origin_x
            self.utm_origin_n = self.png_origin_n - self.origin_y
            self.get_logger().debug(
                f"计算得到 utm_origin: E0={self.utm_origin_e:.3f}, N0={self.utm_origin_n:.3f} "
                f"(map (0,0) 对应的 UTM 坐标)"
            )
        else:
            self.get_logger().warn(
                "尚未获取 PNG origin UTM，因此无法计算 utm_origin，将只发布 map 坐标的路径。"
            )

        # 地图更新后，如果已有起点和终点，也可以立刻重新规划
        self.try_plan()

    def initialpose_callback(self, msg: PoseWithCovarianceStamped):
        # 每次收到新的 initialpose，都更新起点并尝试重新规划
        self.start_pose = msg.pose.pose
        # 提取朝向（使用完整四元数公式）
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        # 完整的四元数到 yaw 转换，结果在 [-π, π] 范围内
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.start_yaw = math.atan2(siny_cosp, cosy_cosp)
        print(f"[起点] 位置: ({self.start_pose.position.x:.2f}, {self.start_pose.position.y:.2f}), "
              f"yaw={math.degrees(self.start_yaw):.1f}°, 四元数: [x={qx:.3f}, y={qy:.3f}, z={qz:.3f}, w={qw:.3f}]")
        self.try_plan()

    def goal_callback(self, msg: PoseStamped):
        # 每次收到新的 goal，都更新终点并尝试重新规划
        self.goal_pose = msg.pose
        # 提取朝向（使用完整四元数公式）
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        # 完整的四元数到 yaw 转换，结果在 [-π, π] 范围内
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.goal_yaw = math.atan2(siny_cosp, cosy_cosp)
        print(f"[终点] 位置: ({self.goal_pose.position.x:.2f}, {self.goal_pose.position.y:.2f}), "
              f"yaw={math.degrees(self.goal_yaw):.1f}°, 四元数: [x={qx:.3f}, y={qy:.3f}, z={qz:.3f}, w={qw:.3f}]")
        self.try_plan()

    # ========== 坐标变换 ==========

    def world_to_grid(self, x: float, y: float):
        """
        世界坐标 (x,y) -> 栅格 (row,col)
        按 OccupancyGrid 约定：origin 在左下角，row 向 +y 增长
        """
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        return row, col

    def grid_to_world(self, row: int, col: int):
        """
        栅格 (row,col) -> 世界坐标 (map 坐标系下的 x,y)
        """
        x = self.origin_x + (col + 0.5) * self.resolution
        y = self.origin_y + (row + 0.5) * self.resolution
        return x, y

    # ========== 规划入口 ==========

    def try_plan(self):
        """
        使用 Hybrid A* 进行路径规划，考虑车辆动力学约束。
        - 状态空间: (x, y, theta)
        - 考虑起点和终点的朝向
        """
        if not self.map_received:
            return
        if self.start_pose is None or self.goal_pose is None:
            return

        sx, sy = self.start_pose.position.x, self.start_pose.position.y
        gx, gy = self.goal_pose.position.x, self.goal_pose.position.y

        # 获取起点和终点朝向
        start_yaw = self.start_yaw if self.start_yaw is not None else math.atan2(gy - sy, gx - sx)
        goal_yaw = self.goal_yaw if self.goal_yaw is not None else math.atan2(gy - sy, gx - sx)

        print("\n" + "="*60)
        print(f"[Hybrid A* 规划]")
        print(f"  起点: ({sx:.2f}, {sy:.2f}), yaw={math.degrees(start_yaw):.1f}°")
        print(f"  终点: ({gx:.2f}, {gy:.2f}), yaw={math.degrees(goal_yaw):.1f}°")
        print("="*60)

        # 检查起点终点是否在地图内且可通行
        sr, sc = self.world_to_grid(sx, sy)
        gr, gc = self.world_to_grid(gx, gy)

        if not self.in_bounds(sr, sc) or not self.in_bounds(gr, gc):
            self.get_logger().warn("起点或终点不在地图范围内，规划失败")
            return

        if not self.is_free(sr, sc):
            self.get_logger().warn("起点在障碍或未知区域，规划失败")
            return
        if not self.is_free(gr, gc):
            self.get_logger().warn("终点在障碍或未知区域，规划失败")
            return

        # 使用 Hybrid A* 规划
        world_path = self.hybrid_astar(
            (sx, sy, start_yaw),
            (gx, gy, goal_yaw)
        )

        if world_path is None or len(world_path) < 2:
            print("[警告] Hybrid A* 未找到可行路径，尝试标准 A*")
            # 回退到标准 A*
            path_rc = self.astar((sr, sc), (gr, gc))
            if path_rc is None:
                print("[错误] 标准 A* 也未找到路径")
                return
            path_rc = self.simplify_path(path_rc)
            world_path = [self.grid_to_world(r, c) for r, c in path_rc]

        # 重采样路径
        world_path = self.resample_path(world_path, self.path_sample_step)
        print(f"[最终路径] 点数={len(world_path)}")

        # 计算每个路径点的朝向（yaw），便于下游控制
        yaws = self.estimate_yaws(world_path)

        # ---- 构造 map 坐标系的 Path 消息 ----
        path_msg = Path()
        path_msg.header.frame_id = self.frame_id  # 一般是 "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for idx, (x, y) in enumerate(world_path):
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = 0.0
            qw, qx, qy, qz = self.yaw_to_quaternion(yaws[idx])
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            path_msg.poses.append(ps)

        self.path_pub.publish(path_msg)
        print(f"[发布路径] /global_path (map 坐标)，点数={len(path_msg.poses)}")

        # ---- 构造 UTM 坐标系的 Path 消息（给 MPC / PP 使用）----
        if self.utm_origin_e is not None and self.utm_origin_n is not None:
            path_msg_utm = Path()
            path_msg_utm.header.frame_id = "utm"
            path_msg_utm.header.stamp = self.get_clock().now().to_msg()

            for idx, (x_map, y_map) in enumerate(world_path):
                # map -> UTM: E = E0 + x_map, N = N0 + y_map
                E = self.utm_origin_e + x_map
                N = self.utm_origin_n + y_map

                ps = PoseStamped()
                ps.header = path_msg_utm.header
                ps.pose.position.x = E
                ps.pose.position.y = N
                ps.pose.position.z = 0.0
                qw, qx, qy, qz = self.yaw_to_quaternion(yaws[idx])
                ps.pose.orientation.x = qx
                ps.pose.orientation.y = qy
                ps.pose.orientation.z = qz
                ps.pose.orientation.w = qw
                path_msg_utm.poses.append(ps)

            self.path_pub_utm.publish(path_msg_utm)
            self.get_logger().debug(
                f"已发布 /global_path_utm (UTM 坐标)，点数={len(path_msg_utm.poses)}"
            )

            # ---- 构造经纬度格式的轨迹（PoseArray）----
            trajectory_msg = PoseArray()
            trajectory_msg.header.stamp = self.get_clock().now().to_msg()
            trajectory_msg.header.frame_id = 'map'

            for idx, (x_map, y_map) in enumerate(world_path):
                E = self.utm_origin_e + x_map
                N = self.utm_origin_n + y_map

                try:
                    lat, lon = utm.to_latlon(E, N, self.utm_zone, self.utm_zone_letter)
                except Exception as e:
                    self.get_logger().warn(f"UTM -> 经纬度转换失败: {e}")
                    continue

                pose = Pose()
                pose.position.x = lon
                pose.position.y = lat
                pose.position.z = 0.0

                yaw = yaws[idx]
                qz = math.sin(yaw / 2.0)
                qw = math.cos(yaw / 2.0)
                pose.orientation.x = 0.0
                pose.orientation.y = 0.0
                pose.orientation.z = qz
                pose.orientation.w = qw

                trajectory_msg.poses.append(pose)

            self.trajectory_pub.publish(trajectory_msg)
            self.get_logger().debug(
                f"已发布 /global_trajectory (经纬度格式)，点数={len(trajectory_msg.poses)}"
            )
        else:
            self.get_logger().warn(
                "未计算出 utm_origin，无法发布 /global_path_utm，如需 UTM 路径请确保 map.yaml 中有 "
                "'# PNG origin UTM: E=..., N=...' 注释并设置 map_yaml_path 参数。"
            )

    # ========== A* & 工具函数 ==========

    def in_bounds(self, r, c):
        return 0 <= r < self.height and 0 <= c < self.width

    def is_free(self, r, c):
        """
        地图值含义：
          0   = free
          100 = obstacle
          -1  = unknown
        为了更安全，这里 unknown 当障碍。
        调试时你可以改成 `return v != 100` 看效果。
        """
        v = int(self.grid[r, c])
        return v == 0

    def astar(self, start_rc, goal_rc):
        H, W = self.height, self.width
        sr, sc = start_rc
        gr, gc = goal_rc

        def h(r, c):
            return math.hypot(r - gr, c - gc)

        # 8 邻域
        moves = [(-1, 0), (1, 0), (0, -1), (0, 1),
                 (-1, -1), (-1, 1), (1, -1), (1, 1)]

        open_set = []
        heapq.heappush(open_set, (0.0, (sr, sc)))
        came_from = {}
        g_score = {(sr, sc): 0.0}
        visited = np.zeros((H, W), dtype=bool)

        while open_set:
            _, (r, c) = heapq.heappop(open_set)
            if visited[r, c]:
                continue
            visited[r, c] = True

            if (r, c) == (gr, gc):
                # 回溯路径
                path = [(r, c)]
                while (r, c) in came_from:
                    r, c = came_from[(r, c)]
                    path.append((r, c))
                path.reverse()
                return path

            for dr, dc in moves:
                nr, nc = r + dr, c + dc
                if not self.in_bounds(nr, nc):
                    continue
                if visited[nr, nc]:
                    continue
                if not self.is_free(nr, nc):
                    continue

                step_cost = math.hypot(dr, dc)
                new_g = g_score[(r, c)] + step_cost

                if (nr, nc) not in g_score or new_g < g_score[(nr, nc)]:
                    g_score[(nr, nc)] = new_g
                    f = new_g + h(nr, nc)
                    heapq.heappush(open_set, (f, (nr, nc)))
                    came_from[(nr, nc)] = (r, c)

        return None

    # ========== Hybrid A* 算法 (优化版) ==========

    def compute_heuristic_map(self, goal_r, goal_c):
        """
        使用优化的 BFS 计算考虑障碍物的距离场
        使用 deque + numpy 加速
        """
        from collections import deque

        dist_map = np.full((self.height, self.width), np.inf, dtype=np.float32)

        # 检查目标是否可通行
        if not self.is_free(goal_r, goal_c):
            print(f"[警告] 目标点在障碍物内，寻找最近自由点")
            return dist_map

        dist_map[goal_r, goal_c] = 0
        queue = deque([(goal_r, goal_c)])

        # 4邻域 (更快，对于启发式足够)
        moves = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        while queue:
            r, c = queue.popleft()
            current_dist = dist_map[r, c]
            next_dist = current_dist + 1

            for dr, dc in moves:
                nr, nc = r + dr, c + dc
                if 0 <= nr < self.height and 0 <= nc < self.width:
                    if self.grid[nr, nc] == 0 and dist_map[nr, nc] == np.inf:
                        dist_map[nr, nc] = next_dist
                        queue.append((nr, nc))

        return dist_map

    def hybrid_astar(self, start_state, goal_state):
        """
        优化版 Hybrid A*：
        1. 使用预计算的2D距离场作为启发式
        2. 更大的步长减少节点数
        3. 分析扩展加速收敛
        """
        sx, sy, s_theta = start_state
        gx, gy, g_theta = goal_state

        print(f"\n[Hybrid A*] ========== 开始规划 ==========")
        print(f"[Hybrid A*] 起点: ({sx:.1f}, {sy:.1f}, {math.degrees(s_theta):.0f}°)")
        print(f"[Hybrid A*] 终点: ({gx:.1f}, {gy:.1f}, {math.degrees(g_theta):.0f}°)")

        start_time = time.time()

        # 目标阈值
        xy_threshold = self.goal_xy_threshold
        theta_threshold = self.goal_theta_threshold

        # 预计算启发式地图
        gr, gc = self.world_to_grid(gx, gy)
        print(f"[Hybrid A*] 正在计算启发式地图...")
        heuristic_map = self.compute_heuristic_map(gr, gc)
        h_time = time.time() - start_time
        print(f"[Hybrid A*] 启发式地图计算完成，耗时 {h_time:.2f}s")

        # 检查目标是否可达
        sr, sc = self.world_to_grid(sx, sy)
        if heuristic_map[sr, sc] == np.inf:
            print(f"[Hybrid A*] 错误：目标不可达（被障碍物隔断）")
            return None

        # 启发式函数 - 使用预计算的距离
        def heuristic(x, y, theta):
            r, c = self.world_to_grid(x, y)
            if not self.in_bounds(r, c):
                return float('inf')
            h = heuristic_map[r, c] * self.resolution  # 转换为米
            # 加上角度差惩罚
            angle_diff = abs(theta - g_theta)
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff
            h += 0.3 * angle_diff * self.wheelbase
            return h

        # 角度离散化
        theta_res = self.theta_resolution
        def theta_to_index(theta):
            theta = theta % (2 * math.pi)
            return int(theta / theta_res)

        # 状态key
        def state_key(x, y, theta):
            r, c = self.world_to_grid(x, y)
            ti = theta_to_index(theta)
            return (r, c, ti)

        # 车辆模型 - 使用更大步长
        step = self.motion_step
        def vehicle_model(x, y, theta, steer):
            if abs(steer) < 0.01:
                return x + step * math.cos(theta), y + step * math.sin(theta), theta
            turn_radius = self.wheelbase / math.tan(abs(steer))
            beta = step / turn_radius * (-1 if steer < 0 else 1)
            new_theta = theta + beta
            dx = turn_radius * (math.sin(new_theta) - math.sin(theta)) * np.sign(steer)
            dy = -turn_radius * (math.cos(new_theta) - math.cos(theta)) * np.sign(steer)
            # 归一化角度
            while new_theta > math.pi: new_theta -= 2 * math.pi
            while new_theta < -math.pi: new_theta += 2 * math.pi
            return x + dx, y + dy, new_theta

        # 快速碰撞检测
        def is_path_free(x1, y1, x2, y2):
            dist = math.hypot(x2 - x1, y2 - y1)
            if dist < 0.01:
                return self.is_world_free_point([x1, y1])
            # 减少采样点数量加速
            steps = max(int(dist / self.resolution), 2)
            for i in range(steps + 1):
                t = i / steps
                r, c = self.world_to_grid(x1 + t * (x2 - x1), y1 + t * (y2 - y1))
                if not self.in_bounds(r, c) or not self.is_free(r, c):
                    return False
            return True

        # 尝试直接连接到目标（分析扩展）
        def try_direct_connect(x, y, theta):
            dist = math.hypot(gx - x, gy - y)
            if dist > 15:  # 太远不尝试
                return None
            # 检查直线是否可行
            if not is_path_free(x, y, gx, gy):
                return None
            # 检查角度是否接近
            target_angle = math.atan2(gy - y, gx - x)
            angle_diff = abs(theta - target_angle)
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff
            if angle_diff < math.radians(30):  # 角度差小于30度可以直连
                return [(x, y), (gx, gy)]
            return None

        # 初始化搜索
        start_key = state_key(sx, sy, s_theta)
        g_cost = {start_key: 0.0}
        came_from = {start_key: None}
        state_map = {start_key: (sx, sy, s_theta)}

        counter = 0
        open_set = []
        heapq.heappush(open_set, (heuristic(sx, sy, s_theta), counter, start_key))
        counter += 1

        visited = set()
        max_iterations = 200000

        print(f"[Hybrid A*] 开始搜索，最大迭代={max_iterations}")
        last_print_time = time.time()

        for iteration in range(max_iterations):
            if not open_set:
                elapsed = time.time() - start_time
                print(f"[Hybrid A*] 失败：开放列表为空，总耗时 {elapsed:.2f}s")
                return None

            _, _, current_key = heapq.heappop(open_set)

            if current_key in visited:
                continue
            visited.add(current_key)

            cx, cy, c_theta = state_map[current_key]

            # 计算到目标的距离
            dist_to_goal = math.hypot(cx - gx, cy - gy)
            angle_to_goal = abs(c_theta - g_theta)
            if angle_to_goal > math.pi:
                angle_to_goal = 2 * math.pi - angle_to_goal

            # 检查是否到达目标
            if dist_to_goal < xy_threshold and angle_to_goal < theta_threshold:
                path = []
                key = current_key
                while key is not None:
                    state = state_map[key]
                    path.append((state[0], state[1]))
                    key = came_from[key]
                path.reverse()
                path.append((gx, gy))
                elapsed = time.time() - start_time
                print(f"[Hybrid A*] 成功! 迭代={iteration}, 总耗时={elapsed:.2f}s, 路径点={len(path)}")
                return path

            # 每隔一段时间尝试直接连接
            if iteration % 100 == 0:
                direct_path = try_direct_connect(cx, cy, c_theta)
                if direct_path:
                    # 回溯已有路径 + 直连
                    path = []
                    key = current_key
                    while key is not None:
                        state = state_map[key]
                        path.append((state[0], state[1]))
                        key = came_from[key]
                    path.reverse()
                    path.append((gx, gy))
                    elapsed = time.time() - start_time
                    print(f"[Hybrid A*] 成功(直连)! 迭代={iteration}, 总耗时={elapsed:.2f}s, 路径点={len(path)}")
                    return path

            # 进度输出 - 每2秒输出一次
            current_time = time.time()
            if current_time - last_print_time > 2.0:
                elapsed = current_time - start_time
                print(f"[Hybrid A*] 进度: 迭代={iteration}, 已访问={len(visited)}, "
                      f"距离={dist_to_goal:.1f}m, 角度差={math.degrees(angle_to_goal):.0f}°, 耗时={elapsed:.1f}s")
                last_print_time = current_time

            # 展开邻居
            current_g = g_cost[current_key]

            for steer in self.steer_angles:
                nx, ny, n_theta = vehicle_model(cx, cy, c_theta, steer)

                if not self.in_bounds(*self.world_to_grid(nx, ny)):
                    continue
                if not is_path_free(cx, cy, nx, ny):
                    continue

                neighbor_key = state_key(nx, ny, n_theta)
                if neighbor_key in visited:
                    continue

                step_cost = step + self.steer_penalty * abs(steer)
                new_g = current_g + step_cost

                if neighbor_key not in g_cost or new_g < g_cost[neighbor_key]:
                    g_cost[neighbor_key] = new_g
                    came_from[neighbor_key] = current_key
                    state_map[neighbor_key] = (nx, ny, n_theta)
                    f = new_g + heuristic(nx, ny, n_theta)
                    heapq.heappush(open_set, (f, counter, neighbor_key))
                    counter += 1

        elapsed = time.time() - start_time
        print(f"[Hybrid A*] 失败：达到最大迭代 {max_iterations}，总耗时 {elapsed:.2f}s")
        return None

    # ----- 路径平滑：line-of-sight -----

    def line_is_free(self, r0, c0, r1, c1):
        """
        Bresenham 直线检查：从 (r0,c0) 到 (r1,c1) 之间是否全是 free
        """
        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        sr = 1 if r1 >= r0 else -1
        sc = 1 if c1 >= c0 else -1

        r, c = r0, c0
        if dr >= dc:
            err = dr / 2.0
            while r != r1:
                if not self.is_free(r, c):
                    return False
                err -= dc
                if err < 0:
                    c += sc
                    err += dr
                r += sr
            return self.is_free(r, c)
        else:
            err = dc / 2.0
            while c != c1:
                if not self.is_free(r, c):
                    return False
                err -= dr
                if err < 0:
                    r += sr
                    err += dc
                c += sc
            return self.is_free(r, c)

    def simplify_path(self, path_rc):
        """
        对 A* 得到的 [ (r,c), ... ] 路径做 line-of-sight 简化
        """
        if not path_rc or len(path_rc) <= 2:
            return path_rc

        simplified = [path_rc[0]]
        idx = 0

        while idx < len(path_rc) - 1:
            next_idx = idx + 1
            # 从当前点开始，尽量向后连直线
            for j in range(idx + 1, len(path_rc)):
                r0, c0 = path_rc[idx]
                r1, c1 = path_rc[j]
                if self.line_is_free(r0, c0, r1, c1):
                    next_idx = j
                else:
                    break
            simplified.append(path_rc[next_idx])
            idx = next_idx

        return simplified

    # ----- 辅助函数 -----

    def is_world_free_point(self, point):
        """
        检查世界坐标点是否在自由区内。
        """
        x, y = point if hasattr(point, '__len__') else (point[0], point[1])
        r, c = self.world_to_grid(x, y)
        if not self.in_bounds(r, c):
            return False
        return self.is_free(r, c)

    def resample_path(self, path, step):
        """
        以固定步长重新采样轨迹，方便控制器追踪。
        """
        if step <= 1e-3 or len(path) < 2:
            return path

        resampled = [path[0]]
        accumulated = 0.0
        prev = np.array(path[0], dtype=np.float64)

        for i in range(1, len(path)):
            segment_end = np.array(path[i], dtype=np.float64)
            seg_vec = segment_end - prev
            seg_len = float(np.linalg.norm(seg_vec))
            if seg_len < 1e-6:
                prev = segment_end
                continue
            direction = seg_vec / seg_len
            remaining = seg_len

            while accumulated + remaining >= step - 1e-9:
                need = step - accumulated
                prev = prev + direction * need
                resampled.append(prev.tolist())
                remaining -= need
                accumulated = 0.0
                if remaining < 1e-6:
                    break

            accumulated += remaining
            prev = segment_end

        if np.linalg.norm(np.array(resampled[-1]) - np.array(path[-1])) > 1e-3:
            resampled.append(path[-1])
        else:
            resampled[-1] = path[-1]
        return resampled

    def estimate_yaws(self, world_path):
        """
        根据连续点计算朝向（yaw）。
        """
        if len(world_path) == 1:
            return [0.0]

        yaws = []
        for i in range(len(world_path)):
            if i == len(world_path) - 1:
                dx = world_path[i][0] - world_path[i - 1][0]
                dy = world_path[i][1] - world_path[i - 1][1]
            else:
                dx = world_path[i + 1][0] - world_path[i][0]
                dy = world_path[i + 1][1] - world_path[i][1]
            yaws.append(math.atan2(dy, dx))
        return yaws

    def yaw_to_quaternion(self, yaw):
        """
        将偏航角转换为四元数 (w,x,y,z)。
        """
        half = yaw * 0.5
        return (
            math.cos(half),
            0.0,
            0.0,
            math.sin(half)
        )


def main(args=None):
    rclpy.init(args=args)
    node = AstarPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
