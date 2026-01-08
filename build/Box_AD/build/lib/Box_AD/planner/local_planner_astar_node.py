#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
局部路径规划器 - 基于A*的避障方案

功能：
- 订阅静态地图 /map (OccupancyGrid)
- 订阅全局路径 /global_path_utm (Path)
- 订阅当前位姿 /pose_utm (PoseStamped)
- 订阅障碍物检测 /fast_obstacles (String, JSON)
- 将检测到的障碍物实时叠加到地图上
- 用A*规划从当前位置到前方目标点的局部路径
- 发布 /local_path (Path)

优点：
- 天然避障，路径质量高
- 可处理任意形状障碍物
- 支持动态障碍物实时更新
"""

import math
import heapq
import json
import os
import re
from typing import List, Optional, Tuple

import cv2
import numpy as np
import utm

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

# 导入轨迹平滑模块
try:
    from Box_AD.planner.trajectory_smoother import TrajectorySmoother, VehicleParams
    SMOOTHER_AVAILABLE = True
except ImportError:
    SMOOTHER_AVAILABLE = False


def yaw_from_pose(pose: Pose) -> float:
    """从 Pose 提取 yaw 角"""
    qz = pose.orientation.z
    qw = pose.orientation.w
    return 2.0 * math.atan2(qz, qw)


class LocalPlannerAstarNode(Node):
    def __init__(self):
        super().__init__('local_planner_astar_node')

        # ========== 参数声明 ==========
        # 车辆尺寸参数（用于计算安全半径）
        self.declare_parameter('vehicle_length', 4)  # 车长（米）
        self.declare_parameter('vehicle_width', 2.0)   # 车宽（米）
        self.declare_parameter('safety_margin', 0.1)   # 额外安全边距（米）
        self.declare_parameter('obstacle_inflation', 0.3)  # 动态障碍物额外膨胀

        # 障碍物处理参数
        self.declare_parameter('min_obstacle_radius', 0.3)  # 最小障碍物半径（小于此值的会被放大）
        self.declare_parameter('obstacle_cluster_dist', 1.5)  # 障碍物聚类距离（米）
        self.declare_parameter('max_obstacle_distance', 50.0)  # 最大障碍物处理距离（米）

        self.declare_parameter('lookahead_distance', 30.0)  # 前视距离
        self.declare_parameter('local_map_size', 60.0)  # 局部地图大小（米）
        self.declare_parameter('replan_frequency', 2.0)  # 重规划频率（降低到2Hz减少CPU占用）
        self.declare_parameter('path_sample_step', 0.3)  # 路径采样步长

        # 轨迹平滑参数
        self.declare_parameter('enable_smoothing', True)
        self.declare_parameter('min_turning_radius', 3.0)
        self.declare_parameter('max_speed', 5.0)

        # 地图和坐标变换
        self.declare_parameter('map_yaml_path', './maps/map.yaml')
        self.declare_parameter('utm_zone', 50)
        self.declare_parameter('utm_zone_letter', 'N')

        # ========== 读取参数 ==========
        # 车辆尺寸
        self.vehicle_length = float(self.get_parameter('vehicle_length').value)
        self.vehicle_width = float(self.get_parameter('vehicle_width').value)
        self.safety_margin = float(self.get_parameter('safety_margin').value)
        self.obstacle_inflation = float(self.get_parameter('obstacle_inflation').value)

        # 根据车辆尺寸计算安全半径（外接圆半径 + 安全边距）
        # 矩形车辆的外接圆半径 = sqrt((length/2)^2 + (width/2)^2)
        self.robot_radius = math.sqrt(
            (self.vehicle_length / 2.0) ** 2 + (self.vehicle_width / 2.0) ** 2
        ) + self.safety_margin

        # 障碍物处理参数
        self.min_obstacle_radius = float(self.get_parameter('min_obstacle_radius').value)
        self.obstacle_cluster_dist = float(self.get_parameter('obstacle_cluster_dist').value)
        self.max_obstacle_distance = float(self.get_parameter('max_obstacle_distance').value)

        self.lookahead_distance = float(self.get_parameter('lookahead_distance').value)
        self.local_map_size = float(self.get_parameter('local_map_size').value)
        self.replan_frequency = float(self.get_parameter('replan_frequency').value)
        self.path_sample_step = float(self.get_parameter('path_sample_step').value)

        # 轨迹平滑参数
        self.enable_smoothing = self.get_parameter('enable_smoothing').value
        self.min_turning_radius = float(self.get_parameter('min_turning_radius').value)
        self.max_speed = float(self.get_parameter('max_speed').value)

        self.map_yaml_path = self.get_parameter('map_yaml_path').get_parameter_value().string_value
        self.utm_zone = int(self.get_parameter('utm_zone').value)
        self.utm_zone_letter = self.get_parameter('utm_zone_letter').get_parameter_value().string_value

        # ========== 地图数据 ==========
        self.map_received = False
        self.static_grid = None  # 静态地图（膨胀后）
        self.width = 0
        self.height = 0
        self.resolution = 0.1
        self.origin_x = 0.0
        self.origin_y = 0.0

        # UTM 原点
        self.png_origin_e = None
        self.png_origin_n = None
        self.utm_origin_e = None
        self.utm_origin_n = None

        if self.map_yaml_path:
            self.load_png_origin_utm(self.map_yaml_path)

        # ========== 状态 ==========
        self.current_pose: Optional[Pose] = None
        self.current_pose_utm: Optional[Tuple[float, float, float]] = None  # (E, N, yaw)
        self.global_path_utm: List[Tuple[float, float]] = []  # [(E, N), ...]
        self.obstacles_utm: List[Tuple[float, float, float]] = []  # [(E, N, radius), ...]

        # 缓存
        self.last_local_path: List[Tuple[float, float, float]] = []
        self.last_closest_idx: int = 0  # 缓存最近点索引，避免每次从头搜索

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

        # ========== 订阅 ==========
        # 静态地图
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, map_qos
        )

        # 全局路径
        self.global_path_sub = self.create_subscription(
            Path, '/global_path_utm', self.global_path_callback, map_qos
        )

        # 当前位姿
        self.pose_sub = self.create_subscription(
            PoseStamped, '/pose_utm', self.pose_callback, 10
        )

        # 障碍物检测
        self.obstacle_sub = self.create_subscription(
            String, '/fast_obstacles', self.obstacle_callback, 10
        )

        # ========== 发布 ==========
        self.local_path_pub = self.create_publisher(Path, '/local_path', 10)
        self.local_path_map_pub = self.create_publisher(Path, '/local_path_in_map', 10)
        self.trajectory_pub = self.create_publisher(PoseArray, 'trajectory', 10)

        # ========== 定时器 ==========
        self.replan_period = 1.0 / max(self.replan_frequency, 0.1)
        self.timer = self.create_timer(self.replan_period, self.replan_callback)

        self.get_logger().info(
            f"LocalPlannerAstar 启动: "
            f"vehicle={self.vehicle_length}x{self.vehicle_width}m, "
            f"robot_radius={self.robot_radius:.2f}m (含安全边距{self.safety_margin}m), "
            f"lookahead={self.lookahead_distance}m, "
            f"replan_freq={self.replan_frequency}Hz"
        )

    # ========== 解析 map.yaml ==========

    def load_png_origin_utm(self, yaml_path: str):
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
                                f"解析 PNG origin UTM: E={self.png_origin_e:.3f}, N={self.png_origin_n:.3f}"
                            )
                            return
        except Exception as e:
            self.get_logger().error(f"解析 map.yaml 失败: {e}")

    # ========== 回调函数 ==========

    def map_callback(self, msg: OccupancyGrid):
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y

        data = np.array(msg.data, dtype=np.int8)
        grid = data.reshape((self.height, self.width))

        # 障碍膨胀
        obst = (grid == 100).astype(np.uint8)
        inflation_cells = int(self.robot_radius / self.resolution)
        kernel_size = 2 * inflation_cells + 1
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        inflated_obst = cv2.dilate(obst, kernel)

        inflated_grid = grid.copy()
        inflated_grid[inflated_obst == 1] = 100

        self.static_grid = inflated_grid
        self.map_received = True

        # 计算 UTM 原点
        if self.png_origin_e is not None and self.png_origin_n is not None:
            self.utm_origin_e = self.png_origin_e - self.origin_x
            self.utm_origin_n = self.png_origin_n - self.origin_y

        self.get_logger().info(
            f"收到地图: {self.width}x{self.height}, res={self.resolution}"
        )

    def global_path_callback(self, msg: Path):
        self.global_path_utm = []
        for ps in msg.poses:
            E = ps.pose.position.x
            N = ps.pose.position.y
            self.global_path_utm.append((E, N))

        # 重置最近点索引缓存
        self.last_closest_idx = 0

        self.get_logger().info(f"收到全局路径: {len(self.global_path_utm)} 点")

    def pose_callback(self, msg: PoseStamped):
        self.current_pose = msg.pose
        E = msg.pose.position.x
        N = msg.pose.position.y
        yaw = yaw_from_pose(msg.pose)
        self.current_pose_utm = (E, N, yaw)

    def obstacle_callback(self, msg: String):
        """解析障碍物，转换到 UTM 坐标系，并进行过滤和聚类"""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        obstacles_rel = data.get("obstacles", [])
        if not obstacles_rel or self.current_pose_utm is None:
            self.obstacles_utm = []
            return

        robot_E, robot_N, robot_yaw = self.current_pose_utm
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)

        # 第一步：转换坐标并过滤过远的障碍物
        raw_obstacles = []
        for obs in obstacles_rel:
            rel_x = obs.get("x", 0.0)
            rel_y = obs.get("y", 0.0)
            radius = obs.get("radius", 0.5)

            # 过滤过远的障碍物
            dist = math.hypot(rel_x, rel_y)
            if dist > self.max_obstacle_distance:
                continue

            # 确保最小半径
            radius = max(radius, self.min_obstacle_radius)

            # 转换到 UTM
            E = robot_E + rel_x * cos_yaw - rel_y * sin_yaw
            N = robot_N + rel_x * sin_yaw + rel_y * cos_yaw

            raw_obstacles.append((E, N, radius))

        # 第二步：聚类相近的障碍物（简单的贪婪聚类）
        self.obstacles_utm = self.cluster_obstacles(raw_obstacles)

    def cluster_obstacles(
        self,
        obstacles: List[Tuple[float, float, float]]
    ) -> List[Tuple[float, float, float]]:
        """将相近的障碍物聚类合并，减少数量并提高避障效果

        Args:
            obstacles: [(E, N, radius), ...]

        Returns:
            聚类后的障碍物列表
        """
        if not obstacles:
            return []

        if len(obstacles) <= 1:
            return obstacles

        # 标记哪些障碍物已被处理
        used = [False] * len(obstacles)
        clustered = []

        for i, (e1, n1, r1) in enumerate(obstacles):
            if used[i]:
                continue

            # 找到所有与当前障碍物相近的障碍物
            cluster_es = [e1]
            cluster_ns = [n1]
            cluster_rs = [r1]
            used[i] = True

            for j, (e2, n2, r2) in enumerate(obstacles):
                if used[j]:
                    continue

                # 计算障碍物边缘距离
                dist = math.hypot(e2 - e1, n2 - n1)
                edge_dist = dist - r1 - r2

                if edge_dist < self.obstacle_cluster_dist:
                    cluster_es.append(e2)
                    cluster_ns.append(n2)
                    cluster_rs.append(r2)
                    used[j] = True

            # 合并聚类：计算质心和包围半径
            center_e = sum(cluster_es) / len(cluster_es)
            center_n = sum(cluster_ns) / len(cluster_ns)

            # 计算能覆盖所有障碍物的最小半径
            max_radius = 0.0
            for e, n, r in zip(cluster_es, cluster_ns, cluster_rs):
                dist_to_center = math.hypot(e - center_e, n - center_n)
                max_radius = max(max_radius, dist_to_center + r)

            clustered.append((center_e, center_n, max_radius))

        return clustered

    # ========== 坐标变换 ==========

    def utm_to_map(self, E: float, N: float) -> Tuple[float, float]:
        """UTM -> map 坐标"""
        if self.utm_origin_e is None:
            return E, N
        x = E - self.utm_origin_e
        y = N - self.utm_origin_n
        return x, y

    def map_to_utm(self, x: float, y: float) -> Tuple[float, float]:
        """map -> UTM 坐标"""
        if self.utm_origin_e is None:
            return x, y
        E = self.utm_origin_e + x
        N = self.utm_origin_n + y
        return E, N

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """map 坐标 -> 栅格坐标"""
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        return row, col

    def grid_to_world(self, row: int, col: int) -> Tuple[float, float]:
        """栅格坐标 -> map 坐标"""
        x = self.origin_x + (col + 0.5) * self.resolution
        y = self.origin_y + (row + 0.5) * self.resolution
        return x, y

    # ========== 重规划 ==========

    def replan_callback(self):
        """定时重规划"""
        if not self.map_received:
            return
        if self.current_pose_utm is None:
            return
        if not self.global_path_utm:
            # 没有全局路径，发布缓存
            if self.last_local_path:
                self.publish_path(self.last_local_path)
            return

        # 1. 找到当前位置在全局路径上的最近点
        robot_E, robot_N, robot_yaw = self.current_pose_utm
        closest_idx = self.find_closest_point(robot_E, robot_N)

        # 2. 找到前视目标点
        goal_idx = self.find_lookahead_point(closest_idx, robot_E, robot_N)
        if goal_idx is None:
            if self.last_local_path:
                self.publish_path(self.last_local_path)
            return

        goal_E, goal_N = self.global_path_utm[goal_idx]

        # 3. 创建带动态障碍物的局部地图
        local_grid = self.create_local_grid_with_obstacles()
        if local_grid is None:
            if self.last_local_path:
                self.publish_path(self.last_local_path)
            return

        # 4. 转换到 map 坐标并执行 A*
        start_x, start_y = self.utm_to_map(robot_E, robot_N)
        goal_x, goal_y = self.utm_to_map(goal_E, goal_N)

        start_rc = self.world_to_grid(start_x, start_y)
        goal_rc = self.world_to_grid(goal_x, goal_y)

        # 检查边界
        if not self.in_bounds(start_rc[0], start_rc[1], local_grid):
            self.get_logger().warn("起点超出地图范围")
            if self.last_local_path:
                self.publish_path(self.last_local_path)
            return

        if not self.in_bounds(goal_rc[0], goal_rc[1], local_grid):
            self.get_logger().warn("终点超出地图范围")
            if self.last_local_path:
                self.publish_path(self.last_local_path)
            return

        # 5. A* 规划
        path_rc = self.astar(start_rc, goal_rc, local_grid)
        if path_rc is None:
            self.get_logger().debug("A* 未找到路径，尝试直接连接")
            # 尝试直接连接
            path_rc = [start_rc, goal_rc]

        # 6. 简化路径
        path_rc = self.simplify_path(path_rc, local_grid)

        # 7. 转换为世界坐标
        world_path = [self.grid_to_world(r, c) for r, c in path_rc]

        # 8. 添加全局路径剩余部分
        full_path = self.append_global_path(world_path, goal_idx)

        # 9. 应用车辆动力学平滑（暂时禁用）
        # if self.smoother is not None and len(full_path) > 2:
        #     try:
        #         trajectory = self.smoother.smooth(full_path, sample_step=self.path_sample_step)
        #         full_path = [(p.x, p.y) for p in trajectory]
        #     except Exception as e:
        #         self.get_logger().warn(f"轨迹平滑失败: {e}")

        # 10. 转换为 UTM 并添加 yaw
        path_with_yaw = []
        for i, (x, y) in enumerate(full_path):
            E, N = self.map_to_utm(x, y)
            if i < len(full_path) - 1:
                nx, ny = full_path[i + 1]
                yaw = math.atan2(ny - y, nx - x)
            else:
                yaw = path_with_yaw[-1][2] if path_with_yaw else robot_yaw
            path_with_yaw.append((E, N, yaw))

        # 11. 发布
        self.last_local_path = path_with_yaw
        self.publish_path(path_with_yaw)

    def find_closest_point(self, E: float, N: float) -> int:
        """找到全局路径上离当前位置最近的点（使用缓存加速）"""
        if not self.global_path_utm:
            return 0

        # 从上次最近点附近开始搜索，通常车辆只会前进
        search_start = max(0, self.last_closest_idx - 5)
        search_end = min(len(self.global_path_utm), self.last_closest_idx + 100)

        min_dist = float('inf')
        closest_idx = self.last_closest_idx

        for i in range(search_start, search_end):
            pe, pn = self.global_path_utm[i]
            dist = (pe - E) ** 2 + (pn - N) ** 2  # 避免sqrt
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        self.last_closest_idx = closest_idx
        return closest_idx

    def find_lookahead_point(self, start_idx: int, E: float, N: float) -> Optional[int]:
        """找到前视距离处的目标点"""
        accumulated_dist = 0.0

        for i in range(start_idx, len(self.global_path_utm) - 1):
            p1 = self.global_path_utm[i]
            p2 = self.global_path_utm[i + 1]
            seg_dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            accumulated_dist += seg_dist

            if accumulated_dist >= self.lookahead_distance:
                return i + 1

        # 如果路径不够长，返回最后一个点
        if len(self.global_path_utm) > start_idx:
            return len(self.global_path_utm) - 1
        return None

    def create_local_grid_with_obstacles(self) -> Optional[np.ndarray]:
        """创建包含动态障碍物的局部栅格地图（优化版）

        修复：正确使用每个障碍物的实际半径进行膨胀
        """
        if self.static_grid is None:
            return None

        # 如果没有动态障碍物，直接返回静态地图引用（避免复制）
        if not self.obstacles_utm:
            return self.static_grid

        # 复制静态地图
        local_grid = self.static_grid.copy()

        # 对每个障碍物单独处理，使用其实际半径
        for obs_E, obs_N, obs_radius in self.obstacles_utm:
            obs_x, obs_y = self.utm_to_map(obs_E, obs_N)
            obs_c = int((obs_x - self.origin_x) / self.resolution)
            obs_r = int((obs_y - self.origin_y) / self.resolution)

            if not (0 <= obs_r < self.height and 0 <= obs_c < self.width):
                continue

            # 总膨胀半径 = 障碍物自身半径 + 车辆安全半径 + 额外膨胀
            total_radius = obs_radius + self.robot_radius + self.obstacle_inflation
            inflation_cells = int(total_radius / self.resolution)

            # 使用OpenCV画填充圆（比逐像素填充更快）
            cv2.circle(local_grid, (obs_c, obs_r), inflation_cells, 100, -1)

        return local_grid

    def append_global_path(
        self,
        local_path: List[Tuple[float, float]],
        goal_idx: int
    ) -> List[Tuple[float, float]]:
        """将全局路径的剩余部分添加到局部路径后面"""
        if not local_path:
            return local_path

        full_path = list(local_path)

        # 从 goal_idx 之后开始添加全局路径点
        for i in range(goal_idx + 1, len(self.global_path_utm)):
            E, N = self.global_path_utm[i]
            x, y = self.utm_to_map(E, N)
            full_path.append((x, y))

        return full_path

    # ========== A* 算法 ==========

    def in_bounds(self, r: int, c: int, grid: np.ndarray) -> bool:
        return 0 <= r < grid.shape[0] and 0 <= c < grid.shape[1]

    def is_free(self, r: int, c: int, grid: np.ndarray) -> bool:
        if not self.in_bounds(r, c, grid):
            return False
        return int(grid[r, c]) == 0

    def astar(
        self,
        start_rc: Tuple[int, int],
        goal_rc: Tuple[int, int],
        grid: np.ndarray,
        max_iterations: int = 10000
    ) -> Optional[List[Tuple[int, int]]]:
        """A* 路径规划（带迭代次数限制）"""
        H, W = grid.shape
        sr, sc = start_rc
        gr, gc = goal_rc

        # 如果起点或终点在障碍物中，尝试找最近的自由点
        if not self.is_free(sr, sc, grid):
            sr, sc = self.find_nearest_free(sr, sc, grid)
            if sr is None:
                return None

        if not self.is_free(gr, gc, grid):
            gr, gc = self.find_nearest_free(gr, gc, grid)
            if gr is None:
                return None

        def h(r, c):
            return abs(r - gr) + abs(c - gc)  # 曼哈顿距离，比欧几里得快

        # 8 邻域
        moves = [(-1, 0), (1, 0), (0, -1), (0, 1),
                 (-1, -1), (-1, 1), (1, -1), (1, 1)]

        open_set = []
        heapq.heappush(open_set, (0.0, (sr, sc)))
        came_from = {}
        g_score = {(sr, sc): 0.0}
        visited = np.zeros((H, W), dtype=bool)

        iterations = 0
        while open_set and iterations < max_iterations:
            iterations += 1
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
                if not self.in_bounds(nr, nc, grid):
                    continue
                if visited[nr, nc]:
                    continue
                if not self.is_free(nr, nc, grid):
                    continue

                step_cost = 1.0 if dr == 0 or dc == 0 else 1.414  # 避免hypot计算
                new_g = g_score[(r, c)] + step_cost

                if (nr, nc) not in g_score or new_g < g_score[(nr, nc)]:
                    g_score[(nr, nc)] = new_g
                    f = new_g + h(nr, nc)
                    heapq.heappush(open_set, (f, (nr, nc)))
                    came_from[(nr, nc)] = (r, c)

        return None

    def find_nearest_free(
        self,
        r: int, c: int,
        grid: np.ndarray,
        max_search: int = 50
    ) -> Optional[Tuple[int, int]]:
        """找到最近的自由栅格"""
        for radius in range(1, max_search):
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    if abs(dr) == radius or abs(dc) == radius:
                        nr, nc = r + dr, c + dc
                        if self.is_free(nr, nc, grid):
                            return nr, nc
        return None, None

    def simplify_path(
        self,
        path_rc: List[Tuple[int, int]],
        grid: np.ndarray
    ) -> List[Tuple[int, int]]:
        """Line-of-sight 路径简化"""
        if not path_rc or len(path_rc) <= 2:
            return path_rc

        simplified = [path_rc[0]]
        idx = 0

        while idx < len(path_rc) - 1:
            next_idx = idx + 1
            for j in range(idx + 1, len(path_rc)):
                r0, c0 = path_rc[idx]
                r1, c1 = path_rc[j]
                if self.line_is_free(r0, c0, r1, c1, grid):
                    next_idx = j
                else:
                    break
            simplified.append(path_rc[next_idx])
            idx = next_idx

        return simplified

    def line_is_free(
        self,
        r0: int, c0: int,
        r1: int, c1: int,
        grid: np.ndarray
    ) -> bool:
        """Bresenham 直线检查"""
        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        sr = 1 if r1 >= r0 else -1
        sc = 1 if c1 >= c0 else -1

        r, c = r0, c0
        if dr >= dc:
            err = dr / 2.0
            while r != r1:
                if not self.is_free(r, c, grid):
                    return False
                err -= dc
                if err < 0:
                    c += sc
                    err += dr
                r += sr
            return self.is_free(r, c, grid)
        else:
            err = dc / 2.0
            while c != c1:
                if not self.is_free(r, c, grid):
                    return False
                err -= dr
                if err < 0:
                    r += sr
                    err += dc
                c += sc
            return self.is_free(r, c, grid)

    def smooth_path(self, world_path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """简单平滑 + 重采样"""
        if len(world_path) <= 2:
            return world_path

        # 重采样
        resampled = [world_path[0]]
        accumulated = 0.0
        prev = np.array(world_path[0], dtype=np.float64)

        for i in range(1, len(world_path)):
            curr = np.array(world_path[i], dtype=np.float64)
            seg_vec = curr - prev
            seg_len = float(np.linalg.norm(seg_vec))

            if seg_len < 1e-6:
                prev = curr
                continue

            direction = seg_vec / seg_len
            remaining = seg_len

            while accumulated + remaining >= self.path_sample_step - 1e-9:
                need = self.path_sample_step - accumulated
                prev = prev + direction * need
                resampled.append(prev.tolist())
                remaining -= need
                accumulated = 0.0
                if remaining < 1e-6:
                    break

            accumulated += remaining
            prev = curr

        if np.linalg.norm(np.array(resampled[-1]) - np.array(world_path[-1])) > 1e-3:
            resampled.append(world_path[-1])

        return resampled

    # ========== 发布 ==========

    def publish_path(self, path_utm: List[Tuple[float, float, float]]):
        """发布路径"""
        now = self.get_clock().now().to_msg()

        # /local_path (UTM)
        path_msg = Path()
        path_msg.header.frame_id = 'utm'
        path_msg.header.stamp = now

        for E, N, yaw in path_utm:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = E
            ps.pose.position.y = N
            ps.pose.position.z = 0.0
            ps.pose.orientation.z = math.sin(yaw * 0.5)
            ps.pose.orientation.w = math.cos(yaw * 0.5)
            path_msg.poses.append(ps)

        self.local_path_pub.publish(path_msg)

        # /local_path_in_map (map 坐标，方便 RViz 显示)
        path_map_msg = Path()
        path_map_msg.header.frame_id = 'map'
        path_map_msg.header.stamp = now

        for E, N, yaw in path_utm:
            x, y = self.utm_to_map(E, N)
            ps = PoseStamped()
            ps.header = path_map_msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = 0.0
            ps.pose.orientation.z = math.sin(yaw * 0.5)
            ps.pose.orientation.w = math.cos(yaw * 0.5)
            path_map_msg.poses.append(ps)

        self.local_path_map_pub.publish(path_map_msg)

        # /trajectory (经纬度格式)
        trajectory_msg = PoseArray()
        trajectory_msg.header.stamp = now
        trajectory_msg.header.frame_id = 'map'

        for E, N, yaw in path_utm:
            try:
                lat, lon = utm.to_latlon(E, N, self.utm_zone, self.utm_zone_letter)
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
    node = LocalPlannerAstarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
