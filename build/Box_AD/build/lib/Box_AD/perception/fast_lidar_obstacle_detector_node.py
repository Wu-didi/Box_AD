#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
fast_lidar_obstacle_detector_node.py

高效激光障碍物探测节点（优化版，无 Open3D、无 DBSCAN）：
- 订阅 /rslidar_points_prev (sensor_msgs/PointCloud2) - 前方雷达
- 订阅 /rslidar_points_left (sensor_msgs/PointCloud2) - 左侧雷达
- 订阅 /rslidar_points_right (sensor_msgs/PointCloud2) - 右侧雷达
- 订阅 /rslidar_points_top (sensor_msgs/PointCloud2) - 顶部雷达
- 订阅 /rslidar_points_back (sensor_msgs/PointCloud2) - 后方雷达
- 粗略估计全局地面高度
- 在 ROI 内用 2D 栅格统计"高于地面的点"
- 输出每个占用栅格的中心作为障碍物位置
- 支持为每个雷达设置不同的最小障碍物高度阈值

优化内容：
- 使用 numpy 直接解析 PointCloud2，避免 Python for 循环
- 使用 numpy.bincount 做网格统计，替代字典累加
- 支持多雷达点云融合
- 每个雷达独立检测，使用各自的高度阈值

发布：
- /fast_obstacles (std_msgs/String, JSON)
    {
      "stamp": {...},
      "frame_id": "...",
      "has_obstacle": true/false,
      "obstacles": [
        {"x":..., "y":..., "z":..., "count": N},
        ...
      ]
    }
"""

import json
import time
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String


# -------------------- 配置 --------------------

class FastConfig:
    def __init__(self):
        # ROI（沿用你之前的默认）局部规划不需要很大
        self.xmin = -2.0
        self.xmax = 40.0
        self.ymin = -15.0
        self.ymax = 15.0
        self.zmin = -3.0
        self.zmax = 3.0

        # 估计地面高度用的距离范围
        self.ground_r_min = 1.0   # 忽略靠太近的
        self.ground_r_max = 20.0  # 太远的也忽略

        # 地面高度的分位数（越小越贴近最低点）
        self.ground_percentile = 15.0

        # 判定障碍物的高度条件：z > ground_z + min_obstacle_height
        # 为五个雷达分别设置不同的高度阈值
        self.min_obstacle_height_prev = 0.5    # 前方雷达：比地面高 0.5m 以上算障碍
        self.min_obstacle_height_left = 1.2    # 左侧雷达：比地面高 1.2m 以上算障碍
        self.min_obstacle_height_right = 1.0   # 右侧雷达：比地面高 0.5m 以上算障碍
        self.min_obstacle_height_top = 0.8     # 顶部雷达：比地面高 0.8m 以上算障碍
        self.min_obstacle_height_back = 0.1    # 后方雷达：比地面高 0.5m 以上算障碍
        self.max_obstacle_height = 5.0         # 超过这个当作高架/桥，可要可不要

        # 有效探测距离范围
        self.obstacle_r_min = 1.0
        self.obstacle_r_max = 60.0

        # 2D 网格分辨率（米）
        self.grid_resolution = 0.5

        # 每个网格最少点数，低于视为噪声
        self.min_points_per_cell = 3

        # 点云降采样：隔 stride 个点取一个（为 1 则不降采样）
        self.sample_stride = 1  # 算力够：1；算力紧张：2 或 3


# -------------------- 工具：PointCloud2 -> numpy (优化版) --------------------

def get_field_offset(msg: PointCloud2, field_name: str) -> Optional[int]:
    """获取指定字段在点云中的字节偏移量"""
    for field in msg.fields:
        if field.name == field_name:
            return field.offset
    return None


def pc2_to_xyz_array_fast(msg: PointCloud2, stride: int = 1) -> np.ndarray:
    """
    高效将 PointCloud2 转为 (N, 3) 的 numpy 数组。
    直接从二进制数据读取，避免 Python for 循环。
    """
    if msg.width * msg.height == 0:
        return np.zeros((0, 3), dtype=np.float32)

    # 获取 x, y, z 字段的偏移量
    x_offset = get_field_offset(msg, 'x')
    y_offset = get_field_offset(msg, 'y')
    z_offset = get_field_offset(msg, 'z')

    if x_offset is None or y_offset is None or z_offset is None:
        return np.zeros((0, 3), dtype=np.float32)

    point_step = msg.point_step
    n_points = msg.width * msg.height

    # 将字节数据转为 numpy 数组
    data = np.frombuffer(msg.data, dtype=np.uint8)

    # 提取 x, y, z（假设都是 float32）
    # 使用 numpy stride tricks 避免复制
    x = np.frombuffer(data[x_offset:].tobytes(), dtype=np.float32,
                      count=n_points, offset=0)[::point_step // 4]
    y = np.frombuffer(data[y_offset:].tobytes(), dtype=np.float32,
                      count=n_points, offset=0)[::point_step // 4]
    z = np.frombuffer(data[z_offset:].tobytes(), dtype=np.float32,
                      count=n_points, offset=0)[::point_step // 4]

    # 更高效的方式：直接按结构化数组读取
    # 创建一个视图，每个点的起始位置
    points_data = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, point_step)

    # 提取 x, y, z
    x = points_data[:, x_offset:x_offset+4].view(np.float32).flatten()
    y = points_data[:, y_offset:y_offset+4].view(np.float32).flatten()
    z = points_data[:, z_offset:z_offset+4].view(np.float32).flatten()

    # 降采样
    if stride > 1:
        x = x[::stride]
        y = y[::stride]
        z = z[::stride]

    # 过滤 NaN 和 Inf
    valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    x = x[valid]
    y = y[valid]
    z = z[valid]

    # 组合成 (N, 3)
    xyz = np.column_stack([x, y, z])
    return xyz


# -------------------- 主检测逻辑（优化版，纯 numpy 向量化） --------------------

def fast_detect_obstacles_optimized(xyz: np.ndarray, cfg: FastConfig, min_obstacle_height: float = None):
    """
    输入：xyz (N,3)
    输出：列表 obstacles = [{"x":..,"y":..,"z":..,"count":..}, ...]

    优化：使用 numpy.bincount 替代 Python 字典累加
    min_obstacle_height: 如果指定，使用该值覆盖 cfg 中的默认值
    """
    # 使用传入的高度阈值，如果没有则使用 prev 的默认值
    if min_obstacle_height is None:
        min_obstacle_height = cfg.min_obstacle_height_prev
    if xyz.shape[0] == 0:
        return []

    x = xyz[:, 0]
    y = xyz[:, 1]
    z = xyz[:, 2]

    # 1) ROI 裁剪
    roi_mask = (
        (x >= cfg.xmin) & (x <= cfg.xmax) &
        (y >= cfg.ymin) & (y <= cfg.ymax) &
        (z >= cfg.zmin) & (z <= cfg.zmax)
    )
    if not np.any(roi_mask):
        return []

    x = x[roi_mask]
    y = y[roi_mask]
    z = z[roi_mask]

    # 2) 粗略估计地面高度（用近处一圈的低分位数）
    r = np.sqrt(x * x + y * y)
    ground_mask = (r >= cfg.ground_r_min) & (r <= cfg.ground_r_max)

    if np.any(ground_mask):
        ground_z = np.percentile(z[ground_mask], cfg.ground_percentile)
    else:
        ground_z = np.percentile(z, cfg.ground_percentile)

    # 3) 挑出疑似障碍物点
    obstacle_mask = (
        (r >= cfg.obstacle_r_min) &
        (r <= cfg.obstacle_r_max) &
        (z >= ground_z + min_obstacle_height) &
        (z <= ground_z + cfg.max_obstacle_height)
    )

    if not np.any(obstacle_mask):
        return []

    xo = x[obstacle_mask]
    yo = y[obstacle_mask]
    zo = z[obstacle_mask]

    # 4) 投到 2D 网格，使用 numpy.bincount 向量化统计
    res = cfg.grid_resolution

    # 计算网格索引
    ix = np.floor((xo - cfg.xmin) / res).astype(np.int32)
    iy = np.floor((yo - cfg.ymin) / res).astype(np.int32)

    # 网格尺寸
    nx = int(np.ceil((cfg.xmax - cfg.xmin) / res))
    ny = int(np.ceil((cfg.ymax - cfg.ymin) / res))

    # 边界检查
    valid_grid = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    if not np.any(valid_grid):
        return []

    ix = ix[valid_grid]
    iy = iy[valid_grid]
    xo = xo[valid_grid]
    yo = yo[valid_grid]
    zo = zo[valid_grid]

    # 一维索引
    flat_idx = ix * ny + iy

    # 使用 bincount 统计（向量化，无 Python 循环）
    total_cells = nx * ny
    counts = np.bincount(flat_idx, minlength=total_cells)
    sum_x = np.bincount(flat_idx, weights=xo, minlength=total_cells)
    sum_y = np.bincount(flat_idx, weights=yo, minlength=total_cells)
    sum_z = np.bincount(flat_idx, weights=zo, minlength=total_cells)

    # 筛选有效格子（点数 >= min_points_per_cell）
    valid_cells = counts >= cfg.min_points_per_cell
    valid_indices = np.where(valid_cells)[0]

    if len(valid_indices) == 0:
        return []

    # 计算中心点坐标
    valid_counts = counts[valid_indices]
    cx = sum_x[valid_indices] / valid_counts
    cy = sum_y[valid_indices] / valid_counts
    cz = sum_z[valid_indices] / valid_counts

    # 构建输出列表（这里还是需要一个循环，但数量很少）
    obstacles = [
        {
            "x": float(cx[i]),
            "y": float(cy[i]),
            "z": float(cz[i]),
            "count": int(valid_counts[i])
        }
        for i in range(len(valid_indices))
    ]

    return obstacles


# -------------------- ROS2 节点 --------------------

class FastLidarObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__("fast_lidar_obstacle_detector")

        self.cfg = FastConfig()

        # 性能统计
        self.timing_enabled = True
        self.timing_count = 0
        self.timing_sum_detect = 0.0

        # 点云缓存（存储最新的点云数据）
        self.cloud_prev: Optional[np.ndarray] = None
        self.cloud_left: Optional[np.ndarray] = None
        self.cloud_right: Optional[np.ndarray] = None
        self.cloud_top: Optional[np.ndarray] = None
        self.cloud_back: Optional[np.ndarray] = None
        self.latest_header = None  # 保存最新的消息头

        # 订阅前方雷达
        self.sub_prev = self.create_subscription(
            PointCloud2,
            "/rslidar_points_prev",
            self.lidar_callback_prev,
            10
        )

        # 订阅左侧雷达
        self.sub_left = self.create_subscription(
            PointCloud2,
            "/rslidar_points_left",
            self.lidar_callback_left,
            10
        )

        # 订阅右侧雷达
        self.sub_right = self.create_subscription(
            PointCloud2,
            "/rslidar_points_right",
            self.lidar_callback_right,
            10
        )

        # 订阅顶部雷达
        self.sub_top = self.create_subscription(
            PointCloud2,
            "/rslidar_points_top",
            self.lidar_callback_top,
            10
        )

        # 订阅后方雷达
        self.sub_back = self.create_subscription(
            PointCloud2,
            "/rslidar_points_back",
            self.lidar_callback_back,
            10
        )

        self.pub = self.create_publisher(
            String,
            "/fast_obstacles",
            10
        )

        self.get_logger().info(
            "FastLidarObstacleDetectorNode started (optimized). "
            "Subscribing /rslidar_points_prev, /rslidar_points_left, /rslidar_points_right, "
            "/rslidar_points_top, /rslidar_points_back, "
            "publishing /fast_obstacles"
        )

    def lidar_callback_prev(self, msg: PointCloud2):
        """前方雷达回调"""
        xyz = pc2_to_xyz_array_fast(msg, stride=self.cfg.sample_stride)
        if xyz.shape[0] > 0:
            self.cloud_prev = xyz
            self.latest_header = msg.header
        self.process_merged_clouds()

    def lidar_callback_left(self, msg: PointCloud2):
        """左侧雷达回调"""
        xyz = pc2_to_xyz_array_fast(msg, stride=self.cfg.sample_stride)
        if xyz.shape[0] > 0:
            self.cloud_left = xyz
            self.latest_header = msg.header
        self.process_merged_clouds()

    def lidar_callback_right(self, msg: PointCloud2):
        """右侧雷达回调"""
        xyz = pc2_to_xyz_array_fast(msg, stride=self.cfg.sample_stride)
        if xyz.shape[0] > 0:
            self.cloud_right = xyz
            self.latest_header = msg.header
        self.process_merged_clouds()

    def lidar_callback_top(self, msg: PointCloud2):
        """顶部雷达回调"""
        xyz = pc2_to_xyz_array_fast(msg, stride=self.cfg.sample_stride)
        if xyz.shape[0] > 0:
            self.cloud_top = xyz
            self.latest_header = msg.header
        self.process_merged_clouds()

    def lidar_callback_back(self, msg: PointCloud2):
        """后方雷达回调"""
        xyz = pc2_to_xyz_array_fast(msg, stride=self.cfg.sample_stride)
        if xyz.shape[0] > 0:
            self.cloud_back = xyz
            self.latest_header = msg.header
        self.process_merged_clouds()

    def process_merged_clouds(self):
        """分别处理五个雷达点云（使用各自的高度阈值），然后合并结果"""
        # 检查是否有任何点云数据
        if self.latest_header is None:
            return

        # 计时开始
        if self.timing_enabled:
            t0 = time.perf_counter()

        # 分别检测五个雷达的障碍物（使用各自的高度阈值）
        obstacles_all = []
        obstacles_prev = []
        obstacles_left = []
        obstacles_right = []
        obstacles_top = []
        obstacles_back = []

        if self.cloud_prev is not None and self.cloud_prev.shape[0] > 0:
            obstacles_prev = fast_detect_obstacles_optimized(
                self.cloud_prev,
                self.cfg,
                min_obstacle_height=self.cfg.min_obstacle_height_prev
            )
            obstacles_all.extend(obstacles_prev)

        if self.cloud_left is not None and self.cloud_left.shape[0] > 0:
            obstacles_left = fast_detect_obstacles_optimized(
                self.cloud_left,
                self.cfg,
                min_obstacle_height=self.cfg.min_obstacle_height_left
            )
            obstacles_all.extend(obstacles_left)

        if self.cloud_right is not None and self.cloud_right.shape[0] > 0:
            obstacles_right = fast_detect_obstacles_optimized(
                self.cloud_right,
                self.cfg,
                min_obstacle_height=self.cfg.min_obstacle_height_right
            )
            obstacles_all.extend(obstacles_right)

        if self.cloud_top is not None and self.cloud_top.shape[0] > 0:
            obstacles_top = fast_detect_obstacles_optimized(
                self.cloud_top,
                self.cfg,
                min_obstacle_height=self.cfg.min_obstacle_height_top
            )
            obstacles_all.extend(obstacles_top)

        if self.cloud_back is not None and self.cloud_back.shape[0] > 0:
            obstacles_back = fast_detect_obstacles_optimized(
                self.cloud_back,
                self.cfg,
                min_obstacle_height=self.cfg.min_obstacle_height_back
            )
            obstacles_all.extend(obstacles_back)

        if self.timing_enabled:
            t1 = time.perf_counter()

        # 统计
        if self.timing_enabled:
            self.timing_count += 1
            self.timing_sum_detect += (t1 - t0)

            # 每100帧输出一次平均耗时（使用 debug 级别，避免刷屏）
            if self.timing_count % 100 == 0:
                avg_detect = self.timing_sum_detect / self.timing_count * 1000
                self.get_logger().debug(
                    f"性能统计(平均): 障碍物检测={avg_detect:.2f}ms"
                )

        result = {
            "stamp": {
                "sec": int(self.latest_header.stamp.sec),
                "nanosec": int(self.latest_header.stamp.nanosec),
            },
            "frame_id": self.latest_header.frame_id,
            "has_obstacle": bool(len(obstacles_all) > 0),
            "obstacles": obstacles_all
        }

        out = String()
        out.data = json.dumps(result)
        self.pub.publish(out)

        if len(obstacles_all) > 0:
            self.get_logger().info(
                f"检测到 {len(obstacles_all)} 个障碍物 "
                f"(前:{len(obstacles_prev)}, "
                f"左:{len(obstacles_left)}, "
                f"右:{len(obstacles_right)}, "
                f"顶:{len(obstacles_top)}, "
                f"后:{len(obstacles_back)})"
            )


def main(args=None):
    rclpy.init(args=args)
    node = FastLidarObstacleDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
