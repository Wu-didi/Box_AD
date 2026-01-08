#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
fast_obstacle_temporal_filter_node.py

功能：
- 订阅 /fast_obstacles（瞬时障碍：每帧重新计算）
- 在时间维度上对障碍栅格做计数滤波，抑制瞬时噪声 / 掉点
- 发布 /fast_obstacles_stable（稳定障碍），供局部规划使用

逻辑：
- 使用与 fast_lidar_obstacle_detector 相同的 ROI 和 grid_resolution
- 对每个障碍点 (x,y) 计算栅格索引 (ix,iy)
- 维护 cell_counts[(ix,iy)]：
    * 当前帧有占用：count += inc_step（不超过 max_count）
    * 当前帧无占用：count -= dec_step（不小于 0）
- 只有 count >= confirm_threshold 的栅格才输出
"""

import json
import math
from typing import Dict, Tuple, Set

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class TemporalFilterConfig:
    def __init__(self):
        # 必须与 fast_lidar_obstacle_detector 里的 FastConfig 保持一致
        self.xmin = 0.0
        self.xmax = 50.0
        self.ymin = -15.0
        self.ymax = 15.0
        self.grid_resolution = 0.5

        # 计数滤波参数
        self.confirm_threshold = 2   # 连续 / 综合计数 >= 2 认为“稳定障碍”
        self.inc_step = 1           # 每帧有占用时 +1
        self.dec_step = 1           # 每帧无占用时 -1
        self.max_count = 5          # 计数上限，防止无限增大

        # 输出时障碍的 z 高度（fast_obstacles 自己带 z，这里可用其均值；也可用固定值）
        self.default_z = 0.0        # 如果不想动 z，可以用 fast_obstacles 原值里的 z


class FastObstacleTemporalFilterNode(Node):
    def __init__(self):
        super().__init__("fast_obstacle_temporal_filter")

        self.cfg = TemporalFilterConfig()

        # (ix, iy) -> count
        self.cell_counts: Dict[Tuple[int, int], int] = {}

        # (ix, iy) -> 最近一次观测到的 (x, y, z)
        self.cell_last_xyz: Dict[Tuple[int, int], Tuple[float, float, float]] = {}

        # 订阅瞬时障碍
        self.sub = self.create_subscription(
            String,
            "/fast_obstacles",
            self.obstacles_callback,
            10
        )

        # 发布稳定障碍
        self.pub = self.create_publisher(
            String,
            "/fast_obstacles_stable",
            10
        )

        self.get_logger().info(
            "FastObstacleTemporalFilterNode started.\n"
            "Sub: /fast_obstacles  ->  Pub: /fast_obstacles_stable"
        )

    def _xy_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        """将 (x, y) 投影到栅格索引 (ix, iy)。"""
        res = self.cfg.grid_resolution
        ix = int(math.floor((x - self.cfg.xmin) / res))
        iy = int(math.floor((y - self.cfg.ymin) / res))
        return ix, iy

    def obstacles_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"Failed to parse JSON from /fast_obstacles: {e}")
            return

        frame_id = data.get("frame_id", "lidar")
        stamp = data.get("stamp", None)
        obstacles = data.get("obstacles", [])

        # 当前帧占用的 cell 集合
        current_cells: Set[Tuple[int, int]] = set()

        for obj in obstacles:
            try:
                x = float(obj["x"])
                y = float(obj["y"])
                z = float(obj.get("z", self.cfg.default_z))
            except KeyError as e:
                self.get_logger().warn(f"Missing key in obstacle: {e}")
                continue

            cell = self._xy_to_cell(x, y)
            current_cells.add(cell)

            # 更新最近一次观测到的位置
            self.cell_last_xyz[cell] = (x, y, z)

            # 更新计数：有占用 -> 自增
            old_c = self.cell_counts.get(cell, 0)
            new_c = min(self.cfg.max_count, old_c + self.cfg.inc_step)
            self.cell_counts[cell] = new_c

        # 对所有历史 cell 做衰减：本帧没出现的格子 count 减少
        for cell in list(self.cell_counts.keys()):
            if cell not in current_cells:
                old_c = self.cell_counts[cell]
                new_c = max(0, old_c - self.cfg.dec_step)
                if new_c == 0:
                    # 计数为 0 时清理掉，防止 dict 无限膨胀
                    del self.cell_counts[cell]
                    if cell in self.cell_last_xyz:
                        del self.cell_last_xyz[cell]
                else:
                    self.cell_counts[cell] = new_c

        # 生成稳定障碍列表：count >= threshold 的 cell
        stable_obstacles = []
        for cell, c in self.cell_counts.items():
            if c < self.cfg.confirm_threshold:
                continue
            if cell not in self.cell_last_xyz:
                continue
            x, y, z = self.cell_last_xyz[cell]
            stable_obstacles.append({
                "x": float(x),
                "y": float(y),
                "z": float(z),
                "count": int(c)  # 这里用的是“计数记忆”，不是点数
            })

        result = {
            "stamp": stamp if isinstance(stamp, dict) else {},  # 原样透传或空 dict
            "frame_id": frame_id,
            "has_obstacle": bool(len(stable_obstacles) > 0),
            "obstacles": stable_obstacles
        }

        out = String()
        out.data = json.dumps(result)
        self.pub.publish(out)

        # debug 日志可以随需开关
        # self.get_logger().info(f"stable cells: {len(stable_obstacles)}")


def main(args=None):
    rclpy.init(args=args)
    node = FastObstacleTemporalFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
