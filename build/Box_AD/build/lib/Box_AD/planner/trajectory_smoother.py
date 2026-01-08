#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
车辆动力学轨迹平滑模块

功能：
1. 基于最小转弯半径约束的路径平滑
2. 曲率连续性优化
3. 三次贝塞尔曲线插值
4. 速度规划（基于曲率限制速度）

适用于：
- local_path_planner_node.py (梯形绕行)
- local_planner_astar_node.py (A*规划)
"""

import math
import numpy as np
from typing import List, Tuple, Optional
from scipy.interpolate import CubicSpline
from dataclasses import dataclass


@dataclass
class VehicleParams:
    """车辆参数"""
    min_turning_radius: float = 3.0   # 最小转弯半径 (m)
    max_curvature: float = 0.33       # 最大曲率 (1/m)，= 1/min_turning_radius
    max_speed: float = 5.0            # 最大速度 (m/s)
    max_lateral_accel: float = 2.0    # 最大横向加速度 (m/s^2)
    wheelbase: float = 2.5            # 轴距 (m)
    safe_distance: float = 1.5        # 与障碍物的安全距离 (m)


class TrajectoryPoint:
    """轨迹点"""
    def __init__(self, x: float, y: float, yaw: float = 0.0,
                 curvature: float = 0.0, speed: float = 0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.curvature = curvature
        self.speed = speed

    def to_tuple(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.yaw)


class TrajectorySmoother:
    """轨迹平滑器（支持障碍物感知）"""

    def __init__(self, vehicle_params: VehicleParams = None):
        self.params = vehicle_params or VehicleParams()
        self.params.max_curvature = 1.0 / max(self.params.min_turning_radius, 0.1)
        self.obstacles: List[Tuple[float, float, float]] = []  # [(x, y, radius), ...]

    def set_obstacles(self, obstacles: List[Tuple[float, float, float]]):
        """设置障碍物列表 [(x, y, radius), ...]"""
        self.obstacles = obstacles if obstacles else []

    def check_collision(self, x: float, y: float) -> Tuple[bool, float]:
        """
        检查点是否与障碍物碰撞

        Returns:
            (is_collision, min_distance): 是否碰撞，最小距离
        """
        if not self.obstacles:
            return False, float('inf')

        min_dist = float('inf')
        safe_dist = self.params.safe_distance

        for ox, oy, radius in self.obstacles:
            dist = math.hypot(x - ox, y - oy) - radius
            min_dist = min(min_dist, dist)

        return min_dist < safe_dist, min_dist

    def check_path_collision(self, path: List[Tuple[float, float]]) -> Tuple[bool, float]:
        """检查整条路径是否碰撞"""
        if not self.obstacles or not path:
            return False, float('inf')

        min_dist = float('inf')
        for x, y in path:
            _, dist = self.check_collision(x, y)
            min_dist = min(min_dist, dist)

        return min_dist < self.params.safe_distance, min_dist

    def smooth(
        self,
        path: List[Tuple[float, float]],
        sample_step: float = 0.3,
        obstacles: List[Tuple[float, float, float]] = None
    ) -> List[TrajectoryPoint]:
        """
        完整的轨迹平滑流程（带障碍物感知）

        Args:
            path: 原始路径点 [(x, y), ...]
            sample_step: 采样步长
            obstacles: 障碍物列表 [(x, y, radius), ...]

        Returns:
            平滑后的轨迹点列表
        """
        if len(path) < 2:
            return [TrajectoryPoint(p[0], p[1]) for p in path]

        # 设置障碍物
        if obstacles is not None:
            self.set_obstacles(obstacles)

        # 保存原始路径用于回退
        original_path = list(path)

        # 1. 带障碍物约束的梯度平滑
        smoothed = self.gradient_smooth_with_obstacles(path)

        # 2. 曲率约束优化（带障碍物检查）
        smoothed = self.curvature_constrained_smooth_with_obstacles(smoothed)

        # 3. 三次样条插值
        smoothed = self.cubic_spline_interpolate(smoothed, sample_step)

        # 4. 碰撞检测 - 如果平滑后碰撞，回退到原始路径
        has_collision, min_dist = self.check_path_collision(smoothed)
        if has_collision:
            # 回退到原始路径，只做简单重采样
            smoothed = self.simple_resample(original_path, sample_step)

        # 5. 计算曲率和航向
        trajectory = self.compute_trajectory_attributes(smoothed)

        # 6. 速度规划
        trajectory = self.plan_speed(trajectory)

        return trajectory

    def simple_resample(
        self,
        path: List[Tuple[float, float]],
        sample_step: float
    ) -> List[Tuple[float, float]]:
        """简单重采样，不改变路径形状"""
        if len(path) < 2:
            return path

        result = [path[0]]
        accumulated = 0.0

        for i in range(1, len(path)):
            p0 = np.array(path[i-1])
            p1 = np.array(path[i])
            seg = p1 - p0
            seg_len = np.linalg.norm(seg)

            if seg_len < 1e-6:
                continue

            direction = seg / seg_len
            pos = 0.0

            while pos + (sample_step - accumulated) <= seg_len:
                pos += (sample_step - accumulated)
                new_pt = p0 + direction * pos
                result.append((float(new_pt[0]), float(new_pt[1])))
                accumulated = 0.0

            accumulated += (seg_len - pos)

        # 确保终点在路径中
        if len(result) < 2 or math.hypot(result[-1][0] - path[-1][0], result[-1][1] - path[-1][1]) > 0.1:
            result.append(path[-1])

        return result

    def gradient_smooth_with_obstacles(
        self,
        path: List[Tuple[float, float]],
        iterations: int = 100,
        weight_data: float = 0.2,
        weight_smooth: float = 0.2,
        weight_obstacle: float = 0.5
    ) -> List[Tuple[float, float]]:
        """
        带障碍物感知的梯度下降平滑

        增加障碍物排斥项，防止平滑后的路径靠近障碍物
        """
        if len(path) <= 2:
            return path

        points = np.array(path, dtype=np.float64)
        original = points.copy()
        smoothed = points.copy()
        safe_dist = self.params.safe_distance

        for _ in range(iterations):
            max_change = 0.0
            for i in range(1, len(points) - 1):
                prev_pt = smoothed[i - 1]
                next_pt = smoothed[i + 1]
                orig_pt = original[i]
                curr_pt = smoothed[i]

                # 数据项：保持接近原始点
                data_term = weight_data * (orig_pt - curr_pt)

                # 平滑项：减少折线
                smooth_term = weight_smooth * (prev_pt + next_pt - 2.0 * curr_pt)

                # 障碍物排斥项
                obstacle_term = np.zeros(2)
                for ox, oy, radius in self.obstacles:
                    obs_center = np.array([ox, oy])
                    diff = curr_pt - obs_center
                    dist = np.linalg.norm(diff)
                    clearance = dist - radius

                    if clearance < safe_dist * 2:  # 在影响范围内
                        if dist > 1e-6:
                            # 排斥力与距离成反比
                            repulsion = (safe_dist * 2 - clearance) / (safe_dist * 2)
                            obstacle_term += weight_obstacle * repulsion * (diff / dist)

                new_pt = curr_pt + data_term + smooth_term + obstacle_term

                # 检查新位置是否安全
                is_collision, _ = self.check_collision(new_pt[0], new_pt[1])
                if is_collision:
                    # 如果新位置碰撞，保持原位置
                    new_pt = curr_pt

                change = np.linalg.norm(new_pt - curr_pt)
                max_change = max(max_change, change)
                smoothed[i] = new_pt

            if max_change < 1e-4:
                break

        return [tuple(p) for p in smoothed]

    def curvature_constrained_smooth_with_obstacles(
        self,
        path: List[Tuple[float, float]],
        iterations: int = 50,
        curvature_weight: float = 0.3
    ) -> List[Tuple[float, float]]:
        """
        带障碍物检查的曲率约束平滑
        """
        if len(path) <= 2:
            return path

        points = np.array(path, dtype=np.float64)
        max_curv = self.params.max_curvature

        for _ in range(iterations):
            changed = False
            for i in range(1, len(points) - 1):
                p0 = points[i - 1]
                p1 = points[i]
                p2 = points[i + 1]

                # 计算曲率
                radius, center = self._compute_turning_radius(p0, p1, p2)

                if radius is not None and radius > 1e-6:
                    curvature = 1.0 / radius

                    if curvature > max_curv and center is not None:
                        # 将点向圆心反方向推
                        direction = p1 - center
                        norm = np.linalg.norm(direction)
                        if norm > 1e-6:
                            direction = direction / norm
                            push_dist = curvature_weight * (curvature - max_curv) / max_curv
                            new_pt = p1 + push_dist * direction

                            # 检查新位置是否安全
                            is_collision, _ = self.check_collision(new_pt[0], new_pt[1])
                            if not is_collision:
                                points[i] = new_pt
                                changed = True
                            # 如果碰撞，保持原位置（宁可曲率大，也不能碰撞）

            if not changed:
                break

        return [tuple(p) for p in points]

    def cubic_spline_interpolate(
        self,
        path: List[Tuple[float, float]],
        sample_step: float = 0.3
    ) -> List[Tuple[float, float]]:
        """
        三次样条插值

        生成曲率连续的平滑曲线
        """
        if len(path) <= 2:
            return path

        points = np.array(path, dtype=np.float64)
        x = points[:, 0]
        y = points[:, 1]

        # 计算累积弧长作为参数
        distances = np.zeros(len(points))
        for i in range(1, len(points)):
            distances[i] = distances[i-1] + np.hypot(x[i] - x[i-1], y[i] - y[i-1])

        total_length = distances[-1]
        if total_length < 1e-6:
            return path

        # 归一化参数 t ∈ [0, 1]
        t = distances / total_length

        try:
            # 创建三次样条（自然边界条件）
            cs_x = CubicSpline(t, x, bc_type='natural')
            cs_y = CubicSpline(t, y, bc_type='natural')

            # 按固定步长采样
            num_samples = max(int(total_length / sample_step), len(path))
            t_new = np.linspace(0, 1, num_samples)

            x_new = cs_x(t_new)
            y_new = cs_y(t_new)

            return [(float(x_new[i]), float(y_new[i])) for i in range(len(x_new))]

        except Exception:
            return path

    def bezier_smooth_corners(
        self,
        path: List[Tuple[float, float]],
        corner_radius: float = None
    ) -> List[Tuple[float, float]]:
        """
        在拐角处使用贝塞尔曲线平滑

        Args:
            path: 原始路径
            corner_radius: 拐角平滑半径，默认使用最小转弯半径
        """
        if len(path) <= 2:
            return path

        if corner_radius is None:
            corner_radius = self.params.min_turning_radius

        result = [path[0]]

        for i in range(1, len(path) - 1):
            p0 = np.array(path[i - 1])
            p1 = np.array(path[i])
            p2 = np.array(path[i + 1])

            # 计算进入和离开方向
            v_in = p1 - p0
            v_out = p2 - p1
            len_in = np.linalg.norm(v_in)
            len_out = np.linalg.norm(v_out)

            if len_in < 1e-6 or len_out < 1e-6:
                result.append(tuple(p1))
                continue

            v_in = v_in / len_in
            v_out = v_out / len_out

            # 计算转角
            cos_angle = np.clip(np.dot(v_in, v_out), -1, 1)
            angle = math.acos(cos_angle)

            # 如果接近直线，不需要平滑
            if angle < 0.1:  # ~6度
                result.append(tuple(p1))
                continue

            # 计算贝塞尔曲线控制点
            # 控制点距离 = 半径 * tan(angle/2)
            ctrl_dist = min(corner_radius * math.tan(angle / 2),
                           len_in * 0.4, len_out * 0.4)

            # 起点和终点
            bezier_start = p1 - v_in * ctrl_dist
            bezier_end = p1 + v_out * ctrl_dist

            # 生成贝塞尔曲线点
            bezier_points = self._quadratic_bezier(
                bezier_start, p1, bezier_end, num_points=10
            )

            result.extend([tuple(p) for p in bezier_points])

        result.append(path[-1])
        return result

    def compute_trajectory_attributes(
        self,
        path: List[Tuple[float, float]]
    ) -> List[TrajectoryPoint]:
        """
        计算轨迹的航向和曲率
        """
        if not path:
            return []

        trajectory = []

        for i, (x, y) in enumerate(path):
            # 计算航向
            if i < len(path) - 1:
                dx = path[i + 1][0] - x
                dy = path[i + 1][1] - y
                yaw = math.atan2(dy, dx)
            else:
                yaw = trajectory[-1].yaw if trajectory else 0.0

            # 计算曲率（使用三点法）
            if 0 < i < len(path) - 1:
                p0 = np.array(path[i - 1])
                p1 = np.array([x, y])
                p2 = np.array(path[i + 1])
                radius, _ = self._compute_turning_radius(p0, p1, p2)
                curvature = 1.0 / radius if radius and radius > 1e-6 else 0.0
            else:
                curvature = 0.0

            trajectory.append(TrajectoryPoint(x, y, yaw, curvature))

        return trajectory

    def plan_speed(
        self,
        trajectory: List[TrajectoryPoint]
    ) -> List[TrajectoryPoint]:
        """
        基于曲率的速度规划

        v = min(v_max, sqrt(a_lat_max / |curvature|))
        """
        if not trajectory:
            return trajectory

        v_max = self.params.max_speed
        a_lat_max = self.params.max_lateral_accel

        for point in trajectory:
            if abs(point.curvature) > 1e-6:
                # 横向加速度约束: a_lat = v^2 * curvature
                # => v = sqrt(a_lat / curvature)
                v_curv = math.sqrt(a_lat_max / abs(point.curvature))
                point.speed = min(v_max, v_curv)
            else:
                point.speed = v_max

        return trajectory

    def _compute_turning_radius(
        self,
        p0: np.ndarray,
        p1: np.ndarray,
        p2: np.ndarray
    ) -> Tuple[Optional[float], Optional[np.ndarray]]:
        """
        计算三点确定的圆的半径和圆心
        """
        x0, y0 = p0
        x1, y1 = p1
        x2, y2 = p2

        d = 2 * (x0 * (y1 - y2) + x1 * (y2 - y0) + x2 * (y0 - y1))
        if abs(d) < 1e-9:
            return None, None

        ux = ((x0**2 + y0**2) * (y1 - y2) +
              (x1**2 + y1**2) * (y2 - y0) +
              (x2**2 + y2**2) * (y0 - y1)) / d
        uy = ((x0**2 + y0**2) * (x2 - x1) +
              (x1**2 + y1**2) * (x0 - x2) +
              (x2**2 + y2**2) * (x1 - x0)) / d

        center = np.array([ux, uy])
        radius = float(np.linalg.norm(center - p0))

        return radius, center

    def _quadratic_bezier(
        self,
        p0: np.ndarray,
        p1: np.ndarray,
        p2: np.ndarray,
        num_points: int = 10
    ) -> List[np.ndarray]:
        """
        二次贝塞尔曲线
        B(t) = (1-t)^2 * P0 + 2(1-t)t * P1 + t^2 * P2
        """
        points = []
        for i in range(num_points + 1):
            t = i / num_points
            mt = 1 - t
            point = mt * mt * p0 + 2 * mt * t * p1 + t * t * p2
            points.append(point)
        return points


def smooth_path(
    path: List[Tuple[float, float]],
    min_turning_radius: float = 3.0,
    sample_step: float = 0.3
) -> List[Tuple[float, float, float]]:
    """
    便捷函数：平滑路径并返回 (x, y, yaw) 列表

    Args:
        path: 原始路径 [(x, y), ...]
        min_turning_radius: 最小转弯半径
        sample_step: 采样步长

    Returns:
        平滑后的轨迹 [(x, y, yaw), ...]
    """
    params = VehicleParams(min_turning_radius=min_turning_radius)
    smoother = TrajectorySmoother(params)
    trajectory = smoother.smooth(path, sample_step)
    return [point.to_tuple() for point in trajectory]


def smooth_path_with_speed(
    path: List[Tuple[float, float]],
    min_turning_radius: float = 3.0,
    max_speed: float = 5.0,
    sample_step: float = 0.3
) -> List[Tuple[float, float, float, float]]:
    """
    便捷函数：平滑路径并返回带速度的轨迹

    Returns:
        [(x, y, yaw, speed), ...]
    """
    params = VehicleParams(min_turning_radius=min_turning_radius, max_speed=max_speed)
    smoother = TrajectorySmoother(params)
    trajectory = smoother.smooth(path, sample_step)
    return [(p.x, p.y, p.yaw, p.speed) for p in trajectory]


# ==================== 测试代码 ====================

if __name__ == '__main__':
    import matplotlib.pyplot as plt

    # 测试路径（带拐角）
    raw_path = [
        (0, 0), (10, 0), (15, 5), (20, 5), (25, 10), (35, 10), (40, 5), (50, 5)
    ]

    # 创建平滑器
    params = VehicleParams(min_turning_radius=3.0, max_speed=5.0)
    smoother = TrajectorySmoother(params)

    # 平滑
    trajectory = smoother.smooth(raw_path, sample_step=0.5)

    # 绘图
    fig, axes = plt.subplots(1, 3, figsize=(15, 4))

    # 路径对比
    ax1 = axes[0]
    ax1.set_title('Path Smoothing')
    raw_x = [p[0] for p in raw_path]
    raw_y = [p[1] for p in raw_path]
    ax1.plot(raw_x, raw_y, 'r--o', label='Raw', markersize=8)

    smooth_x = [p.x for p in trajectory]
    smooth_y = [p.y for p in trajectory]
    ax1.plot(smooth_x, smooth_y, 'g-', linewidth=2, label='Smoothed')
    ax1.legend()
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)

    # 曲率
    ax2 = axes[1]
    ax2.set_title('Curvature Profile')
    curvatures = [p.curvature for p in trajectory]
    ax2.plot(curvatures, 'b-')
    ax2.axhline(y=params.max_curvature, color='r', linestyle='--', label='Max Curvature')
    ax2.axhline(y=-params.max_curvature, color='r', linestyle='--')
    ax2.set_ylabel('Curvature (1/m)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # 速度
    ax3 = axes[2]
    ax3.set_title('Speed Profile')
    speeds = [p.speed for p in trajectory]
    ax3.plot(speeds, 'g-')
    ax3.axhline(y=params.max_speed, color='r', linestyle='--', label='Max Speed')
    ax3.set_ylabel('Speed (m/s)')
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('/home/wudi/slam/Box_AD/trajectory_smoothing_demo.png', dpi=150)
    print("Demo saved to: trajectory_smoothing_demo.png")
    plt.close()
