#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Hybrid A* 仿真测试脚本
- 加载地图
- 测试不同场景（直行、转弯、掉头）
- 可视化结果
"""

import math
import heapq
import time
import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrow, Rectangle
import yaml
import os


class HybridAstarTest:
    def __init__(self, map_yaml_path):
        """加载地图"""
        self.load_map(map_yaml_path)

        # 车辆参数
        self.wheelbase = 2.85       # 轴距(m)
        self.max_steer = 1.047      # 最大转向角 60度
        self.vehicle_length = 4.53  # 车长(m)
        self.vehicle_width = 1.9    # 车宽(m)

        # Hybrid A* 参数 - 优化版
        self.motion_step = 1.5      # 增大步长，加快搜索
        self.num_steers = 5         # 转向离散数
        self.theta_resolution = math.radians(15)  # 15度分辨率，减少状态数
        self.robot_radius = 1.5     # 膨胀半径

        # 生成转向角列表
        self.steer_angles = np.linspace(-self.max_steer, self.max_steer, self.num_steers).tolist()

        # 膨胀地图
        self.inflate_map()

    def load_map(self, yaml_path):
        """加载地图"""
        map_dir = os.path.dirname(yaml_path)
        with open(yaml_path, 'r') as f:
            map_config = yaml.safe_load(f)

        image_path = os.path.join(map_dir, map_config['image'])
        self.resolution = map_config['resolution']
        self.origin_x = map_config['origin'][0]
        self.origin_y = map_config['origin'][1]

        # 加载图像
        img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise RuntimeError(f"无法加载地图: {image_path}")

        # 转换为栅格：白色=free(0), 黑色=obstacle(100), 灰色=unknown(-1)
        self.grid = np.zeros_like(img, dtype=np.int8)
        self.grid[img < 50] = 100      # 黑色 -> 障碍
        self.grid[img > 200] = 0       # 白色 -> free
        self.grid[(img >= 50) & (img <= 200)] = -1  # 灰色 -> unknown

        self.height, self.width = self.grid.shape
        self.original_grid = self.grid.copy()

        print(f"地图加载成功: {self.width}x{self.height}, 分辨率={self.resolution}m")

    def inflate_map(self):
        """膨胀障碍物"""
        obst = (self.original_grid == 100).astype(np.uint8)
        inflation_cells = int(self.robot_radius / self.resolution)
        kernel_size = 2 * inflation_cells + 1
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        inflated = cv2.dilate(obst, kernel)

        self.grid = self.original_grid.copy()
        self.grid[inflated == 1] = 100
        print(f"障碍膨胀完成: inflation_cells={inflation_cells}")

    def world_to_grid(self, x, y):
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        return row, col

    def grid_to_world(self, row, col):
        x = self.origin_x + (col + 0.5) * self.resolution
        y = self.origin_y + (row + 0.5) * self.resolution
        return x, y

    def in_bounds(self, r, c):
        return 0 <= r < self.height and 0 <= c < self.width

    def is_free(self, r, c):
        if not self.in_bounds(r, c):
            return False
        return self.grid[r, c] == 0

    def is_world_free(self, x, y):
        r, c = self.world_to_grid(x, y)
        return self.is_free(r, c)

    def check_path_collision(self, x1, y1, x2, y2):
        """检查路径段是否无碰撞"""
        dist = math.hypot(x2 - x1, y2 - y1)
        if dist < 0.01:
            return self.is_world_free(x1, y1)
        steps = max(int(dist / (self.resolution * 0.5)), 2)
        for i in range(steps + 1):
            t = i / steps
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            if not self.is_world_free(x, y):
                return False
        return True

    def vehicle_model(self, x, y, theta, steer, direction=1):
        """Ackermann 运动学模型"""
        step = self.motion_step * direction

        if abs(steer) < 0.01:
            new_x = x + step * math.cos(theta)
            new_y = y + step * math.sin(theta)
            new_theta = theta
        else:
            # 转弯半径
            turn_radius = self.wheelbase / math.tan(abs(steer))
            # 角度变化
            beta = step / turn_radius
            if steer < 0:
                beta = -beta

            new_theta = theta + beta
            # 使用圆弧运动公式
            if abs(beta) > 0.001:
                new_x = x + turn_radius * (math.sin(new_theta) - math.sin(theta)) * np.sign(steer)
                new_y = y - turn_radius * (math.cos(new_theta) - math.cos(theta)) * np.sign(steer)
            else:
                new_x = x + step * math.cos(theta)
                new_y = y + step * math.sin(theta)

        # 归一化角度
        while new_theta > math.pi:
            new_theta -= 2 * math.pi
        while new_theta < -math.pi:
            new_theta += 2 * math.pi

        return new_x, new_y, new_theta

    def hybrid_astar(self, start, goal, allow_reverse=True):
        """
        优化版 Hybrid A*

        start: (x, y, theta)
        goal: (x, y, theta)
        """
        sx, sy, s_theta = start
        gx, gy, g_theta = goal

        print(f"\n[Hybrid A*] 起点: ({sx:.1f}, {sy:.1f}, {math.degrees(s_theta):.0f}°)")
        print(f"[Hybrid A*] 终点: ({gx:.1f}, {gy:.1f}, {math.degrees(g_theta):.0f}°)")

        # 检查起点终点
        if not self.is_world_free(sx, sy):
            print("[错误] 起点在障碍物内")
            return None
        if not self.is_world_free(gx, gy):
            print("[错误] 终点在障碍物内")
            return None

        # 目标阈值
        xy_threshold = 2.0  # 位置误差
        theta_threshold = math.radians(30)  # 朝向误差

        # 离散化函数
        def theta_index(theta):
            theta = theta % (2 * math.pi)
            return int(theta / self.theta_resolution)

        def state_key(x, y, theta):
            r, c = self.world_to_grid(x, y)
            ti = theta_index(theta)
            return (r, c, ti)

        # 启发式：欧几里得距离 + 朝向惩罚
        def heuristic(x, y, theta):
            dist = math.hypot(x - gx, y - gy)
            # 朝向差异
            angle_diff = abs(theta - g_theta)
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff
            # 需要掉头时增加惩罚
            return dist + 0.5 * angle_diff * self.wheelbase

        # 优先队列
        counter = 0
        open_set = []

        start_key = state_key(sx, sy, s_theta)
        g_cost = {start_key: 0.0}
        came_from = {start_key: None}
        state_map = {start_key: (sx, sy, s_theta)}

        heapq.heappush(open_set, (heuristic(sx, sy, s_theta), counter, start_key))
        counter += 1

        visited = set()
        max_iterations = 100000

        # 允许的运动方向
        directions = [1, -1] if allow_reverse else [1]

        start_time = time.time()

        for iteration in range(max_iterations):
            if not open_set:
                print(f"[Hybrid A*] 开放列表为空")
                break

            _, _, current_key = heapq.heappop(open_set)

            if current_key in visited:
                continue
            visited.add(current_key)

            cx, cy, c_theta = state_map[current_key]

            # 检查是否到达目标
            dist_to_goal = math.hypot(cx - gx, cy - gy)
            angle_to_goal = abs(c_theta - g_theta)
            if angle_to_goal > math.pi:
                angle_to_goal = 2 * math.pi - angle_to_goal

            if dist_to_goal < xy_threshold and angle_to_goal < theta_threshold:
                # 回溯路径
                path = []
                key = current_key
                while key is not None:
                    path.append(state_map[key])
                    key = came_from[key]
                path.reverse()

                elapsed = time.time() - start_time
                print(f"[Hybrid A*] 找到路径! 迭代={iteration}, 时间={elapsed:.2f}s, 路径点={len(path)}")
                return path

            # 展开邻居
            current_g = g_cost[current_key]

            for direction in directions:
                for steer in self.steer_angles:
                    nx, ny, n_theta = self.vehicle_model(cx, cy, c_theta, steer, direction)

                    # 检查碰撞
                    if not self.check_path_collision(cx, cy, nx, ny):
                        continue

                    neighbor_key = state_key(nx, ny, n_theta)
                    if neighbor_key in visited:
                        continue

                    # 计算代价
                    step_cost = self.motion_step
                    if direction < 0:
                        step_cost *= 1.5  # 倒车惩罚
                    step_cost += 0.2 * abs(steer)  # 转向惩罚

                    new_g = current_g + step_cost

                    if neighbor_key not in g_cost or new_g < g_cost[neighbor_key]:
                        g_cost[neighbor_key] = new_g
                        came_from[neighbor_key] = current_key
                        state_map[neighbor_key] = (nx, ny, n_theta)
                        f = new_g + heuristic(nx, ny, n_theta)
                        heapq.heappush(open_set, (f, counter, neighbor_key))
                        counter += 1

            if iteration % 10000 == 0 and iteration > 0:
                elapsed = time.time() - start_time
                print(f"[Hybrid A*] 迭代={iteration}, 已访问={len(visited)}, 距离={dist_to_goal:.1f}m, 时间={elapsed:.1f}s")

        elapsed = time.time() - start_time
        print(f"[Hybrid A*] 搜索失败, 时间={elapsed:.2f}s")
        return None

    def visualize(self, path=None, start=None, goal=None, title="Hybrid A* Test"):
        """可视化地图和路径"""
        fig, ax = plt.subplots(1, 1, figsize=(12, 10))

        # 显示地图
        display_grid = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        display_grid[self.grid == 0] = [255, 255, 255]    # free = 白
        display_grid[self.grid == 100] = [0, 0, 0]        # obstacle = 黑
        display_grid[self.grid == -1] = [128, 128, 128]   # unknown = 灰

        # 翻转y轴以匹配世界坐标
        ax.imshow(display_grid, origin='lower',
                  extent=[self.origin_x, self.origin_x + self.width * self.resolution,
                          self.origin_y, self.origin_y + self.height * self.resolution])

        # 画路径
        if path is not None and len(path) > 0:
            xs = [p[0] for p in path]
            ys = [p[1] for p in path]
            ax.plot(xs, ys, 'b-', linewidth=2, label='Path')

            # 每隔几个点画一个箭头表示朝向
            for i in range(0, len(path), max(1, len(path)//20)):
                x, y, theta = path[i]
                dx = 1.0 * math.cos(theta)
                dy = 1.0 * math.sin(theta)
                ax.arrow(x, y, dx, dy, head_width=0.5, head_length=0.3, fc='blue', ec='blue')

        # 画起点
        if start is not None:
            sx, sy, s_theta = start
            ax.plot(sx, sy, 'go', markersize=15, label='Start')
            dx = 2.0 * math.cos(s_theta)
            dy = 2.0 * math.sin(s_theta)
            ax.arrow(sx, sy, dx, dy, head_width=0.8, head_length=0.5, fc='green', ec='green', linewidth=2)

        # 画终点
        if goal is not None:
            gx, gy, g_theta = goal
            ax.plot(gx, gy, 'ro', markersize=15, label='Goal')
            dx = 2.0 * math.cos(g_theta)
            dy = 2.0 * math.sin(g_theta)
            ax.arrow(gx, gy, dx, dy, head_width=0.8, head_length=0.5, fc='red', ec='red', linewidth=2)

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title(title)
        ax.legend()
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        save_path = f'/home/nvidia/vcii/wudi/Box_AD/Box_AD/planner/{title.replace(" ", "_").replace(":", "")}.png'
        plt.savefig(save_path, dpi=150)
        print(f"可视化结果已保存到 {save_path}")
        plt.close()


def main():
    # 加载地图
    map_yaml = '/home/nvidia/vcii/wudi/Box_AD/maps/map.yaml'
    planner = HybridAstarTest(map_yaml)

    print(f"地图范围: X=[{planner.origin_x:.1f}, {planner.origin_x + planner.width * planner.resolution:.1f}]")
    print(f"          Y=[{planner.origin_y:.1f}, {planner.origin_y + planner.height * planner.resolution:.1f}]")

    # 自动找到可通行的点
    free_points = []
    for r in range(50, planner.height - 50, 30):
        for c in range(50, planner.width - 50, 30):
            if planner.is_free(r, c):
                x, y = planner.grid_to_world(r, c)
                free_points.append((x, y))

    print(f"找到 {len(free_points)} 个可用测试点")

    if len(free_points) < 4:
        print("可用测试点太少!")
        return

    # 按y坐标排序，选择不同区域的点
    free_points.sort(key=lambda p: p[1])

    # 测试场景1: 直行（选择y方向间隔大的两点）
    print("\n" + "="*60)
    print("测试场景1: 直行")
    print("="*60)

    p1 = free_points[len(free_points)//4]
    p2 = free_points[3*len(free_points)//4]

    # 计算方向
    direction = math.atan2(p2[1] - p1[1], p2[0] - p1[0])

    start = (p1[0], p1[1], direction)
    goal = (p2[0], p2[1], direction)

    print(f"起点: ({p1[0]:.1f}, {p1[1]:.1f}), 终点: ({p2[0]:.1f}, {p2[1]:.1f})")
    print(f"直线距离: {math.hypot(p2[0]-p1[0], p2[1]-p1[1]):.1f}m")

    path1 = planner.hybrid_astar(start, goal, allow_reverse=False)
    planner.visualize(path1, start, goal, "Test 1: Straight")

    # 测试场景2: 90度转弯
    print("\n" + "="*60)
    print("测试场景2: 90度转弯")
    print("="*60)

    p3 = free_points[len(free_points)//3]
    p4 = free_points[len(free_points)//2]

    start = (p3[0], p3[1], 0.0)              # 朝东
    goal = (p4[0], p4[1], math.pi/2)         # 朝北

    print(f"起点: ({p3[0]:.1f}, {p3[1]:.1f}) 朝东, 终点: ({p4[0]:.1f}, {p4[1]:.1f}) 朝北")

    path2 = planner.hybrid_astar(start, goal, allow_reverse=False)
    planner.visualize(path2, start, goal, "Test 2: 90-degree Turn")

    # 测试场景3: 掉头
    print("\n" + "="*60)
    print("测试场景3: 掉头 (180度)")
    print("="*60)

    p5 = free_points[len(free_points)//2]

    start = (p5[0], p5[1], 0.0)              # 朝东
    goal = (p5[0] + 15, p5[1], math.pi)      # 朝西，前进15m后掉头

    print(f"起点: ({p5[0]:.1f}, {p5[1]:.1f}) 朝东, 终点: ({p5[0]+15:.1f}, {p5[1]:.1f}) 朝西")

    path3 = planner.hybrid_astar(start, goal, allow_reverse=False)
    planner.visualize(path3, start, goal, "Test 3: U-Turn (180-degree)")


if __name__ == '__main__':
    main()
