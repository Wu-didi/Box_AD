#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Hybrid A* 测试脚本 v2 - 更可靠的测试
"""

import math
import heapq
import time
import cv2
import numpy as np
import matplotlib.pyplot as plt
import yaml
import os


class HybridAstarTest:
    def __init__(self, map_yaml_path):
        self.load_map(map_yaml_path)

        # 车辆参数
        self.wheelbase = 2.85
        self.max_steer = 1.047  # 60度

        # Hybrid A* 参数
        self.motion_step = 1.5
        self.num_steers = 5
        self.theta_resolution = math.radians(15)
        self.robot_radius = 1.5

        self.steer_angles = np.linspace(-self.max_steer, self.max_steer, self.num_steers).tolist()

        # 膨胀地图
        self.inflate_map()

    def load_map(self, yaml_path):
        map_dir = os.path.dirname(yaml_path)
        with open(yaml_path, 'r') as f:
            map_config = yaml.safe_load(f)

        image_path = os.path.join(map_dir, map_config['image'])
        self.resolution = map_config['resolution']
        self.origin_x = map_config['origin'][0]
        self.origin_y = map_config['origin'][1]

        img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise RuntimeError(f"无法加载地图: {image_path}")

        self.grid = np.zeros_like(img, dtype=np.int8)
        self.grid[img < 50] = 100
        self.grid[img > 200] = 0
        self.grid[(img >= 50) & (img <= 200)] = -1

        self.height, self.width = self.grid.shape
        self.original_grid = self.grid.copy()
        print(f"地图: {self.width}x{self.height}, 分辨率={self.resolution}m")

    def inflate_map(self):
        obst = (self.original_grid == 100).astype(np.uint8)
        inflation_cells = int(self.robot_radius / self.resolution)
        kernel_size = 2 * inflation_cells + 1
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        inflated = cv2.dilate(obst, kernel)
        self.grid = self.original_grid.copy()
        self.grid[inflated == 1] = 100

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

    def find_free_point_near(self, x, y, search_radius=10.0):
        """在指定位置附近找一个自由点"""
        for r in np.arange(0, search_radius, self.resolution * 2):
            for angle in np.linspace(0, 2*math.pi, 16):
                nx = x + r * math.cos(angle)
                ny = y + r * math.sin(angle)
                if self.is_world_free(nx, ny):
                    return nx, ny
        return None

    def check_path_collision(self, x1, y1, x2, y2):
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

    def vehicle_model(self, x, y, theta, steer):
        step = self.motion_step
        if abs(steer) < 0.01:
            new_x = x + step * math.cos(theta)
            new_y = y + step * math.sin(theta)
            new_theta = theta
        else:
            turn_radius = self.wheelbase / math.tan(abs(steer))
            beta = step / turn_radius
            if steer < 0:
                beta = -beta
            new_theta = theta + beta
            new_x = x + turn_radius * (math.sin(new_theta) - math.sin(theta)) * np.sign(steer)
            new_y = y - turn_radius * (math.cos(new_theta) - math.cos(theta)) * np.sign(steer)

        while new_theta > math.pi:
            new_theta -= 2 * math.pi
        while new_theta < -math.pi:
            new_theta += 2 * math.pi
        return new_x, new_y, new_theta

    def hybrid_astar(self, start, goal):
        """Hybrid A* - 只允许前进"""
        sx, sy, s_theta = start
        gx, gy, g_theta = goal

        # 检查起点终点
        if not self.is_world_free(sx, sy):
            print(f"[错误] 起点 ({sx:.1f}, {sy:.1f}) 在障碍物内!")
            return None
        if not self.is_world_free(gx, gy):
            print(f"[错误] 终点 ({gx:.1f}, {gy:.1f}) 在障碍物内!")
            return None

        xy_threshold = 2.0
        theta_threshold = math.radians(30)

        def theta_index(theta):
            theta = theta % (2 * math.pi)
            return int(theta / self.theta_resolution)

        def heuristic(x, y, theta):
            dist = math.hypot(x - gx, y - gy)
            angle_diff = abs(theta - g_theta)
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff
            return dist + 0.5 * angle_diff * self.wheelbase

        def state_key(x, y, theta):
            r, c = self.world_to_grid(x, y)
            ti = theta_index(theta)
            return (r, c, ti)

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
        start_time = time.time()

        for iteration in range(max_iterations):
            if not open_set:
                print(f"[Hybrid A*] 开放列表为空")
                return None

            _, _, current_key = heapq.heappop(open_set)
            if current_key in visited:
                continue
            visited.add(current_key)

            cx, cy, c_theta = state_map[current_key]
            dist_to_goal = math.hypot(cx - gx, cy - gy)
            angle_to_goal = abs(c_theta - g_theta)
            if angle_to_goal > math.pi:
                angle_to_goal = 2 * math.pi - angle_to_goal

            if dist_to_goal < xy_threshold and angle_to_goal < theta_threshold:
                path = []
                key = current_key
                while key is not None:
                    state = state_map[key]
                    path.append(state)  # (x, y, theta)
                    key = came_from[key]
                path.reverse()
                path.append((gx, gy, g_theta))

                elapsed = time.time() - start_time
                print(f"[Hybrid A*] 成功! 迭代={iteration}, 时间={elapsed:.2f}s, 点数={len(path)}")
                return path

            current_g = g_cost[current_key]

            for steer in self.steer_angles:
                nx, ny, n_theta = self.vehicle_model(cx, cy, c_theta, steer)

                if not self.in_bounds(*self.world_to_grid(nx, ny)):
                    continue
                if not self.check_path_collision(cx, cy, nx, ny):
                    continue

                neighbor_key = state_key(nx, ny, n_theta)
                if neighbor_key in visited:
                    continue

                step_cost = self.motion_step + 0.2 * abs(steer)
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
                print(f"  迭代={iteration}, 距离={dist_to_goal:.1f}m, 时间={elapsed:.1f}s")

        print(f"[Hybrid A*] 达到最大迭代")
        return None

    def visualize(self, path, start, goal, title, zoom=True):
        """可视化 - 放大显示路径区域"""
        fig, ax = plt.subplots(1, 1, figsize=(14, 10))

        # 显示地图
        display_grid = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        display_grid[self.grid == 0] = [255, 255, 255]
        display_grid[self.grid == 100] = [50, 50, 50]
        display_grid[self.grid == -1] = [200, 200, 200]

        extent = [self.origin_x, self.origin_x + self.width * self.resolution,
                  self.origin_y, self.origin_y + self.height * self.resolution]
        ax.imshow(display_grid, origin='lower', extent=extent)

        # 画路径
        if path is not None and len(path) > 1:
            xs = [p[0] for p in path]
            ys = [p[1] for p in path]
            ax.plot(xs, ys, 'b-', linewidth=3, label=f'Path ({len(path)} points)', zorder=10)

            # 画朝向箭头
            for i in range(0, len(path), max(1, len(path)//10)):
                x, y, theta = path[i]
                dx = 1.5 * math.cos(theta)
                dy = 1.5 * math.sin(theta)
                ax.arrow(x, y, dx, dy, head_width=0.8, head_length=0.4,
                        fc='blue', ec='blue', zorder=11)

        # 画起点
        sx, sy, s_theta = start
        ax.plot(sx, sy, 'go', markersize=20, label='Start', zorder=12)
        dx = 3.0 * math.cos(s_theta)
        dy = 3.0 * math.sin(s_theta)
        ax.arrow(sx, sy, dx, dy, head_width=1.2, head_length=0.6,
                fc='green', ec='darkgreen', linewidth=3, zorder=13)

        # 画终点
        gx, gy, g_theta = goal
        ax.plot(gx, gy, 'ro', markersize=20, label='Goal', zorder=12)
        dx = 3.0 * math.cos(g_theta)
        dy = 3.0 * math.sin(g_theta)
        ax.arrow(gx, gy, dx, dy, head_width=1.2, head_length=0.6,
                fc='red', ec='darkred', linewidth=3, zorder=13)

        # 放大到路径区域
        if zoom and path is not None and len(path) > 1:
            all_x = [p[0] for p in path] + [sx, gx]
            all_y = [p[1] for p in path] + [sy, gy]
            margin = 15
            ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
            ax.set_ylim(min(all_y) - margin, max(all_y) + margin)
        elif zoom:
            margin = 20
            ax.set_xlim(min(sx, gx) - margin, max(sx, gx) + margin)
            ax.set_ylim(min(sy, gy) - margin, max(sy, gy) + margin)

        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        ax.set_title(title, fontsize=14)
        ax.legend(fontsize=12)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        save_path = f'/home/nvidia/vcii/wudi/Box_AD/Box_AD/planner/test_{title.replace(" ", "_")}.png'
        plt.savefig(save_path, dpi=150)
        print(f"已保存: {save_path}")
        plt.close()


def main():
    map_yaml = '/home/nvidia/vcii/wudi/Box_AD/maps/map.yaml'
    planner = HybridAstarTest(map_yaml)

    # 手动选择在地图白色区域（道路）上的点
    # 根据地图，道路主要在 x=-60 到 x=20, y=-140 到 y=80 范围内

    # 先扫描找到大片自由区域
    print("\n扫描地图中的自由区域...")
    free_regions = []
    for r in range(100, planner.height - 100, 20):
        for c in range(100, planner.width - 100, 20):
            if planner.is_free(r, c):
                # 检查周围是否也是自由的（找大片空地）
                free_count = 0
                for dr in range(-5, 6):
                    for dc in range(-5, 6):
                        if planner.is_free(r+dr, c+dc):
                            free_count += 1
                if free_count > 80:  # 大部分都是自由的
                    x, y = planner.grid_to_world(r, c)
                    free_regions.append((x, y, free_count))

    free_regions.sort(key=lambda p: p[2], reverse=True)
    print(f"找到 {len(free_regions)} 个大片自由区域")

    if len(free_regions) < 2:
        print("没有足够的自由区域!")
        return

    # 选择最大的几个自由区域进行测试
    for i, (x, y, count) in enumerate(free_regions[:5]):
        print(f"  区域{i+1}: ({x:.1f}, {y:.1f}), 自由度={count}")

    # 测试1: 短距离直行
    print("\n" + "="*60)
    print("测试1: 短距离直行 (约20m)")
    print("="*60)

    p1 = free_regions[0]
    start = (p1[0], p1[1], math.radians(90))  # 朝北
    goal = (p1[0], p1[1] + 20, math.radians(90))  # 向北20m

    # 确保终点也是自由的
    if not planner.is_world_free(goal[0], goal[1]):
        fp = planner.find_free_point_near(goal[0], goal[1])
        if fp:
            goal = (fp[0], fp[1], math.radians(90))

    print(f"起点: ({start[0]:.1f}, {start[1]:.1f}), 终点: ({goal[0]:.1f}, {goal[1]:.1f})")
    path1 = planner.hybrid_astar(start, goal)
    planner.visualize(path1, start, goal, "1_straight_20m")

    # 测试2: 90度左转
    print("\n" + "="*60)
    print("测试2: 90度左转")
    print("="*60)

    p2 = free_regions[1] if len(free_regions) > 1 else free_regions[0]
    start = (p2[0], p2[1], math.radians(0))   # 朝东
    goal = (p2[0] + 15, p2[1] + 15, math.radians(90))  # 朝北

    if not planner.is_world_free(goal[0], goal[1]):
        fp = planner.find_free_point_near(goal[0], goal[1])
        if fp:
            goal = (fp[0], fp[1], math.radians(90))

    print(f"起点: ({start[0]:.1f}, {start[1]:.1f}) 朝东")
    print(f"终点: ({goal[0]:.1f}, {goal[1]:.1f}) 朝北")
    path2 = planner.hybrid_astar(start, goal)
    planner.visualize(path2, start, goal, "2_turn_90deg")

    # 测试3: 大角度转弯 (135度)
    print("\n" + "="*60)
    print("测试3: 135度转弯")
    print("="*60)

    p3 = free_regions[2] if len(free_regions) > 2 else free_regions[0]
    start = (p3[0], p3[1], math.radians(0))   # 朝东
    goal = (p3[0] - 10, p3[1] + 20, math.radians(135))  # 朝西北

    if not planner.is_world_free(goal[0], goal[1]):
        fp = planner.find_free_point_near(goal[0], goal[1])
        if fp:
            goal = (fp[0], fp[1], math.radians(135))

    print(f"起点: ({start[0]:.1f}, {start[1]:.1f}) 朝东 (0°)")
    print(f"终点: ({goal[0]:.1f}, {goal[1]:.1f}) 朝西北 (135°)")
    path3 = planner.hybrid_astar(start, goal)
    planner.visualize(path3, start, goal, "3_turn_135deg")

    # 测试4: 掉头 (180度) - 不允许倒车，需要画U型弯
    print("\n" + "="*60)
    print("测试4: 掉头 180度 (禁止倒车，需要U型弯)")
    print("="*60)

    # 找一个有足够空间掉头的区域
    p4 = free_regions[0]  # 使用最大的自由区域
    start = (p4[0], p4[1], math.radians(0))   # 朝东
    goal = (p4[0] + 5, p4[1], math.radians(180))  # 朝西，向前5m后掉头

    # 确保终点自由
    if not planner.is_world_free(goal[0], goal[1]):
        fp = planner.find_free_point_near(goal[0], goal[1])
        if fp:
            goal = (fp[0], fp[1], math.radians(180))

    print(f"起点: ({start[0]:.1f}, {start[1]:.1f}) 朝东 (0°)")
    print(f"终点: ({goal[0]:.1f}, {goal[1]:.1f}) 朝西 (180°)")
    print("注意: 禁止倒车，车辆需要画U型弯掉头")
    path4 = planner.hybrid_astar(start, goal)
    planner.visualize(path4, start, goal, "4_uturn_180deg")


if __name__ == '__main__':
    main()
