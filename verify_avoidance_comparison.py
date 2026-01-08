#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
避障算法对比验证

对比两种避障算法：
1. A* 局部规划（考虑静态地图+动态障碍物）
2. 梯形绕行方案（考虑静态地图+动态障碍物）

两种算法都使用相同的：
- 起点、终点（从全局路径取）
- 静态地图（膨胀后）
- 动态障碍物
"""

import math
import heapq
import csv
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import cv2
import sys
from typing import List, Tuple, Optional

sys.path.insert(0, '/home/wudi/slam/Box_AD')


# ==================== 地图加载 ====================

def load_map(map_path, yaml_path=None):
    """加载地图"""
    img = Image.open(map_path)
    map_array = np.array(img)

    if len(map_array.shape) == 3:
        map_array = np.mean(map_array, axis=2).astype(np.uint8)

    resolution = 0.1
    origin_x = -72.603497
    origin_y = -144.173262
    png_origin_e = 671775.357
    png_origin_n = 3529827.330

    if yaml_path:
        try:
            import re
            with open(yaml_path, 'r') as f:
                for line in f:
                    line = line.strip()
                    if line.startswith('resolution:'):
                        resolution = float(line.split(':')[1].strip())
                    elif line.startswith('origin:'):
                        parts = line.split('[')[1].split(']')[0].split(',')
                        origin_x = float(parts[0])
                        origin_y = float(parts[1])
                    elif 'PNG origin UTM' in line:
                        m = re.search(r"E\s*=\s*([0-9.+\-eE]+).*N\s*=\s*([0-9.+\-eE]+)", line)
                        if m:
                            png_origin_e = float(m.group(1))
                            png_origin_n = float(m.group(2))
        except Exception as e:
            print(f"解析 yaml 失败: {e}")

    height, width = map_array.shape
    grid = np.zeros((height, width), dtype=np.int8)
    obstacle_pixel_thresh = int(255 * 0.35)
    grid[map_array < obstacle_pixel_thresh] = 100

    return grid, resolution, origin_x, origin_y, png_origin_e, png_origin_n, map_array


def inflate_obstacles(grid: np.ndarray, radius_cells: int) -> np.ndarray:
    obst = (grid == 100).astype(np.uint8)
    kernel_size = 2 * radius_cells + 1
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    inflated = cv2.dilate(obst, kernel)
    result = grid.copy()
    result[inflated == 1] = 100
    return result


def add_dynamic_obstacles_to_grid(grid, obstacles, resolution, origin_x, origin_y, height, robot_radius=2.0):
    """将动态障碍物添加到栅格地图"""
    result = grid.copy()
    for ox, oy, obs_radius in obstacles:
        col = int((ox - origin_x) / resolution)
        row = height - 1 - int((oy - origin_y) / resolution)
        inflation_radius = obs_radius + robot_radius
        inflation_cells = int(inflation_radius / resolution)
        for dr in range(-inflation_cells, inflation_cells + 1):
            for dc in range(-inflation_cells, inflation_cells + 1):
                if dr * dr + dc * dc <= inflation_cells * inflation_cells:
                    nr, nc = row + dr, col + dc
                    if 0 <= nr < result.shape[0] and 0 <= nc < result.shape[1]:
                        result[nr, nc] = 100
    return result


# ==================== 坐标转换 ====================

def utm_to_map(E, N, png_origin_e, png_origin_n, origin_x, origin_y):
    x = (E - png_origin_e) + origin_x
    y = (N - png_origin_n) + origin_y
    return x, y


def world_to_grid(x, y, resolution, origin_x, origin_y, height):
    col = int((x - origin_x) / resolution)
    row = height - 1 - int((y - origin_y) / resolution)
    return row, col


def grid_to_world(row, col, resolution, origin_x, origin_y, height):
    x = origin_x + (col + 0.5) * resolution
    y = origin_y + (height - 1 - row + 0.5) * resolution
    return x, y


# ==================== A* 规划 ====================

def astar(start_rc, goal_rc, grid):
    H, W = grid.shape
    sr, sc = start_rc
    gr, gc = goal_rc

    def in_bounds(r, c):
        return 0 <= r < H and 0 <= c < W

    def is_free(r, c):
        return in_bounds(r, c) and int(grid[r, c]) == 0

    def h(r, c):
        return math.hypot(r - gr, c - gc)

    moves = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]

    if not is_free(sr, sc):
        sr, sc = find_nearest_free(sr, sc, grid)
        if sr is None:
            return None

    if not is_free(gr, gc):
        gr, gc = find_nearest_free(gr, gc, grid)
        if gr is None:
            return None

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
            path = [(r, c)]
            while (r, c) in came_from:
                r, c = came_from[(r, c)]
                path.append((r, c))
            path.reverse()
            return path

        for dr, dc in moves:
            nr, nc = r + dr, c + dc
            if not in_bounds(nr, nc) or visited[nr, nc] or not is_free(nr, nc):
                continue
            step_cost = math.hypot(dr, dc)
            new_g = g_score[(r, c)] + step_cost
            if (nr, nc) not in g_score or new_g < g_score[(nr, nc)]:
                g_score[(nr, nc)] = new_g
                heapq.heappush(open_set, (new_g + h(nr, nc), (nr, nc)))
                came_from[(nr, nc)] = (r, c)

    return None


def find_nearest_free(r, c, grid, max_search=50):
    H, W = grid.shape
    for radius in range(1, max_search):
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                if abs(dr) == radius or abs(dc) == radius:
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < H and 0 <= nc < W and grid[nr, nc] == 0:
                        return nr, nc
    return None, None


def simplify_path(path_rc, grid):
    if not path_rc or len(path_rc) <= 2:
        return path_rc

    def line_is_free(r0, c0, r1, c1):
        steps = max(abs(r1 - r0), abs(c1 - c0))
        if steps == 0:
            return grid[r0, c0] == 0
        for i in range(steps + 1):
            t = i / steps
            r = int(r0 + t * (r1 - r0))
            c = int(c0 + t * (c1 - c0))
            if grid[r, c] != 0:
                return False
        return True

    simplified = [path_rc[0]]
    idx = 0
    while idx < len(path_rc) - 1:
        next_idx = idx + 1
        for j in range(idx + 1, len(path_rc)):
            if line_is_free(path_rc[idx][0], path_rc[idx][1], path_rc[j][0], path_rc[j][1]):
                next_idx = j
            else:
                break
        simplified.append(path_rc[next_idx])
        idx = next_idx
    return simplified


# ==================== 梯形绕行（考虑静态地图） ====================

def plan_trapezoid_path_with_map(
    start, goal, obstacles,
    grid, resolution, origin_x, origin_y, height,
    step=0.5, vehicle_width=1.0, safe_margin=1.0
):
    """
    梯形绕行规划（考虑静态地图）

    尝试两个方向（上/下），选择不碰撞的方向
    如果都碰撞，选择碰撞少的方向
    """
    if not obstacles:
        # 无障碍物，直线连接
        dist = math.hypot(goal[0] - start[0], goal[1] - start[1])
        num = max(int(dist / step), 2)
        return [(start[0] + i/num * (goal[0]-start[0]),
                 start[1] + i/num * (goal[1]-start[1])) for i in range(num+1)]

    # 计算两个方向需要的偏移量
    max_offset_up = 0.0
    max_offset_down = 0.0

    for obs_x, obs_y, obs_radius in obstacles:
        safe_dist = obs_radius + vehicle_width + safe_margin
        diff = obs_y - start[1]

        if diff >= 0:
            offset_up = diff + safe_dist
            offset_down = safe_dist - diff if diff < safe_dist else safe_dist
        else:
            offset_down = abs(diff) + safe_dist
            offset_up = safe_dist - abs(diff) if abs(diff) < safe_dist else safe_dist

        max_offset_up = max(max_offset_up, offset_up)
        max_offset_down = max(max_offset_down, offset_down)

    # 确保最小偏移量
    min_offset = safe_margin + vehicle_width + 0.5
    max_offset_up = max(max_offset_up, min_offset)
    max_offset_down = max(max_offset_down, min_offset)

    # 计算绕行区间
    min_obs_x = min(obs[0] - obs[2] for obs in obstacles)
    max_obs_x = max(obs[0] + obs[2] for obs in obstacles)

    offset_end_x = min_obs_x - 3.0
    parallel_end_x = max_obs_x + 3.0
    offset_end_x = max(start[0] + 1.0, min(offset_end_x, goal[0] - 2.0))
    parallel_end_x = max(offset_end_x + 1.0, min(parallel_end_x, goal[0] - 1.0))

    def generate_trapezoid_path(lateral_offset):
        """生成梯形路径"""
        bypass_y = start[1] + lateral_offset
        path = []

        # 1. 偏移段
        dist1 = math.hypot(offset_end_x - start[0], bypass_y - start[1])
        num1 = max(int(dist1 / step), 2)
        for i in range(num1 + 1):
            t = i / num1
            path.append((start[0] + t * (offset_end_x - start[0]),
                         start[1] + t * (bypass_y - start[1])))

        # 2. 平行段
        dist2 = parallel_end_x - offset_end_x
        num2 = max(int(dist2 / step), 2)
        for i in range(1, num2 + 1):
            t = i / num2
            path.append((offset_end_x + t * (parallel_end_x - offset_end_x), bypass_y))

        # 3. 回归段
        dist3 = math.hypot(goal[0] - parallel_end_x, goal[1] - bypass_y)
        num3 = max(int(dist3 / step), 2)
        for i in range(1, num3 + 1):
            t = i / num3
            path.append((parallel_end_x + t * (goal[0] - parallel_end_x),
                         bypass_y + t * (goal[1] - bypass_y)))

        return path

    def count_collision(path):
        """计算路径与栅格地图的碰撞点数"""
        collision_count = 0
        for x, y in path:
            row, col = world_to_grid(x, y, resolution, origin_x, origin_y, height)
            if 0 <= row < grid.shape[0] and 0 <= col < grid.shape[1]:
                if grid[row, col] != 0:
                    collision_count += 1
        return collision_count

    # 尝试向上绕行（+Y）
    path_up = generate_trapezoid_path(max_offset_up)
    collision_up = count_collision(path_up)

    # 尝试向下绕行（-Y）
    path_down = generate_trapezoid_path(-max_offset_down)
    collision_down = count_collision(path_down)

    # 选择碰撞少的方向，如果相同则选择偏移量小的
    if collision_up == 0 and collision_down == 0:
        # 都不碰撞，选偏移小的
        return path_up if max_offset_up <= max_offset_down else path_down
    elif collision_up == 0:
        return path_up
    elif collision_down == 0:
        return path_down
    else:
        # 都碰撞，选碰撞少的
        return path_up if collision_up <= collision_down else path_down


# ==================== 辅助函数 ====================

def check_path_collision(path, grid, resolution, origin_x, origin_y, height):
    collision_count = 0
    for x, y in path:
        row, col = world_to_grid(x, y, resolution, origin_x, origin_y, height)
        if 0 <= row < grid.shape[0] and 0 <= col < grid.shape[1]:
            if grid[row, col] != 0:
                collision_count += 1
    return collision_count > 0, collision_count


def path_length(path):
    if len(path) < 2:
        return 0
    return sum(math.hypot(path[i+1][0]-path[i][0], path[i+1][1]-path[i][1])
              for i in range(len(path)-1))


def min_clearance(path, obstacles):
    """计算路径到动态障碍物的最小距离"""
    if not path or not obstacles:
        return float('inf')
    min_c = float('inf')
    for px, py in path:
        for ox, oy, r in obstacles:
            c = math.hypot(px - ox, py - oy) - r
            min_c = min(min_c, c)
    return min_c


# ==================== 主函数 ====================

def main():
    print("=" * 70)
    print("避障算法对比验证: A* vs 梯形绕行")
    print("（两种算法都考虑静态地图）")
    print("=" * 70)

    # 1. 加载全局轨迹
    traj_path = '/home/wudi/slam/get_pc_from_db3/processed_shiyanzhongxin_0327_with_yaw_ck.csv'
    global_path_utm = []

    with open(traj_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            global_path_utm.append((float(row['cx']), float(row['cy'])))

    print(f"全局轨迹点数: {len(global_path_utm)}")

    # 2. 加载地图
    map_path = '/home/wudi/slam/Box_AD/maps/map_2d.png'
    yaml_path = '/home/wudi/slam/Box_AD/maps/map.yaml'

    grid, resolution, origin_x, origin_y, png_origin_e, png_origin_n, map_img = load_map(map_path, yaml_path)
    height, width = grid.shape
    print(f"地图大小: {width}x{height}, 分辨率: {resolution}m")

    robot_radius = 2.0
    inflation_cells = int(robot_radius / resolution)
    static_grid = inflate_obstacles(grid, inflation_cells)

    # 3. 转换全局路径到地图坐标
    global_path_map = []
    for e, n in global_path_utm:
        x, y = utm_to_map(e, n, png_origin_e, png_origin_n, origin_x, origin_y)
        global_path_map.append((x, y))

    # 4. 定义多种测试场景
    test_scenarios = [
        {
            'name': '1. Single obstacle',
            'desc': 'One obstacle on path',
            'obstacle_indices': [130],
            'obstacle_offsets': [(0, 0)],
            'obstacle_radii': [1.5],
        },
        {
            'name': '2. Two obstacles',
            'desc': 'Two spaced obstacles',
            'obstacle_indices': [120, 145],
            'obstacle_offsets': [(0, 0), (0, 0)],
            'obstacle_radii': [1.2, 1.2],
        },
        {
            'name': '3. Consecutive',
            'desc': 'Close obstacles in row',
            'obstacle_indices': [125, 130, 135],
            'obstacle_offsets': [(0, 1), (0, 0), (0, -1)],
            'obstacle_radii': [1.0, 1.0, 1.0],
        },
        {
            'name': '4. Large obstacle',
            'desc': 'One large obstacle',
            'obstacle_indices': [140],
            'obstacle_offsets': [(0, 0)],
            'obstacle_radii': [2.5],
        },
        {
            'name': '5. Both sides',
            'desc': 'Obstacles on both sides',
            'obstacle_indices': [125, 140],
            'obstacle_offsets': [(0, 3), (0, -3)],
            'obstacle_radii': [1.5, 1.5],
        },
        {
            'name': '6. Dense cluster',
            'desc': 'Multiple close obstacles',
            'obstacle_indices': [128, 132, 136, 140],
            'obstacle_offsets': [(0, 0.5), (0, -0.5), (0, 0.5), (0, -0.5)],
            'obstacle_radii': [0.8, 0.8, 0.8, 0.8],
        },
    ]

    # 5. 创建可视化
    n_scenarios = len(test_scenarios)
    fig, axes = plt.subplots(n_scenarios, 2, figsize=(16, 4 * n_scenarios))

    results = []

    for idx, scenario in enumerate(test_scenarios):
        print(f"\n{scenario['name']}: {scenario['desc']}")

        # 创建动态障碍物
        dynamic_obstacles = []
        for i, obs_idx in enumerate(scenario['obstacle_indices']):
            if obs_idx < len(global_path_map):
                ox, oy = global_path_map[obs_idx]
                ox += scenario['obstacle_offsets'][i][0]
                oy += scenario['obstacle_offsets'][i][1]
                dynamic_obstacles.append((ox, oy, scenario['obstacle_radii'][i]))

        # 确定起点终点（从全局路径取）
        min_idx = min(scenario['obstacle_indices']) - 15
        max_idx = max(scenario['obstacle_indices']) + 15
        min_idx = max(0, min_idx)
        max_idx = min(len(global_path_map) - 1, max_idx)

        start = global_path_map[min_idx]
        goal = global_path_map[max_idx]
        global_segment = global_path_map[min_idx:max_idx+1]

        # 创建包含静态+动态障碍物的地图
        combined_grid = add_dynamic_obstacles_to_grid(
            static_grid, dynamic_obstacles, resolution, origin_x, origin_y, height, robot_radius
        )

        # === A* 规划 ===
        start_rc = world_to_grid(start[0], start[1], resolution, origin_x, origin_y, height)
        goal_rc = world_to_grid(goal[0], goal[1], resolution, origin_x, origin_y, height)

        astar_path_rc = astar(start_rc, goal_rc, combined_grid)
        if astar_path_rc:
            astar_path_rc = simplify_path(astar_path_rc, combined_grid)
            astar_path = [grid_to_world(r, c, resolution, origin_x, origin_y, height)
                         for r, c in astar_path_rc]
            astar_collision, astar_collision_count = check_path_collision(
                astar_path, combined_grid, resolution, origin_x, origin_y, height
            )
            astar_len = path_length(astar_path)
            astar_clearance = min_clearance(astar_path, dynamic_obstacles)
            astar_status = "OK" if not astar_collision else f"COLLISION({astar_collision_count})"
        else:
            astar_path = None
            astar_status = "FAILED"
            astar_len = 0
            astar_clearance = 0

        # === 梯形绕行（考虑静态地图） ===
        trapezoid_path = plan_trapezoid_path_with_map(
            start, goal, dynamic_obstacles,
            combined_grid, resolution, origin_x, origin_y, height
        )
        trap_collision, trap_collision_count = check_path_collision(
            trapezoid_path, combined_grid, resolution, origin_x, origin_y, height
        )
        trap_len = path_length(trapezoid_path)
        trap_clearance = min_clearance(trapezoid_path, dynamic_obstacles)
        trap_status = "OK" if not trap_collision else f"COLLISION({trap_collision_count})"

        global_len = path_length(global_segment)

        # 记录结果
        results.append({
            'name': scenario['name'],
            'astar_status': astar_status,
            'astar_len': astar_len,
            'astar_clearance': astar_clearance,
            'trap_status': trap_status,
            'trap_len': trap_len,
            'trap_clearance': trap_clearance,
            'global_len': global_len,
        })

        print(f"  Global: {global_len:.1f}m")
        print(f"  A*: {astar_status}, len={astar_len:.1f}m, clearance={astar_clearance:.2f}m")
        print(f"  Trapezoid: {trap_status}, len={trap_len:.1f}m, clearance={trap_clearance:.2f}m")

        # === 绘图 ===
        all_x = [p[0] for p in global_segment] + [o[0] for o in dynamic_obstacles]
        all_y = [p[1] for p in global_segment] + [o[1] for o in dynamic_obstacles]
        margin = 10
        x_min, x_max = min(all_x) - margin, max(all_x) + margin
        y_min, y_max = min(all_y) - margin, max(all_y) + margin

        # 左图：A*
        ax1 = axes[idx, 0]
        ax1.set_title(f"{scenario['name']}\nA*: [{astar_status}] len={astar_len:.1f}m",
                     fontsize=10, fontweight='bold')

        ax1.imshow(map_img, cmap='gray', extent=[
            origin_x, origin_x + width * resolution,
            origin_y, origin_y + height * resolution
        ])
        ax1.set_xlim(x_min, x_max)
        ax1.set_ylim(y_min, y_max)

        # 全局路径
        gx = [p[0] for p in global_segment]
        gy = [p[1] for p in global_segment]
        ax1.plot(gx, gy, 'b--', linewidth=2, label='Global', alpha=0.6)

        # A* 路径
        if astar_path:
            ax_x = [p[0] for p in astar_path]
            ax_y = [p[1] for p in astar_path]
            color = 'green' if astar_status == "OK" else 'orange'
            ax1.plot(ax_x, ax_y, color=color, linewidth=3, label='A*')

        # 障碍物
        for ox, oy, r in dynamic_obstacles:
            circle = plt.Circle((ox, oy), r, color='red', alpha=0.7)
            ax1.add_patch(circle)
            safe_circle = plt.Circle((ox, oy), r + robot_radius, color='orange',
                                      fill=False, linestyle='--', linewidth=1.5)
            ax1.add_patch(safe_circle)

        ax1.plot(start[0], start[1], 'go', markersize=8, label='Start')
        ax1.plot(goal[0], goal[1], 'r*', markersize=10, label='Goal')
        ax1.legend(loc='upper right', fontsize=8)
        ax1.set_aspect('equal')
        ax1.grid(True, alpha=0.3)

        # 右图：梯形绕行
        ax2 = axes[idx, 1]
        ax2.set_title(f"Trapezoid: [{trap_status}] len={trap_len:.1f}m",
                     fontsize=10, fontweight='bold')

        ax2.imshow(map_img, cmap='gray', extent=[
            origin_x, origin_x + width * resolution,
            origin_y, origin_y + height * resolution
        ])
        ax2.set_xlim(x_min, x_max)
        ax2.set_ylim(y_min, y_max)

        # 全局路径
        ax2.plot(gx, gy, 'b--', linewidth=2, label='Global', alpha=0.6)

        # 梯形路径
        tx = [p[0] for p in trapezoid_path]
        ty = [p[1] for p in trapezoid_path]
        color = 'green' if trap_status == "OK" else 'orange'
        ax2.plot(tx, ty, color=color, linewidth=3, label='Trapezoid')

        # 障碍物
        for ox, oy, r in dynamic_obstacles:
            circle = plt.Circle((ox, oy), r, color='red', alpha=0.7)
            ax2.add_patch(circle)
            safe_circle = plt.Circle((ox, oy), r + robot_radius, color='orange',
                                      fill=False, linestyle='--', linewidth=1.5)
            ax2.add_patch(safe_circle)

        ax2.plot(start[0], start[1], 'go', markersize=8, label='Start')
        ax2.plot(goal[0], goal[1], 'r*', markersize=10, label='Goal')
        ax2.legend(loc='upper right', fontsize=8)
        ax2.set_aspect('equal')
        ax2.grid(True, alpha=0.3)

    # 打印汇总结果
    print("\n" + "=" * 70)
    print("Summary Results:")
    print("=" * 70)
    print(f"{'Scenario':<20} {'A* Status':<15} {'A* Len':<8} {'Trap Status':<15} {'Trap Len':<8}")
    print("-" * 70)
    for r in results:
        print(f"{r['name']:<20} {r['astar_status']:<15} {r['astar_len']:<8.1f} {r['trap_status']:<15} {r['trap_len']:<8.1f}")

    # 统计
    astar_ok = sum(1 for r in results if r['astar_status'] == 'OK')
    trap_ok = sum(1 for r in results if r['trap_status'] == 'OK')
    print("-" * 70)
    print(f"Success rate: A* = {astar_ok}/{len(results)}, Trapezoid = {trap_ok}/{len(results)}")

    plt.suptitle('Obstacle Avoidance: A* vs Trapezoid (Both consider static map)',
                fontsize=14, fontweight='bold')
    plt.tight_layout()

    output_path = '/home/wudi/slam/Box_AD/avoidance_comparison.png'
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"\n结果保存到: {output_path}")

    plt.close()


if __name__ == '__main__':
    main()
