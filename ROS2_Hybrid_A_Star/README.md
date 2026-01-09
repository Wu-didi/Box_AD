# ROS2 Hybrid A* 路径规划

ROS2 版本的 Hybrid A* 路径规划算法，适用于车辆路径规划场景。

## 特性

- ✅ 考虑车辆运动学约束（阿克曼转向模型）
- ✅ Reeds-Shepp曲线用于解析扩展
- ✅ 精确的碰撞检测（考虑车辆矩形轮廓）
- ✅ 支持前进和后退
- ✅ ROS2 Humble/Foxy 兼容

## 依赖

- ROS2 (Humble/Foxy)
- Eigen3
- glog

```bash
sudo apt-get install ros-${ROS_DISTRO}-nav-msgs ros-${ROS_DISTRO}-geometry-msgs ros-${ROS_DISTRO}-visualization-msgs
sudo apt-get install libeigen3-dev libgoogle-glog-dev
```

## 编译

```bash
# 进入工作空间
cd ~/ros2_ws/src

# 复制或链接 ROS2_Hybrid_A_Star 包到工作空间
# cp -r /path/to/ROS2_Hybrid_A_Star .

# 编译
cd ~/ros2_ws
colcon build --packages-select ros2_hybrid_a_star

# source
source install/setup.bash
```

## 使用方法

### 1. 启动节点

```bash
ros2 launch ros2_hybrid_a_star hybrid_a_star.launch.py
```

### 2. 发布地图

需要发布 `/map` 话题 (nav_msgs/OccupancyGrid)：

```bash
# 示例：使用 map_server 发布地图
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/path/to/map.yaml
```

### 3. 设置起点和终点

在 RViz2 中：
- 使用 "2D Pose Estimate" 设置起点
- 使用 "2D Goal Pose" 设置终点

或通过命令行：

```bash
# 发布起点 (initialpose)
ros2 topic pub /initialpose geometry_msgs/PoseWithCovarianceStamped ...

# 发布终点 (goal_pose)
ros2 topic pub /goal_pose geometry_msgs/PoseStamped ...
```

### 4. 查看规划结果

规划的路径会发布到 `/global_path` 话题，在 RViz2 中添加 Path 显示。

## 话题接口

### 订阅的话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/map` | nav_msgs/OccupancyGrid | 栅格地图 |
| `/initialpose` | geometry_msgs/PoseWithCovarianceStamped | 起点位姿 |
| `/goal_pose` | geometry_msgs/PoseStamped | 终点位姿 |

### 发布的话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/global_path` | nav_msgs/Path | 规划的路径 |
| `/search_tree` | visualization_msgs/MarkerArray | 搜索树可视化 |

## 参数配置

在 launch 文件中可以配置以下参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `vehicle_length` | 4.7 | 车辆长度 (米) |
| `vehicle_width` | 2.0 | 车辆宽度 (米) |
| `rear_axle_dist` | 1.3 | 后轴到车尾距离 (米) |
| `wheelbase` | 2.85 | 轴距 (米) |
| `max_steer_angle` | 60.0 | 最大转向角 (度) |
| `segment_length` | 1.6 | 扩展步长 (米) |
| `steering_penalty` | 1.05 | 转向惩罚系数 |
| `reversing_penalty` | 2.0 | 倒车惩罚系数 |
| `shot_distance` | 10.0 | 解析扩展距离 (米) |

## RViz2 配置

在 RViz2 中添加以下显示：

1. **Map** - 显示 `/map` 话题
2. **Path** - 显示 `/global_path` 话题
3. **MarkerArray** - 显示 `/search_tree` 话题（可选）
4. **Pose** - 显示当前起点/终点

## 与 Box_AD 集成

将此包集成到 Box_AD 项目中：

```bash
# 在 box_ad.launch.py 中添加节点
hybrid_a_star_node = Node(
    package='ros2_hybrid_a_star',
    executable='hybrid_a_star_node',
    name='hybrid_a_star_planner',
    output='screen',
    parameters=[{
        'vehicle_length': 4.53,
        'vehicle_width': 1.9,
        'wheelbase': 2.85,
        # ... 其他参数
    }]
)
```

## 算法说明

Hybrid A* 是一种基于状态栅格的路径规划算法，结合了：

1. **A* 搜索** - 在离散状态空间中搜索
2. **车辆运动学** - 使用简化的阿克曼转向模型
3. **Reeds-Shepp 曲线** - 用于解析扩展，加速收敛
4. **碰撞检测** - 考虑车辆矩形轮廓

状态空间：(x, y, θ)
- x, y: 位置
- θ: 朝向角

## 性能优化建议

1. 调大 `segment_length` 可加速搜索，但路径更粗糙
2. 减小 `state_grid_resolution` 可提高路径精度，但搜索变慢
3. 调整 `shot_distance` 控制何时尝试 Reeds-Shepp 连接

## 常见问题

### Q: 编译报错找不到 Eigen？
A: 安装 Eigen3 开发库：`sudo apt-get install libeigen3-dev`

### Q: 找不到路径？
A: 检查起点/终点是否在障碍物内，或被障碍物隔断

### Q: 规划很慢？
A: 调大 `segment_length` 或 `state_grid_resolution`

## 参考文献

1. Dolgov, D., et al. "Practical Search Techniques in Path Planning for Autonomous Driving." (2008)
2. Reeds, J. A., & Shepp, L. A. "Optimal paths for a car that goes both forwards and backwards." (1990)

## 致谢

- 原始 ROS1 实现：[zm0612/Hybrid_A_Star](https://github.com/zm0612/Hybrid_A_Star)
- ROS2 移植：Box_AD 项目

## License

BSD License
