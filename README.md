# Box_AD - 模块化ROS2自动驾驶系统

Box_AD是一个基于ROS2 Humble的模块化自动驾驶系统，提供从感知到控制的完整自动驾驶栈。

## 目录

- [系统架构](#系统架构)
- [环境要求](#环境要求)
- [快速开始](#快速开始)
- [节点详细说明](#节点详细说明)
  - [感知模块](#感知模块)
  - [定位模块](#定位模块)
  - [规划模块](#规划模块)
  - [轨迹转换模块](#轨迹转换模块)
  - [循迹控制模块](#循迹控制模块)
  - [可视化模块](#可视化模块)
  - [地图与环境模块](#地图与环境模块)
- [Launch文件启动](#launch文件启动)
- [完整启动示例](#完整启动示例)
- [常见问题](#常见问题)

## 系统架构

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          可视化层                                        │
│      ego_visualizer, path_visualizer, fast_obstacle_rviz_in_map         │
└─────────────────────────────────────────────────────────────────────────┘
                                    ▲
┌─────────────────────────────────────────────────────────────────────────┐
│                          控制层 (二选一)                                 │
│  ┌─────────────────────────┐    ┌─────────────────────────┐             │
│  │  MPC循迹 (mpc_node)     │    │  简易循迹 (follow_node)  │             │
│  │  - 高精度轨迹跟踪       │    │  - 简单易用              │             │
│  │  - 需要曲率信息         │    │  - 经纬度输入            │             │
│  └─────────────────────────┘    └─────────────────────────┘             │
└─────────────────────────────────────────────────────────────────────────┘
                                    ▲
┌─────────────────────────────────────────────────────────────────────────┐
│                       轨迹转换层 (根据控制器选择)                         │
│  ┌─────────────────────────┐    ┌─────────────────────────┐             │
│  │  local_path_to_mpc      │    │  local_path_to_follow   │             │
│  │  - UTM坐标 + 曲率计算   │    │  - UTM → 经纬度转换     │             │
│  │  - /mpc_trajectory      │    │  - /trajectory          │             │
│  └─────────────────────────┘    └─────────────────────────┘             │
└─────────────────────────────────────────────────────────────────────────┘
                                    ▲
┌─────────────────────────────────────────────────────────────────────────┐
│                           规划层                                         │
│  全局: A* (astar_planner_node) → /global_path_utm                       │
│  局部: DWA (local_path_planner_node) → /local_path (UTM, 避障后轨迹)    │
└─────────────────────────────────────────────────────────────────────────┘
                                    ▲
┌──────────────────┬──────────────────┬────────────────────────────────────┐
│     感知         │      定位        │          地图/环境                  │
│  激光雷达检测    │   GPS/IMU融合    │        PNG地图发布                  │
│  障碍物聚类      │   UTM坐标转换    │        点云处理                     │
└──────────────────┴──────────────────┴────────────────────────────────────┘
                                    ▲
┌─────────────────────────────────────────────────────────────────────────┐
│                         传感器驱动                                       │
│            CAN总线 (imu_node), 激光雷达, GPS                            │
└─────────────────────────────────────────────────────────────────────────┘
```

### 数据流图

```
                          /global_path_utm
astar_planner_node ─────────────────────────────> local_path_planner_node
                                                          │
                                                          │ /local_path (UTM)
                                                          ▼
                                    ┌─────────────────────┴─────────────────────┐
                                    │                                           │
                                    ▼                                           ▼
                          local_path_to_mpc                          local_path_to_follow
                                    │                                           │
                                    │ /mpc_trajectory                           │ /trajectory
                                    │ (PoseArray, UTM+曲率)                     │ (PoseArray, 经纬度)
                                    ▼                                           ▼
                              mpc_node_v2                                  follow_node
                                    │                                           │
                                    └───────────────────┬───────────────────────┘
                                                        │
                                                        ▼
                                                   CAN总线控制
```

## 环境要求

### 系统要求

- Ubuntu 22.04
- ROS2 Humble
- Python 3.8+

### 依赖安装

```bash
# 安装Python依赖
pip3 install numpy opencv-python open3d pyproj python-can

# 安装ROS2依赖 (如需要)
sudo apt install ros-humble-slam-toolbox ros-humble-nav2-map-server
```

### CAN总线配置 (IMU/GPS节点需要)

```bash
# 配置CAN接口
sudo ip link set can0 up type can bitrate 500000
ip link show can0  # 验证状态
```

## 快速开始

### 1. 编译包

```bash
cd ~/slam/Box_AD
colcon build --packages-select Box_AD
source install/setup.bash
```

### 2. 使用Launch文件一键启动

```bash
# 启动完整系统 (推荐)
ros2 launch Box_AD box_ad.launch.py

# 自定义参数启动
ros2 launch Box_AD box_ad.launch.py \
  map_image_path:=/path/to/map.png \
  map_resolution:=0.1 \
  max_speed:=2.0
```

### 3. 最小启动示例 (手动)

```bash
# 终端1: 启动地图发布器
ros2 run Box_AD png_map_publisher

# 终端2: 启动定位 (需要GPS/IMU数据)
ros2 run Box_AD gps_imu_to_utm_pose

# 终端3: 启动全局规划
ros2 run Box_AD astar_planner_node

# 终端4: RViz可视化
rviz2
```

## 节点详细说明

### 感知模块

#### 1. `fast_lidar_obstacle_detector`

**功能**: 实时轻量级障碍物检测，基于网格的占据检测

**输入**:

- `/lidar_point_prev` (sensor_msgs/PointCloud2) - 原始激光雷达点云

**输出**:

- `/fast_obstacles` (std_msgs/String) - JSON格式障碍物数据

**参数**:

```bash
--ros-args \
  -p xmin:=0.0 \        # ROI最小X (米)
  -p xmax:=40.0 \       # ROI最大X (米)
  -p ymin:=-15.0 \      # ROI最小Y (米)
  -p ymax:=15.0 \       # ROI最大Y (米)
  -p grid_resolution:=0.5 \  # 网格分辨率 (米)
  -p min_obstacle_height:=0.6 \  # 最小障碍物高度 (米)
  -p sample_stride:=1   # 点云抽样步长
```

**启动命令**:

```bash
ros2 run Box_AD fast_lidar_obstacle_detector
```

**输出JSON格式**:

```json
{
  "stamp": {"sec": 123, "nanosec": 456},
  "frame_id": "base_link",
  "has_obstacle": true,
  "obstacles": [
    {"x": 5.2, "y": -1.3, "z": 0.5, "count": 15}
  ]
}
```

---

#### 2. `fast_obstacle_temporal_filter_node`

**功能**: 多帧时序滤波，减少噪声和误检

**输入**:

- `/fast_obstacles` (std_msgs/String) - 瞬时检测结果

**输出**:

- `/fast_obstacles_stable` (std_msgs/String) - 滤波后稳定障碍物

**启动命令**:

```bash
ros2 run Box_AD fast_obstacle_temporal_filter_node
```

---

#### 3. `fast_obstacle_rviz_in_map`

**功能**: 将base_link坐标系下的障碍物转换到map坐标系进行可视化

**输入**:

- `/pose_utm` (geometry_msgs/PoseStamped) - 自车位置
- `/fast_obstacles` (std_msgs/String) - 障碍物检测结果

**输出**:

- `/fast_obstacle_markers` (visualization_msgs/Marker) - map坐标系下的点标记

**启动命令**:

```bash
ros2 run Box_AD fast_obstacle_rviz_in_map \
  --ros-args -p map_yaml_path:=/path/to/map.yaml
```

---

### 定位模块

#### 4. `publish_imu`

**功能**: CAN总线驱动，采集IMU和GPS传感器数据

**输出**:

- `/IMU` (sensor_msgs/Imu) - 50Hz IMU数据
- `/gps/fix` (sensor_msgs/NavSatFix) - GPS位置

**启动命令**:

```bash
sudo ip link set can0 up type can bitrate 500000
ros2 run Box_AD publish_imu
```

---

#### 5. `gps_imu_to_utm_pose`

**功能**: 融合GPS和IMU数据，输出UTM坐标系下的姿态

**输入**:

- `/IMU` (sensor_msgs/Imu) - IMU姿态
- `/gps/fix` (sensor_msgs/NavSatFix) - GPS位置

**输出**:

- `/pose_utm` (geometry_msgs/PoseStamped) - UTM坐标系下的姿态

**启动命令**:

```bash
ros2 run Box_AD gps_imu_to_utm_pose
```

---

#### 6. `imu_gps_logger`

**功能**: 将IMU和GPS数据记录到CSV文件

**输出**:

- `logs/imu.csv`: 时间戳, roll, pitch, yaw, 角速度xyz, 加速度xyz
- `logs/gps.csv`: 时间戳, 纬度, 经度, 高度

**启动命令**:

```bash
ros2 run Box_AD imu_gps_logger
```

---

### 规划模块

#### 7. `astar_planner_node`

**功能**: 全局路径规划，使用A*算法结合车辆动力学约束

**输入**:

- `/map` (nav_msgs/OccupancyGrid) - 静态地图
- `/initialpose` (geometry_msgs/PoseWithCovarianceStamped) - 起点
- `/goal_pose` (geometry_msgs/PoseStamped) - 终点

**输出**:

- `/global_path` (nav_msgs/Path) - map坐标系路径 (可视化用)
- `/global_path_utm` (nav_msgs/Path) - UTM坐标系路径 (局部规划器使用)

**参数**:

```bash
--ros-args \
  -p robot_radius:=0.5 \           # 障碍物膨胀距离 (米)
  -p min_turning_radius:=2.0 \     # 最小转弯半径 (米)
  -p smooth_iterations:=150        # 平滑迭代次数
```

**启动命令**:

```bash
ros2 run Box_AD astar_planner_node \
  --ros-args \
  -p robot_radius:=0.7 \
  -p min_turning_radius:=3.0
```

---

#### 8. `local_path_planner_node`

**功能**: 局部轨迹规划，DWA采样，动态避障

**输入**:

- `/global_path_utm` (nav_msgs/Path) - 全局参考路径
- `/pose_utm` (geometry_msgs/PoseStamped) - 自车姿态
- `/fast_obstacles` (std_msgs/String) - 动态障碍物

**输出**:

- `/local_path` (nav_msgs/Path) - 避障后的局部轨迹 (UTM坐标)
- `/update_global_path` (nav_msgs/Path) - 原始全局路径 (用于可视化)

**关键参数**:

```bash
--ros-args \
  -p lookahead_distance:=10.0 \  # 前瞻距离 (米)
  -p max_speed:=2.0 \            # 最大线速度 (米/秒)
  -p robot_radius:=0.7 \         # 碰撞检测半径 (米)
  -p predict_time:=2.0 \         # 轨迹预测时间 (秒)
  -p predict_dt:=0.1 \           # 预测时间步长 (秒)
  -p velocity_samples:=15 \      # 速度采样数
  -p yaw_samples:=21             # 角速度采样数
```

**代价函数权重**:

```bash
  -p path_follow_weight:=1.0 \   # 跟踪全局路径
  -p heading_weight:=2.0 \       # 朝向对齐
  -p clearance_weight:=3.0 \     # 与障碍物保持距离
  -p ttc_weight:=5.0             # 避免碰撞
```

**启动命令**:

```bash
ros2 run Box_AD local_path_planner_node \
  --ros-args \
  -p max_speed:=2.0 \
  -p robot_radius:=0.7
```

---

### 轨迹转换模块

#### 9. `local_path_to_mpc`

**功能**: 将局部规划轨迹转换为MPC控制器所需格式，并计算曲率

**输入**:

- `/local_path` (nav_msgs/Path) - UTM坐标的局部轨迹

**输出**:

- `/mpc_trajectory` (geometry_msgs/PoseArray) - MPC所需格式
  - `position.x, y`: UTM坐标
  - `orientation.x`: 曲率 (ck)
  - `orientation.z, w`: 航向四元数

**启动命令**:

```bash
ros2 run Box_AD local_path_to_mpc
```

**使用场景**: 配合MPC循迹控制器使用

---

#### 10. `local_path_to_follow`

**功能**: 将UTM坐标的局部轨迹转换为经纬度坐标，供简易循迹控制器使用

**输入**:

- `/local_path` (nav_msgs/Path) - UTM坐标的局部轨迹

**输出**:

- `/trajectory` (geometry_msgs/PoseArray) - 经纬度坐标
  - `position.x`: 经度 (longitude)
  - `position.y`: 纬度 (latitude)
  - `orientation.z, w`: 航向四元数

**参数**:

```bash
--ros-args \
  -p central_longitude:=118.8170043 \  # 投影中央经线
  -p central_latitude:=31.8926311      # 投影原点纬度
```

**启动命令**:

```bash
ros2 run Box_AD local_path_to_follow
```

**依赖**: 需要安装 `pyproj` 库

```bash
pip install pyproj
```

**使用场景**: 配合follow_node简易循迹控制器使用

---

### 循迹控制模块

> 注: 循迹控制模块位于 `follow_traj_wd` 子包中

#### 11. `mpc_node_v2` (MPC循迹)

**功能**: 基于模型预测控制的高精度轨迹跟踪

**输入**:

- `/mpc_trajectory` (geometry_msgs/PoseArray) - 带曲率的轨迹
- `/vehicle_state` (Float32MultiArray) - 车辆状态

**输出**:

- CAN总线控制指令

**特点**:
- 高精度轨迹跟踪
- 考虑车辆动力学模型
- 需要曲率信息

---

#### 12. `follow_node` (简易循迹)

**功能**: 基于纯跟踪的简易轨迹跟踪

**输入**:

- `/trajectory` (geometry_msgs/PoseArray) - 经纬度轨迹
- `/vehicle_state` (Float32MultiArray) - 车辆状态

**输出**:

- `/my_planner_action` (Float32MultiArray) - 控制指令

**特点**:
- 简单易用
- 输入为经纬度坐标
- 适合低速场景

---

### 可视化模块

#### 13. `ego_visualizer`

**功能**: 在RViz中显示自车3D模型

**输入**:

- `/pose_utm` (geometry_msgs/PoseStamped) - UTM坐标系自车位置

**输出**:

- `/ego_marker` (visualization_msgs/Marker) - 3D车辆模型

**启动命令**:

```bash
ros2 run Box_AD ego_visualizer \
  --ros-args -p map_yaml_path:=/path/to/map.yaml
```

---

#### 14. `path_visualizer`

**功能**: 将所有路径转换到map坐标系统一可视化

**输入**:

- `/global_path_utm`, `/local_path`, `/update_global_path` (nav_msgs/Path)

**输出**:

- `/global_path_utm_in_map`, `/local_path_in_map`, `/update_global_path_in_map` (nav_msgs/Path)

**启动命令**:

```bash
ros2 run Box_AD path_visualizer \
  --ros-args -p map_yaml_path:=/path/to/map.yaml
```

---

### 地图与环境模块

#### 15. `png_map_publisher`

**功能**: 将PNG图像转换为ROS OccupancyGrid地图

**输出**:

- `/map` (nav_msgs/OccupancyGrid) - 静态地图 (1Hz)

**参数**:

```bash
--ros-args \
  -p image_path:=/path/to/map.png \  # PNG文件路径
  -p resolution:=0.1 \                # 米/像素
  -p origin_x:=0.0 \                  # 地图原点X
  -p origin_y:=0.0                    # 地图原点Y
```

**启动命令**:

```bash
ros2 run Box_AD png_map_publisher \
  --ros-args \
  -p image_path:=/home/user/my_map.png \
  -p resolution:=0.1
```

---

## Launch文件启动

### 使用launch文件一键启动系统

```bash
# 默认配置启动
ros2 launch Box_AD box_ad.launch.py

# 自定义参数
ros2 launch Box_AD box_ad.launch.py \
  map_image_path:=/path/to/map.png \
  map_resolution:=0.1 \
  map_origin_x:=-50.0 \
  map_origin_y:=-80.0 \
  max_speed:=2.0 \
  robot_radius:=0.7 \
  use_obstacle_detector:=true \
  use_planner:=true \
  use_visualizer:=true
```

### Launch参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `map_image_path` | - | 地图PNG文件路径 |
| `map_resolution` | 0.1 | 地图分辨率 (米/像素) |
| `map_origin_x` | 0.0 | 地图原点X |
| `map_origin_y` | 0.0 | 地图原点Y |
| `robot_radius` | 0.7 | 机器人半径 (米) |
| `max_speed` | 2.0 | 最大速度 (米/秒) |
| `lookahead_distance` | 10.0 | 前瞻距离 (米) |
| `use_obstacle_detector` | true | 是否启动障碍物检测 |
| `use_planner` | true | 是否启动规划模块 |
| `use_visualizer` | true | 是否启动可视化 |

---

## 完整启动示例

### 场景1: MPC循迹 (高精度)

```bash
# 终端1: 启动核心系统
ros2 launch Box_AD box_ad.launch.py

# 终端2: 启动轨迹转换 (UTM+曲率)
ros2 run Box_AD local_path_to_mpc

# 终端3: 启动MPC控制器 (在follow_traj_wd包中)
ros2 run follow_traj_wd mpc_node_v2
```

### 场景2: 简易循迹 (经纬度)

```bash
# 终端1: 启动核心系统
ros2 launch Box_AD box_ad.launch.py

# 终端2: 启动轨迹转换 (经纬度)
ros2 run Box_AD local_path_to_follow

# 终端3: 启动简易控制器 (在follow_traj_wd包中)
ros2 run follow_traj_wd follow_node
```

### 场景3: 手动分步启动

```bash
# 终端1: CAN总线传感器
sudo ip link set can0 up type can bitrate 500000
ros2 run Box_AD publish_imu

# 终端2: GPS/IMU定位
ros2 run Box_AD gps_imu_to_utm_pose

# 终端3: 地图服务器
ros2 run Box_AD png_map_publisher \
  --ros-args -p image_path:=/path/to/map.png

# 终端4: 激光雷达障碍物检测
ros2 run Box_AD fast_lidar_obstacle_detector

# 终端5: 全局规划器
ros2 run Box_AD astar_planner_node

# 终端6: 局部规划器
ros2 run Box_AD local_path_planner_node

# 终端7: 轨迹转换 (选择一个)
ros2 run Box_AD local_path_to_mpc    # MPC用
# 或
ros2 run Box_AD local_path_to_follow  # follow_node用

# 终端8: 可视化
ros2 run Box_AD ego_visualizer
ros2 run Box_AD path_visualizer

# 终端9: RViz
rviz2
```

---

## 重要话题参考

### 传感器话题

| 话题 | 类型 | 描述 |
|------|------|------|
| `/IMU` | sensor_msgs/Imu | 50Hz IMU数据 |
| `/gps/fix` | sensor_msgs/NavSatFix | GPS定位 |
| `/lidar_point_prev` | sensor_msgs/PointCloud2 | 原始点云 |

### 定位话题

| 话题 | 类型 | 描述 |
|------|------|------|
| `/pose_utm` | geometry_msgs/PoseStamped | UTM坐标系姿态 |

### 感知话题

| 话题 | 类型 | 描述 |
|------|------|------|
| `/fast_obstacles` | std_msgs/String | 障碍物检测JSON |

### 规划话题

| 话题 | 类型 | 描述 |
|------|------|------|
| `/map` | nav_msgs/OccupancyGrid | 静态地图 |
| `/global_path` | nav_msgs/Path | 全局路径 (map坐标) |
| `/global_path_utm` | nav_msgs/Path | 全局路径 (UTM坐标) |
| `/local_path` | nav_msgs/Path | 局部轨迹 (UTM坐标) |
| `/update_global_path` | nav_msgs/Path | 原始全局路径 |

### 循迹控制话题

| 话题 | 类型 | 描述 |
|------|------|------|
| `/mpc_trajectory` | geometry_msgs/PoseArray | MPC轨迹 (UTM+曲率) |
| `/trajectory` | geometry_msgs/PoseArray | 简易循迹轨迹 (经纬度) |

### 可视化话题

| 话题 | 类型 | 描述 |
|------|------|------|
| `/ego_marker` | visualization_msgs/Marker | 自车模型 |
| `/fast_obstacle_markers` | visualization_msgs/Marker | 障碍物标记 |
| `/*_in_map` | nav_msgs/Path | map坐标系路径 |

---

## 常见问题

### 1. pyproj未安装

**症状**: `local_path_to_follow` 启动失败

**解决**:
```bash
pip install pyproj
```

### 2. 投影参数不匹配

**症状**: 轨迹偏移严重

**解决**: 确保 `local_path_to_follow` 的投影参数与 `gps_imu_to_utm_pose` 一致

```bash
ros2 run Box_AD local_path_to_follow \
  --ros-args \
  -p central_longitude:=118.8170043 \
  -p central_latitude:=31.8926311
```

### 3. MPC没有收到曲率

**症状**: MPC控制不稳定

**检查**: 确保启动了 `local_path_to_mpc` 节点

```bash
ros2 topic echo /mpc_trajectory --once
# 检查 orientation.x 是否有曲率值
```

### 4. 局部规划没有输出

**检查清单**:
1. 是否有全局路径: `ros2 topic echo /global_path_utm --once`
2. 是否有定位: `ros2 topic echo /pose_utm --once`
3. 是否有障碍物数据过期警告

### 5. CPU使用率高

**优化策略**:
- 增大 `predict_dt` (如 0.1 → 0.2)
- 减少 `velocity_samples` 和 `yaw_samples`
- 使用 `fast_` 系列节点

---

## 许可证

TODO: License declaration

## 维护者

- wudi (164662525@qq.com)

## 贡献

欢迎提交Issue和Pull Request!
