# 测试脚本使用指南

## 📋 可用的测试脚本

我已经为你创建了几个测试脚本，位于 `ROS2_Hybrid_A_Star/` 目录：

### 1️⃣ `run_test.sh` - 基础启动测试 ⭐推荐先用这个

**用途：** 启动节点并显示实时日志，方便调试

**使用方法：**
```bash
cd /home/nvidia/vcii/wudi/Box_AD/ROS2_Hybrid_A_Star
./run_test.sh
```

**会做什么：**
- ✅ 检查并编译包（如果需要）
- ✅ 使用正确的参数启动节点
- ✅ 显示实时日志
- ✅ 检查话题状态
- ✅ 显示下一步操作提示

**你需要做什么：**
在另一个终端运行：
```bash
# 发布地图
ros2 run Box_AD png_map_publisher --ros-args \
    -p image_path:=/home/nvidia/vcii/wudi/Box_AD/maps/map_2d.png \
    -p resolution:=0.1

# 然后发布起点终点
cd /home/nvidia/vcii/wudi/Box_AD/ROS2_Hybrid_A_Star
./publish_test_poses.sh
```

---

### 2️⃣ `test_full.sh` - 完整自动化测试

**用途：** 自动启动所有组件并测试完整流程

**使用方法：**
```bash
cd /home/nvidia/vcii/wudi/Box_AD/ROS2_Hybrid_A_Star
./test_full.sh
```

**会做什么：**
- ✅ 启动 Hybrid A* 节点（后台）
- ✅ 启动地图发布器（后台）
- ✅ 自动发布测试起点和终点
- ✅ 等待规划完成
- ✅ 显示测试结果
- ✅ 自动清理进程

**优点：** 一键完成所有测试，适合快速验证

---

### 3️⃣ `test_topics.sh` - 话题检查

**用途：** 检查节点的话题状态

**使用方法：**
```bash
# 先启动节点，然后在另一个终端运行：
./test_topics.sh
```

**会显示：**
- 节点是否在运行
- 订阅的话题列表
- 发布的话题列表
- 话题类型信息

---

### 4️⃣ `publish_test_poses.sh` - 发布测试姿态

**用途：** 发布预设的测试起点和终点

**使用方法：**
```bash
./publish_test_poses.sh
```

**测试点位：**
- 起点：(10, 10, 0°)
- 终点：(50, 50, 45°)

---

## 🚀 快速开始（推荐流程）

### 方式A：手动分步测试（推荐调试时用）

**终端1：**
```bash
cd /home/nvidia/vcii/wudi/Box_AD/ROS2_Hybrid_A_Star
./run_test.sh
```

**终端2：**
```bash
# 发布地图
ros2 run Box_AD png_map_publisher --ros-args \
    -p image_path:=/home/nvidia/vcii/wudi/Box_AD/maps/map_2d.png \
    -p resolution:=0.1 \
    -p origin_x:=0.0 \
    -p origin_y:=0.0
```

**终端3：**
```bash
cd /home/nvidia/vcii/wudi/Box_AD/ROS2_Hybrid_A_Star

# 等地图加载后（终端1显示"Map processed"），发布起点终点
./publish_test_poses.sh

# 然后查看路径
ros2 topic echo /global_path --once
```

---

### 方式B：一键自动测试（推荐快速验证用）

**单个终端：**
```bash
cd /home/nvidia/vcii/wudi/Box_AD/ROS2_Hybrid_A_Star
./test_full.sh
```

等待10-15秒，脚本会自动完成所有测试并显示结果。

---

## ✅ 成功的标志

如果测试成功，你会看到：

```
[hybrid_a_star_node]: Hybrid A* Node initialized. Vehicle: L=4.53m W=1.90m WB=2.85m
[hybrid_a_star_node]: Loaded PNG origin UTM: E=XXX, N=XXX
[hybrid_a_star_node]: Received map: 1000x1000, resolution: 0.10
[hybrid_a_star_node]: Map processed and planner initialized.
[hybrid_a_star_node]: Received start: (10.00, 10.00, 0.0°)
[hybrid_a_star_node]: Received goal: (50.00, 50.00, 45.0°)
[hybrid_a_star_node]: Starting Hybrid A* path planning...
[hybrid_a_star_node]: Hybrid A* search completed: time=1234.56 ms, path_length=56.78 m
[hybrid_a_star_node]: Path planning succeeded! Points: XX, Time: XXX ms
[hybrid_a_star_node]: Published /global_path (map frame)
[hybrid_a_star_node]: Published /global_path_utm (UTM frame)
[hybrid_a_star_node]: Published /global_trajectory (lat/lon)
```

---

## ❌ 常见问题

### 问题1：节点崩溃

**现象：**
```
terminate called after throwing an instance of...
```

**解决：**
检查日志文件：
```bash
cat /tmp/hybrid_astar_node.log
```

### 问题2：找不到地图

**现象：**
```
Cannot open map.yaml
```

**解决：**
确认地图文件路径：
```bash
ls /home/nvidia/vcii/wudi/Box_AD/maps/
```

### 问题3：规划失败

**现象：**
```
Path planning failed!
```

**可能原因：**
- 起点/终点在障碍物内
- 地图未加载
- 参数设置不合理

**解决：**
使用安全的测试点，或在 RViz2 中手动选择起点终点。

---

## 📊 查看结果

### 命令行查看：

```bash
# 查看路径点数
ros2 topic echo /global_path --once | grep -c "position:"

# 查看第一个路径点
ros2 topic echo /global_path --once | head -30

# 查看所有话题
ros2 topic list | grep global
```

### RViz2 可视化：

```bash
# 启动 RViz2
rviz2

# 然后添加：
# 1. Fixed Frame: map
# 2. Add -> By topic -> /map -> Map
# 3. Add -> By topic -> /global_path -> Path
# 4. Add -> By topic -> /search_tree -> MarkerArray
```

---

## 🔧 调试技巧

### 1. 查看实时日志：

```bash
# 如果用 test_full.sh，查看日志文件
tail -f /tmp/hybrid_astar_node.log
```

### 2. 检查话题频率：

```bash
ros2 topic hz /global_path
```

### 3. 检查节点参数：

```bash
ros2 param list /hybrid_a_star_node
ros2 param get /hybrid_a_star_node vehicle_length
```

### 4. 调整参数重新测试：

```bash
# 修改 run_test.sh 中的参数，例如增大步长加速：
-p segment_length:=2.0 \
-p state_grid_resolution:=1.5 \
```

---

## 📝 总结

- **首次测试**：用 `run_test.sh`，看实时日志
- **快速验证**：用 `test_full.sh`，全自动
- **调试问题**：用 `test_topics.sh` 检查话题
- **手动测试**：用 `publish_test_poses.sh` 发布姿态

选择适合你的方式，开始测试吧！ 🚀
