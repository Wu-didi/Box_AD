# Hybrid A* 仅前进模式修改说明

## 修改原因
车辆不支持倒车功能，因此需要禁用 Hybrid A* 算法中的所有倒车逻辑。

## 修改内容

### 1. 禁用倒车邻居节点生成
**文件**: `src/hybrid_a_star.cpp`
**位置**: 第 402-426 行
**修改**: 注释掉 `GetNeighborNodes()` 函数中生成 BACKWARD 方向邻居节点的代码

```cpp
// backward - DISABLED: Vehicle cannot reverse
// has_obstacle = false;
// intermediate_state.clear();
// ... (完整代码已注释)
```

**效果**: 算法只会扩展前进方向的节点，不再考虑倒车方向。

### 2. 禁用 Reeds-Shepp 曲线
**文件**: `src/hybrid_a_star.cpp`
**位置**: 第 546-578 行
**修改**: 注释掉 `Search()` 函数中的 Reeds-Shepp 路径分析扩展

```cpp
// DISABLED: Reeds-Shepp path analytical expansion (contains reverse motions)
// Vehicle cannot reverse, so RS path is disabled
// if ((current_node_ptr->state_.head(2) - goal_node_ptr->state_.head(2)).norm() <= shot_distance_) {
//     ... (完整代码已注释)
// }
```

**效果**: 不再使用 Reeds-Shepp 曲线进行 "shot to goal" 优化，因为 RS 曲线本质上包含倒车段。

### 3. 更新参数文件
**文件**: `config/params.yaml`
**修改**: 添加说明注释

```yaml
# 注意：本配置已禁用倒车功能（车辆不支持倒车）
reversing_penalty: 2.0  # 未使用（倒车功能已禁用）
shot_distance: 10.0     # 未使用（Reeds-Shepp 路径已禁用）
```

## 影响和限制

### 正面影响
- ✅ 生成的路径保证只包含前进动作
- ✅ 符合车辆实际物理约束
- ✅ 简化了规划器的搜索空间

### 可能的限制
- ⚠️ **可能无法规划到某些狭窄空间** - 某些位置可能需要倒车才能到达
- ⚠️ **规划时间可能略微增加** - RS 曲线提供了快速的启发式路径
- ⚠️ **路径可能更长** - 无法利用倒车走捷径

### 建议
- 确保起点和终点都在可前进到达的位置
- 避免将目标点设置在需要倒车才能进入的狭窄区域
- 如果规划失败，考虑调整终点位置

## 验证方法

### 1. 检查生成的路径
```bash
ros2 topic echo /global_path --once
```

查看路径中每个点的速度方向，确保没有负值。

### 2. 可视化
在 RViz2 中查看 `/global_path`，确保路径是连续的前进轨迹。

### 3. 日志检查
查看节点日志，不应该出现 "BACKWARD" 或 "reverse" 相关的信息。

## 恢复倒车功能
如果将来需要恢复倒车功能：

1. 取消注释 `src/hybrid_a_star.cpp` 第 402-426 行（倒车邻居节点）
2. 取消注释 `src/hybrid_a_star.cpp` 第 546-578 行（RS 路径）
3. 重新编译：`colcon build --symlink-install`

## 编译和测试

### 重新编译
```bash
cd /home/nvidia/vcii/wudi/Box_AD/ROS2_Hybrid_A_Star
colcon build --symlink-install
```

### 测试
```bash
# 方式1：使用测试脚本
./run_test.sh

# 方式2：使用完整启动脚本
cd /home/nvidia/vcii/wudi/Box_AD
./start_box_ad_ros2.sh
```

## 修改历史
- 2026-01-09: 初始修改，禁用倒车功能
