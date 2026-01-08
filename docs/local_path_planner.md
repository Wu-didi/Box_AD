# Local Path Planner（`local_path_planner_node`）

`pcd2scan_slam_toolbox/local_path_planner_node.py` 是一个“Hybrid DWA/Lattice”局部路径规划器，它直接读取全局导航路径、机器人姿态、速度与激光雷达数据，实时输出可执行的 `cmd_vel` 以及一条经过避障修正的局部轨迹。节点的目标是 **在遵循全局导航意图的同时，考虑动力学约束和动态障碍**，从而让机器人在复杂环境下保持平顺的运动。

## 节点职责

- 缓存 `nav_msgs/Path` 格式的全局路径并维护最近点索引，避免每次从头遍历。
- 读取定位（`PoseStamped`/`PoseWithCovarianceStamped`/`Odometry`）与速度，构造当前运动状态。
- 使用激光点云估计局部障碍，并通过连续帧计算简单的速度预测，得到未来一段时间的障碍分布。
- 在 Dynamic Window 内采样若干 `(v, w)`，结合 lattice 偏置生成候选轨迹，并仿真 `predict_time` 秒。
- 依据航向误差、路径偏差、速度收益、最小清距以及 TTC（time-to-collision）对轨迹打分，剔除不可行项。
- 同步发布 `/local_path`、`/update_global_path`、`/cmd_vel`；若失效或到达终点则输出零速。

## 数据接口

| 类型 | 话题 | 消息 | 说明 |
|------|------|------|------|
| 订阅 | `/global_path` | `nav_msgs/Path` | 上游全局规划结果，通过 `TRANSIENT_LOCAL` QoS 确保掉线后可重连。 |
| 订阅 | `pose_topic` | `PoseStamped` / `PoseWithCovarianceStamped` / `Odometry` | 通过 `pose_type` 参数指定具体格式。 |
| 订阅 | `velocity_topic` | `nav_msgs/Odometry` | 当 `pose_type` 不是 `Odometry` 时单独订阅速度。 |
| 订阅 | `laser_topic` | `sensor_msgs/LaserScan` | 局部障碍观测，支持距离外推。 |
| 发布 | `/local_path` | `nav_msgs/Path` | 评分最高的候选轨迹，方便在 RViz 观察。 |
| 发布 | `/update_global_path` | `nav_msgs/Path` | “最佳局部轨迹 + 全局剩余路段” 拼接后的路径。 |
| 发布 | `/cmd_vel` | `geometry_msgs/Twist` | 控制输入，线速度写入 `linear.x`，角速度写入 `angular.z`。 |

## 关键参数（含默认值）

| 参数 | 默认值 | 作用 |
|------|--------|------|
| `global_path_topic` | `global_path` | 全局路径订阅话题。 |
| `local_path_topic` | `local_path` | 局部轨迹发布话题。 |
| `cmd_vel_topic` | `cmd_vel` | 速度指令发布话题。 |
| `update_global_path_topic` | `update_global_path` | 避障后路径发布话题。 |
| `pose_topic` | `amcl_pose` | 位姿订阅话题。 |
| `pose_type` | `PoseWithCovarianceStamped` | 指定位姿消息类型。 |
| `velocity_topic` | `odom` | 单独获取速度的 Odometry 话题。 |
| `laser_topic` | `scan` | 激光雷达话题。 |
| `lookahead_distance` | `6.0` m | 沿全局路径寻找目标点的前瞻距离。 |
| `goal_tolerance` | `0.5` m | 与终点距离低于该值时视为到达。 |
| `robot_radius` | `0.7` m | 膨胀后的机器人半径，影响清距判断。 |
| `control_frequency` | `10.0` Hz | 控制循环频率。 |
| `predict_time` | `2.0` s | 候选轨迹的仿真时间。 |
| `predict_dt` | `0.1` s | 仿真步长。 |
| `velocity_samples` | `8` | 线速度采样数量。 |
| `yaw_samples` | `12` | 角速度采样数量。 |
| `max_speed` / `min_speed` | `2.0 / 0.0` m/s | 线速度范围。 |
| `max_acceleration` / `max_deceleration` | `1.5 / 2.0` m/s² | 线速度的加减速约束。 |
| `max_yaw_rate` / `max_yaw_acceleration` | `1.2` rad/s / `2.5` rad/s² | 角速度约束。 |
| `path_follow_weight` | `1.2` | 路径偏差权重。 |
| `heading_weight` | `1.4` | 航向误差权重。 |
| `velocity_weight` | `0.4` | 速度奖励权重。 |
| `clearance_weight` | `2.4` | 清距代价权重。 |
| `ttc_weight` | `1.0` | 碰撞时间代价权重。 |
| `obstacle_max_range` | `15.0` m | 忽略超过该距离的激光点。 |
| `dynamic_obstacle_horizon` | `1.0` s | 激光帧间的线性预测时间。 |
| `lattice_yaw_bias` | `[-0.8,-0.4,0.0,0.4,0.8]` rad/s | 额外角速度偏置，用于覆盖不同的转向分支。 |

> `control_frequency` 越高，动态窗口跨度越小，轨迹更平滑，但 DWA 采样计算量也会增加，可配合调低 `velocity_samples` 与 `yaw_samples`。

## 控制循环概述

1. **路径与位姿同步**  
   - 接收全局路径后缓存所有点 `(x, y)` 并保存 `frame_id`，同时重置最近索引。  
   - 位姿与路径 frame 不一致时会打印一次警告，随后假定已对齐。

2. **目标点选择**  
   - 以当前坐标为起点，从 `closest_path_idx` 开始沿路径累计弧长；当距离超过 `lookahead_distance` 或到达终点，记下目标 `x/y` 与朝向。  
   - 同步保存 `target_index`，供 `/update_global_path` 在避障段后继续拼接全局余量。

3. **动态窗口采样 + Lattice 偏置**  
   - 根据当前速度和动力学约束计算 `[v_min,v_max] × [w_min,w_max]`。  
   - 使用等距采样，随后将期望的“目标追踪角速度”与 `lattice_yaw_bias` 相加，覆盖直行、轻弯、急弯等驾驶意图。

4. **轨迹仿真**  
   - 对每对 `(v,w)`，按 `predict_dt` 在 `predict_time` 内前向积分，生成 `PoseStamped` 列表；该列表既是评分对象，也是最终的 `/local_path`。  
   - 姿态通过 `yaw_from_pose` 统一转换为四元数，以保证 RViz 中可视化正确。

5. **障碍建模与外推**  
   - 在 `scan_callback` 中将 `LaserScan` 投影至机器人坐标系 `(x,y)`，若上一帧存在对应光束，则用 `range_rate = (range_now - range_prev) / dt` 线性预测 `dynamic_obstacle_horizon` 秒后的距离。  
   - 通过 `update_obstacle_world_coordinates()` 将相对坐标转换到世界系，供碰撞检测使用。

6. **代价评估**  
   - **Heading**：最终朝向与目标朝向的差值（归一化到 `[0,1]`）。  
   - **Path**：轨迹终点与目标点之间的距离，按 `lookahead_distance` 归一化。  
   - **Velocity**：偏离 `max_speed` 越多代价越大，用于鼓励高速运行。  
   - **Clearance**：若最小清距小于 `robot_radius * 0.9` 直接丢弃；在 `4×robot_radius` 内以倒数形式惩罚。  
   - **TTC**：首次距离小于 `1.1×robot_radius` 的时间倒数，提前阻止潜在碰撞。  
   - 五项加权叠加为 `score`，取最小值作为最佳轨迹。

7. **输出与融合**  
   - 发布 `/local_path`：最佳轨迹全量点集。  
   - 发布 `/cmd_vel`：对应的 `(v, w)`，仅取第一控制步。  
   - 发布 `/update_global_path`：把局部轨迹与全局路径中 `target_index+1` 之后的点拼接，从而提供一条“已经绕过障碍”的路径给其他模块。  
   - 若无可行解或 `reached_goal()`，直接输出零速并发布当前姿态构成的静止轨迹，保证下游不会继续执行旧命令。

## 内部模块

- **Global Path Manager**：负责缓存 `Path`、维护 `closest_path_idx` 与 `target_index`，并提供 `reached_goal()`、`find_target_point()` 等辅助函数。  
- **Pose Subscription Helper**：根据 `pose_type` 创建不同的订阅器，保证 `current_pose` 与 `current_linear_vel`/`current_angular_vel` 始终有效。若位姿本身为 `Odometry`，无需重复订阅速度。  
- **Obstacle Projector**：利用雷达帧间差分预测动态障碍，减少“追尾”风险；预测点在世界系中由 `update_obstacle_world_coordinates()` 实时更新。  
- **TrajectoryCandidate 数据结构**：使用 `dataclass` 聚合轨迹、清距、碰撞时间、目标索引和综合得分，方便在 `publish_updated_global_path()` 中同时携带所需上下文。  
- **Fail-safe 逻辑**：`plan_once()` 无解时立即执行 `publish_stop()`，持续输出零速，避免机器人因上一周期命令继续运动。

## 启动示例

```bash
ros2 run pcd2scan_slam_toolbox local_path_planner_node \
  --ros-args \
  -p global_path_topic:=/global_path \
  -p local_path_topic:=/local_path \
  -p cmd_vel_topic:=/cmd_vel \
  -p pose_topic:=/amcl_pose \
  -p pose_type:=PoseWithCovarianceStamped \
  -p velocity_topic:=/odom \
  -p laser_topic:=/scan \
  -p lookahead_distance:=6.0 \
  -p robot_radius:=0.7 \
  -p control_frequency:=10.0 \
  -p predict_time:=2.0 \
  -p velocity_samples:=8 \
  -p yaw_samples:=12 \
  -p max_speed:=2.0 \
  -p max_acceleration:=1.5
```

> 如果希望调整到不同命名空间，可在 launch 文件中通过 `remap` 或参数覆盖（例如 `-p global_path_topic:=/planner/global_path` ）。

## 调试与优化建议

- 在 RViz 中同时显示 `/global_path`、`/local_path`、`/update_global_path` 与 `/scan`，能快速判断是否因 frame 或话题配置问题导致轨迹异常。  
- 若频繁进入 fail-safe（输出零速），可适当降低 `robot_radius` 或 `clearance_weight`、`ttc_weight`，亦可增加 `predict_time` 让规划器看到更远的绕障机会。  
- `velocity_samples`、`yaw_samples` 与 `predict_dt` 决定了每轮仿真数量；在算力有限时可降低采样密度或缩短 `predict_time`。  
- 调整 `lattice_yaw_bias` 能塑造不同驾驶风格：增加偏置值可提升左/右转偏好；在狭窄通道中可减小偏置以保持路径贴合。  
- `dynamic_obstacle_horizon` 越大，对高速障碍越敏感，但可能加剧对噪声的放大，需要配合 `obstacle_max_range` 或激光滤波器。  
- 若激光位于不同坐标系，请确保 TF 树有效或在上游进行转换，否则 `update_obstacle_world_coordinates()` 的结果会偏移，导致误判。
