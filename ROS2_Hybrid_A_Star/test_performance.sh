#!/bin/bash
# 性能测试脚本

echo "========================================="
echo "Hybrid A* 性能测试"
echo "========================================="
echo ""

# 测试不同距离的规划
declare -a start_points=("10 10" "20 20" "30 30")
declare -a goal_points=("50 50" "70 70" "90 90")

for i in "${!start_points[@]}"; do
    start=(${start_points[$i]})
    goal=(${goal_points[$i]})

    echo "测试 $((i+1)): 起点(${start[0]}, ${start[1]}) -> 终点(${goal[0]}, ${goal[1]})"

    # 发布起点
    ros2 topic pub --once /initialpose geometry_msgs/PoseWithCovarianceStamped "
header: {frame_id: 'map'}
pose:
  pose:
    position: {x: ${start[0]}.0, y: ${start[1]}.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
" > /dev/null 2>&1

    sleep 0.5

    # 发布终点并计时
    start_time=$(date +%s%N)
    ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped "
header: {frame_id: 'map'}
pose:
  position: {x: ${goal[0]}.0, y: ${goal[1]}.0, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
" > /dev/null 2>&1

    # 等待路径
    sleep 3

    # 检查路径点数
    path_points=$(ros2 topic echo /global_path --once 2>/dev/null | grep -c "position:")

    echo "  - 路径点数: $path_points"
    echo ""
done

echo "========================================="
echo "测试完成！请查看节点日志了解详细规划时间"
echo "========================================="
