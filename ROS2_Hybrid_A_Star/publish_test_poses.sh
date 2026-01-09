#!/bin/bash
# 发布测试起点和终点

echo "发布起点 (10, 10, 0°)..."
ros2 topic pub --once /initialpose geometry_msgs/PoseWithCovarianceStamped "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'map'
pose:
  pose:
    position:
      x: 10.0
      y: 10.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
"

sleep 1

echo "发布终点 (50, 50, 45°)..."
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'map'
pose:
  position:
    x: 50.0
    y: 50.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.38268343  # sin(45°/2)
    w: 0.92387953  # cos(45°/2)
"

echo ""
echo "起点和终点已发布！"
echo "请在终端1查看规划日志"
echo "如果成功，应该看到："
echo "  - Starting Hybrid A* path planning..."
echo "  - Path planning succeeded! Points: XXX, Time: XXX ms"
echo "  - Published /global_path (map frame)"
echo "  - Published /global_path_utm (UTM frame)"
echo "  - Published /global_trajectory (lat/lon)"
