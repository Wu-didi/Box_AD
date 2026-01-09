#!/bin/bash
# Hybrid A* 话题测试脚本

echo "========================================="
echo "1. 检查 Hybrid A* 节点是否在运行"
echo "========================================="
ros2 node list | grep hybrid

echo ""
echo "========================================="
echo "2. 检查订阅的话题（输入）"
echo "========================================="
echo "应该订阅："
echo "  - /map (地图)"
echo "  - /initialpose (起点)"
echo "  - /goal_pose (终点)"
echo ""
ros2 topic list | grep -E "map|initialpose|goal_pose"

echo ""
echo "========================================="
echo "3. 检查发布的话题（输出）"
echo "========================================="
echo "应该发布："
echo "  - /global_path (map坐标)"
echo "  - /global_path_utm (UTM坐标)"
echo "  - /global_trajectory (经纬度)"
echo "  - /search_tree (搜索树可视化)"
echo ""
ros2 topic list | grep -E "global_path|global_trajectory|search_tree"

echo ""
echo "========================================="
echo "4. 查看节点详细信息"
echo "========================================="
ros2 node info /hybrid_a_star_node

echo ""
echo "========================================="
echo "5. 检查话题类型"
echo "========================================="
echo "/global_path 类型："
ros2 topic info /global_path
echo ""
echo "/global_path_utm 类型："
ros2 topic info /global_path_utm
echo ""
echo "/global_trajectory 类型："
ros2 topic info /global_trajectory
