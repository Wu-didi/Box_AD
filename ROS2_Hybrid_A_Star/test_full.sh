#!/bin/bash
# ROS2 Hybrid A* 完整测试脚本（需要在多个终端运行或使用tmux）

set -e

echo "=================================================="
echo "   ROS2 Hybrid A* 完整功能测试"
echo "=================================================="
echo ""

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

cd /home/nvidia/vcii/wudi/Box_AD/ROS2_Hybrid_A_Star

# 检查是否已编译
if [ ! -d "install" ]; then
    echo -e "${YELLOW}正在编译...${NC}"
    colcon build --symlink-install
fi

source install/setup.bash

echo -e "${BLUE}提示：${NC}此脚本会在后台启动节点"
echo "如果你想看实时日志，请使用: ${YELLOW}./run_test.sh${NC}"
echo ""
read -p "按 Enter 继续，或 Ctrl+C 取消..."

# 1. 启动节点
echo ""
echo -e "${YELLOW}[1/4] 启动 Hybrid A* 节点...${NC}"
ros2 run ros2_hybrid_a_star hybrid_a_star_node --ros-args \
    --params-file config/params.yaml \
    > /tmp/hybrid_astar_node.log 2>&1 &

NODE_PID=$!
echo -e "${GREEN}✓ 节点已启动 (PID: $NODE_PID)${NC}"
echo "  日志文件: /tmp/hybrid_astar_node.log"

# 清理函数
cleanup() {
    echo ""
    echo -e "${YELLOW}清理进程...${NC}"
    kill $NODE_PID 2>/dev/null || true
    kill $MAP_PID 2>/dev/null || true
    echo -e "${GREEN}✓ 清理完成${NC}"
    exit
}

trap cleanup EXIT INT TERM

sleep 3

# 检查节点是否运行
if ! ps -p $NODE_PID > /dev/null; then
    echo -e "${RED}✗ 节点启动失败！${NC}"
    echo "查看日志："
    cat /tmp/hybrid_astar_node.log
    exit 1
fi

# 2. 启动地图发布器（如果存在）
echo ""
echo -e "${YELLOW}[2/4] 尝试启动地图发布器...${NC}"

if ros2 pkg list | grep -q Box_AD; then
    echo "使用 Box_AD 的地图发布器..."
    ros2 run Box_AD png_map_publisher --ros-args \
        -p image_path:=/home/nvidia/vcii/wudi/Box_AD/maps/map_2d.png \
        -p resolution:=0.1 \
        -p origin_x:=0.0 \
        -p origin_y:=0.0 \
        > /tmp/map_publisher.log 2>&1 &
    MAP_PID=$!
    echo -e "${GREEN}✓ 地图发布器已启动 (PID: $MAP_PID)${NC}"
    sleep 2
elif [ -f "/home/nvidia/vcii/wudi/Box_AD/maps/map.yaml" ]; then
    echo "使用 nav2_map_server..."
    ros2 run nav2_map_server map_server --ros-args \
        -p yaml_filename:=/home/nvidia/vcii/wudi/Box_AD/maps/map.yaml \
        > /tmp/map_publisher.log 2>&1 &
    MAP_PID=$!
    echo -e "${GREEN}✓ 地图服务器已启动 (PID: $MAP_PID)${NC}"
    sleep 2
else
    echo -e "${RED}✗ 未找到地图发布器${NC}"
    echo "请手动在另一个终端启动地图发布器"
    MAP_PID=""
fi

# 等待地图加载
echo ""
echo -e "${YELLOW}等待地图加载 (5秒)...${NC}"
sleep 5

# 检查日志
echo ""
echo -e "${YELLOW}[3/4] 检查节点状态...${NC}"
echo "节点日志 (最后20行):"
echo "-----------------------------------"
tail -20 /tmp/hybrid_astar_node.log
echo "-----------------------------------"

if grep -q "Map processed and planner initialized" /tmp/hybrid_astar_node.log; then
    echo -e "${GREEN}✓ 地图已加载并初始化完成${NC}"
else
    echo -e "${YELLOW}○ 地图可能还未加载${NC}"
fi

# 3. 发布测试起点和终点
echo ""
echo -e "${YELLOW}[4/4] 发布测试起点和终点...${NC}"
sleep 2

echo "发布起点: (10, 10, 0°)"
ros2 topic pub --once /initialpose geometry_msgs/PoseWithCovarianceStamped "
header: {frame_id: 'map'}
pose:
  pose:
    position: {x: 10.0, y: 10.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
" > /dev/null 2>&1

sleep 1

echo "发布终点: (50, 50, 45°)"
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped "
header: {frame_id: 'map'}
pose:
  position: {x: 50.0, y: 50.0, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.38268343, w: 0.92387953}
" > /dev/null 2>&1

# 等待规划完成
echo ""
echo -e "${YELLOW}等待路径规划 (10秒)...${NC}"
sleep 10

# 检查结果
echo ""
echo "=================================================="
echo -e "${BLUE}测试结果${NC}"
echo "=================================================="

echo ""
echo "节点日志 (规划相关):"
echo "-----------------------------------"
tail -30 /tmp/hybrid_astar_node.log | grep -E "Received|planning|succeeded|failed|Published" || echo "未找到规划相关日志"
echo "-----------------------------------"

echo ""
echo "话题检查:"
for topic in "/global_path" "/global_path_utm" "/global_trajectory"; do
    echo -n "  $topic: "
    if timeout 2 ros2 topic echo $topic --once > /dev/null 2>&1; then
        echo -e "${GREEN}✓ 有数据${NC}"
    else
        echo -e "${RED}✗ 无数据${NC}"
    fi
done

echo ""
echo "路径统计:"
path_points=$(timeout 3 ros2 topic echo /global_path --once 2>/dev/null | grep -c "position:" || echo "0")
echo "  路径点数: $path_points"

if [ "$path_points" -gt 0 ]; then
    echo ""
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}  ✓ 测试成功！${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""
    echo "生成的路径包含 $path_points 个点"
    echo ""
    echo "查看路径:"
    echo "  ${YELLOW}ros2 topic echo /global_path --once${NC}"
    echo ""
    echo "可视化:"
    echo "  ${YELLOW}rviz2${NC}"
    echo "  然后添加 Path 显示，订阅 /global_path"
else
    echo ""
    echo -e "${RED}========================================${NC}"
    echo -e "${RED}  ✗ 测试失败或未完成${NC}"
    echo -e "${RED}========================================${NC}"
    echo ""
    echo "可能的原因:"
    echo "  1. 地图未正确加载"
    echo "  2. 起点或终点在障碍物内"
    echo "  3. 规划超时"
    echo ""
    echo "查看完整日志:"
    echo "  ${YELLOW}cat /tmp/hybrid_astar_node.log${NC}"
fi

echo ""
echo "进程状态:"
echo "  Hybrid A* 节点: PID $NODE_PID"
[ -n "$MAP_PID" ] && echo "  地图发布器: PID $MAP_PID"
echo ""
echo "按 Enter 清理并退出，或 Ctrl+C 保持运行..."
read

cleanup
