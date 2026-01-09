#!/bin/bash
# ROS2 Hybrid A* 一键测试脚本

set -e  # 遇到错误立即退出

echo "=================================================="
echo "   ROS2 Hybrid A* 一键测试脚本"
echo "=================================================="
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 进入正确的目录
cd /home/nvidia/vcii/wudi/Box_AD/ROS2_Hybrid_A_Star

echo -e "${YELLOW}[1/5] 检查编译状态...${NC}"
if [ ! -d "install" ]; then
    echo -e "${YELLOW}未找到编译结果，开始编译...${NC}"
    colcon build --symlink-install
    if [ $? -ne 0 ]; then
        echo -e "${RED}编译失败！${NC}"
        exit 1
    fi
    echo -e "${GREEN}✓ 编译成功${NC}"
else
    echo -e "${GREEN}✓ 已编译${NC}"
fi

# Source 环境
source install/setup.bash

echo ""
echo -e "${YELLOW}[2/5] 启动 Hybrid A* 节点...${NC}"
echo "使用以下参数："
echo "  - 车长: 4.53m, 车宽: 1.9m, 轴距: 2.85m"
echo "  - 最大转向角: 60度"
echo "  - map.yaml: /home/nvidia/vcii/wudi/Box_AD/maps/map.yaml"
echo ""

# 使用参数文件启动，避免命令行参数类型问题
ros2 run ros2_hybrid_a_star hybrid_a_star_node --ros-args \
    --params-file config/params.yaml &

NODE_PID=$!
echo -e "${GREEN}✓ 节点已启动 (PID: $NODE_PID)${NC}"

# 等待节点启动
echo ""
echo -e "${YELLOW}[3/5] 等待节点初始化 (5秒)...${NC}"
sleep 5

# 检查节点是否还在运行
if ! ps -p $NODE_PID > /dev/null; then
    echo -e "${RED}✗ 节点启动失败或已崩溃！${NC}"
    echo "请查看上方的错误信息"
    exit 1
fi

echo ""
echo -e "${YELLOW}[4/5] 检查话题...${NC}"
sleep 2  # 再等2秒让节点完全注册

# 检查节点
echo -n "  节点: "
if ros2 node list 2>/dev/null | grep -q "hybrid_a_star_node"; then
    echo -e "${GREEN}✓ hybrid_a_star_node${NC}"
else
    echo -e "${YELLOW}○ 节点正在注册中...${NC}"
    sleep 2
    if ros2 node list 2>/dev/null | grep -q "hybrid_a_star_node"; then
        echo -e "${GREEN}  ✓ 现在找到了${NC}"
    else
        echo -e "${RED}  ✗ 仍未找到，但节点进程存在 (PID: $NODE_PID)${NC}"
        echo -e "${YELLOW}  这可能是正常的，继续检查话题...${NC}"
    fi
fi

# 检查订阅的话题
echo "  订阅话题:"
for topic in "/map" "/initialpose" "/goal_pose"; do
    echo -n "    $topic: "
    if ros2 topic list | grep -q "^${topic}$"; then
        echo -e "${GREEN}✓${NC}"
    else
        echo -e "${YELLOW}○ (等待)${NC}"
    fi
done

# 检查发布的话题
echo "  发布话题:"
for topic in "/global_path" "/global_path_utm" "/global_trajectory" "/search_tree"; do
    echo -n "    $topic: "
    if ros2 topic list | grep -q "^${topic}$"; then
        echo -e "${GREEN}✓${NC}"
    else
        echo -e "${RED}✗${NC}"
    fi
done

echo ""
echo -e "${YELLOW}[5/5] 查看节点信息...${NC}"
ros2 node info /hybrid_a_star_node | head -20

echo ""
echo "=================================================="
echo -e "${GREEN}测试脚本完成！${NC}"
echo "=================================================="
echo ""
echo "节点正在后台运行 (PID: $NODE_PID)"
echo ""
echo -e "${BLUE}下一步操作：${NC}"
echo "  1. 在另一个终端发布地图："
echo "     ${YELLOW}ros2 run Box_AD png_map_publisher --ros-args -p image_path:=/home/nvidia/vcii/wudi/Box_AD/maps/map_2d.png -p resolution:=0.1${NC}"
echo ""
echo "  2. 发布测试起点和终点："
echo "     ${YELLOW}cd /home/nvidia/vcii/wudi/Box_AD/ROS2_Hybrid_A_Star${NC}"
echo "     ${YELLOW}./publish_test_poses.sh${NC}"
echo ""
echo "  3. 查看实时日志："
echo "     ${YELLOW}ros2 topic echo /global_path --once${NC}"
echo ""
echo "  4. 停止节点："
echo "     ${YELLOW}kill $NODE_PID${NC}"
echo ""
echo "  5. 运行完整测试（需要另开终端发布地图）："
echo "     ${YELLOW}./test_with_map.sh${NC}"
echo ""

# 等待用户中断
echo "按 Ctrl+C 停止节点..."
wait $NODE_PID
