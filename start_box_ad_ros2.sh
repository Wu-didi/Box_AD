#!/bin/bash
# Box_AD ROS2 (with Hybrid A*) 启动脚本

set -e

echo "=========================================="
echo "  Box_AD ROS2 自动驾驶栈启动脚本"
echo "  使用 Hybrid A* 全局规划"
echo "=========================================="
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 工作空间路径
BOX_AD_WS="/home/nvidia/vcii/wudi/Box_AD"
HYBRID_ASTAR_WS="$BOX_AD_WS/ROS2_Hybrid_A_Star"

cd "$BOX_AD_WS"

# 1. 检查 ROS2_Hybrid_A_Star 是否已编译
echo -e "${YELLOW}[1/4] 检查 Hybrid A* 编译状态...${NC}"
if [ ! -d "$HYBRID_ASTAR_WS/install" ]; then
    echo -e "${YELLOW}未找到 Hybrid A* 编译结果，开始编译...${NC}"
    cd "$HYBRID_ASTAR_WS"
    colcon build --symlink-install
    if [ $? -ne 0 ]; then
        echo -e "${RED}✗ Hybrid A* 编译失败！${NC}"
        exit 1
    fi
    echo -e "${GREEN}✓ Hybrid A* 编译成功${NC}"
    cd "$BOX_AD_WS"
else
    echo -e "${GREEN}✓ Hybrid A* 已编译${NC}"
fi

# 2. 检查 Box_AD 是否已编译
echo ""
echo -e "${YELLOW}[2/4] 检查 Box_AD 编译状态...${NC}"
if [ ! -d "$BOX_AD_WS/install/Box_AD" ]; then
    echo -e "${YELLOW}未找到 Box_AD 编译结果，开始编译...${NC}"
    colcon build --symlink-install
    if [ $? -ne 0 ]; then
        echo -e "${RED}✗ Box_AD 编译失败！${NC}"
        exit 1
    fi
    echo -e "${GREEN}✓ Box_AD 编译成功${NC}"
else
    echo -e "${GREEN}✓ Box_AD 已编译${NC}"
fi

# 3. Source 环境
echo ""
echo -e "${YELLOW}[3/4] 配置 ROS2 环境...${NC}"

# Source ROS2 基础环境
source /opt/ros/foxy/setup.bash
echo -e "  ${GREEN}✓${NC} ROS2 Foxy"

# Source Hybrid A* 环境（关键步骤！）
if [ -f "$HYBRID_ASTAR_WS/install/setup.bash" ]; then
    source "$HYBRID_ASTAR_WS/install/setup.bash"
    echo -e "  ${GREEN}✓${NC} Hybrid A* 包"
else
    echo -e "  ${RED}✗${NC} Hybrid A* 环境文件不存在"
    exit 1
fi

# Source Box_AD 环境
if [ -f "$BOX_AD_WS/install/setup.bash" ]; then
    source "$BOX_AD_WS/install/setup.bash"
    echo -e "  ${GREEN}✓${NC} Box_AD 包"
else
    echo -e "  ${RED}✗${NC} Box_AD 环境文件不存在"
    exit 1
fi

# 4. 验证包可见性
echo ""
echo -e "${YELLOW}[4/4] 验证包可见性...${NC}"

if ros2 pkg list | grep -q "^ros2_hybrid_a_star$"; then
    echo -e "  ${GREEN}✓${NC} ros2_hybrid_a_star 包已找到"
else
    echo -e "  ${RED}✗${NC} ros2_hybrid_a_star 包未找到"
    echo ""
    echo "当前可见的包："
    ros2 pkg list | grep -E "(Box_AD|hybrid)" || echo "  无相关包"
    exit 1
fi

if ros2 pkg list | grep -q "^Box_AD$"; then
    echo -e "  ${GREEN}✓${NC} Box_AD 包已找到"
else
    echo -e "  ${RED}✗${NC} Box_AD 包未找到"
    exit 1
fi

# 5. 启动系统
echo ""
echo "=========================================="
echo -e "${GREEN}环境配置完成！${NC}"
echo "=========================================="
echo ""
echo -e "${BLUE}启动参数说明：${NC}"
echo "  默认启动: ./start_box_ad_ros2.sh"
echo "  自定义参数: 见下方示例"
echo ""
echo -e "${BLUE}示例：${NC}"
echo "  # 基础启动"
echo "  ros2 launch Box_AD box_ad_ros2.launch.py"
echo ""
echo "  # 自定义地图和速度"
echo "  ros2 launch Box_AD box_ad_ros2.launch.py \\"
echo "      map_path:=./maps/map_2d.png \\"
echo "      max_speed:=3.0"
echo ""
echo "  # 禁用部分模块"
echo "  ros2 launch Box_AD box_ad_ros2.launch.py \\"
echo "      use_imu:=false \\"
echo "      use_perception:=false"
echo ""

# 询问是否立即启动
read -p "是否立即启动 Box_AD ROS2 系统？(y/n): " -n 1 -r
echo ""

if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo ""
    echo -e "${GREEN}正在启动...${NC}"
    echo ""
    ros2 launch Box_AD box_ad_ros2.launch.py
else
    echo ""
    echo "环境已配置完成，你可以手动运行："
    echo -e "${YELLOW}ros2 launch Box_AD box_ad_ros2.launch.py${NC}"
    echo ""
    echo "注意：需要在当前终端运行，或重新 source 环境："
    echo -e "${YELLOW}source $HYBRID_ASTAR_WS/install/setup.bash${NC}"
    echo -e "${YELLOW}source $BOX_AD_WS/install/setup.bash${NC}"
fi
