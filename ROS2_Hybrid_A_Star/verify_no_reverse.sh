#!/bin/bash
# 验证倒车功能已禁用

set -e

echo "=========================================="
echo "  验证 Hybrid A* 仅前进模式"
echo "=========================================="
echo ""

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

cd /home/nvidia/vcii/wudi/Box_AD/ROS2_Hybrid_A_Star

echo -e "${YELLOW}[1/3] 检查代码修改...${NC}"

# 检查 BACKWARD 代码是否已注释
if grep -q "// backward - DISABLED: Vehicle cannot reverse" src/hybrid_a_star.cpp; then
    echo -e "  ${GREEN}✓${NC} 倒车邻居节点生成已禁用"
else
    echo -e "  ${RED}✗${NC} 倒车邻居节点代码可能未注释"
fi

# 检查 RS path 是否已注释
if grep -q "// DISABLED: Reeds-Shepp path analytical expansion" src/hybrid_a_star.cpp; then
    echo -e "  ${GREEN}✓${NC} Reeds-Shepp 路径已禁用"
else
    echo -e "  ${RED}✗${NC} Reeds-Shepp 路径可能未注释"
fi

# 检查参数文件
if grep -q "# 注意：本配置已禁用倒车功能" config/params.yaml; then
    echo -e "  ${GREEN}✓${NC} 参数文件已更新说明"
else
    echo -e "  ${YELLOW}○${NC} 参数文件未添加说明"
fi

echo ""
echo -e "${YELLOW}[2/3] 检查编译状态...${NC}"

if [ -f "install/ros2_hybrid_a_star/lib/ros2_hybrid_a_star/hybrid_a_star_node" ]; then
    FILE_DATE=$(stat -c %y install/ros2_hybrid_a_star/lib/ros2_hybrid_a_star/hybrid_a_star_node | cut -d' ' -f1,2)
    echo -e "  ${GREEN}✓${NC} 可执行文件存在"
    echo -e "    编译时间: $FILE_DATE"
else
    echo -e "  ${RED}✗${NC} 可执行文件不存在"
    exit 1
fi

echo ""
echo -e "${YELLOW}[3/3] 代码分析...${NC}"

# 统计 BACKWARD 出现次数（应该只在注释中）
BACKWARD_COUNT=$(grep -c "BACKWARD" src/hybrid_a_star.cpp || echo "0")
BACKWARD_ACTIVE=$(grep "BACKWARD" src/hybrid_a_star.cpp | grep -v "//" | wc -l || echo "0")

echo "  src/hybrid_a_star.cpp 中 'BACKWARD' 出现次数: $BACKWARD_COUNT"
echo "  其中活跃（未注释）代码: $BACKWARD_ACTIVE"

if [ "$BACKWARD_ACTIVE" -eq 0 ]; then
    echo -e "  ${GREEN}✓${NC} 所有倒车相关代码已禁用"
else
    echo -e "  ${YELLOW}⚠${NC} 仍有 $BACKWARD_ACTIVE 处倒车代码未注释"
fi

echo ""
echo "=========================================="
echo -e "${GREEN}验证完成！${NC}"
echo "=========================================="
echo ""
echo -e "${BLUE}修改总结：${NC}"
echo "  1. 倒车邻居节点生成：已禁用"
echo "  2. Reeds-Shepp 路径：已禁用"
echo "  3. 规划器现在只生成前进路径"
echo ""
echo -e "${BLUE}注意事项：${NC}"
echo "  - 某些狭窄位置可能无法规划（需要倒车）"
echo "  - 确保起点和终点在可前进到达的位置"
echo "  - 如果规划失败，考虑调整目标点位置"
echo ""
echo -e "${BLUE}测试建议：${NC}"
echo "  1. 运行完整测试："
echo -e "     ${YELLOW}cd /home/nvidia/vcii/wudi/Box_AD${NC}"
echo -e "     ${YELLOW}./start_box_ad_ros2.sh${NC}"
echo ""
echo "  2. 在 RViz2 中可视化路径，确认只有前进动作"
echo ""
echo "详细修改说明请参考："
echo -e "  ${YELLOW}cat FORWARD_ONLY_MODIFICATIONS.md${NC}"
echo ""
