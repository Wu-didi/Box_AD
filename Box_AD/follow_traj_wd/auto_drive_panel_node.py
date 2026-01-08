#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
智驾控制面板节点

在 RViz 中显示一个可点击的 Interactive Marker 按钮，
用于启动/停止 can_node 进程，实现智驾模式的开关。

使用方式:
  1. 启动节点: ros2 run Box_AD auto_drive_panel
  2. 在 RViz 中添加 InteractiveMarkers 显示
  3. 设置 Update Topic 为 /auto_drive_panel/update
  4. 点击按钮启动/停止智驾
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from visualization_msgs.msg import InteractiveMarkerFeedback
from interactive_markers import InteractiveMarkerServer
import subprocess
import signal
import os


class AutoDrivePanelNode(Node):
    def __init__(self):
        super().__init__('auto_drive_panel')

        # can_node 进程
        self.can_node_process = None
        self.auto_drive_enabled = False

        # 状态发布
        self.status_pub = self.create_publisher(Bool, 'auto_drive_status', 10)
        self.log_pub = self.create_publisher(String, 'auto_drive_log', 10)

        # 定时检查进程状态
        self.check_timer = self.create_timer(1.0, self.check_process_status)

        # Interactive Marker Server
        self.server = InteractiveMarkerServer(self, 'auto_drive_panel')

        # 创建按钮
        self.create_button()

        self.get_logger().info('=' * 50)
        self.get_logger().info('智驾控制面板已启动')
        self.get_logger().info('RViz 设置: 添加 InteractiveMarkers')
        self.get_logger().info('Update Topic: /auto_drive_panel/update')
        self.get_logger().info('=' * 50)

    def check_process_status(self):
        """检查 can_node 进程状态"""
        if self.can_node_process is not None:
            poll = self.can_node_process.poll()
            if poll is not None:
                # 进程已退出
                self.get_logger().warn(f'can_node 进程已退出，返回码: {poll}')
                self.can_node_process = None
                self.auto_drive_enabled = False
                self.update_button()

        # 发布状态
        status_msg = Bool()
        status_msg.data = self.auto_drive_enabled
        self.status_pub.publish(status_msg)

    def create_button(self):
        """创建 Interactive Marker 按钮"""
        int_marker = self._build_marker()
        self.server.insert(int_marker, feedback_callback=self.button_callback)
        self.server.applyChanges()

    def update_button(self):
        """更新按钮外观"""
        int_marker = self._build_marker()
        self.server.insert(int_marker, feedback_callback=self.button_callback)
        self.server.applyChanges()

    def _build_marker(self):
        """构建 Interactive Marker"""
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = 'map'
        int_marker.name = 'auto_drive_button'
        int_marker.pose.position.x = 0.0
        int_marker.pose.position.y = 0.0
        int_marker.pose.position.z = 5.0
        int_marker.scale = 3.0

        # 按钮外观
        button_marker = Marker()
        button_marker.type = Marker.CUBE
        button_marker.scale.x = 6.0
        button_marker.scale.y = 3.0
        button_marker.scale.z = 0.5

        # 文字
        text_marker = Marker()
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.pose.position.z = 1.2
        text_marker.scale.z = 1.2
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0

        if self.auto_drive_enabled:
            # 绿色 = 运行中
            button_marker.color.r = 0.0
            button_marker.color.g = 0.8
            button_marker.color.b = 0.0
            button_marker.color.a = 0.9
            text_marker.text = '智驾运行中\n[点击停止]'
            int_marker.description = '点击停止智驾'
        else:
            # 红色 = 已停止
            button_marker.color.r = 0.8
            button_marker.color.g = 0.0
            button_marker.color.b = 0.0
            button_marker.color.a = 0.9
            text_marker.text = '智驾已停止\n[点击启动]'
            int_marker.description = '点击启动智驾'

        # 控制
        button_control = InteractiveMarkerControl()
        button_control.interaction_mode = InteractiveMarkerControl.BUTTON
        button_control.always_visible = True
        button_control.markers.append(button_marker)
        button_control.markers.append(text_marker)
        int_marker.controls.append(button_control)

        return int_marker

    def button_callback(self, feedback):
        """处理按钮点击"""
        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            if self.auto_drive_enabled:
                self.stop_can_node()
            else:
                self.start_can_node()

    def start_can_node(self):
        """启动 can_node 进程"""
        if self.can_node_process is not None:
            self.get_logger().warn('can_node 已在运行')
            return

        try:
            # 启动 can_node
            self.can_node_process = subprocess.Popen(
                ['ros2', 'run', 'Box_AD', 'can_node'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # 创建新的进程组，方便后续终止
            )
            self.auto_drive_enabled = True
            self.get_logger().info('can_node 已启动，PID: %d' % self.can_node_process.pid)
            self._publish_log('智驾已启动')
            self.update_button()

        except Exception as e:
            self.get_logger().error(f'启动 can_node 失败: {e}')
            self._publish_log(f'启动失败: {e}')

    def stop_can_node(self):
        """停止 can_node 进程"""
        if self.can_node_process is None:
            self.get_logger().warn('can_node 未在运行')
            return

        try:
            # 发送 SIGTERM 给整个进程组
            os.killpg(os.getpgid(self.can_node_process.pid), signal.SIGTERM)
            self.can_node_process.wait(timeout=3.0)
            self.get_logger().info('can_node 已停止')
            self._publish_log('智驾已停止')

        except subprocess.TimeoutExpired:
            # 强制终止
            os.killpg(os.getpgid(self.can_node_process.pid), signal.SIGKILL)
            self.get_logger().warn('can_node 强制终止')
            self._publish_log('智驾强制停止')

        except Exception as e:
            self.get_logger().error(f'停止 can_node 失败: {e}')
            self._publish_log(f'停止失败: {e}')

        finally:
            self.can_node_process = None
            self.auto_drive_enabled = False
            self.update_button()

    def _publish_log(self, msg):
        """发布日志消息"""
        log_msg = String()
        log_msg.data = msg
        self.log_pub.publish(log_msg)

    def destroy_node(self):
        """清理资源"""
        # 停止 can_node
        if self.can_node_process is not None:
            self.stop_can_node()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AutoDrivePanelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
