#!/usr/bin/env python3
"""
テストスクリプト: joy_safety_gateの動作確認

このスクリプトは、診断メッセージとJoyメッセージを送信してsafety gateの動作をテストします。
"""

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from sensor_msgs.msg import Joy
import time


class JoySafetyGateTestPublisher(Node):
    def __init__(self):
        super().__init__('joy_safety_gate_test_publisher')
        
        # Publishers
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.joy_pub = self.create_publisher(Joy, 'joy_in', 10)
        
        # Subscriber
        self.joy_sub = self.create_subscription(
            Joy, 'joy_out', self.joy_out_callback, 10)
        
        self.received_joy = None
        
    def joy_out_callback(self, msg):
        """joy_outトピックの出力を監視"""
        self.received_joy = msg
        self.get_logger().info(
            f'Received joy_out: axes={msg.axes[:2]}, buttons={msg.buttons[:2]}')
    
    def publish_diagnostics(self, level, name="test_diagnostic", message="Test"):
        """診断メッセージを送信"""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        status = DiagnosticStatus()
        status.level = level
        status.name = name
        status.message = message
        
        diag_array.status.append(status)
        self.diag_pub.publish(diag_array)
        
        level_str = {
            DiagnosticStatus.OK: "OK",
            DiagnosticStatus.WARN: "WARN",
            DiagnosticStatus.ERROR: "ERROR",
            DiagnosticStatus.STALE: "STALE"
        }.get(level, "UNKNOWN")
        
        self.get_logger().info(f'Published diagnostic: {name} = {level_str}')
    
    def publish_joy(self, axes, buttons):
        """Joyメッセージを送信"""
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = axes
        msg.buttons = buttons
        self.joy_pub.publish(msg)
        self.get_logger().info(
            f'Published joy_in: axes={axes[:2]}, buttons={buttons[:2]}')


def main():
    rclpy.init()
    node = JoySafetyGateTestPublisher()
    
    print("\n=== Joy Safety Gate Test ===\n")
    
    # Test 1: OKステータスで通過するか
    print("Test 1: Sending OK diagnostic and Joy input...")
    node.publish_diagnostics(DiagnosticStatus.OK, "test_sensor", "Everything OK")
    time.sleep(0.5)
    node.publish_joy([0.0, 0.5, 0.0, 0.0], [0, 0, 0, 0])
    time.sleep(1.0)
    rclpy.spin_once(node, timeout_sec=0.5)
    
    # Test 2: ERRORステータスでブロックされるか
    print("\nTest 2: Sending ERROR diagnostic and Joy input...")
    node.publish_diagnostics(DiagnosticStatus.ERROR, "test_sensor", "Error detected!")
    time.sleep(0.5)
    node.publish_joy([0.0, 0.8, 0.0, 0.0], [0, 0, 0, 0])
    time.sleep(1.0)
    rclpy.spin_once(node, timeout_sec=0.5)
    print("    -> Joy message should be BLOCKED (no output on joy_out)")
    
    # Test 3: 再びOKに戻して通過するか
    print("\nTest 3: Sending OK diagnostic again and Joy input...")
    node.publish_diagnostics(DiagnosticStatus.OK, "test_sensor", "Recovered")
    time.sleep(0.5)
    node.publish_joy([0.3, 0.0, 0.0, 0.5], [1, 0, 0, 0])
    time.sleep(1.0)
    rclpy.spin_once(node, timeout_sec=0.5)
    
    # Test 4: WARNステータス（デフォルト設定では通過）
    print("\nTest 4: Sending WARN diagnostic and Joy input...")
    node.publish_diagnostics(DiagnosticStatus.WARN, "test_sensor", "Warning")
    time.sleep(0.5)
    node.publish_joy([0.0, 0.3, 0.0, 0.0], [0, 0, 0, 0])
    time.sleep(1.0)
    rclpy.spin_once(node, timeout_sec=0.5)
    print("    -> Joy message should PASS (WARN not in default block_levels)")
    
    print("\n=== Test completed ===\n")
    print("Check the joy_safety_gate node logs for detailed information.")
    print("Monitor joy_out topic: ros2 topic echo /joy_out")
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
