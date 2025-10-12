#!/usr/bin/env python3
"""
テストスクリプト: diagnostic_safety_gateの動作確認

このスクリプトは、診断メッセージを送信してsafety gateの動作をテストします。
"""

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist
import time


class DiagnosticTestPublisher(Node):
    def __init__(self):
        super().__init__('diagnostic_test_publisher')
        
        # Publishers
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_in', 10)
        
        # Subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        self.received_cmd_vel = None
        
    def cmd_vel_callback(self, msg):
        """cmd_velトピックの出力を監視"""
        self.received_cmd_vel = msg
        self.get_logger().info(
            f'Received cmd_vel: linear.x={msg.linear.x:.3f}, angular.z={msg.angular.z:.3f}')
    
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
    
    def publish_cmd_vel(self, linear_x, angular_z):
        """コマンドを送信"""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info(
            f'Published cmd_vel_in: linear.x={linear_x:.3f}, angular.z={angular_z:.3f}')


def main():
    rclpy.init()
    node = DiagnosticTestPublisher()
    
    print("\n=== Diagnostic Safety Gate Test ===\n")
    
    # Test 1: OKステータスで通過するか
    print("Test 1: Sending OK diagnostic and forward command...")
    node.publish_diagnostics(DiagnosticStatus.OK, "test_sensor", "Everything OK")
    time.sleep(0.5)
    node.publish_cmd_vel(0.5, 0.0)
    time.sleep(1.0)
    rclpy.spin_once(node, timeout_sec=0.1)
    
    # Test 2: ERRORステータスでブロックされるか
    print("\nTest 2: Sending ERROR diagnostic and forward command...")
    node.publish_diagnostics(DiagnosticStatus.ERROR, "test_sensor", "Error detected!")
    time.sleep(0.5)
    node.publish_cmd_vel(0.5, 0.0)
    time.sleep(1.0)
    rclpy.spin_once(node, timeout_sec=0.1)
    
    # Test 3: 再びOKに戻して通過するか
    print("\nTest 3: Sending OK diagnostic again and forward command...")
    node.publish_diagnostics(DiagnosticStatus.OK, "test_sensor", "Recovered")
    time.sleep(0.5)
    node.publish_cmd_vel(0.3, 0.5)
    time.sleep(1.0)
    rclpy.spin_once(node, timeout_sec=0.1)
    
    print("\n=== Test completed ===\n")
    print("Check the diagnostic_safety_gate node logs for detailed information.")
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
