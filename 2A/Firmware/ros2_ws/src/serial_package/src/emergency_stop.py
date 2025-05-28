#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import serial
import time

class EmergencyStop(Node):
    def __init__(self):
        super().__init__('emergency_stop')
        
        # Configuration
        self.stop_cmd_str = "00000000000000000000000001"
        self.serial_port = '/dev/serial0'
        
        # Initialize serial
        self.ser = serial.Serial(
            self.serial_port,
            baudrate=115200,
            timeout=0.001,
            write_timeout=0
        )
        
        # Publisher for keyboard_commands
        self.pub_keyboard_commands = self.create_publisher(String, '/keyboard_commands', 10)
        
        # Subscription
        self.create_subscription(
            Bool, 
            '/stop_moteur', 
            self.stop_callback, 
            10  # QoS depth
        )
        
        self.get_logger().info("⏱️ Emergency stop ready (100Hz response)")

    def stop_callback(self, msg):
        """Send stop command when triggered"""
        if msg.data:
            # Try to send through serial
            try:
                self.ser.write(self.stop_cmd_str.encode('utf-8'))
                self.ser.flush()
                self.get_logger().warning("🛑 EMERGENCY STOP SENT", throttle_duration_sec=0.1)
            except Exception as e:
                self.get_logger().error(f"Serial write failed: {str(e)}")
            
            # Publish to keyboard_commands
            stop_msg = String()
            stop_msg.data = self.stop_cmd_str
            self.pub_keyboard_commands.publish(stop_msg)
            self.get_logger().info("Published stop command to /keyboard_commands")

    def destroy_node(self):
        self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
