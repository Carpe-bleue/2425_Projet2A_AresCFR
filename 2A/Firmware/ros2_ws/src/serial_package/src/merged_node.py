#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import serial
import subprocess
import os

class BagPlayerController(Node):
    def __init__(self):
        super().__init__('bag_player_controller')
        
        # Configuration
        self.bags_dir = os.path.expanduser('~/ros2_ws/src/serial_package/bags')
        self.serial_port_path = '/dev/serial0'
        self.stop_command = '00000000000000000000000011'
        self.init_command = '00000000000000000000000011'
        
        # State variables
        self.selected_bag = None
        self.tirette_state = True
        self.motors_stopped = False
        self.bag_process = None
        self.bag_playing = False
        self.prev_stop_state = False
        
        # Initialize serial port
        self.serial_port = self.initialize_serial()
        
        # Subscribers
        self.create_subscription(String, '/file_stm_choisi', self.file_stm_callback, 10)
        self.create_subscription(Bool, '/tirette_value', self.tirette_callback, 10)
        self.create_subscription(Bool, '/stop_moteur', self.stop_moteur_callback, 10)
        self.create_subscription(String, '/keyboard_commands', self.keyboard_callback, 10)
        
        self.get_logger().info("Node initialized and ready")

    def initialize_serial(self):
        try:
            ser = serial.Serial(self.serial_port_path, baudrate=115200, timeout=1)
            self.get_logger().info(f"Serial port {self.serial_port_path} opened successfully")
            try:
                ser.write(self.init_command.encode())
                self.get_logger().info(f"Sent initial command: {self.init_command}")
            except Exception as e:
                self.get_logger().error(f"Failed to send initial command: {e}")
            return ser
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            return None

    # --- REQUIRED CALLBACK METHODS ---
    def file_stm_callback(self, msg):
        try:
            bag_number = int(msg.data)
            if 0 <= bag_number <= 6:
                self.selected_bag = bag_number
                self.get_logger().info(f"Received valid bag selection: {self.selected_bag}")
                self.check_tirette_state()
            else:
                self.get_logger().warn(f"Invalid bag number received: {bag_number}")
        except ValueError:
            self.get_logger().error(f"Non-numeric bag selection received: {msg.data}")

    def tirette_callback(self, msg):
        new_state = msg.data
        if self.tirette_state != new_state:
            self.get_logger().info(f"Tirette state changed to: {new_state}")
            self.tirette_state = new_state
            self.check_tirette_state()

    def check_tirette_state(self):
        if self.selected_bag is not None and not self.tirette_state:
            self.play_selected_bag()

    def play_selected_bag(self):
        bag_path = os.path.join(self.bags_dir, str(self.selected_bag))
        if not os.path.exists(bag_path):
            self.get_logger().error(f"Bag {self.selected_bag} not found at {bag_path}")
            return

        if self.bag_process:
            self.bag_process.terminate()

        try:
            self.bag_process = subprocess.Popen(
                ['ros2', 'bag', 'play', bag_path],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT
            )
            self.bag_playing = True
            self.get_logger().info(f"Started playing bag: {self.selected_bag}")
            self.selected_bag = None
        except Exception as e:
            self.get_logger().error(f"Failed to play bag: {e}")

    def stop_moteur_callback(self, msg):
        current_state = msg.data
        if current_state != self.prev_stop_state:
            self.prev_stop_state = current_state
            
            if current_state:
                self.motors_stopped = True
                if self.bag_process:
                    self.bag_process.terminate()
                    self.bag_process = None
                    self.get_logger().info("Stopped bag playback")
                
                if self.serial_port and self.serial_port.is_open:
                    try:
                        self.serial_port.write(self.stop_command.encode())
                        self.get_logger().info("Sent emergency stop command")
                    except Exception as e:
                        self.get_logger().error(f"Failed to send stop command: {e}")
                
                self.bag_playing = False
            else:
                if self.motors_stopped:
                    self.motors_stopped = False
                    self.get_logger().info("Motor operations resumed")

    def keyboard_callback(self, msg):
        if self.motors_stopped or not self.serial_port or not self.serial_port.is_open:
            return

        try:
            self.serial_port.write(msg.data.encode())
            self.get_logger().debug(f"Sent command: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Failed to send command: {e}")

    def destroy_node(self):
        if self.bag_process:
            self.bag_process.terminate()
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    controller = BagPlayerController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
