#!/usr/bin/env python3
import numpy as np
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from rcl_interfaces.msg import SetParametersResult

class TestSafetyController(Node):

    def __init__(self):
        super().__init__("test_safety_controller")
        # Declare parameters to make them available for use
        self.declare_parameter("scan_topic", "default")
        self.declare_parameter("drive_topic", "default")
        self.declare_parameter("velocity", 0.5)
        self.declare_parameter("is_stopped", 0.0)

        # Fetch constants from the ROS parameter server
        # This is necessary for the tests to be able to test varying parameters!
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.SENT = self.get_parameter('is_stopped').get_parameter_value().double_value
        
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.timer = self.create_timer(0.05, self.on_timer)
        self.safety_sub = self.create_subscription(
            AckermannDriveStamped,
            self.DRIVE_TOPIC,
            self.listener_callback,
            10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, 
            self.DRIVE_TOPIC,
            10)
        self.STOPPED = 0

    def listener_callback(self, msg):
        if msg.drive.speed == 0:
            self.STOPPED = 1

    def on_timer(self):
        if not self.STOPPED:
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = self.get_clock().now().to_msg()
            drive_msg.header.frame_id = "base_link"

            drive_msg.drive.steering_angle = 0.0
            drive_msg.drive.steering_angle_velocity = 0.0 # change as quick as possible
            drive_msg.drive.speed = self.VELOCITY # m/s
            drive_msg.drive.acceleration = 0.0 # change as quick as possible
            drive_msg.drive.jerk = 0.0 # change as quick as possible        

            self.drive_pub.publish(drive_msg)
    
    def parameters_callback(self, params):
        """
        DO NOT MODIFY THIS CALLBACK FUNCTION!
        
        This is used by the test cases to modify the parameters during testing. 
        It's called whenever a parameter is set via 'ros2 param set'.
        """
        for param in params:
            if param.name == "is_stopped":
                self.STOPPED = 0
                self.get_logger().info(f"RESET STOPPED FLAG")
        return SetParametersResult(successful=True)


def main():
    rclpy.init()
    test_safety_controller = TestSafetyController()
    rclpy.spin(test_safety_controller)
    test_safety_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    