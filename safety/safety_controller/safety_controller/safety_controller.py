#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from rcl_interfaces.msg import SetParametersResult

class SafetyController(Node):

    def __init__(self):
        super().__init__("safety_controller")
        # Declare parameters to make them available for use
        self.declare_parameter("scan_topic", "default")
        self.declare_parameter("drive_topic", "default")

        # Fetch constants from the ROS parameter server
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
		
        # This activates the parameters_callback function so that the tests are able
        # to change the parameters during testing.
        # self.add_on_set_parameters_callback(self.parameters_callback)
  
        self.cmd_sub = self.create_subscription(
            AckermannDriveStamped,
            self.SCAN_TOPIC,
            self.listener_callback,
            10)
        self.safety_pub = self.create_publisher(
            AckermannDriveStamped, 
            self.DRIVE_TOPIC,
            10)
        self.DT = 0.1 # in seconds

    def listener_callback(self, msg):
        drive_cmd = msg.drive
        drive_ang = drive_cmd.steering_angle
        drive_speed = drive_cmd.speed

        predicted_dist = drive_speed * self.DT

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "base_link"

        drive_msg.drive.steering_angle = 0.0 # set everything else to 0
        drive_msg.drive.steering_angle_velocity = 0.0 # set everything else to 0
        drive_msg.drive.speed = 0.0 # STOP THE CAR
        drive_msg.drive.acceleration = 0.0 # set everything else to 0
        drive_msg.drive.jerk = 0.0 # set everything else to 0      

        self.drive_pub.publish(drive_msg)

    def parameters_callback(self, params):
        """
        DO NOT MODIFY THIS CALLBACK FUNCTION!
        
        This is used by the test cases to modify the parameters during testing. 
        It's called whenever a parameter is set via 'ros2 param set'.
        """
        pass
        # for param in params:
        #     if param.name == 'side':
        #         self.SIDE = param.value
        #         self.get_logger().info(f"Updated side to {self.SIDE}")
        #     elif param.name == 'velocity':
        #         self.VELOCITY = param.value
        #         self.get_logger().info(f"Updated velocity to {self.VELOCITY}")
        #     elif param.name == 'desired_distance':
        #         self.DESIRED_DISTANCE = param.value
        #         self.get_logger().info(f"Updated desired_distance to {self.DESIRED_DISTANCE}")
        # return SetParametersResult(successful=True)


def main():
    rclpy.init()
    safety_controller = SafetyController()
    rclpy.spin(safety_controller)
    safety_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    