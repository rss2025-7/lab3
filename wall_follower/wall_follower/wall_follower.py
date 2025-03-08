#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

from wall_follower.visualization_tools import VisualizationTools
from .lidar_reading import compute_least_squares_line
import math


class WallFollower(Node):

    def __init__(self):
        super().__init__("wall_follower")
        # Declare parameters to make them available for use
        # DO NOT MODIFY THIS!
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("drive_topic", "/vesc/high_level/input/nav0")
        self.declare_parameter("side", 1)
        self.declare_parameter("velocity", 2.0)
        self.declare_parameter("desired_distance", 0.5)

        # Fetch constants from the ROS parameter server
        # DO NOT MODIFY THIS! This is necessary for the tests to be able to test varying parameters!
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value
        self.WALL_TOPIC = "/wall"

        # This activates the parameters_callback function so that the tests are able
        # to change the parameters during testing.
        # DO NOT MODIFY THIS!
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.Kp = 1
        self.Kd = 0.1
        self.Ki = 0.0

        self.prev_error = 0.0
        self.integral_error = 0.0
        self.last_time = self.get_clock().now()

        self.drive_publisher_ = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 10)
        self.lidar_subscriber_ = self.create_subscription(LaserScan, self.SCAN_TOPIC, self.listener_callback, 10)

        # FOR EVALUATION
        self.err_publisher_ = self.create_publisher(Float32, "wall_follower_logs", 10)

        self.line_pub = self.create_publisher(Marker, self.WALL_TOPIC, 1)

    def listener_callback(self, scan_msg):
        # self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        # self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        # self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value

        # self.get_logger().info(f'Side: {self.SIDE}')
        # -1: right, 1: left
        cutoff = 100.0
        if self.SIDE < 0:
            # start_angle = np.deg2rad(10)
            start_angle = -2.0
            end_angle = 0

            self.Kp = 2.5
            self.Kd = 0.5
            self.Ki = 0.0
            cutoff = 7.0

        else:
            start_angle = 0
            end_angle = 2.0
            # 5 -> 3 -> 2.5
            self.Kp = 2.5
            self.Kd = 0.5
            self.Ki = 0.0

            # self.Kp = 0.3
            # self.Kd = 0.25
            # self.Ki = 0.0
            cutoff = 7.0
        # self.Kp = 2.5
        # self.Kd = 0.5
        # self.Ki = 0.0
        # cutoff = 7.0
        # self.get_logger().info(f'Side: {self.SIDE}, Cutoff: {cutoff}, Kp: {self.Kp},   Kd: {self.Kd},   Ki: {self.Ki}')
        x, y, m, b = compute_least_squares_line(scan_msg, start_angle, end_angle, cutoff)
        VisualizationTools.plot_line(x, y, self.line_pub, frame="/laser")

        if m is None:
            self.get_logger().info('unable to compute line')
            return

        distance = abs(b)/math.sqrt(1 + m ** 2)

        if self.SIDE > 0:
            # Wall on the left: if measured distance > desired_distance, error is positive.
            error = distance - self.DESIRED_DISTANCE
        else:
            # Wall on the right: reverse the sign
            error = self.DESIRED_DISTANCE - distance


        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0.0:
            dt = 0.01

        self.integral_error += error * dt
        derivative_error = (error - self.prev_error) / dt
        self.prev_error = error

        pid_output = self.Kp * error + self.Kd * derivative_error + self.Ki * self.integral_error

        steering_angle = pid_output

        drive_command = AckermannDriveStamped()
        drive_command.drive.speed = float(self.VELOCITY)
        # drive_command.drive.speed = 0.0

        drive_command.drive.acceleration = 0.0
        drive_command.drive.steering_angle = float(steering_angle)
        # drive_command.drive.steering_angle = 0.0
        drive_command.drive.steering_angle_velocity = 0.0

        self.drive_publisher_.publish(drive_command)
        # self.get_logger().info(
        #     f"Distance: {distance:.2f}, Error: {error:.2f}, Steering: {steering_angle:.2f}"
        # )

        msg = Float32()
        msg.data = error
        self.err_publisher_.publish(msg)

        # TODO: Write your callback functions here

    def parameters_callback(self, params):
        """
        DO NOT MODIFY THIS CALLBACK FUNCTION!

        This is used by the test cases to modify the parameters during testing.
        It's called whenever a parameter is set via 'ros2 param set'.
        """
        for param in params:
            if param.name == 'side':
                self.SIDE = param.value
                self.get_logger().info(f"Updated side to {self.SIDE}")
            elif param.name == 'velocity':
                self.VELOCITY = param.value
                self.get_logger().info(f"Updated velocity to {self.VELOCITY}")
            elif param.name == 'desired_distance':
                self.DESIRED_DISTANCE = param.value
                self.get_logger().info(f"Updated desired_distance to {self.DESIRED_DISTANCE}")
        return SetParametersResult(successful=True)


def main():
    rclpy.init()
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
