#!/usr/bin/env python3
import numpy as np
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from rcl_interfaces.msg import SetParametersResult

from wall_follower.visualization_tools import VisualizationTools

class WallFollower(Node):

    def __init__(self):
        super().__init__("wall_follower")
        # Declare parameters to make them available for use
        self.declare_parameter("scan_topic", "default")
        self.declare_parameter("drive_topic", "default")
        self.declare_parameter("side", "default")
        self.declare_parameter("velocity", "default")
        self.declare_parameter("desired_distance", "default")
        self.declare_parameter("kP", 0.0)
        self.declare_parameter("kI", 0.0)
        self.declare_parameter("kD", 0.0)
        self.WALL_TOPIC = "/wall"

        # Fetch constants from the ROS parameter server
        # This is necessary for the tests to be able to test varying parameters!
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value

        # Initialize your publishers and subscribers here
        self.lidar_sub = self.create_subscription(
            LaserScan,
            self.SCAN_TOPIC,
            self.listener_callback,
            10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, 
            self.DRIVE_TOPIC,
            10)
        self.line_pub = self.create_publisher(Marker, self.WALL_TOPIC, 1)

        self.kP = self.get_parameter('kP').get_parameter_value().double_value
        self.kI = self.get_parameter('kI').get_parameter_value().double_value
        self.kD = self.get_parameter('kD').get_parameter_value().double_value
        self.prev_error = 0
        self.error = 0
        self.integral = 0
        self.derivative = 0
        self.prev_time = time.perf_counter()

    def listener_callback(self, msg):
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value
        
        # Chop data to just the left or right half
        angles = np.linspace(start=msg.angle_min, 
                    stop=msg.angle_max, 
                    num=int(np.round((msg.angle_max-msg.angle_min)/msg.angle_increment+1)),
                    endpoint=True)
        useful_scans = np.array(msg.ranges)

        if self.SIDE == 1:
            in_range_ind = np.where((angles >= -np.pi/12) & (angles <= 1.4))
        else:
            in_range_ind = np.where((angles <= np.pi/12) & (angles >= -1.4))
        angles = angles[in_range_ind]
        useful_scans = useful_scans[in_range_ind]

        # Discard out-of-range data
        in_range_ind = np.where((useful_scans >= msg.range_min) & (useful_scans <= 0.8*msg.range_max))
        angles = angles[in_range_ind]
        useful_scans = useful_scans[in_range_ind]

        # Calculate distance from wall based on angle + range
        dist_from_wall = np.multiply(useful_scans, np.sin(angles)) # y
        dist_along_wall = np.multiply(useful_scans, np.cos(angles)) # x
        
        A = np.vstack([dist_along_wall, np.ones(len(dist_along_wall))]).T
        slope, offset = np.linalg.lstsq(A, dist_from_wall, rcond=None)[0]

        current_dist = abs(offset) / ((slope**2)+1.5)**(1/2)

        x_values = np.linspace(-0.5, 0.5, num=20)  # Generate evenly spaced x values between 0 and 1
        y_values = slope * x_values + offset
        VisualizationTools.plot_line(x_values, y_values, self.line_pub, frame="/laser", color=(0.0,0.0,1.0))

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "base_link"

        curr_time = time.perf_counter()
        delta_time = curr_time - self.prev_time
        self.prev_time = curr_time
        angle = self.PID_drive(current_dist, delta_time, self.SIDE) # radians
        drive_msg.drive.steering_angle = angle
        # self.get_logger().info(f"Steering angle: {angle}")
        drive_msg.drive.steering_angle_velocity = 0.0 # change as quick as possible
        drive_msg.drive.speed = self.VELOCITY # m/s
        drive_msg.drive.acceleration = 0.0 # change as quick as possible
        drive_msg.drive.jerk = 0.0 # change as quick as possible        

        self.drive_pub.publish(drive_msg)

    def PID_drive(self, current_dist, delta_time, dir):
        if dir == 1:
            self.error = - (self.DESIRED_DISTANCE - current_dist)
        else:
            self.error = self.DESIRED_DISTANCE - current_dist
        self.integral = self.integral + self.error * delta_time
        self.derivative = (self.error - self.prev_error) / delta_time
        delta = self.kP*self.error + self.kI*self.integral + self.kD*self.derivative

        if delta > np.pi/3:
            delta = np.pi/3
        if delta < -np.pi/3:
            delta = -np.pi/3

        # self.get_logger().info(f"Params [{self.kP}, {self.kD}, {self.kI}], curr {current_dist}, err {self.error}, int {self.integral}, der {self.derivative}, del {delta}, time {delta_time}, side={dir}")
        return delta
    
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
    