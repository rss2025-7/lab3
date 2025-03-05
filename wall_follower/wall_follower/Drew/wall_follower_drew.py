#!/usr/bin/env python3
import numpy as np
import rclpy
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
        # DO NOT MODIFY THIS! 
        self.declare_parameter("scan_topic", "default")
        self.declare_parameter("drive_topic", "default")
        self.declare_parameter("side", "default")
        self.declare_parameter("velocity", "default")
        self.declare_parameter("desired_distance", "default")

        # Fetch constants from the ROS parameter server
        # DO NOT MODIFY THIS! This is necessary for the tests to be able to test varying parameters!
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value
		
        # This activates the parameters_callback function so that the tests are able
        # to change the parameters during testing.
        # DO NOT MODIFY THIS! 
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.Kp = 10
        self.Ki = 0.5
        self.Kd = 1.3

     
        self.error_prev = 0.0
        self.error_integral = 0.0
        self.last_time = self.get_clock().now()
  
        # TODO: Initialize your publishers and subscribers here
        self.publisher = self.create_publisher(AckermannDriveStamped, self.get_parameter('drive_topic').get_parameter_value().string_value, 10)
        self.subscriber = self.create_subscription(LaserScan, self.get_parameter('scan_topic').get_parameter_value().string_value, 
                                                 self.listener_callback, 10) 

    

        # TODO: Write your callback functions here    

    def listener_callback(self, scan):
        ### Reading LaserScan
        time = scan.header.stamp
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment
        ranges = np.array(scan.ranges)
        angles = angle_min + np.arange(len(ranges)) * angle_increment
        x0,y0 = ranges*np.cos(angles), ranges*np.sin(angles)

        ###Slicing Ranges according to obstacles and wall side
        if self.SIDE == 1:
            valid_side = y0 > 0
        elif self.SIDE == -1:
            valid_side = y0 < 0
        valid_range = (ranges < 3)
        valid_x = x0 > -2

        indices = np.where(valid_side & valid_range & valid_x)
        
        seen_ranges = ranges[indices]
        if len(seen_ranges) == 0:
            valid_x = x0 > -2
            valid_range = (ranges < 20)
            indices = np.where(valid_side & valid_range & valid_x)
            seen_ranges = ranges[indices]

        xwall = seen_ranges * np.cos(angles[indices])
        ywall = seen_ranges * np.sin(angles[indices])

        ### Linear Regression
        Awall = np.vstack([xwall, np.ones(len(xwall))]).T
        m, b = np.linalg.lstsq(Awall, ywall, rcond=None)[0]

        x = xwall
        y = m*x + b
        wall_distance = self.compute_wall_distance(m,b)

        ### PID 
        error = self.DESIRED_DISTANCE - wall_distance

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # seconds
        self.last_time = current_time
      

        self.error_integral += error * dt
        derivative = (error - self.error_prev) / dt
        self.error_prev = error

        steering_angle = (self.Kp * error + self.Ki * self.error_integral + self.Kd * derivative)

        if self.SIDE == 1:
            steering_angle *= -1




        drivemsg = AckermannDriveStamped()
        drivemsg.header.stamp = self.get_clock().now().to_msg()
        drivemsg.header.frame_id = "base_link"

        drivemsg.drive.steering_angle = steering_angle
        drivemsg.drive.steering_angle_velocity = 0.0
        # drivemsg.drive.speed = self.get_parameter("velocity").get_parameter_value()
        drivemsg.drive.speed = self.VELOCITY
        drivemsg.drive.acceleration = 0.
        drivemsg.drive.jerk = 0.

        self.publisher.publish(drivemsg)
            
    
    def parameters_callback(self, params):
        """
        DO NOT MODIFY THIS CALLBACK FUNCTION!
        
        This is used by the test cases to modify the parameters during testing. 
        It's called whenever a parameter is set via 'ros2 param set'.
        """
        for param in params:
            if param.name == 'side':
                self.error_integral = 0.0
                self.error_prev = 0.0
                self.last_time = self.get_clock().now()
                self.SIDE = param.value
                self.get_logger().info(f"Updated side to {self.SIDE}")
            elif param.name == 'velocity':
                self.VELOCITY = param.value
                self.get_logger().info(f"Updated velocity to {self.VELOCITY}")
            elif param.name == 'desired_distance':
                self.DESIRED_DISTANCE = param.value
                self.get_logger().info(f"Updated desired_distance to {self.DESIRED_DISTANCE}")
        return SetParametersResult(successful=True)
    
    def compute_wall_distance(self, m, b):
        A, B, C = m, -1, b
        return np.abs(C)/np.sqrt((A**2) + (B**2))



def main():
    rclpy.init()
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    