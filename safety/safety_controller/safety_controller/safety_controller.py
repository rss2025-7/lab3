#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan

class SafetyController(Node):

    def __init__(self):
        super().__init__("safety_controller")
        # Declare parameters to make them available for use
        self.declare_parameter("cmd_topic", "default")
        self.declare_parameter("laser_topic", "default")
        self.declare_parameter("drive_topic", "default")

        # Fetch constants from the ROS parameter server
        self.CMD_TOPIC = self.get_parameter('cmd_topic').get_parameter_value().string_value
        self.LASER_TOPIC = self.get_parameter('laser_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value

        self.cmd_sub = self.create_subscription(
            AckermannDriveStamped,
            self.CMD_TOPIC,
            self.listener_callback,
            10)
        self.lidar_sub = self.create_subscription(
            LaserScan,
            self.LASER_TOPIC,
            self.laser_callback,
            10)
        self.safety_pub = self.create_publisher(
            AckermannDriveStamped,
            self.DRIVE_TOPIC,
            10)

        self.prev_cmd = None
        self.angle_memory = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.weights = [0.1, 0.2, 0.3, 0.4, 0.5]

        # Data to be saved by laser_callback()
        self.angles = np.array([])
        self.ranges = np.array([])

        # PARAMETERS TODO CAN CHANGE
        # Delta time, assumed increment for predicting where robot will be
        # 0.5 -> 0.25 -> 0.2
        self.DT = 0.25 # in seconds
        # Angle range, range of laserscan data we will scan over
        # rangle calculated as drive_command.steering_angle +/- ang_range
        # 0.2 -> 0.15 -> 0.1 -> 0.075 -> 0.15 -> .2 -> 0.1
        self.ang_range = 0.1 # in radians
        # Tolerance, car will stop if predicted distance from wall is <= tolerance
        self.dist_tolerance = 0.25 # in meters
        # Danger threshold, car will stop if this % of range data reads
        # within the dist_ range
        self.danger_threshold = 0.1 # 20 percent

    def laser_callback(self, msg):
        # Save most recent laser data
        self.angles = np.linspace(start=msg.angle_min,
            stop=msg.angle_max,
            num=int(np.round((msg.angle_max-msg.angle_min)/msg.angle_increment+1)),
            endpoint=True)

        self.ranges = np.array(msg.ranges)
        self.ranges = np.clip(self.ranges, a_min=msg.range_min, a_max=msg.range_max)

    def listener_callback(self, msg):
        if self.prev_cmd is not None:
            # self.get_logger().info(f"Entered callback")
            drive_ang = self.prev_cmd.steering_angle
            drive_speed = self.prev_cmd.speed

            self.angle_memory[:-1] = self.angle_memory[1:]
            self.angle_memory[-1] = drive_ang
            drive_ang_adj = np.sum(self.angle_memory*self.weights)

            predicted_dist = drive_speed * self.DT
            min_safe_dist = predicted_dist + self.dist_tolerance

            inds_to_check = np.where((self.angles >= drive_ang_adj - self.ang_range) &
                                    (self.angles <= drive_ang_adj + self.ang_range))
            ranges_to_check = self.ranges[inds_to_check]
            danger_rating = np.sum(ranges_to_check < min_safe_dist) / float(ranges_to_check.size)

            # self.get_logger().info(f"{self.danger_threshold}, {danger_rating}")
            if danger_rating > self.danger_threshold:
                # self.get_logger().info(f"STOPPED!")
                drive_msg = AckermannDriveStamped()
                drive_msg.header.stamp = self.get_clock().now().to_msg()
                drive_msg.header.frame_id = "base_link"

                drive_msg.drive.steering_angle = 0.0 # set everything else to 0
                drive_msg.drive.steering_angle_velocity = 0.0 # set everything else to 0
                drive_msg.drive.speed = 0.0 # STOP THE CAR
                drive_msg.drive.acceleration = 0.0 # set everything else to 0
                drive_msg.drive.jerk = 0.0 # set everything else to 0

                self.safety_pub.publish(drive_msg)
        self.prev_cmd = msg.drive

def main():
    rclpy.init()
    safety_controller = SafetyController()
    rclpy.spin(safety_controller)
    safety_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
