import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import math


class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        # Topics for publishing and subscribing
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # Create subscribers and publishers
        self.lidar_sub = self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        # TODO: set PID gains
        # self.kp = 
        # self.kd = 
        # self.ki = 

        # TODO: set Desired distances to the walls
        # self.desired_distance_right = 
        
        # TODO: set Lookahead distance
        # self.lookahead_distance = 

        # History storage
        self.prev_error = 0.0
        self.integral = 0.0

        # Initial values
        self.prev_time = self.get_clock().now()
        

    def get_range(self, range_data, angle):
        """
        Return the corresponding range measurement at a given angle, handling NaNs and infinities.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle
        """

        # TODO: implement
        
        return 100.0  # Return a large number if NaN or inf

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall on the right side using two angles.

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """
        
        # TODO: implement
        
        error = 0.0
        return error

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control.

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """        
        current_time = self.get_clock().now()
        delta_time = (current_time - self.prev_time).nanoseconds / 1e9  # Convert to seconds

        # TODO: Get error derivative & error integral

        # TODO: Compute steering angle using PID control with kp, ki, kd
        steering_angle = 0.0
        
        # TODO: Adjust driving speed depending on the steering angle
        velocity = velocity
        

        # Publish the steering angle and driving speed via the Ackermann drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

        # Update previous error and time
        self.prev_error = error
        self.prev_time = current_time

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculates the error and publishes the drive message.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        error = self.get_error(msg, self.desired_distance_right)
        velocity = 1.0  # Default velocity
        self.pid_control(error, velocity)


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
