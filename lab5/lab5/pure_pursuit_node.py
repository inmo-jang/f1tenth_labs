#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np

from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math

import os
import csv

LOOKAHEAD_DISTANCE = 1.20
HEADING_THRESHOLD = np.radians(45)  # Define threshold in radians (±45 degrees here)

WAYPOINTS_FILENAME = 'waypoints.csv'
WAYPOINTS_INTERVAL = 100


PACKAGE_NAME = 'lab5'
waypoint_filepath = os.path.join(os.getcwd(), PACKAGE_NAME, WAYPOINTS_FILENAME)



class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        # Topics for publishing and subscribing
        self.drive_topic = '/drive'
        self.waypoints_marker_topic = '/waypoints_marker'
        self.target_marker_topic = '/target_marker'
        self.odom_topic = '/ego_racecar/odom'

        # Publisher for drive command
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        # Publisher for visualization markers
        self.waypoints_marker_pub = self.create_publisher(Marker, self.waypoints_marker_topic, 10)
        self.target_marker_pub = self.create_publisher(Marker, self.target_marker_topic, 10)

        # Subscriber to vehicle odometry
        self.create_subscription(Odometry, self.odom_topic, self.pose_callback, 10)

        # Load waypoints from CSV file
        self.load_waypoints(waypoint_filepath, WAYPOINTS_INTERVAL)
        
        # Initialize variables
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.heading_angle = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        
        # TODO: Define additional variables if necessary

    def load_waypoints(self, filename, interval=100):
        """Loads waypoints from a CSV file with a specified interval."""
        self.waypoints_x = []
        self.waypoints_y = []
        with open(filename, mode='r') as file:
            reader = csv.reader(file)
            for i, row in enumerate(reader):
                if i % interval == 0:  # Only load waypoints at the specified interval
                    x, y, heading, speed = map(float, row)
                    self.waypoints_x.append(x)
                    self.waypoints_y.append(y)
        self.get_logger().info("Loaded waypoints from file with interval: {}".format(interval))

    def pose_callback(self, odometry_msg):
        """Callback to process vehicle pose and publish control commands."""
        # Extract current position and heading
        self.pos_x = odometry_msg.pose.pose.position.x
        self.pos_y = odometry_msg.pose.pose.position.y
        q = odometry_msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.heading_angle = math.atan2(siny_cosp, cosy_cosp)


        # TODO - Step 1: Find the target goal point (self.target_x, self.target_y) with LOOKAHEAD_DISTANCE

        # TODO - Step 2: Calculate the steering angle and speed
        steering_angle = 0.0
        speed = 0.0
                
        # TODO - Step 3: Publish the speed and steering angle as AckermannDriveStamped()
        self.publish_drive(speed, steering_angle)
        self.publish_markers()



        


    def publish_drive(self, speed, steering_angle):
        """Publishes the drive command with speed and steering_angle."""
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg) 
        self.get_logger().info(f"Steering Angle: {steering_angle*180/np.pi}, Speed: {drive_msg.drive.speed}")
        

    def publish_markers(self):
        """Publishes markers for visualization in RViz."""
        # Publish marker for waypoints
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.color.a = 1.0
        marker.color.b = 1.0
        marker.points = [Point(x=x, y=y, z=0.0) for x, y in zip(self.waypoints_x, self.waypoints_y)]
        self.waypoints_marker_pub.publish(marker)

        # Publish marker for target waypoint
        target_marker = Marker()
        target_marker.header.frame_id = "map"
        target_marker.type = Marker.POINTS
        target_marker.action = Marker.ADD
        target_marker.scale.x = 0.2
        target_marker.scale.y = 0.2
        target_marker.color.a = 1.0
        target_marker.color.r = 1.0
        target_marker.points = [Point(x=self.target_x, y=self.target_y, z=0.0)]
        self.target_marker_pub.publish(target_marker)

def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)
    
    pure_pursuit_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
