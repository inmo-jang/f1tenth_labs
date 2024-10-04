import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from visualization_msgs.msg import Marker


class GapFollow(Node):
    def __init__(self):
        super().__init__('gap_follow_node')
        
        # Topics for publishing and subscribing
        self.lidarscan_topic = '/scan'
        self.drive_topic = '/drive'
        self.best_point_marker_topic = '/best_point_marker'
        self.bubble_marker_topic = '/bubble_point_marker'
        
        # Create subscribers and publishers
        self.lidar_sub = self.create_subscription(LaserScan, self.lidarscan_topic, self.scan_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.best_point_marker_pub = self.create_publisher(Marker, self.best_point_marker_topic, 10)
        self.bubble_marker_pub = self.create_publisher(Marker, self.bubble_marker_topic, 10)
      
        
        # TODO: Define additional variables if necessary

        


    def publish_best_point_marker(self, best_point_idx, best_point_distance, angle_min, angle_increment):
        """ Publish a visualization marker for the best point in Rviz """
        # Calculate angle of best point
        best_point_angle = angle_min + best_point_idx * angle_increment

        # Convert polar coordinates to Cartesian for visualization
        x = best_point_distance * np.cos(best_point_angle)
        y = best_point_distance * np.sin(best_point_angle)

        # Create the Marker message
        marker = Marker()
        marker.header.frame_id = "ego_racecar/laser"  # Set the reference frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "best_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0  # LiDAR is in 2D plane, so z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Scale the marker (make it a small sphere)
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        # Set the color of the marker
        marker.color.a = 1.0  # Alpha channel
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        # Publish the Marker
        self.best_point_marker_pub.publish(marker)
        
    def publish_closest_bubble_marker(self, bubble_idx, bubble_distance, bubble_radius, angle_min, angle_increment):
        """ Publish a visualization marker for the closest bubble point in Rviz """
        # Calculate angle of best point
        bubble_point_angle = angle_min + bubble_idx * angle_increment

        # Convert polar coordinates to Cartesian for visualization
        x = bubble_distance * np.cos(bubble_point_angle)
        y = bubble_distance * np.sin(bubble_point_angle)

        # Create the Marker message
        marker = Marker()
        marker.header.frame_id = "ego_racecar/laser"  # Set the reference frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "bubble_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0  # LiDAR is in 2D plane, so z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Scale the marker (make it a small sphere)
        marker.scale.x = bubble_radius 
        marker.scale.y = bubble_radius 
        marker.scale.z = bubble_radius 

        # Set the color of the marker
        marker.color.a = 0.5  # Alpha channel
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        # Publish the Marker
        self.bubble_marker_pub.publish(marker)
                


    def scan_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message """
        ranges = np.array(data.ranges)



        # TODO: Preprocess LiDAR scan ranges                
                
        # TODO: Add safety bubbles at disparities

        # TODO: Find the best point (lidar index)
        best_point = 540
        best_point_distance = 2.0        
        
        # TODO: Set the steering angle        
        steering_angle = 0.0

        # Publish the drive command
        self.reactive_control(steering_angle)
        
        # (For Debug)
        # Publish the best point as a Marker                       
        self.publish_best_point_marker(best_point, best_point_distance, data.angle_min, data.angle_increment)
        
        
        # Publish the closest bubble as a Marker                 
        bubble_index = 540 
        bubble_distance = 1.0 
        bubble_radius = 0.5
        self.publish_closest_bubble_marker(bubble_index, bubble_distance, bubble_radius, data.angle_min, data.angle_increment)
        

    def reactive_control(self, steering_angle):
        """ Calculate and publish the steering angle and speed based on the best gap angle """
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering_angle

        # TODO: Determine speed if necessary
        drive_msg.drive.speed = 0.0

        # Publish the message
        self.drive_pub.publish(drive_msg)
        self.get_logger().info(f"Steering Angle: {drive_msg.drive.steering_angle*180/np.pi}, Speed: {drive_msg.drive.speed}")


def main(args=None):
    rclpy.init(args=args)
    print("Reactive Gap Follower Node Initialized")
    reactive_node = GapFollow()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
