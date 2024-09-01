#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool

class SafetyNode(Node):

    def __init__(self):
        super().__init__('safety_node')
        
        self.brake_publisher = self.create_publisher(Bool, '/brake_bool', 10)
        # TODO: Add self.drive_publisher -- AckermannDriveSamped 라는 ROS message를 이용하여 /drive 라는 topic을 출력하는 publisher를 만들기
        
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        # TODO: Add self.odom_subscriber -- /ego_racecar/odom 를 subscribe해서 ego vehicle의 longitudinal speed 값을 self.speed 에 계속 업데이트. 이때 odom_callback()을 정의해서 할 것. 
        
        self.lidar_data = None
        self.speed = 0.0

        self.timer = self.create_timer(0.01, self.check_ttc)
        
    def odom_callback(self, odom_msg):
        # TODO: update current speed
        pass        

    def scan_callback(self, scan_msg):
        self.lidar_data = scan_msg

    def check_ttc(self):
        # TODO: calculate TTC
        
        # TODO: publish commannd to brake if necessary           
            
        # self.publish_stop_drive()
        self.publish_brake(True)
        pass

    def publish_brake(self, should_brake):
        brake_msg = Bool()
        brake_msg.data = should_brake
        self.brake_publisher.publish(brake_msg)
        
    def publish_stop_drive(self):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = 0.0
        self.drive_publisher.publish(drive_msg)        
        

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    print("Run safety_node")
    rclpy.spin(safety_node)
    
    # Cleanup
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()