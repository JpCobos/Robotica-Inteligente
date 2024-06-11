import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class LaserScanSub(Node):

    def __init__(self):
        super().__init__('laser_scan_subscriber')

        self.subscription = self.create_subscription(LaserScan, "base_scan", self.lidar_cb, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)  # Initialize the publisher

        self.lidar_data = LaserScan()  # To store LIDAR data

        self.update_interval = 1.0
        self.timer = self.create_timer(self.update_interval, self.timer_callback)

        self.get_logger().info("LaserScanSub node has been started")

        self.turn_threshold = 1.0  # Distance to start turning
        self.stop_threshold = 0.5  # Distance to stop
        self.default_linear_speed = 0.2
        self.default_angular_speed = 0.5
        self.proportional_gain = 1.0  # Gain for proportional control
    
    def lidar_cb(self, msg):
        self.lidar_data = msg

    def timer_callback(self):
        scan_ranges = self.lidar_data.ranges
        min_angle = self.lidar_data.angle_min
        angle_step = self.lidar_data.angle_increment
        
        # Filter out invalid (non-finite) ranges
        valid_ranges = [r for r in scan_ranges if np.isfinite(r)]
        
        velocity_cmd = Twist()

        if valid_ranges:
            closest_distance = min(valid_ranges)
            closest_index = scan_ranges.index(closest_distance)
            closest_angle = min_angle + closest_index * angle_step
            
            self.get_logger().info(f"Closest distance: {closest_distance}")
            self.get_logger().info(f"Angle to closest object: {closest_angle}")
            
            # Case 2: Turn if within turning threshold
            if closest_distance < self.turn_threshold:
                velocity_cmd.linear.x = 0.0
                velocity_cmd.angular.z = self.proportional_gain * closest_angle

            # Case 4: Stop if within stopping threshold
            if closest_distance < self.stop_threshold:
                velocity_cmd.linear.x = 0.0
                velocity_cmd.angular.z = 0.0

            # Case 3: Move forward with proportional turning adjustment
            else:
                velocity_cmd.linear.x = self.default_linear_speed
                velocity_cmd.angular.z = self.proportional_gain * closest_angle
            
        # Case 1:No valid object detected
        else:
            self.get_logger().info("No objects detected in range.")
            velocity_cmd.linear.x = 0.0
            velocity_cmd.angular.z = 0.0

        self.publisher.publish(velocity_cmd)  # Publish the velocity command

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
