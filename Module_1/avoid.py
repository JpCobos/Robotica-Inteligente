import rclpy  

from rclpy.node import Node  

from geometry_msgs.msg import Twist  

from sensor_msgs.msg import LaserScan  

import numpy as np  

  

class LaserScanSub(Node):  

    def __init__(self):  

        super().__init__('avoid_obstacles_1')  

        self.sub = self.create_subscription(LaserScan, "base_scan", self.lidar_cb, 10)  

        self.pub_cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)  

        self.lidar = LaserScan() # Data from lidar will be stored here.  

        timer_period = 10.0  

        self.timer = self.create_timer(timer_period, self.timer_callback)  

        self.robot_vel = Twist() #Robot velocity  

        self.get_logger().info("Node initialized!!!")  

  

    def timer_callback(self):  

        if self.lidar.ranges: #if we have data inside the lidar message  

            closest_range, closest_angle = self.get_closest_object()  

             

             

            # Fill the speed message and publish it  

            self.pub_cmd_vel.publish(self.robot_vel)  

  

    def lidar_cb(self, lidar_msg):  

        ## This function receives the ROS LaserScan message  

        self.lidar =  lidar_msg 

         

    def get_closest_object(self):  

        closest_range = 0 

        closest_angle = 0 

        return closest_range, closest_angle  

  

def main(args=None):  

    rclpy.init(args=args)  

    m_p=LaserScanSub()  

    rclpy.spin(m_p)  

    m_p.destroy_node()  

    rclpy.shutdown()  

   

if __name__ == '__main__':  

    main()  