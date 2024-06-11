""" This program publishes the radius and center of the detected ball   

    The radius will be zero if there is no detected object  

    published topics:  

        /processed_img [Image] 

    subscribed topics:  

        /robot/camera1/image_raw    [Image]  

"""  

import rclpy 

from rclpy.node import Node 

import cv2 

import numpy as np 

from cv_bridge import CvBridge 

from sensor_msgs.msg import Image 

from geometry_msgs.msg import Twist 

  

class CVExample(Node): 

    def __init__(self): 

        super().__init__('ros_color_tracker') 

 

        self.bridge = CvBridge() 

  
        self.sub = self.create_subscription(Image, 'image_raw', self.camera_callback, 10) 
        #self.sub = self.create_subscription(Image, 'robot/camera1/image_raw', self.camera_callback, 10) 

        self.pub = self.create_publisher(Image, 'processed_img', 10) 

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10) 

 

        self.kw = 0.002 

        self.kv = 0.004  

        self.max_radius = 200 #[pixels] 

        self.robot_vel = Twist() 

         

        self.image_received_flag = False #This flag is to ensure we received at least one image  

        dt = 0.1 

        self.timer = self.create_timer(dt, self.timer_callback) 

        self.get_logger().info('ros_color_tracker Node started') 

  

    def camera_callback(self, msg): 

        try:  

            # We select bgr8 because its the OpenCV encoding by default  

            self.cv_img= self.bridge.imgmsg_to_cv2(msg, "bgr8")  

            self.image_received_flag = True  

        except: 

            self.get_logger().info('Failed to get an image') 

  

  

    def timer_callback(self):
        if self.image_received_flag:
            # Create a copy of the image
            image = self.cv_img.copy()
            xcenter = image.shape[1] // 2  # image width / 2

            # Convert the image to HSV color space
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # Define HSV ranges for each color
            # Blue

            # Red
            min_red = np.array([0, 100, 100])
            max_red = np.array([10, 255, 255])
            # Yellow
            min_yellow = np.array([20, 100, 100])
            max_yellow = np.array([30, 255, 255])

            # Green
            min_green = np.array([35, 100, 100])
            max_green = np.array([85, 255, 255])

            # Create masks for each color range
            mask_red = cv2.inRange(hsv, min_red, max_red)
            mask_yellow = cv2.inRange(hsv, min_yellow, max_yellow)
            mask_green = cv2.inRange(hsv, min_green, max_green)

            # Find contours in each mask
            contours_red, _ = cv2.findContours(mask_red.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours_yellow, _ = cv2.findContours(mask_yellow.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours_green, _ = cv2.findContours(mask_green.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Initialize center and radius
            x, y, radius = 0.0, 0.0, 0.0

            # Draw contours on the image
            # Blue contours

            # Red contours
            cv2.drawContours(image, contours_red, -1, (0, 0, 255), 2)  # Draw contours in red

            # Yellow contours
            cv2.drawContours(image, contours_yellow, -1, (0, 255, 255), 2)  # Draw contours in yellow

            # Green contours
            cv2.drawContours(image, contours_green, -1, (0, 255, 0), 2)  # Draw contours in green

            # Handle red mask: stop the robot if red color is detected
            if contours_red:
                # Stop the robot if red color is detected
                self.robot_vel.linear.x = 0.0  # Stop the robot
                self.robot_vel.angular.z = 0.0  # Stop rotation
                self.cmd_vel_pub.publish(self.robot_vel)
                self.get_logger().info("Red detected: stopping robot.")
                return  # Skip further processing as we are stopping the robot

            # Handle yellow mask: slow down the robot if yellow color is detected
            if contours_yellow:
                self.robot_vel.linear.x = 0.1  # Set lower linear speed
                self.robot_vel.angular.z = 0.0  # Stop rotation
                self.cmd_vel_pub.publish(self.robot_vel)
                self.get_logger().info("Yellow detected: slowing down.")
                # Continue processing as the robot is just slowing down, not stopping

            # Handle green mask: continue the robot's normal speed
            if contours_green:
                # Set linear and angular velocity based on green object detection
                for c in contours_green:
                    ((x, y), radius) = cv2.minEnclosingCircle(c)
                    if radius > 10:  # Only consider contours with a minimum radius
                        M = cv2.moments(c)
                        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                        d = xcenter - center[0]
                        r = self.max_radius - radius
                        w = self.kw * d
                        v = self.kv * r

                        # Set robot velocities
                        self.robot_vel.linear.x = v
                        self.robot_vel.angular.z = w

                        # Publish the velocities
                        self.cmd_vel_pub.publish(self.robot_vel)
                        self.get_logger().info(f"Green detected: x: {x}, y: {y}, radius: {radius}")
                        break  # If green is found, process it and break from loop

            # Publish the processed image with contours drawn
            self.pub.publish(self.bridge.cv2_to_imgmsg(image, 'bgr8'))

      

def main(args=None): 

    rclpy.init(args=args) 

    cv_e = CVExample() 

    rclpy.spin(cv_e) 

    cv_e.destroy_node() 

    rclpy.shutdown() 

  

if __name__ == '__main__': 

    main() 