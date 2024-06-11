import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np

class CombinedNode(Node):
    def __init__(self):
        super().__init__('combined_node')

        # Initialize publishers and subscribers
        self.bridge = CvBridge()
        self.sub_image = self.create_subscription(Image, 'image_raw', self.camera_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        self.pub = self.create_publisher(Image, 'processed_img', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Define goals (positions) as a list of dictionaries with x, y coordinates
        self.goals = [
            {'x': 1.0, 'y': 1.0},
            {'x': 1.0, 'y': -1.0},
            {'x': -1.0, 'y': -1.0},
            {'x': -1.0, 'y': 1.0},
        ]
        self.current_goal_index = 0

        # Robot velocity
        self.robot_vel = Twist()
        self.kw = 0.002
        self.kv = 0.004
        self.max_radius = 200  # [pixels]

        # PID control parameters
        self.kp_linear = 0.5  # Proportional coefficient for linear speed
        self.kp_angular = 1.0  # Proportional coefficient for angular speed
        
        # Flag to check if an image has been received
        self.image_received_flag = False

        # Create a timer to call the timer_callback method periodically
        dt = 0.1
        self.timer = self.create_timer(dt, self.timer_callback)
        self.get_logger().info('Combined Node initialized')

    def camera_callback(self, msg):
        try:
            # Convert the image from ROS format to OpenCV format
            self.cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.image_received_flag = True
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')

    def odometry_callback(self, msg):
        # Process the odometry data
        self.robot_position = msg.pose.pose.position
        self.robot_orientation = msg.pose.pose.orientation
        
        # Print the robot's current location (position and orientation)
        self.get_logger().info(f"Robot Location - x: {self.robot_position.x}, y: {self.robot_position.y}, z: {self.robot_position.z}")
        self.get_logger().info(f"Orientation - x: {self.robot_orientation.x}, y: {self.robot_orientation.y}, z: {self.robot_orientation.z}, w: {self.robot_orientation.w}")

    def timer_callback(self):
        # Combine processing of the image data, odometry data, and navigation logic
        self.process_image_and_navigate()

        # Publish the processed image
        if hasattr(self, 'cv_img'):
            self.pub.publish(self.bridge.cv2_to_imgmsg(self.cv_img, 'bgr8'))

    def process_image_and_navigate(self):
        # Ensure the cv_img attribute exists
        if hasattr(self, 'cv_img'):
            # Convert the image to HSV color space
            hsv = cv2.cvtColor(self.cv_img, cv2.COLOR_BGR2HSV)

            # Define HSV ranges for each color
            min_red = np.array([0, 100, 100])
            max_red = np.array([10, 255, 255])
            min_yellow = np.array([20, 100, 100])
            max_yellow = np.array([30, 255, 255])
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

            # Draw contours on the image
            cv2.drawContours(self.cv_img, contours_red, -1, (0, 0, 255), 2)  # Red
            cv2.drawContours(self.cv_img, contours_yellow, -1, (0, 255, 255), 2)  # Yellow
            cv2.drawContours(self.cv_img, contours_green, -1, (0, 255, 0), 2)  # Green

            # Handle color detection and navigation
            self.handle_colors_and_navigation(contours_red, contours_yellow, contours_green)

    def handle_colors_and_navigation(self, contours_red, contours_yellow, contours_green):
        # Check if we have odometry data
        if not hasattr(self, 'robot_position') or not hasattr(self, 'robot_orientation'):
            return

        # Get the current goal position
        current_goal = self.goals[self.current_goal_index]

        # Calculate the distance to the goal
        goal_x = current_goal['x']
        goal_y = current_goal['y']
        current_x = self.robot_position.x
        current_y = self.robot_position.y
        distance = np.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)

        # Calculate the angle to the goal
        angle_to_goal = np.arctan2(goal_y - current_y, goal_x - current_x)

        # If the robot is close to the goal
        if distance < 0.1:
            self.get_logger().info(f"Reached goal {self.current_goal_index}: x = {goal_x}, y = {goal_y}")

            # Increment the goal index to move to the next goal
            self.current_goal_index += 1

            # If all goals are reached, reset the goal index
            if self.current_goal_index >= len(self.goals):
                self.current_goal_index = 0

            # Stop the robot when all goals are completed
            self.robot_vel.linear.x = 0.0
            self.robot_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(self.robot_vel)
            self.get_logger().info("All goals completed! Robot has stopped.")

        else:
            # Calculate linear and angular velocities for navigation
            self.robot_vel.linear.x = self.kp_linear * distance
            self.robot_vel.angular.z = self.kp_angular * (angle_to_goal - self.calculate_robot_yaw())

            # Check for detected colors and adjust robot velocity if needed
            if contours_red:
                # Stop the robot if red is detected
                self.robot_vel.linear.x = 0.0
                self.robot_vel.angular.z = 0.0
                self.cmd_vel_pub.publish(self.robot_vel)
                self.get_logger().info("Red detected: stopping robot.")

            elif contours_yellow:
                # Slow down the robot if yellow is detected
                self.robot_vel.linear.x = 0.1
                self.robot_vel.angular.z = 0.0
                self.cmd_vel_pub.publish(self.robot_vel)
                self.get_logger().info("Yellow detected: slowing down.")

            elif contours_green:
                # Maintain normal speed if green is detected
                for c in contours_green:
                    ((x, y), radius) = cv2.minEnclosingCircle(c)
                    if radius > 10:
                        M = cv2.moments(c)
                        center_x = int(M["m10"] / M["m00"])
                        center_y = int(M["m01"] / M["m00"])

                        # Calculate the angle and distance errors
                        distance_error = np.sqrt((center_x - current_x)**2 + (center_y - current_y)**2)
                        angle_error = angle_to_goal - np.arctan2(center_y - current_y, center_x - current_x)

                        # Calculate new linear and angular velocities
                        v = self.kv * (self.max_radius - radius)
                        w = self.kw * angle_error

                        # Set the velocities
                        self.robot_vel.linear.x = v
                        self.robot_vel.angular.z = w

                        # Publish the velocities
                        self.cmd_vel_pub.publish(self.robot_vel)
                        self.get_logger().info(f"Green detected: x: {x}, y: {y}, radius: {radius}")

                        # Break the loop as we only need one green contour to take action
                        break
            else:
                # Publish the robot velocities for goal navigation
                self.cmd_vel_pub.publish(self.robot_vel)

    def calculate_robot_yaw(self):
        # Calculate the yaw (rotation around the z-axis) from the orientation quaternion
        q = self.robot_orientation
        yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return yaw

def main(args=None):
    rclpy.init(args=args)
    node = CombinedNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
