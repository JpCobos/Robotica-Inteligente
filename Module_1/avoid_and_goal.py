import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan
import numpy as np

class RobotControl(Node):
    def __init__(self, posiciones_objetivo):
        super().__init__('robot_control')
        self.sub_lidar = self.create_subscription(LaserScan, "base_scan", self.lidar_cb, 10)
        self.pub_cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)
        self.sub_pose = self.create_subscription(Pose2D, 'pose', self.pose_callback, 10)

        self.lidar = LaserScan()  # Data from lidar will be stored here.
        self.robot_vel = Twist()  # Robot velocity
        self.posiciones_objetivo = posiciones_objetivo
        self.indice_objetivo = 0
        self.objetivo_x, self.objetivo_y = self.posiciones_objetivo[self.indice_objetivo]

        # Obstacle avoidance parameters
        self.max_range = 15.0  # Maximum range before avoiding obstacles

        # Navigation parameters
        self.Kp_lineal = 0.1
        self.Kp_angular = 1
        self.velocidad_lineal_maxima = 0.2
        self.velocidad_angular_maxima = 0.1
        self.tolerancia_lineal = 0.2
        self.tolerancia_angular = 0.2

        self.objetivo_alcanzado = False
        self.mensaje_pose = Pose2D()

    def lidar_cb(self, lidar_msg):
        self.lidar = lidar_msg
        self.avoid_obstacle()

    def pose_callback(self, mensaje_pose):
        self.mensaje_pose = mensaje_pose
        self.navigate(self.mensaje_pose)

    def avoid_obstacle(self):
        if self.lidar.ranges:
            min_distance, closest_angle = self.get_closest_object()
            self.get_logger().info("Minimum Distance: %f" % min_distance)  # Log the minimum distance for debugging

            if min_distance < self.max_range:  # Only avoid obstacles if they are within the maximum range
                if min_distance < 2:  # Reduce the threshold distance for obstacle avoidance
                    # Calculate angular velocity based on the angle of the closest obstacle
                    if -1.57 <= closest_angle <= 1.57:  # Only consider obstacles within -90 to +90 degrees
                        if closest_angle < 0:
                            self.robot_vel.angular.z = 1.0  # Increase angular velocity for more aggressive turning
                        else:
                            self.robot_vel.angular.z = -1.0  # Increase angular velocity for more aggressive turning
                        # Stop linear motion
                        self.robot_vel.linear.x = 0.0
                    else:
                        # Continue moving forward if no obstacle is directly in front or sides
                        self.robot_vel.linear.x = 0.5
                        self.robot_vel.angular.z = 0.0
                else:
                    # Continue moving forward if no obstacle is within the maximum range
                    self.robot_vel.linear.x = 0.5
                    self.robot_vel.angular.z = 0.0

            self.pub_cmd_vel.publish(self.robot_vel)

    def navigate(self, mensaje_pose):
        self.mensaje_pose = mensaje_pose
        self.get_logger().info("Siguiendo Ruta")
        error_distancia = np.sqrt((self.objetivo_x - self.mensaje_pose.x) ** 2 + (self.objetivo_y - self.mensaje_pose.y) ** 2)
        error_ángulo = np.arctan2(self.objetivo_y - self.mensaje_pose.y, self.objetivo_x - self.mensaje_pose.x) - self.mensaje_pose.theta
        error_ángulo = np.arctan2(np.sin(error_ángulo), np.cos(error_ángulo))

        self.get_logger().info(f'Posición actual: x={mensaje_pose.x:.2f}, y={mensaje_pose.y:.2f}, theta={mensaje_pose.theta:.2f}')
        self.get_logger().info(f'Posición objetivo: x={self.objetivo_x:.2f}, y={self.objetivo_y:.2f}')
        self.get_logger().info(f'Error de distancia: {error_distancia:.2f} m, error de ángulo: {error_ángulo:.2f} rad')

        distancia_objetivo = np.sqrt(self.objetivo_x ** 2 + self.objetivo_y ** 2)
        if error_distancia <= 0.1 * distancia_objetivo:
            self.objetivo_alcanzado = True
            self.robot_vel.linear.x = 0.0
            self.robot_vel.angular.z = 0.0
            self.pub_cmd_vel.publish(self.robot_vel)
            self.get_logger().info(f'Objetivo completado')

            if self.indice_objetivo < len(self.posiciones_objetivo) - 1:
                self.indice_objetivo += 1
                self.objetivo_x, self.objetivo_y = self.posiciones_objetivo[self.indice_objetivo]
                self.objetivo_alcanzado = False
        else:
            self.robot_vel.linear.x = self.Kp_lineal * error_distancia
            self.robot_vel.angular.z = self.Kp_angular * error_ángulo

            self.robot_vel.linear.x = max(min(self.robot_vel.linear.x, self.velocidad_lineal_maxima), -self.velocidad_lineal_maxima)
            self.robot_vel.angular.z = max(min(self.robot_vel.angular.z, self.velocidad_angular_maxima), -self.velocidad_angular_maxima)

            self.pub_cmd_vel.publish(self.robot_vel)

    def get_closest_object(self):
        # Find the closest object detected by the LIDAR within a specific angular window in front and sides of the robot
        ranges_len = len(self.lidar.ranges)
        if ranges_len == 0:
            return float('inf'), 0.0

        # Define the angular window in radians for obstacles in front and sides (-90 degrees to +90 degrees)
        front_angle_window = np.pi / 2  # 90 degrees to each side of the front
        angle_min = -front_angle_window
        angle_max = front_angle_window

        # Get the indices that correspond to this angular window
        angle_increment = self.lidar.angle_increment
        min_index = int((angle_min - self.lidar.angle_min) / angle_increment)
        max_index = int((angle_max - self.lidar.angle_min) / angle_increment)

        # Get the range data within the angular window
        new_ranges = self.lidar.ranges[min_index:max_index]

        range_closest = min(new_ranges)
        index = new_ranges.index(range_closest)

        # Calculate the angle of the closest obstacle
        closest_angle = angle_min + (index * angle_increment)
        return range_closest, closest_angle

def main(args=None):
    rclpy.init(args=args)
    posiciones_objetivo = [(10.0, 0.0)]
    robot_control = RobotControl(posiciones_objetivo)
    rclpy.spin(robot_control)
    robot_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
