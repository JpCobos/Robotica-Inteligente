import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import Twist  
from std_msgs.msg import String  

class PubSubClass(Node): 

    def __init__(self): 
        super().__init__('pub_sub_with_classes') 
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10) 
        self.sub = self.create_subscription(String, "string_topic", self.listener_callback, 10) 
        timer_period = 0.5 
        self.timer = self.create_timer(timer_period, self.timer_callback) 
        self.get_logger().info("Node initialized!!!") 
        self.vel = Twist() 
        self.my_string = "stop" 
        self.sequence_state = 0
        self.sequence_timer = 0

    def timer_callback(self): 
        if self.my_string == "move forward": 
            self.get_logger().info("moving forward") 
            self.vel.linear.x = 0.1 
            self.vel.angular.z = 0.0 
        elif self.my_string == "move back": 
            self.get_logger().info("moving back") 
            self.vel.linear.x = -0.1 
            self.vel.angular.z = 0.0 
        elif self.my_string == "turn left": 
            self.get_logger().info("turning left") 
            self.vel.linear.x = 0.0 
            self.vel.angular.z = 0.1 
        elif self.my_string == "turn right": 
            self.get_logger().info("turning right") 
            self.vel.linear.x = 0.0 
            self.vel.angular.z = -0.1 
        elif self.my_string == "stop": 
            self.get_logger().info("stopped") 
            self.vel.linear.x = 0.0 
            self.vel.angular.z = 0.0 
        elif self.my_string == "complex movement":
            if self.sequence_state == 0:
                self.get_logger().info("Sequence: turning right")
                self.vel.linear.x = 0.0
                self.vel.angular.z = -0.1
                self.sequence_timer += 1
                if self.sequence_timer > 4:  # Adjust based on timing needed
                    self.sequence_timer = 0
                    self.sequence_state += 1
            elif self.sequence_state == 1:
                self.get_logger().info("Sequence: moving forward")
                self.vel.linear.x = 0.1
                self.vel.angular.z = 0.0
                self.sequence_timer += 1
                if self.sequence_timer > 4:  # Adjust based on timing needed
                    self.sequence_timer = 0
                    self.sequence_state += 1
            elif self.sequence_state == 2:
                self.get_logger().info("Sequence: turning left")
                self.vel.linear.x = 0.0
                self.vel.angular.z = 0.1
                self.sequence_timer += 1
                if self.sequence_timer > 4:  # Adjust based on timing needed
                    self.sequence_timer = 0
                    self.sequence_state += 1
            elif self.sequence_state == 3:
                self.get_logger().info("Sequence: moving back")
                self.vel.linear.x = -0.1
                self.vel.angular.z = 0.0
                self.sequence_timer += 1
                if self.sequence_timer > 4:  # Adjust based on timing needed
                    self.sequence_timer = 0
                    self.sequence_state += 1
            elif self.sequence_state == 4:
                self.get_logger().info("Sequence: stopped")
                self.vel.linear.x = 0.0
                self.vel.angular.z = 0.0
                self.sequence_state = 0  # Reset sequence
                self.my_string = "stop"  # Reset command to stop
        self.cmd_vel_pub.publish(self.vel)

    def listener_callback(self, msg): 
        self.my_string = msg.data 
        self.get_logger().info("I received this message in the callback: " + self.my_string) 

def main(args=None): 
    rclpy.init(args=args) 
    m_p = PubSubClass() 
    rclpy.spin(m_p) 
    m_p.destroy_node() 
    rclpy.shutdown() 

if __name__ == '__main__': 
    main()
