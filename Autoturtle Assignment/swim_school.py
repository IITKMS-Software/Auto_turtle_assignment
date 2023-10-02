import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class TurtleControl(Node):
    def __init__(self):
        super().__init__('turtle_control')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer_ = self.create_timer(1.0, self.publish_twist)
        self.phase = 0
        self.linear_vel= float(input("linear velocity: "))
        self.angular_vel= float(input("angular velocity: "))

    def publish_twist(self):
        twist_msg = Twist()
        
        if self.phase < 1:
            twist_msg.linear.x = self.linear_vel # Forward motion
            twist_msg.angular.z = self.angular_vel # Turning right
        else:
            twist_msg.linear.x = self.linear_vel  # Forward motion
            twist_msg.angular.z = -self.angular_vel  # Turning left
        
        self.publisher_.publish(twist_msg)

        self.phase += 1
        if self.phase > 1:
            self.phase = 0

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
