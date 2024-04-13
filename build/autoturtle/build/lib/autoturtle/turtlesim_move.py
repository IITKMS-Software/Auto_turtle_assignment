import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ControlTurtlesim(Node):
    def __init__(self):
        super().__init__('ControlTurtlesim')
        self.get_logger().info(" Press CTRL+c to stop moving the Turtle")
        self.cmd_vel = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_twist)
        self.get_logger().info("Set rate 10Hz")
        self.move_cmd = Twist()
        self.move_cmd.linear.x = 2.4
        self.move_cmd.angular.z = 1.0

    def publish_twist(self):
        self.cmd_vel.publish(self.move_cmd)

    def shutdown(self):
        self.get_logger().info("Stopping the turtle")
        stop_cmd = Twist()
        self.cmd_vel.publish(stop_cmd)
        self.get_logger().info("End of the swim for this Turtle.")

def main():
    rclpy.init()
    control_turtlesim = ControlTurtlesim()
    try:
        rclpy.spin(control_turtlesim)
    except KeyboardInterrupt:
        control_turtlesim.get_logger().info("User interrupted with Ctrl+C.")
    finally:
        control_turtlesim.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()