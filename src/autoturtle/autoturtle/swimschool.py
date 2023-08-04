import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute
import random

class ControlTurtlesim(Node):
    def __init__(self):
        super().__init__('ControlTurtlesim')
        self.get_logger().info(" Press CTRL+c to stop moving the Turtle")
        self.cmd_vel = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(1, self.publish_twist)
        self.get_logger().info("Set rate 10Hz")
        self.move_cmd = Twist()
        self.move_cmd.linear.x = 1.2
        self.move_cmd.angular.z = 1.0
        self.count = 0
        self.teleport()

    def publish_twist(self):
        self.count += 1
        self.cmd_vel.publish(self.move_cmd)
        if(self.count % 126 == 0):
            self.move_cmd.angular.z *= -1
        

    def shutdown(self):
        self.get_logger().info("Stopping the turtle")
        stop_cmd = Twist()
        self.cmd_vel.publish(stop_cmd)
        self.get_logger().info("End of the swim for this Turtle.")

    def teleport(self):
        teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        request = TeleportAbsolute.Request()
        request.x = float(random.randrange(0, 11))
        request.y = float(random.randrange(0, 11))
        future = teleport_client.call_async(request)


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