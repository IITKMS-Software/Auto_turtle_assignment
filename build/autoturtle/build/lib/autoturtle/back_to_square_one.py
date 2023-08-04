import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math 

class ControlTurtlesim(Node):
    def __init__(self):
        super().__init__('ControlTurtlesim')
        self.get_logger().info(" Press CTRL+c to stop moving the Turtle")
        self.cmd_vel = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.publish_twist)
        self.get_logger().info("Set rate 10Hz")
        self.move_cmd = Twist()
        self.vel = 2.0
        self.move_cmd.angular.z = 0.0
        self.current_side = 0

    def publish_twist(self):
        # if self.count % 4 == 0:
        #     self.move_cmd.linear.x = self.vel
        #     self.move_cmd.linear.y = 0.0
        # elif self.count % 4 == 1:
        #     self.move_cmd.linear.x = 0.0
        #     self.move_cmd.linear.y = self.vel
        # elif self.count % 4 == 2:
        #     self.move_cmd.linear.x = -self.vel
        #     self.move_cmd.linear.y = 0.0
        # else:
        #     self.move_cmd.linear.x = 0.0
        #     self.move_cmd.linear.y = -self.vel
        
        # self.cmd_vel.publish(self.move_cmd)

        # self.count += 1
        if self.current_side < 4:
            self.move_cmd.linear.x = self.vel
            self.cmd_vel.publish(self.move_cmd)
            self.current_side += 1

        elif self.current_side == 4:
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = math.pi / 2  # Approximately 90 degrees in radians
            self.cmd_vel.publish(self.move_cmd)
            self.move_cmd.angular.z = 0.0
            self.current_side = 0
        

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