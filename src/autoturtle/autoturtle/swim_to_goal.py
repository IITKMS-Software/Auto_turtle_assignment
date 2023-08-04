import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class ControlTurtlesim(Node):
	def __init__(self, x, y, tol):
		super().__init__('ControlTurtlesim')
		self.get_logger().info(" Press CTRL+c to stop moving the Turtle")

		self.cmd_vel = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
		self.subscriber = self.create_subscription(Pose, '/turtle1/pose', self.callback, 100)

		self.timer = self.create_timer(0.2, self.publish_twist)
		self.pose = Pose()
		
		self.x = x
		self.y = y
		self.tol = tol
   
	def callback(self, data):
		self.pose = data
		self.pose.x = round(self.pose.x, 4)
		self.pose.y = round(self.pose.y, 4)
		self.pose.theta = round(self.pose.theta, 2)
		
	def publish_twist(self):
		msg = Twist()
		emp = Twist()

		self.get_logger().info(f"pose  {self.pose}")

		if self.dist()<self.tol:
			self.cmd_vel.publish(emp)
		else:
			msg.linear.x = 0.5 * self.dist()
			msg.angular.z= 2.5 * (self.angle() - self.pose.theta)
			self.cmd_vel.publish(msg)
			self.get_logger().info(f"Twist: Linear={msg.linear}, Angular={msg.angular}")
			self.get_logger().info(f"Angle: {self.angle()*180/math.pi}")
   
	def dist(self):
		return math.sqrt((self.pose.x - self.x)**2+ (self.pose.y - self.y)**2)
	
	def angle(self):
		y = (self.y - self.pose.y)
		x = (self.x - self.pose.x)
		return math.atan2(y,x)
	
def main():
	rclpy.init()
	x = float(input("Enter x: "))
	y = float(input("Enter y: "))
	tol = float(input("Enter tolerance: "))
	control_turtlesim = ControlTurtlesim(x, y, tol)

	try:
		rclpy.spin(control_turtlesim)
	except KeyboardInterrupt:
		control_turtlesim.get_logger().info("User interrupted with Ctrl+C.")
	finally:
		control_turtlesim.shutdown()
		rclpy.shutdown()

if __name__ == '__main__':
	main()
	