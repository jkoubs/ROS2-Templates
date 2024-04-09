import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityPublisher(Node):

    def __init__(self):
        super().__init__('velocity_publisher') # invoke the constructor of the parent class
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.1  # Linear velocity in the x-axis (forward direction)
        msg.angular.z = 0.1  # Angular velocity in the z-axis (rotational motion)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: Linear: %f, Angular: %f' % (msg.linear.x, msg.angular.z))
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    velocity_publisher = VelocityPublisher()
    rclpy.spin(velocity_publisher)
    velocity_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
