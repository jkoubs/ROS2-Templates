import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class MoveUntilObstacle(Node):

    def __init__(self):
        super().__init__('move_until_obstacle')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription_ = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.subscription_  # prevent unused variable warning
        self.linear_vel = 0.2
        self.angular_vel = 0.0

    def scan_callback(self, msg):
        # Check if there is an obstacle within 1 meter ahead
        if msg.ranges[len(msg.ranges) // 2] < 1.0:
            self.linear_vel = 0.0
            self.angular_vel = 0.2  # Turn right if obstacle detected
            self.get_logger().info('Obstacle detected, turning right')
        else:
            self.linear_vel = 0.2
            self.angular_vel = 0.0
            self.get_logger().info('Moving forward')

    def send_cmd_vel(self):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = self.linear_vel
        cmd_vel_msg.angular.z = self.angular_vel
        self.publisher_.publish(cmd_vel_msg)
        self.get_logger().info('Publishing cmd_vel: Linear: %f, Angular: %f' % (self.linear_vel, self.angular_vel))


def main(args=None):
    rclpy.init(args=args)
    move_until_obstacle = MoveUntilObstacle()
    # Publish velocity commands every 0.1 second
    timer_period = 0.1
    timer = move_until_obstacle.create_timer(timer_period, move_until_obstacle.send_cmd_vel)
    rclpy.spin(move_until_obstacle)
    move_until_obstacle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
