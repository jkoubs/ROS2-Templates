import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LaserScanSubscriber(Node):

    def __init__(self):
        super().__init__('laser_scan_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.subscription  # prevent unused variable warning

    def scan_callback(self, msg):
        self.get_logger().info('Received LaserScan Data\n'
                               'Range: %f\n'
                               'Angle Min: %f\n'
                               'Angle Max: %f\n'
                               'Time Increment: %f\n'
                               'Scan Time: %f\n'
                               'Range Min: %f\n'
                               'Range Max: %f' %
                               (msg.ranges[0], msg.angle_min, msg.angle_max,
                                msg.time_increment, msg.scan_time,
                                msg.range_min, msg.range_max))


def main(args=None):
    rclpy.init(args=args)
    laser_scan_subscriber = LaserScanSubscriber()
    rclpy.spin(laser_scan_subscriber)
    laser_scan_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
