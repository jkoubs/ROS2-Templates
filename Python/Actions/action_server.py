import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class NavigateToPoseActionServer(Node):

    def __init__(self):
        super().__init__('navigate_to_pose_action_server')
        self.action_server = ActionServer(self, NavigateToPose, 'navigate_to_pose', self.execute_callback)
        self.subscription = self.create_subscription(PoseStamped, 'navigation_goal', self.goal_callback, 10)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Received navigation goal request')
        feedback_msg = NavigateToPose.Feedback()
        feedback_msg.current_distance_remaining = 10.0

        # Simulate navigation progress
        while feedback_msg.current_distance_remaining > 0.0:
            feedback_msg.current_distance_remaining -= 1.0
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info('Navigating... Distance remaining: %.2f' % feedback_msg.current_distance_remaining)
            self.get_logger().info('Simulating navigation progress...')
            self.rate.sleep()

        result = NavigateToPose.Result()
        result.arrived = True
        self.get_logger().info('Navigation to pose completed successfully')
        goal_handle.succeed(result)

    def goal_callback(self, msg):
        self.get_logger().info('Received navigation goal: [%f, %f, %f]' % (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))


def main(args=None):
    rclpy.init(args=args)
    navigate_to_pose_action_server = NavigateToPoseActionServer()
    rclpy.spin(navigate_to_pose_action_server)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
