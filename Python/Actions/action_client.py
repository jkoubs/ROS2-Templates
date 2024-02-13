import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import time

class NavigateToPoseActionClient:

    def __init__(self):
        self.node = rclpy.create_node('navigate_to_pose_action_client')
        self.action_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')

    def send_goal(self):
        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().info('Action server not available, waiting...')
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = 1.0
        goal_msg.pose.pose.position.y = 1.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.node.get_logger().info('Sending navigation goal...')
        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        
        while not self.send_goal_future.done():
            time.sleep(1)

        if self.send_goal_future.result() is not None:
            self.node.get_logger().info('Goal response received')
        else:
            self.node.get_logger().info('Goal response not received')

    def feedback_callback(self, feedback_msg):
        self.node.get_logger().info('Received feedback: Distance remaining - %.2f' % feedback_msg.current_distance_remaining)

    def destroy(self):
        self.action_client.destroy()
        self.node.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    action_client = NavigateToPoseActionClient()
    action_client.send_goal()
    action_client.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
