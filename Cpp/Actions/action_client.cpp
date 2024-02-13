#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <chrono>

using namespace std::chrono_literals;

class NavigateToPoseActionClient : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    NavigateToPoseActionClient() : Node("navigate_to_pose_action_client"), action_client_(rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose"))
    {
        send_goal();
    }

private:
    void send_goal()
    {
        while (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Action server not available, waiting...");
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.pose.position.x = 1.0;
        goal_msg.pose.pose.position.y = 1.0;
        goal_msg.pose.pose.orientation.w = 1.0;

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.feedback_callback = std::bind(&NavigateToPoseActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

        auto future_goal_handle = action_client_->async_send_goal(goal_msg, send_goal_options);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_goal_handle) != rclcpp::executor::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send goal to action server");
            return;
        }

        goal_handle_ = future_goal_handle.get();
        if (!goal_handle_)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the action server");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Goal sent to action server");
    }

    void feedback_callback(GoalHandleNavigateToPose::SharedPtr goal_handle, const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Received feedback: Distance remaining - %.2f", feedback->current_distance_remaining);
    }

    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    GoalHandleNavigateToPose::SharedPtr goal_handle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigateToPoseActionClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
