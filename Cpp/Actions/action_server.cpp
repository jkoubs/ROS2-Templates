#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

class NavigateToPoseActionServer : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;

    NavigateToPoseActionServer()
        : Node("navigate_to_pose_action_server"),
          action_server_(rclcpp_action::create_server<NavigateToPose>(
              this, "navigate_to_pose", std::bind(&NavigateToPoseActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
              std::bind(&NavigateToPoseActionServer::handle_cancel, this, std::placeholders::_1),
              std::bind(&NavigateToPoseActionServer::handle_accepted, this, std::placeholders::_1)))
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "navigation_goal", 10, std::bind(&NavigateToPoseActionServer::goal_callback, this, std::placeholders::_1));
    }

private:
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received navigation goal: [%f, %f, %f]", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const NavigateToPose::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received navigation goal request");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
    {
        using namespace std::placeholders;
        std::thread{std::bind(&NavigateToPoseActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
    {
        auto result = std::make_shared<NavigateToPose::Result>();
        RCLCPP_INFO(this->get_logger(), "Executing navigation to pose...");
        goal_handle->succeed(result);
    }

    rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigateToPoseActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
