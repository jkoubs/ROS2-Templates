#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class MoveUntilObstacle : public rclcpp::Node
{
public:
    MoveUntilObstacle() : Node("move_until_obstacle")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&MoveUntilObstacle::scan_callback, this, std::placeholders::_1));

        linear_vel_ = 0.2;
        angular_vel_ = 0.0;
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Check if there is an obstacle within 1 meter ahead
        if (msg->ranges[msg->ranges.size() / 2] < 1.0)
        {
            linear_vel_ = 0.0;
            angular_vel_ = 0.2; // Turn right if obstacle detected
            RCLCPP_INFO(this->get_logger(), "Obstacle detected, turning right");
        }
        else
        {
            linear_vel_ = 0.2;
            angular_vel_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "Moving forward");
        }
        send_cmd_vel();
    }

    void send_cmd_vel()
    {
        auto cmd_vel_msg = geometry_msgs::msg::Twist();
        cmd_vel_msg.linear.x = linear_vel_;
        cmd_vel_msg.angular.z = angular_vel_;
        publisher_->publish(cmd_vel_msg);
        RCLCPP_INFO(this->get_logger(), "Publishing cmd_vel: Linear: %f, Angular: %f", linear_vel_, angular_vel_);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    double linear_vel_;
    double angular_vel_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto move_until_obstacle = std::make_shared<MoveUntilObstacle>();
    rclcpp::spin(move_until_obstacle);
    rclcpp::shutdown();
    return 0;
}
