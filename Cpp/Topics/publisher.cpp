#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class VelocityPublisher : public rclcpp::Node
{
public:
    VelocityPublisher() : Node("velocity_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&VelocityPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 0.1;   // Linear velocity in the x-axis (forward direction)
        message.angular.z = 0.1;  // Angular velocity in the z-axis (rotational motion)
        RCLCPP_INFO(this->get_logger(), "Publishing: Linear: %f, Angular: %f", message.linear.x, message.angular.z);
        publisher_->publish(message);
        ++count_;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VelocityPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
