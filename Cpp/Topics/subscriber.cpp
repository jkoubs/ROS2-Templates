#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LaserScanSubscriber : public rclcpp::Node
{
public:
    LaserScanSubscriber() : Node("laser_scan_subscriber")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LaserScanSubscriber::scan_callback, this, std::placeholders::_1));
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received LaserScan Data\n"
                                        "Range: %f\n"
                                        "Angle Min: %f\n"
                                        "Angle Max: %f\n"
                                        "Time Increment: %f\n"
                                        "Scan Time: %f\n"
                                        "Range Min: %f\n"
                                        "Range Max: %f",
                    msg->ranges[0], msg->angle_min, msg->angle_max,
                    msg->time_increment, msg->scan_time,
                    msg->range_min, msg->range_max);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserScanSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
