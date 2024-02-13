#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class AddTwoIntsClient : public rclcpp::Node
{
public:
    AddTwoIntsClient() : Node("add_two_ints_client")
    {
        client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }
    }

    void send_request(int a, int b)
    {
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;
        auto future = client_->async_send_request(request);
        auto response_callback = [this](rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future) {
            if (future.wait_for(std::chrono::seconds(1)) == std::future_status::ready) {
                auto response = future.get();
                RCLCPP_INFO(this->get_logger(), "Sum: %ld", response->sum);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Service call failed");
            }
        };
        future.then(response_callback);
    }

private:
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsClient>();
    node->send_request(1, 2);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
