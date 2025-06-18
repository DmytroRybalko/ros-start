/*
add_two_ints_client.cpp

This node implements a ROS 2 service client for the "add_two_ints" service (example_interfaces::srv::AddTwoInts).

- The client creates a request with two integers (a and b) and sends it to the "add_two_ints" service.
- It waits for the server to respond with the sum.
- The client logs the request and the received result using RCLCPP_INFO.

Example:
If the client sends a=3 and b=5, it will receive sum=8 from the server and log "Result: 8".
*/

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
 
class AddTwoIntsClientNode : public rclcpp::Node
{
public:
    AddTwoIntsClientNode() : Node("add_two_ints_client") 
    {
        client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
    }

    void callAddTwoInts(int a, int b)
    {
        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server...");
        }

        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        client_->async_send_request(request,
                                    std::bind(&AddTwoIntsClientNode::callbackCallAddTwoInts, this, std::placeholders::_1));
    }
 
private:
    void callbackCallAddTwoInts(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future)
    {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Sum: %d", (int)response->sum);
    }

    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsClientNode>();
    node->callAddTwoInts(10, 5);
    node->callAddTwoInts(7, 17);
    node->callAddTwoInts(11, 23);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}