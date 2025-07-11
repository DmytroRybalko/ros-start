/*
This node implements a ROS 2 service server for the "add_two_ints" service (example_interfaces::srv::AddTwoInts).

- The server waits for service requests on the "add_two_ints" topic.
- When a request is received, the callbackAddTwoInts() function is called.
- The callback adds the two integers from the request (a and b) and sets the result in the response (sum).
- The server logs each request and result using RCLCPP_INFO.

Example:
If a client sends a=3 and b=5, the server responds with sum=8 and logs "3 + 5 = 8".
*/

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
 
class AddTwoIntsServerNode : public rclcpp::Node
{
public:
    AddTwoIntsServerNode() : Node("add_two_ints_server")
    {
        server_ = this->create_service<example_interfaces::srv::AddTwoInts>(
            "add_two_ints",
            std::bind(
                &AddTwoIntsServerNode::callbackAddTwoInts,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );
        RCLCPP_INFO(this->get_logger(), "Add Two Ints Service has been started.");
    }
 
private:
    void callbackAddTwoInts(const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
                            const example_interfaces::srv::AddTwoInts::Response::SharedPtr response)
    {
        // Process request
        response->sum = request->a + request->b;
        RCLCPP_INFO(this->get_logger(), "%d + %d = %d",
                     (int)request->a, (int)request->b, (int)response->sum);
    } 

    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server_;
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}