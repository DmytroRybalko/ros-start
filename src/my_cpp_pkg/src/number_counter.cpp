/*
This node subscribes to the "number" topic (example_interfaces::msg::Int64), 
keeps a running sum of all received numbers, and publishes the sum to the "number_count" topic.

- counter_publisher_: Publishes the running sum to "number_count".
- number_subscriber_: Subscribes to "number" and receives Int64 messages.
- counter_: Stores the running sum.

When a message is received on "number", callbackNumber() adds its data to counter_,
creates a new Int64 message with the updated sum, and publishes it to "number_count".

Example:
If messages 2, 3, 5 are received on "number", this node will publish 2, 5, 10 on "number_count".

How to run service reset_counter:
1) ros2 run my_cpp_pkg number_publisher
2) ros2 run my_cpp_pkg number_counter
3) ros2 topic echo /number_count
4) ros2 service call /reset_counter example_interfaces/srv/SetBool "{data: true}"
*/

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"
 
class NumberCounterNode : public rclcpp::Node
{
public:
    NumberCounterNode() : Node("number_counter"), counter_(0)
    {
        counter_publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);
        number_subscriber_ = this->create_subscription<example_interfaces::msg::Int64>(
            "number", 10, std::bind(&NumberCounterNode::callbackNumber, this, std::placeholders::_1));

        server_ = this->create_service<example_interfaces::srv::SetBool>(
            "reset_counter",
            std::bind(
                &NumberCounterNode::callbackResetCounter,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );
        RCLCPP_INFO(this->get_logger(), "Reset counter Service has been started.");
    }
 
private:
    void callbackNumber(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        counter_ += msg->data;
        auto newMsg = example_interfaces::msg::Int64();
        newMsg.data = counter_;
        counter_publisher_->publish(newMsg);
    }

    void callbackResetCounter(const example_interfaces::srv::SetBool::Request::SharedPtr request,
                              const example_interfaces::srv::SetBool::Response::SharedPtr response)
    {
        (void)request; //unused
        counter_ = 0;
        response->success = true;
        response->message = "Counter has been reset";
        RCLCPP_INFO(this->get_logger(), "Counter is reset to %d", counter_);
    }

    int counter_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr counter_publisher_;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr number_subscriber_;
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr server_;
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}