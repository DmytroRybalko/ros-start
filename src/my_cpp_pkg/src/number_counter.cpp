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
*/

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
 
class NumberCounterNode : public rclcpp::Node
{
public:
    NumberCounterNode() : Node("number_counter"), counter_(0)
    {
        counter_publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);
        number_subscriber_ = this->create_subscription<example_interfaces::msg::Int64>(
            "number", 10, std::bind(&NumberCounterNode::callbackNumber, this, std::placeholders::_1));
    }
 
private:
    void callbackNumber(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        counter_ += msg->data;
        auto newMsg = example_interfaces::msg::Int64();
        newMsg.data = counter_;
        counter_publisher_->publish(newMsg);
    }

    int counter_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr counter_publisher_;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr number_subscriber_;
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}