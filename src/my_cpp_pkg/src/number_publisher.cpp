/*
This node publishes an integer value to the "number" topic (example_interfaces::msg::Int64) at a fixed interval.

- number_publisher_: Publishes integer messages to the "number" topic.
- number_timer_: Triggers the periodic publishing.
- number_: Stores the value to be published.

Every time the timer fires, publishNumber() creates a new Int64 message with the value of number_ and publishes it to "number".

Example:
If number_ is set to 2, this node will publish 2 on "number" every second.
*/

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
 
class NumberPublisherNode : public rclcpp::Node
{
public:
    NumberPublisherNode() : Node("number_publisher"), number_(2)
    {
        number_publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
        number_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&NumberPublisherNode::publishNumber, this));
        RCLCPP_INFO(this->get_logger(), "Number Publisher has been started");
    }
 
private:
    void publishNumber()
    {
        auto msg = example_interfaces::msg::Int64();
        msg.data = number_;
        number_publisher_->publish(msg);
    }

    int number_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr number_publisher_;
    rclcpp::TimerBase::SharedPtr number_timer_;
 
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisherNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
