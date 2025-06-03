#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
 
class NumberCounterNode : public rclcpp::Node
{
public:
    NumberCounterNode() : Node("number_counter")
    {
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("data", 12);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&NumberCounterNode::publish_data, this));
        subscriber_ = this->create_subscription<example_interfaces::msg::Int64>("data", 12,
                std::bind(&NumberCounterNode::callbackNumberPublisher, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Number counter has been started");
    }
 
private:
    void callbackNumberPublisher(const example_interfaces::msg::Int64::SharedPtr sub_msg)
    {
        RCLCPP_INFO(this->get_logger(), "Recived: %ld", sub_msg->data);
    }

    void publish_data()
    {
        auto pub_msg = example_interfaces::msg::Int64();
        pub_msg.data = 12; //sub_msg->data * 2;
        publisher_->publish(pub_msg);
    }

    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}