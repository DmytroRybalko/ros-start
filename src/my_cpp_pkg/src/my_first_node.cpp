#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("my_first_node"), counter_{0}
    {
        RCLCPP_INFO(this->get_logger(), "Hello, ROS 2!");
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MyNode::timerCallback, this));
    }
private:
    void timerCallback()
    {
        RCLCPP_INFO(this->get_logger(), "Timer callback executed %d times", counter_);
        counter_++;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
// This is a simple ROS 2 node that logs "Hello, ROS 2!" to the console.
