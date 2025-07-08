#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"
 
class RobotNewsStationNode : public rclcpp::Node
{
public:
    RobotNewsStationNode() : Node("robot_news_station")
    {
        this->declare_parameter("robot_name", "New_name");
        this->get_parameter("robot_name", robot_name_);
        publisher_ = this->create_publisher<example_interfaces::msg::String>("robot_news", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&RobotNewsStationNode::publish_news, this));
        RCLCPP_INFO(this->get_logger(), "Robot News Station has been started");
    }
 
private:
    void publish_news()
    {
        auto msg = example_interfaces::msg::String();
        msg.data = std::string("Hi, this is ") + robot_name_ + std::string(" from the Robot News Station!");
        publisher_->publish(msg);
    }

    std::string robot_name_;
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
 
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotNewsStationNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

// ros2 topic echo /robot_news
// ros2 node info /robot_news_station
// ros2 node list
// ros2 run my_cpp_pkg robot_news_station --ros-args -p robot_name:='ROS3' 