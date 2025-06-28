#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/led_panel_state.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"
 
class LedPanelNode : public rclcpp::Node 
{
public:
    LedPanelNode() : Node("led_panel_publisher")
    {
        pub_ = this->create_publisher<my_robot_interfaces::msg::LedPanelState>(
            "led_panel_state", 10);
        server_ = this->create_service<my_robot_interfaces::srv::SetLed>(
            "set_led_panel",
            std::bind(
                &LedPanelNode::callbackLedPanelState,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&LedPanelNode::publisherLedPanelState, this));
        RCLCPP_INFO(this->get_logger(), "Something about LED panel state");
    }
 
private:
    
    void publisherLedPanelState()
    {
        auto msg = my_robot_interfaces::msg::LedPanelState();
        msg.led_states = led_states;
        pub_->publish(msg);
    }

    void callbackLedPanelState(
        const my_robot_interfaces::srv::SetLed::Request::SharedPtr request,
        const my_robot_interfaces::srv::SetLed::Response::SharedPtr response)
    {
        // We need to read led_number and set state for that led_number
        // Check if led_number is valid
        if (request->led_number >= 0 && request->led_number <static_cast<int64_t>(led_states.size())) {
            led_states[request->led_number] = request->state;
            response->success = true;
        } else {
            response->success = false;
        }
    }
    
    rclcpp::Service<my_robot_interfaces::srv::SetLed>::SharedPtr server_;
    rclcpp::Publisher<my_robot_interfaces::msg::LedPanelState>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<int64_t> led_states = {0, 0, 0};
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LedPanelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}