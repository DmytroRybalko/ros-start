#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/led_panel_state.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"
 
class BatteryNode : public rclcpp::Node
{
public:
    BatteryNode() : Node("battery"),
        battery_state_("full"),
        last_time_battery_state_changed_(this->now().seconds()) 
    {   
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(0.1),
            std::bind(&BatteryNode::checkBatteryState, this));

        client_ = this->create_client<my_robot_interfaces::srv::SetLed>("set_led_panel");
    }

    void checkBatteryState()
    {
        double time_now = this->now().seconds();
        //int random_led = std::rand() % 3;
        if (battery_state_ == "full")
        {
            if (time_now - last_time_battery_state_changed_ > 4.0)
            {
                battery_state_ = "empty";
                RCLCPP_INFO(this->get_logger(), "Battery is empty! Charging...");
                // Generate a random number between 0 and 2
                //random_led = std::rand() % 3;
                this->callSetLed(2, 1);
                last_time_battery_state_changed_ = time_now;
            }
        }
        else if (battery_state_ == "empty")
        {
            if (time_now - last_time_battery_state_changed_ > 6.0)
            {
                battery_state_ = "full";
                RCLCPP_INFO(this->get_logger(), "Battery is now full.");
                this->callSetLed(2, 0);
                last_time_battery_state_changed_ = time_now;
            }
        }
    }

    void callSetLed(int led_number, int state)
    {
        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server...");
        }

        auto request = std::make_shared<my_robot_interfaces::srv::SetLed::Request>();
        request->led_number = led_number;
        request->state = state;

        client_->async_send_request(request,
                                    std::bind(&BatteryNode::callbackBattery, this, std::placeholders::_1));
    }
 
private:
    void callbackBattery(rclcpp::Client<my_robot_interfaces::srv::SetLed>::SharedFuture future)
    {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Result: %s", (bool)response->success ? "LED turned on" : "LED not charged");
    }

    std::string battery_state_;
    double last_time_battery_state_changed_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<my_robot_interfaces::srv::SetLed>::SharedPtr client_;
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryNode>();
    //node->callSetLed(1,5);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}