#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class LedBatteryClientNode : public rclcpp::Node  
{
public:
    LedBatteryClientNode() : Node("led_battery_client")   
    {
        client_ = this->create_client<my_robot_interfaces::srv::SetLed>("set_led");
    }

    void callSetLed(int ledNumber, bool state)
    {
        while (!client_->wait_for_service(1s))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server...");
        }

        auto request = std::make_shared<my_robot_interfaces::srv::SetLed::Request>();
        request->led_number = ledNumber;
        request->state = state;

        client_ -> async_send_request(request, std::bind(&LedBatteryClientNode::callbackCallSetLeds, this, _1));
    }

private:

    void callbackCallSetLeds(rclcpp::Client<my_robot_interfaces::srv::SetLed>::SharedFuture future)
    {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Response: %d", response->success);
    }

    rclcpp::Client<my_robot_interfaces::srv::SetLed>::SharedPtr client_;

};

int main(int argsc, char **argv){
    rclcpp::init(argsc, argv);
    auto node = std::make_shared<LedBatteryClientNode>();  
    node -> callSetLed(10,true);
    node -> callSetLed(56,false);
    node -> callSetLed(34,true);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}