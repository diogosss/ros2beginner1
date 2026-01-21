#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"
#include "my_robot_interfaces/msg/led_panel_state.hpp"
#include <cinttypes>

using namespace std::placeholders;
using namespace std::chrono_literals;

class LedPanelServerNode : public rclcpp::Node  
{
public:
    LedPanelServerNode() : Node("led_panel_server")   
    {
        server_ = this-> create_service<my_robot_interfaces::srv::SetLed>(
            "set_led"
            ,std::bind(&LedPanelServerNode::callbackSetLed,
            this,
            _1,
            _2
            ));

        publisher_ = this->create_publisher<my_robot_interfaces::msg::LedPanelState>("led_panel_state",10);

        RCLCPP_INFO(this->get_logger(), "Set Led server c++ has been started");
    }
private:

    void callbackSetLed(
        const  my_robot_interfaces::srv::SetLed::Request::SharedPtr request,
        const my_robot_interfaces::srv::SetLed::Response::SharedPtr response
    )
        {
            this->publishLedPanelSate(request->led_number,request->state);
            response->success = true;
            RCLCPP_INFO(this->get_logger(), "Received Led: %" PRIi64"  State: %d", request->led_number, request->state);
        }

    void publishLedPanelSate(int64_t ledNumber, bool state)
    {
        std::stringstream mess;
        mess << "Data received: ";
        mess << "Led: " << ledNumber;
        mess << " ,State: "<< std::boolalpha << state;
        std::string mensaje = mess.str();

        auto msg = my_robot_interfaces::msg::LedPanelState();
        msg.message = mensaje;
        publisher_->publish(msg);
    }

    rclcpp::Service<my_robot_interfaces::srv::SetLed>::SharedPtr server_;
    rclcpp::Publisher<my_robot_interfaces::msg::LedPanelState>::SharedPtr publisher_;

};

int main(int argsc, char **argv){
    rclcpp::init(argsc, argv);
    auto node = std::make_shared<LedPanelServerNode>();  
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}