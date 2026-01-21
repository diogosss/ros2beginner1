#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"


using namespace std::placeholders;
using namespace std::chrono_literals;

class NumberCounterNode : public rclcpp::Node  
{
public:
    NumberCounterNode() : Node("number_counter"), counter_(0)   
    {
        //iniciar la suscripcion
        subscriber_ = this->create_subscription<example_interfaces::msg::Int64>(
            "number",
            10,
            std::bind(&NumberCounterNode::callbackNumberPublisher, this, _1));

        //iniciar publiser
        publisher_ = this -> create_publisher<example_interfaces::msg::Int64>("number_count",10);
        //timer_ = this -> create_wall_timer(0.5s, std::bind(&NumberCounterNode::publishCounter, this));  //publicar con timer

        //iniciar server
        server_ = this-> create_service<example_interfaces::srv::SetBool>(
            "reset_counter"
            ,std::bind(&NumberCounterNode::callbackResetCounter,
            this,
            _1,
            _2
            ));

        //logs        
        RCLCPP_INFO(this->get_logger(), "Number Counter Node has been started 2");
        
    }
private:

    void callbackNumberPublisher(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "%ld", msg->data);
        counter_++;
        //publicar cuando se llame a esta metodo
        auto new_msg = example_interfaces::msg::Int64();
        new_msg.data = std::int64_t(counter_);
        publisher_ -> publish(new_msg);

    }

    void publishCounter()
    {
        auto msg = example_interfaces::msg::Int64();
        msg.data = std::int64_t(counter_);
        publisher_ -> publish(msg);
        
    }

    void callbackResetCounter(
        const  example_interfaces::srv::SetBool::Request::SharedPtr request,
        const example_interfaces::srv::SetBool::Response::SharedPtr response
    )
        {            
            if(request->data)
            {
                RCLCPP_INFO(this->get_logger(), "Received True");
                response->message = "Received True";
                response->success = true;
                counter_ = 0;
            }else{
                RCLCPP_INFO(this->get_logger(), "Received False");
                response->message = "Received False";
                response->success = true;
            }
        }


    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;

    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;

    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr server_;



};

int main(int argsc, char **argv){
    rclcpp::init(argsc, argv);
    auto node = std::make_shared<NumberCounterNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}