#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

using namespace std::chrono_literals;

class NumberPublisherNode : public rclcpp::Node 
{
public:
    NumberPublisherNode() : Node("number_publisher")
    {
        //declarar parametris
        this->declare_parameter("number",2);
        this->declare_parameter("timer_period",1.0);
        //usar 
        number_ = this->get_parameter("number").as_int();
        double timer_period_ = this->get_parameter("timer_period").as_double();

        number_pub_ = number_; //asignar a la q se estaba usando o simplificar qda como ejemplo

        publisher_ = this -> create_publisher<example_interfaces::msg::Int64>("number",10);
        timer_ = this -> create_wall_timer(
            std::chrono::duration<double>(timer_period_),
            std::bind(&NumberPublisherNode::publishNumber, this));
        RCLCPP_INFO(this->get_logger(), "Number Publisher Node has been started");

    }
private:

    void publishNumber()
    {
        auto msg = example_interfaces::msg::Int64();
        msg.data = std::int64_t(number_pub_);
        publisher_ -> publish(msg);
    }


    int number_;
    std::int64_t number_pub_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argsc, char **argv){
    rclcpp::init(argsc, argv);
    auto node = std::make_shared<NumberPublisherNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}