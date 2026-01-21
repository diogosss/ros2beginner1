#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("cpp_test"), counter_(0)
    {
        RCLCPP_INFO(this -> get_logger(), "Hello World construc");
        timer_ = this -> create_wall_timer(std::chrono::seconds(1),
                                            std::bind(&MyNode::timerCallback, this));
    }
private:
    void timerCallback(){
        RCLCPP_INFO(this->get_logger(), "Hello %d", counter_);
        counter_++;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
};

int main(int argsc, char **argv){
    rclcpp::init(argsc, argv);
    //
    //auto node = std::make_shared<rclcpp::Node>("cpp_test");//pointer to a node
    //RCLCPP_INFO(node->get_logger(), "Hello World");
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}