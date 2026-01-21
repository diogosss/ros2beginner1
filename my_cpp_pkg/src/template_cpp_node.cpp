#include "rclcpp/rclcpp.hpp"

class MyCustomNode : public rclcpp::Node  //ModifyName
{
public:
    MyCustomNode() : Node("node_name")   //ModifyName
    {

    }
private:

};

int main(int argsc, char **argv){
    rclcpp::init(argsc, argv);
    auto node = std::make_shared<MyCustomNode>();  //ModifyName
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}