#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "components_cpp/number_publisher.hpp"

using namespace std::chrono_literals;

namespace my_namespace {


NumberPublisher::NumberPublisher(const rclcpp::NodeOptions &options) : Node("number_publisher", options)
{
    number_ = 2;
    //this->get_node_base_interface();
    number_publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
    number_timer_ = this->create_wall_timer(1000ms,
                                            std::bind(&NumberPublisher::publishNumber, this));
    RCLCPP_INFO(this->get_logger(), "Number publisher has been started.");
}

void NumberPublisher::publishNumber()
{
    auto msg = example_interfaces::msg::Int64();
    msg.data = number_;
    number_publisher_->publish(msg);
}


} //namesoace my_namespace

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_namespace::NumberPublisher)


// diogo@DiegoS:~/ros2_ws$ ros2 component types
// components_cpp
//   my_namespace::NumberPublisher  <- debe aparecer asi luego del colcon build

//PROBAR ABRIR 
//ros2 run rclcpp_components component_container  <-terminal 1
// ros2 node list  <-terminal 2  -> /ComponentManager

//ros2 component load /ComponentManager components_cpp my_namespace::NumberPublisher <-terminal 3 cargar el Node
// ros2 node list
// ros2 topic list
// ros2 topic echo /number

// ros2 component list
// /ComponentManager
//   1  /number_publisher

// ros2 component unload /ComponentManager 1
// Unloaded component 1 from '/ComponentManager' container node

// ros2 component list
// /ComponentManager

//ros2 component load /ComponentManager components_cpp my_namespace::NumberPublisher -r __node:=abc

// ros2 component list
// /ComponentManager
//   2  /number_publisher
//   3  /abc

// ros2 node list
//     /ComponentManager
//     /abc
//     /number_publisher

//************************ */
//Para debugging
//ros2 component standalone components_cpp my_namespace::NumberPublisher
//************************ */


//PROBAR ABRIR  MultiThreading
//ros2 run rclcpp_components component_container_mt

//ros2 run rclcpp_components component_container_isolated  -> cada nodo se añade a otro single thread executor