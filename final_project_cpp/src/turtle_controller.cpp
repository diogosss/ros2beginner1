#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class TurtleControllerNode : public rclcpp::Node  
{
public:
    TurtleControllerNode() : Node("turtle_controller_client")   
    {
        kill_client_ = this->create_client<turtlesim::srv::Kill>("kill");
        spawn_client_ = this->create_client<turtlesim::srv::Spawn>("spawn");
    }

    void killTurtle(std::string name)
    {
        while (!kill_client_->wait_for_service(1s))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server...");
        }

        auto request = std::make_shared<turtlesim::srv::Kill::Request>();
        request->name = name;

        kill_client_ -> async_send_request(request, std::bind(&TurtleControllerNode::callbackKillTurtle, this, _1));
    }

    void spawnTurtle(double x, double y, double theta, std::string name)
    {
        while (!spawn_client_->wait_for_service(1s))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server...");
        }

        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->name = name;
        request->x = x;
        request->y = y;
        request->theta = theta;

        spawn_client_ -> async_send_request(request, std::bind(&TurtleControllerNode::callbackSpawnTurtle, this, _1));
    }

private:

    void callbackKillTurtle(rclcpp::Client<turtlesim::srv::Kill>::SharedFuture future)
    {
        auto response = future.get();
        //RCLCPP_INFO(this->get_logger(), "Kill Turtle Response: %d", (int)response->sum);
    }

    void callbackSpawnTurtle(rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future)
    {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Spawn Turtle Response: %s", response->name.c_str());
    }

    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_client_;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;

};

int main(int argsc, char **argv){
    rclcpp::init(argsc, argv);
    auto node = std::make_shared<TurtleControllerNode>();  
    node -> killTurtle("turtle1");
    node -> spawnTurtle(5.0,5.0, 0.0, "turtle2");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}