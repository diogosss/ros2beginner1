#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class TurtleController : public rclcpp::Node  
{
public:
    TurtleController() : Node("turtle_controller")   
    {
        this->declare_parameter("turtle_name", rclcpp::PARAMETER_STRING);
        turtle_name_ = this->get_parameter("turtle_name").as_string();

        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        kill_turtle_client_ = this->create_client<turtlesim::srv::Kill>("/kill", rclcpp::ServicesQoS(), cb_group_);
        spawn_turtle_client_ = this->create_client<turtlesim::srv::Spawn>("/spawn",rclcpp::ServicesQoS(), cb_group_);

        //Test desde constructor con threads
        spawn_turtle_thread_ = std::thread(std::bind(&TurtleController::spawnTurtle, this, 5.5, 5.5, 0.0));
        

    }

    

private:

    void killTurtle()
    {
        while (!kill_turtle_client_->wait_for_service(1s))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server...");
        }

        auto request = std::make_shared<turtlesim::srv::Kill::Request>();
        request->name = turtle_name_;

        RCLCPP_WARN(this->get_logger(), "Trying to remove turtle...");

        kill_turtle_client_ -> async_send_request(request, std::bind(&TurtleController::callbackKillTurtle, this, _1));
    }

    void spawnTurtle(double x, double y, double theta)
    {
        while (!spawn_turtle_client_->wait_for_service(1s))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server...");
        }

        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->name = turtle_name_;
        request->x = x;
        request->y = y;
        request->theta = theta;
        RCLCPP_WARN(this->get_logger(), "Trying to spawn turtle...");

        spawn_turtle_client_ -> async_send_request(request, std::bind(&TurtleController::callbackSpawnTurtle, this, _1));

        //ejecutar el metodo de Kill luego de spawn
        std::this_thread::sleep_for(std::chrono::seconds(3));
        kill_turtle_thread_ = std::thread(std::bind(&TurtleController::killTurtle, this));
    }

    void callbackKillTurtle(rclcpp::Client<turtlesim::srv::Kill>::SharedFuture future)
    {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), " Turtle  has been removed");
    }

    void callbackSpawnTurtle(rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future)
    {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Spawn Turtle Response: %s", response->name.c_str());
    }

    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_turtle_client_;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_turtle_client_;
    std::string turtle_name_;

    std::thread spawn_turtle_thread_;
    std::thread kill_turtle_thread_;

    rclcpp::CallbackGroup::SharedPtr cb_group_;

};

int main(int argsc, char **argv){
    rclcpp::init(argsc, argv);
    auto node = std::make_shared<TurtleController>();  
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}