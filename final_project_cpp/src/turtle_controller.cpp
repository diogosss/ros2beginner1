#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/move_turtle.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
using TurtleControllerI = my_robot_interfaces::action::MoveTurtle;
using TurtleControllerGoalHandle = rclcpp_action::ServerGoalHandle<TurtleControllerI>;

class TurtleController : public rclcpp::Node  
{
public:
    TurtleController() : Node("turtle_controller")   
    {
        this->declare_parameter("turtle_name", rclcpp::PARAMETER_STRING);
        turtle_name_ = this->get_parameter("turtle_name").as_string();

        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        //Service server para kill y spawn
        kill_turtle_client_ = this->create_client<turtlesim::srv::Kill>("/kill", rclcpp::ServicesQoS(), cb_group_);
        spawn_turtle_client_ = this->create_client<turtlesim::srv::Spawn>("/spawn",rclcpp::ServicesQoS(), cb_group_);

        //Publisher Vel to turtle sim
        publisher_ = this -> create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel",10);


        //Action server init
        move_turtle_server_ = rclcpp_action::create_server<TurtleControllerI>(
            this,
            "move_turtle",
            std::bind(&TurtleController::goal_callback, this, _1, _2),
            std::bind(&TurtleController::cancel_callback, this, _1),
            std::bind(&TurtleController::handle_accepted_callback, this, _1),
            rcl_action_server_get_default_options(),
            cb_group_
        );
        RCLCPP_INFO(this->get_logger(), "Action server has been started...");
        

    }

    

private:

    rclcpp_action::GoalResponse goal_callback(
        const rclcpp_action::GoalUUID &uuid,
         std::shared_ptr<const TurtleControllerI::Goal> goal)
    {
        (void) uuid;
        RCLCPP_INFO(this->get_logger(), "Received a goal");

        //Policy2: refuse new goal if one goal is active
        //Lock del thread   -> googlear scope en c++ lock y mutex
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(goal_handle_){
                if(goal_handle_->is_active()){
                    RCLCPP_WARN(this->get_logger(), "A goal is still active, reject new goal...");
                    return rclcpp_action::GoalResponse::REJECT;
                }
            }
        }              

        if(goal->linear_vel_x <= 0.0 || goal->linear_vel_y <= 0.0 || goal->duration_sec<= 0.0){
            RCLCPP_WARN(this->get_logger(), "Rejecting the goal");
            return rclcpp_action::GoalResponse::REJECT;
        }           


        RCLCPP_INFO(this->get_logger(), "Accepting the goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse cancel_callback(
        const std::shared_ptr<TurtleControllerGoalHandle> goal_handle)
    {
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_callback(const std::shared_ptr<TurtleControllerGoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing the goal...");
        execute_goal(goal_handle);
    }

    void execute_goal(const std::shared_ptr<TurtleControllerGoalHandle> goal_handle)
    {
        //Lock del thread
        {
            std::lock_guard<std::mutex> lock(mutex_);
            this->goal_handle_ = goal_handle;
        }        

        //Get reuqest from goal
        double linear_vel_x = goal_handle->get_goal()->linear_vel_x;
        double linear_vel_y = goal_handle->get_goal()->linear_vel_y;
        double duration_sec = goal_handle->get_goal()->duration_sec;

        //Execute the action
        // 1. Definir una frecuencia de control alta (10Hz es estándar para teleop)
        rclcpp::Rate loop_rate(10);
        // 2. Guardar el tiempo de inicio
        auto start_time = this->now();
        //Feedback
        auto feedback = std::make_shared<TurtleControllerI::Feedback>();
        //set the final state an return result
        auto result = std::make_shared<TurtleControllerI::Result>();

        //Ejecutar los callbacks de control de turtlesim

        //Ejecutar el callback de movimiento
        RCLCPP_INFO(this->get_logger(), "Iniciando movimiento por %f segundos", duration_sec);
        // 3. Bucle de control dinámico
        while (rclcpp::ok() && (this->now() - start_time).seconds() < duration_sec) {
            //Verificar si se cancelo         
            if(goal_handle->is_canceling()){
                goal_handle->canceled(result);
                return;
            }
            // Publicar la velocidad en cada iteración
            publishTurtleVelocity();
            // Enviar feedback opcional al cliente
            // auto feedback = std::make_shared<MyAction::Feedback>();
            // feedback->time_left = duration_sec - (this->now() - start_time).seconds();
            // goal_handle->publish_feedback(feedback);
            loop_rate.sleep();
        }
        
        result->state = true;
        //goal_handle->abort(result);
        goal_handle->succeed(result);
        
    }

    void publishTurtleVelocity()
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = 1.0;  // Velocidad constante
        msg.angular.z = 1.0; // Giro constante
        publisher_->publish(msg);
    }

    void stop_turtle()
    {
        auto msg = geometry_msgs::msg::Twist(); // Se inicializa todo en 0.0 por defecto
        publisher_->publish(msg);
    }

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

    rclcpp_action::Server<TurtleControllerI>::SharedPtr move_turtle_server_;
    std::shared_ptr<TurtleControllerGoalHandle> goal_handle_;
    std::mutex mutex_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;


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



//ros2 run final_project_cpp turtle_controller --ros-args -p turtle_name:=abx
//ros2 action send_goal /move_turtle my_robot_interfaces/action/MoveTurtle "{linear_vel_x: 1.0, linear_vel_y: 1.0, duration_sec: 9.0}"