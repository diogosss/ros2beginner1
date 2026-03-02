#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/move_turtle.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
using LifecycleCallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using Twist = geometry_msgs::msg::Twist;
using MoveTurtle = my_robot_interfaces::action::MoveTurtle;
using MoveTurtleGoalHandle = rclcpp_action::ServerGoalHandle<MoveTurtle>;

class TurtleController : public rclcpp_lifecycle::LifecycleNode 
{ 
public:
    TurtleController() : LifecycleNode("turtle_controller")   
    {  
        server_activated_ = false;
        //declaraa parametro
        this->declare_parameter("turtle_name", rclcpp::PARAMETER_STRING);
        turtle_name_="";
        server_activated_ = false;
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);       
    }

    LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(this->get_logger(), "IN on_configure");
        //Obtener p}valor paramtro
        turtle_name_ = this->get_parameter("turtle_name").as_string();
        //Service server para kill y spawn
        kill_turtle_client_ = this->create_client<turtlesim::srv::Kill>("/kill", rclcpp::ServicesQoS(), cb_group_);
        spawn_turtle_client_ = this->create_client<turtlesim::srv::Spawn>("/spawn",rclcpp::ServicesQoS(), cb_group_);
     
        //Publisher Vel to turtle sim
        cmd_vel_publisher_ = this -> create_publisher<Twist>("/"+turtle_name_+"/cmd_vel",10);

        //Action server init
        move_turtle_server_ = rclcpp_action::create_server<MoveTurtle>(
            this,
            "move_turtle",
            std::bind(&TurtleController::goal_callback, this, _1, _2),
            std::bind(&TurtleController::cancel_callback, this, _1),
            std::bind(&TurtleController::handle_accepted_callback, this, _1),
            rcl_action_server_get_default_options(),
            cb_group_
        );
        //Spawn Turtle
        spawnTurtle();
        RCLCPP_INFO(this->get_logger(), "Action server has been started...");
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(this->get_logger(), "IN on_activate");
        
        server_activated_ = true;
        rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);
        return LifecycleCallbackReturn::SUCCESS;
    }


    LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(this->get_logger(), "Deactivate node");
        server_activated_ = false;
        // {
        //     std::lock_guard<std::mutex> lock(mutex_);
        //     if (goal_handle_) {
        //         if (goal_handle_->is_active()) {
        //             preempted_goal_id_ = goal_handle_->get_goal_id();
        //         }
        //     }
        // }
        rclcpp_lifecycle::LifecycleNode::on_deactivate(previous_state);
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        turtle_name_ = "";
        this->undeclare_parameter("robot_name");
        killTurtle();
        move_turtle_server_.reset();
        spawn_turtle_client_.reset();
        cmd_vel_publisher_.reset();
        kill_turtle_client_.reset();
        
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        turtle_name_ = "";
        server_activated_ = false;
        this->undeclare_parameter("robot_name");
        killTurtle();
        move_turtle_server_.reset();
        spawn_turtle_client_.reset();
        cmd_vel_publisher_.reset();
        kill_turtle_client_.reset();        
        return LifecycleCallbackReturn::SUCCESS;
    }



private:

    rclcpp_action::GoalResponse goal_callback(
        const rclcpp_action::GoalUUID &uuid,
         std::shared_ptr<const MoveTurtle::Goal> goal)
    {
        (void) uuid;
        RCLCPP_INFO(this->get_logger(), "Received a goal");

        //Check if node is Activated
        if(!server_activated_){
            RCLCPP_WARN(this->get_logger(), "Server not activated yet");
            return rclcpp_action::GoalResponse::REJECT;
        }

        //Policy2: refuse new goal if one goal is active
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(goal_handle_){
                if(goal_handle_->is_active()){
                    RCLCPP_WARN(this->get_logger(), "A goal is still active, reject new goal...");
                    return rclcpp_action::GoalResponse::REJECT;
                }
            }
        }          
        
        //Validate new goal
        if((fabs(goal->linear_vel_x)>3.0) 
            || (fabs(goal->angular_vel_z)>2.0)
            || (goal->duration_sec <= 0.0))
        {
            RCLCPP_WARN(this->get_logger(), "Invalid goall...");
            return rclcpp_action::GoalResponse::REJECT;
        }
    
        RCLCPP_INFO(this->get_logger(), "Accepting the goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse cancel_callback(
        const std::shared_ptr<MoveTurtleGoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_callback(const std::shared_ptr<MoveTurtleGoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing the goal...");
        execute_goal(goal_handle);
    }

    void execute_goal(const std::shared_ptr<MoveTurtleGoalHandle> goal_handle)
    {
        //Lock del thread
        {
            std::lock_guard<std::mutex> lock(mutex_);
            this->goal_handle_ = goal_handle;
        }        

        //Get reuqest from goal
        double linear_vel_x = goal_handle->get_goal()->linear_vel_x;
        double angular_vel_z = goal_handle->get_goal()->angular_vel_z;
        double duration_sec = goal_handle->get_goal()->duration_sec;

        //Execute the action
        
        //Feedback
        auto feedback = std::make_shared<MoveTurtle::Feedback>();
        //set the final state an return result
        auto result = std::make_shared<MoveTurtle::Result>();

        //**--Ejecutar los callbacks de control de turtlesim kill spawn

        //**--Ejecutar el callback de movimiento
        // 1. Definir una frecuencia de control alta (10Hz es estándar para teleop)
        rclcpp::Rate loop_rate(10);
        // 2. Guardar el tiempo de inicio
        auto start_time = this->now();
        RCLCPP_INFO(this->get_logger(), "Iniciando movimiento por %f segundos", duration_sec);
        // 3. Bucle de control dinámico
        while (rclcpp::ok() && (this->now() - start_time).seconds() < duration_sec) {
            //Si el robot se desactiva
            if(!server_activated_){
                stop_turtle();
                result->success = false;
                result->message = "Aborted server was deactivated";
                goal_handle->abort(result);
                return;
            }
            //Verificar si se cancelo         
            if(goal_handle->is_canceling())
            {
                stop_turtle();
                goal_handle->canceled(result);
                return;
            }
            // Publicar la velocidad en cada iteración
            publishTurtleVelocity(linear_vel_x,angular_vel_z);
            // Enviar feedback opcional al cliente
            // auto feedback = std::make_shared<MyAction::Feedback>();
            // feedback->time_left = duration_sec - (this->now() - start_time).seconds();
            // goal_handle->publish_feedback(feedback);
            loop_rate.sleep();
        }
        
        result->success = true;
        result->message = "Success";
        stop_turtle();
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Meta alcanzada con éxito.");
        
    }

    void publishTurtleVelocity(int linear_vel_x, int angular_vel_z)
    {
        auto msg = Twist();
        msg.linear.x = linear_vel_x;  // Velocidad constante
        msg.angular.z = angular_vel_z; // Giro constante
        cmd_vel_publisher_->publish(msg);
    }

    void stop_turtle()
    {
        auto msg = Twist(); // Se inicializa todo en 0.0 por defecto
        cmd_vel_publisher_->publish(msg);
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

    void spawnTurtle()
    {
        while (!spawn_turtle_client_->wait_for_service(1s))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server...");
        }

        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->name = turtle_name_;
        request->x = 5.5;
        request->y = 5.5;
        request->theta = 0;
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

    rclcpp::CallbackGroup::SharedPtr cb_group_;

    rclcpp_action::Server<MoveTurtle>::SharedPtr move_turtle_server_;
    std::shared_ptr<MoveTurtleGoalHandle> goal_handle_;
    std::mutex mutex_;
    rclcpp::Publisher<Twist>::SharedPtr cmd_vel_publisher_;
    bool server_activated_;


};

int main(int argsc, char **argv){
    rclcpp::init(argsc, argv);
    auto node = std::make_shared<TurtleController>();  
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}



//ros2 run final_project_cpp turtle_controller --ros-args -p turtle_name:=abx
//ros2 lifecycle nodes
//   /turtle_controller
//ros2 lifecycle set /turtle_controller configure
//ros2 lifecycle set /turtle_controller activate
//ros2 action send_goal /move_turtle my_robot_interfaces/action/MoveTurtle "{linear_vel_x: 1.0, angular_vel_z: 1.53, duration_sec: 2.0}"