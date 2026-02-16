#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/move_robot.hpp"

using MoveRobot = my_robot_interfaces::action::MoveRobot;
using MoveRobotGoalHandle = rclcpp_action::ServerGoalHandle<MoveRobot>;
using namespace std::placeholders;

//V0 - Server
class MoveRobotServerNode : public rclcpp::Node
{
public:
    MoveRobotServerNode() : Node("count_until_server"), robot_position_(50)   
    {
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        move_robot_server_ = rclcpp_action::create_server<MoveRobot>(
            this,
            "move_robot",
            std::bind(&MoveRobotServerNode::goal_callback, this, _1, _2),
            std::bind(&MoveRobotServerNode::cancel_callback, this, _1),
            std::bind(&MoveRobotServerNode::handle_accepted_callback, this, _1),
            rcl_action_server_get_default_options(),
            cb_group_
        );

        RCLCPP_INFO(this->get_logger(), "Action server has been started...");
        RCLCPP_INFO(this->get_logger(), "Robot position: %d", robot_position_);
    }
private:

    rclcpp_action::GoalResponse goal_callback(
        const rclcpp_action::GoalUUID &uuid,
         std::shared_ptr<const MoveRobot::Goal> goal)
         {
            (void) uuid;
            RCLCPP_INFO(this->get_logger(), "Received a goal");            

            if((goal->position > 100 || goal->position < 0) || (goal->velocity <=0)){
                RCLCPP_WARN(this->get_logger(), "Rejecting the goal");
                return rclcpp_action::GoalResponse::REJECT;
            }            

            //Policy3: Preempt the existing goal when receiving new valid goal
            //Lock del thread   -> googlear scope en c++ lock y mutex
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if(goal_handle_){
                    if(goal_handle_->is_active()){
                        preemted_goal_id_ = goal_handle_->get_goal_id();
                        RCLCPP_WARN(this->get_logger(), "A new goal is received, Abort current goal & accept new goal...");
                    }
                }
            }  

            RCLCPP_INFO(this->get_logger(), "Accepting the goal");
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
         }

    rclcpp_action::CancelResponse cancel_callback(
        const std::shared_ptr<MoveRobotGoalHandle> goal_handle)
         {
            RCLCPP_INFO(this->get_logger(), "Received cancel request");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
         }

    void handle_accepted_callback(const std::shared_ptr<MoveRobotGoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing the goal...");
        execute_goal(goal_handle);
    }

    void execute_goal(const std::shared_ptr<MoveRobotGoalHandle> goal_handle)
    {
        //Lock del thread
        {
            std::lock_guard<std::mutex> lock(mutex_);
            this->goal_handle_ = goal_handle;
        }       
        
        //Initial position

        //Get reuqest from goal
        int goal_position = goal_handle->get_goal()->position;
        int velocity = goal_handle->get_goal()->velocity;

        //Execute the action
        rclcpp::Rate loop_rate(1.0);

        auto feedback = std::make_shared<MoveRobot::Feedback>();
        //set the final state an return result
        auto result = std::make_shared<MoveRobot::Result>();
        //****Logica del robot*****
        while(rclcpp::ok){
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if(goal_handle->get_goal_id()== preemted_goal_id_)
                {
                    result->position = robot_position_;
                    result->message = "Aborted-> Preemted by another goal";
                    goal_handle->abort(result);
                    return;
                }                
            }            
            if(goal_handle->is_canceling()){
                result->position = robot_position_;
                result->message = "Canceled";
                goal_handle->canceled(result);
                return;
            }

            int diff = goal_position - robot_position_;

            if(diff==0)
            {
                result->position = robot_position_;
                result->message = "Finished";
                goal_handle->succeed(result);
                return;
            }
            if(diff>0)
            {
                if(diff>=velocity)
                {
                    robot_position_ += velocity;
                }
                else
                {
                    robot_position_ += diff;
                }
            }
            else
            {
                if(abs(diff)>=velocity)
                {
                    robot_position_ -= velocity;
                }
                else
                {
                    robot_position_ -= abs(diff);
                }
            }
            
            
            RCLCPP_INFO(this->get_logger(), "Robot positiiom: %d", robot_position_);
            feedback->current_position = robot_position_;
            goal_handle->publish_feedback(feedback);
            loop_rate.sleep();
        }           

        
        result->position = robot_position_;
        result->message = "Finished";
        //goal_handle->abort(result);
        goal_handle->succeed(result);
        
    }

    rclcpp_action::Server<MoveRobot>::SharedPtr move_robot_server_;

    rclcpp::CallbackGroup::SharedPtr cb_group_;

    std::shared_ptr<MoveRobotGoalHandle> goal_handle_;

    std::mutex mutex_;

    rclcpp_action::GoalUUID preemted_goal_id_;

    // 2. Aquí es el mejor lugar para colocar la variable
    // Representa el estado interno del robot, por lo que debe ser privada.
    int robot_position_; 
    
    // Si usas múltiples hilos (MultiThreadedExecutor), 
    // considera usar std::atomic para evitar condiciones de carrera:
    // std::atomic<int> robot_position_;

};

int main(int argsc, char **argv){
    rclcpp::init(argsc, argv);
    auto node = std::make_shared<MoveRobotServerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}