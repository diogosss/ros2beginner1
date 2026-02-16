#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/count_until.hpp"
#include "my_robot_interfaces/action/move_robot.hpp"
#include "example_interfaces/msg/int64.hpp"

using MoveRobot = my_robot_interfaces::action::MoveRobot;
using MoveRobotGoalHandle = rclcpp_action::ClientGoalHandle<MoveRobot>;
using namespace std::placeholders;

//V0 - Client
class MoveRobotClientNode : public rclcpp::Node
{
public:
    MoveRobotClientNode() : Node("count_until_client")
    {
        //cliente action
        move_robot_client_ = rclcpp_action::create_client<MoveRobot>(this, "move_robot");

        //iniciar la suscripcion
        subscriber_ = this->create_subscription<example_interfaces::msg::Int64>(
            "cancel_goal",
            10,
            std::bind(&MoveRobotClientNode::callbackSendCancelPublisher, this, _1));

        RCLCPP_INFO(this->get_logger(), "Client Action Node has been started ...");

    }

    void send_goal(int target_position, int velocity)
    {
        //wait for the action server
        move_robot_client_->wait_for_action_server();

        //Craer el goal
        auto goal = MoveRobot::Goal();
        goal.position = target_position;
        goal.velocity= velocity;

        // Add callbacks register the functions or goalscallbacks
        auto options = rclcpp_action::Client<MoveRobot>::SendGoalOptions();
        options.result_callback = std::bind(&MoveRobotClientNode::goal_result_callback, this, _1);
        options.goal_response_callback = std::bind(&MoveRobotClientNode::goal_response_callback,this, _1);
        options.feedback_callback = std::bind(&MoveRobotClientNode::feedback_callback,this, _1, _2);

        //Enviar el goal
        RCLCPP_INFO(this->get_logger(), "Sending a goal...");
        move_robot_client_->async_send_goal(goal, options);

    }

private:

    //Callbak del timer de prueba del canceling
    void cancel_action_callback()
    {
        if(this->goal_handle_)
        {
            RCLCPP_INFO(this->get_logger(), "Sending a cancel goal..");
            move_robot_client_->async_cancel_goal(goal_handle_);
            goal_handle_.reset();
        }
        
    }

    //Callback para obtener el feedback
    void feedback_callback(const MoveRobotGoalHandle::SharedPtr &goal_handle, const std::shared_ptr<const MoveRobot::Feedback> feedback){

        (void)goal_handle;
        int number = feedback->current_position;
        RCLCPP_INFO(this->get_logger(), "Got feedback: %d",number);
    }

    //Callback to know if the goal was accepted or rejected
    void goal_response_callback(const MoveRobotGoalHandle::SharedPtr &goal_handle)
    {
        if(!goal_handle){
            RCLCPP_WARN(this->get_logger(), "Goal was rejected.");
        }else{
            this->goal_handle_ = goal_handle; //obtener el goal hadle luego de qe el server acepte y usarlo en el metodo timerCallback
            RCLCPP_INFO(this->get_logger(), "Goal was accepted.");
        }
    }

    //Callback received the result once the goal is done
    void goal_result_callback(const MoveRobotGoalHandle::WrappedResult &result)
    {
        auto status = result.code;
        if(status == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(this->get_logger(), "Succeeded..");
        }else if(status == rclcpp_action::ResultCode::ABORTED)
        {
            RCLCPP_ERROR(this->get_logger(), "Aborted..");
        }else if(status == rclcpp_action::ResultCode::CANCELED)
        {
            RCLCPP_WARN(this->get_logger(), "Canceled..");
        }
        int reached_number = result.result->position;
        std::string message = result.result->message;
        RCLCPP_INFO(this->get_logger(), "Result Position: %d",reached_number);
        RCLCPP_INFO(this->get_logger(), "Result Message: %s",message.c_str());
    }

    void callbackSendCancelPublisher(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "%ld", msg->data);
        //Enviar una señal de cancel para test Canceling goal
        // Verificamos si existe un goal activo para cancelar
        if (!goal_handle_) {
            RCLCPP_WARN(this->get_logger(), "No hay ninguna meta activa para cancelar.");
            return;
        }

        // Llamamos directamente al método de cancelación
        this->cancel_action_callback();
        //publicar cuando se llame a esta metodo
        rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;

    }

    rclcpp_action::Client<MoveRobot>::SharedPtr move_robot_client_;


    MoveRobotGoalHandle::SharedPtr goal_handle_;

    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;


};

int main(int argsc, char **argv){
    rclcpp::init(argsc, argv);
    auto node = std::make_shared<MoveRobotClientNode>();
    node->send_goal(77,7);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}