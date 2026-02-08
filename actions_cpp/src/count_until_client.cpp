#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/count_until.hpp"

using CountUntil = my_robot_interfaces::action::CountUntil;
using CountUntilGoalHandle = rclcpp_action::ClientGoalHandle<CountUntil>;
using namespace std::placeholders;

//V0 - Client
class CountUntilClientNode : public rclcpp::Node
{
public:
    CountUntilClientNode() : Node("count_until_client")
    {
        count_until_client_ = rclcpp_action::create_client<CountUntil>(this, "count_until");
    }

    void send_goal(int target_number, double period)
    {
        //wait for the action server
        count_until_client_->wait_for_action_server();

        //Craer el goal
        auto goal = CountUntil::Goal();
        goal.target_number = target_number;
        goal.period= period;

        // Add callbacks register the functions or goalscallbacks
        auto options = rclcpp_action::Client<CountUntil>::SendGoalOptions();
        options.result_callback = std::bind(&CountUntilClientNode::goal_result_callback, this, _1);
        options.goal_response_callback = std::bind(&CountUntilClientNode::goal_response_callback,this, _1);
        options.feedback_callback = std::bind(&CountUntilClientNode::feedback_callback,this, _1, _2);

        //Enviar el goal
        RCLCPP_INFO(this->get_logger(), "Sending a goal...");
        count_until_client_->async_send_goal(goal, options);

        //Enviar una señal de cancel para test Canceling goal
        //timer_ = this->create_wall_timer(std::chrono::seconds(2),std::bind(&CountUntilClientNode::timer_callback, this));
    }

private:

    //Callbak del timer de prueba del canceling
    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Sending a cancel goal..");
        count_until_client_->async_cancel_goal(goal_handle_);
        timer_->cancel();    
    }

    //Callback para obtener el feedback
    void feedback_callback(const CountUntilGoalHandle::SharedPtr &goal_handle, const std::shared_ptr<const CountUntil::Feedback> feedback){

        (void)goal_handle;
        int number = feedback->current_number;
        RCLCPP_INFO(this->get_logger(), "Got feedback: %d",number);
    }

    //Callback to know if the goal was accepted or rejected
    void goal_response_callback(const CountUntilGoalHandle::SharedPtr &goal_handle)
    {
        if(!goal_handle){
            RCLCPP_WARN(this->get_logger(), "Goal was rejected.");
        }else{
            this->goal_handle_ = goal_handle; //obtener el goal hadle luego de qe el server acepte y usarlo en el metodo timerCallback
            RCLCPP_INFO(this->get_logger(), "Goal was accepted.");
        }
    }

    //Callback received the result once the goal is done
    void goal_result_callback(const CountUntilGoalHandle::WrappedResult &result)
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
        int reached_number = result.result->reached_number;
        RCLCPP_INFO(this->get_logger(), "Result: %d",reached_number);
    }

    rclcpp_action::Client<CountUntil>::SharedPtr count_until_client_;

    rclcpp::TimerBase::SharedPtr timer_;

    CountUntilGoalHandle::SharedPtr goal_handle_;

};

int main(int argsc, char **argv){
    rclcpp::init(argsc, argv);
    auto node = std::make_shared<CountUntilClientNode>();
    node->send_goal(9,0.7);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}