#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "bumperbot_msgs/action/fibonacci.hpp"
#include <rclcpp_components/register_node_macro.hpp> //for the macto at the end of the file
#include <thread>

using namespace std::placeholders;

namespace tibidabot_cpp_examples {
class SimpleActionServer : public rclcpp::Node
{
    public: 

    SimpleActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    :Node("simple_action_server", options)
    {
        action_server_ = rclcpp_action::create_server<bumperbot_msgs::action::Fibonacci>(this, std::string("fibonacci"), 
            std::bind(&SimpleActionServer::goalCallback, this, _1,_2),
            std::bind(&SimpleActionServer::cancelCallback, this, _1),
            std::bind(&SimpleActionServer::acceptedCallback, this, _1)
        );

            RCLCPP_INFO(get_logger(), "starting the action server");
    }

    private:
    rclcpp_action::Server<bumperbot_msgs::action::Fibonacci>::SharedPtr action_server_;

    rclcpp_action::GoalResponse goalCallback(const rclcpp_action::GoalUUID & uuid, 
                                            std::shared_ptr<const bumperbot_msgs::action::Fibonacci::Goal> goal)
    {
        RCLCPP_INFO(get_logger(), "received goal request with order %d", goal->order);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    void acceptedCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<bumperbot_msgs::action::Fibonacci>> goal_handle)
    {

        //we create a new thread in order not to block the client flow. 
        //The thread accept as input the function executed in the thread itself and
        //the parameter it accept, which is the goal_handle in this case
        //The "dethach" is just to say that this function is not blocking the main thread
        std::thread{std::bind(&SimpleActionServer::execute, this, _1), goal_handle}.detach();
    }   

    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<bumperbot_msgs::action::Fibonacci>> goal_handle)
    {
        RCLCPP_INFO(get_logger(), "executing goal");

        rclcpp::Rate loop_rate{1}; //allows to separete two execution by an amount of time (1sec in this case)
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<bumperbot_msgs::action::Fibonacci::Feedback>();
        auto & sequence = feedback->partial_sequence;
        sequence.push_back(0);
        sequence.push_back(1);
        auto result = std::make_shared<bumperbot_msgs::action::Fibonacci::Result>();

        for(int i = 0; (i < goal->order) && rclcpp::ok(); i++ )
        {
            if(goal_handle->is_canceling())
            {
                result->sequence = sequence;
                goal_handle->canceled(result);
                RCLCPP_INFO(get_logger(), "goal canceled");
                return;
            }
            sequence.push_back(sequence[i] + sequence[i-1]);
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(get_logger(), "publishing feedback");
            loop_rate.sleep();
        }

        if(rclcpp::ok())
        {
            result->sequence = sequence;
            goal_handle->succeed(result);
            RCLCPP_INFO(get_logger(), "goal succeded");

        }
        
    }

    rclcpp_action::CancelResponse cancelCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<bumperbot_msgs::action::Fibonacci>> goal_handle)
    {
        RCLCPP_INFO(get_logger(), "receive request to cancel the goal");

        return rclcpp_action::CancelResponse::ACCEPT;
    }
    


};
}

RCLCPP_COMPONENTS_REGISTER_NODE(tibidabot_cpp_examples::SimpleActionServer)