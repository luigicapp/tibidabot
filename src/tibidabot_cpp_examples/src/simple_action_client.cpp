#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "rclcpp_components/register_node_macro.hpp"
#include "bumperbot_msgs/action/fibonacci.hpp"

using namespace std::placeholders;

namespace tibidabot_cpp_examples
{
    class SimpleActionClient : public rclcpp::Node
    {
        public:

        SimpleActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()):
        Node("simple_action_client", options)
        {
            client_ = rclcpp_action::create_client<bumperbot_msgs::action::Fibonacci>(this, "fibonacci");
            timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&SimpleActionClient::timerCallback, this));

        }

        private:

        rclcpp_action::Client<bumperbot_msgs::action::Fibonacci>::SharedPtr client_;
        rclcpp::TimerBase::SharedPtr timer_;

        void timerCallback()
        {
            timer_->cancel();//because we want to execute it only once

            if(!client_->wait_for_action_server())//check if server exists
            {
                RCLCPP_INFO(get_logger(), "action server not available after waiting");
                rclcpp::shutdown();
            }
            
            auto goal_msg = bumperbot_msgs::action::Fibonacci::Goal();
            goal_msg.order = 10;
            RCLCPP_INFO(get_logger(),"sending goal");

            auto send_goal_option = rclcpp_action::Client<bumperbot_msgs::action::Fibonacci>::SendGoalOptions();

            send_goal_option.goal_response_callback = std::bind(&SimpleActionClient::goalCallback, this, _1); //callback when goal will be received
            send_goal_option.feedback_callback = std::bind(&SimpleActionClient::feedbackCallback, this, _1, _2);//callback when feedback will be received
            send_goal_option.result_callback = std::bind(&SimpleActionClient::resultCallback, this, _1);//callback when result will be received

            client_->async_send_goal(goal_msg, send_goal_option);
        }

        void goalCallback(const rclcpp_action::ClientGoalHandle<bumperbot_msgs::action::Fibonacci>::SharedPtr & goal_handle)
        {
            if(!goal_handle) //check if not empty
            {
                RCLCPP_ERROR(get_logger(),"goal rejected by server");
            }
            else
            {
                RCLCPP_INFO(get_logger(),"goal accepted by server");
            }
        }   

        void feedbackCallback( rclcpp_action::ClientGoalHandle<bumperbot_msgs::action::Fibonacci>::SharedPtr , 
        const std::shared_ptr<const bumperbot_msgs::action::Fibonacci::Feedback> feedback)
        {
            std::stringstream ss;
            ss << "Next number in sequence received: ";
            for(auto number: feedback->partial_sequence)
            {
                ss << number << " ";
            }
            RCLCPP_INFO(get_logger(),  ss.str().c_str());
        }
        
        void resultCallback( const rclcpp_action::ClientGoalHandle<bumperbot_msgs::action::Fibonacci>::WrappedResult result)
        {
            switch(result.code)
            {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(get_logger(), "goal was aborted");
                    return;
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(get_logger(), "goal was canceled");
                    return;
                    break;
                default:
                    RCLCPP_ERROR(get_logger(), "unkown result code");
                    return;
            }

            std::stringstream ss;
            ss << "Result received: ";

            for(auto number : result.result->sequence)
            {
                ss << number << " ";
            }

            RCLCPP_INFO(get_logger(), ss.str().c_str());
            rclcpp::shutdown();
        }
        
    };
};
//main function where we use all the funcionalities of the class.
//This instruction register the node

RCLCPP_COMPONENTS_REGISTER_NODE(tibidabot_cpp_examples::SimpleActionClient)