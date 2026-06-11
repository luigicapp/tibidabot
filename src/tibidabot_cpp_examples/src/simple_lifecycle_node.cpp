#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

class SimpleLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    SimpleLifecycleNode(const std::string & node_name, bool use_intra_process_comms = false)
        : rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(use_intra_process_comms))
    {
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override
    {
        sub_ = create_subscription<std_msgs::msg::String>("topic", 10, std::bind(&SimpleLifecycleNode::msgCallback, this, std::placeholders::_1)); 
        RCLCPP_INFO(get_logger(), "on_configure() is called from state %s.", previous_state.label().c_str());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override
    {
        sub_.reset();
        RCLCPP_INFO(get_logger(), "on_shutdown() is called from state %s.", previous_state.label().c_str());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override
    {
        sub_.reset();
        RCLCPP_INFO(get_logger(), "on_cleanup() is called from state %s.", previous_state.label().c_str());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override
    {
        LifecycleNode::on_activate(previous_state);
        RCLCPP_INFO(get_logger(), "on_activate() is called from state %s.", previous_state.label().c_str());
        std::this_thread::sleep_for(2s);
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override
    {
        LifecycleNode::on_deactivate(previous_state);
        RCLCPP_INFO(get_logger(), "on_deactivate() is called from state %s.", previous_state.label().c_str());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }


    void msgCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        auto state = get_current_state();
        if(state.label() == "active")
        {
            RCLCPP_INFO_STREAM(get_logger(),"lifecycle node heard "<< msg->data.c_str() );
            return;
        }
    }

private:

rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    std::shared_ptr<SimpleLifecycleNode> node = std::make_shared<SimpleLifecycleNode>("simple_lifecycle_node");
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    return 0;
}