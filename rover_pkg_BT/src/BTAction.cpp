#include "custom_msg/action/drill_terrain.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace rover_pkg_BT
{
class SendAction : public BT::AsyncActionNode
{
public:
    using DrillTerrain = custom_msg::action::DrillTerrain;
    SendAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config), node_(rclcpp::Node::make_shared("bt_action_client"))
    {
        client_ = rclcpp_action::create_client<DrillTerrain>(node_, "a_s");
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("goal") };
    }

    BT::NodeStatus tick() override
    {
        // Get goal from the BT input port
        auto goal = getInput<std::string>("goal");
        if (!goal)
        {
            throw BT::RuntimeError("missing required input [goal]: ", goal.error());
        }

        // Check if the action server is available
        if (!client_->wait_for_action_server(std::chrono::seconds(2)))
        {
            RCLCPP_ERROR(node_->get_logger(), "Action server not available");
            return BT::NodeStatus::FAILURE;
        }

        // Create the goal and send the action request
        auto goal_msg = DrillTerrain::Goal();
        goal_msg.extend_to_percentage = 10;

        auto send_goal_options = rclcpp_action::Client<DrillTerrain>::SendGoalOptions();
        send_goal_options.result_callback = [this](auto & result) {
            RCLCPP_INFO(node_->get_logger(), "Goal completed with result: %d", 2);
            switch (result.code)
            {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    status_ = BT::NodeStatus::SUCCESS;
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    status_ = BT::NodeStatus::FAILURE;
                    return;
                case rclcpp_action::ResultCode::CANCELED:
                    status_ = BT::NodeStatus::FAILURE;
                    return;
                default:
                    status_ = BT::NodeStatus::FAILURE;
                    return;
            }
        };

        auto future_goal_handle = client_->async_send_goal(goal_msg, send_goal_options);

        // Wait for result
        if (rclcpp::spin_until_future_complete(node_, future_goal_handle) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to send goal");
            return BT::NodeStatus::FAILURE;
        }

        auto goal_handle = future_goal_handle.get();
        if (!goal_handle)
        {
            RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by the server");
            return BT::NodeStatus::FAILURE;
        }

        // Wait for the result
        auto future_result = client_->async_get_result(goal_handle);
        if (rclcpp::spin_until_future_complete(node_, future_result) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get action result");
            return BT::NodeStatus::FAILURE;
        }

        return status_;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<DrillTerrain>::SharedPtr client_;
    BT::NodeStatus status_ = BT::NodeStatus::RUNNING;
};

int main(int argc, char** argv)
{
    std::cout << "A" << std::endl;

    rclcpp::init(argc, argv);
    std::cout << "A" << std::endl;

    BT::BehaviorTreeFactory factory;

    // Register your custom action node
    factory.registerNodeType<SendAction>("SendAction");
    std::cout << "A" << std::endl;

    // Load the BT from an XML file
    auto tree = factory.createTreeFromFile("tree.xml");
    std::cout << "A" << std::endl;

    // Create a blackboard (shared memory for nodes)
    auto blackboard = BT::Blackboard::create();
    tree.tickRoot();
    std::cout << "A" << std::endl;
    return 0;
}
}

