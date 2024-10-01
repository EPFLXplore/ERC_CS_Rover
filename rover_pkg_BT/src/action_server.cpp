#include <functional>
#include <memory>
#include <thread>

#include "custom_msg/action/drill_terrain.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace rover_pkg_BT
{
class FibonacciActionServer : public rclcpp::Node
{
public:
  using DrillTerrain = custom_msg::action::DrillTerrain;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<DrillTerrain>;

  explicit FibonacciActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<DrillTerrain>(
      this,
      "a_s",
      std::bind(&FibonacciActionServer::handle_goal, this, _1, _2),
      std::bind(&FibonacciActionServer::handle_cancel, this, _1),
      std::bind(&FibonacciActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<DrillTerrain>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const DrillTerrain::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", 2);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&FibonacciActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<DrillTerrain::Feedback>();
    auto & sequence = feedback->current_status;
    auto & warning_type = feedback->warning_type;
    auto & warning_message = feedback->warning_message;
    auto result = std::make_shared<DrillTerrain::Result>();

    for (int i = 1; (i < 3) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->result = "bla";
        result->error_type = 1;
        result->error_message = "error";
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
        result->result = "serégeroip";
        result->error_type = 0;
        result->error_message = "no error";
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};

}

RCLCPP_COMPONENTS_REGISTER_NODE(rover_pkg_BT::FibonacciActionServer)