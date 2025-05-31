#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <my_robot_interfaces/action/count_until.hpp>

using namespace std::placeholders;

class CountUntilServerNode : public rclcpp::Node
{
public:
  CountUntilServerNode() : Node("count_until_server")
  {
    pCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    pActionServer_ =
        rclcpp_action::create_server<my_robot_interfaces::action::CountUntil>(
            this, "count_until",
            std::bind(&CountUntilServerNode::cbGoal, this, _1, _2),
            std::bind(&CountUntilServerNode::cbCancel, this, _1),
            std::bind(&CountUntilServerNode::cbHandleAccepted, this, _1),
            rcl_action_server_get_default_options(),
            pCallbackGroup_);

    RCLCPP_INFO(this->get_logger(), "Count Until Server started.");
  }

private:
  rclcpp_action::GoalResponse cbGoal(const rclcpp_action::GoalUUID & /*uuid*/, my_robot_interfaces::action::CountUntil::Goal::ConstSharedPtr pGoal)
  {
    RCLCPP_INFO(this->get_logger(), "Received a goal.");

    //// Policy: refuse new goal if another is active
    // {
    //   std::lock_guard<std::mutex> lock(mutexGoalHandle_);
    //   if (pGoalHandle_ && pGoalHandle_->is_active())
    //   {
    //     RCLCPP_INFO(this->get_logger(), "Rejecting goal because a goal is already active.");
    //     return rclcpp_action::GoalResponse::REJECT;
    //   }
    // }

    if (pGoal->target <= 0)
    {
      RCLCPP_INFO(this->get_logger(), "Rejecting goal because target is not positive.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (pGoal->period <= 0)
    {
      RCLCPP_INFO(this->get_logger(), "Rejecting goal because period is not positive.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    //// Policy: preempt previous goal when receiving new goal
    // {
    //   std::lock_guard<std::mutex> lock(mutexGoalHandle_);
    //   if (pGoalHandle_ && pGoalHandle_->is_active())
    //   {
    //     RCLCPP_INFO(this->get_logger(), "Aborting previous goal.");
    //     preemptingGoalID_ = pGoalHandle_->get_goal_id();
    //   }
    // }

    RCLCPP_INFO(this->get_logger(), "Accepting goal.");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse cbCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<my_robot_interfaces::action::CountUntil>> /*pGoalHandle*/)
  {
    RCLCPP_INFO(this->get_logger(), "Received a cancel request.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void cbHandleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<my_robot_interfaces::action::CountUntil>> pGoalHandle)
  {
    {
      std::lock_guard<std::mutex> lock(mutexGoalHandle_);
      pGoalHandle_ = pGoalHandle;
    }
    executeGoal(pGoalHandle);
  }

  void executeGoal(const std::shared_ptr<rclcpp_action::ServerGoalHandle<my_robot_interfaces::action::CountUntil>> pGoalHandle)
  {
    const auto target = pGoalHandle->get_goal()->target;
    const auto period = pGoalHandle->get_goal()->period;

    auto pResult = std::make_shared<my_robot_interfaces::action::CountUntil::Result>();
    pResult->count = 0;

    auto pFeedback = std::make_shared<my_robot_interfaces::action::CountUntil::Feedback>();
    pFeedback->count = pResult->count;
    pGoalHandle->publish_feedback(pFeedback);

    rclcpp::Rate loopRate(1.0 / period);
    for (int i = 0; i < target; ++i)
    {
      loopRate.sleep();

      // {
      //   std::lock_guard<std::mutex> lock(mutexGoalHandle_);
      //   if (pGoalHandle->get_goal_id() == preemptingGoalID_)
      //   {
      //     pGoalHandle->abort(pResult);
      //     return;
      //   }
      // }

      if (pGoalHandle->is_canceling())
      {
        RCLCPP_INFO(this->get_logger(), "Cancelling goal.");
        pGoalHandle->canceled(pResult);
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Counted to %ld.", ++(pResult->count));

      pFeedback->count = pResult->count;
      pGoalHandle->publish_feedback(pFeedback);
    }

    pGoalHandle->succeed(pResult);
  }

  rclcpp_action::Server<my_robot_interfaces::action::CountUntil>::SharedPtr pActionServer_;
  rclcpp::CallbackGroup::SharedPtr pCallbackGroup_;

  std::mutex mutexGoalHandle_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<my_robot_interfaces::action::CountUntil>> pGoalHandle_;

  // rclcpp_action::GoalUUID preemptingGoalID_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto pNode = std::make_shared<CountUntilServerNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(pNode);
  executor.spin();

  // rclcpp::spin(std::make_shared<CountUntilServerNode>());

  rclcpp::shutdown();
  return 0;
}
