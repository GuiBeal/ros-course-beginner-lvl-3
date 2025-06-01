#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <my_robot_interfaces/action/move_robot.hpp>

using namespace std::placeholders;

using MoveRobotAction = my_robot_interfaces::action::MoveRobot;
using GoalHandle = rclcpp_action::ServerGoalHandle<MoveRobotAction>;

class MoveRobotServerNode : public rclcpp::Node
{
public:
  MoveRobotServerNode() : Node("move_robot_server")
  {
    pCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    pActionServer_ = rclcpp_action::create_server<MoveRobotAction>(
        this, "move_robot",
        std::bind(&MoveRobotServerNode::cbHandleGoal, this, _1, _2),
        std::bind(&MoveRobotServerNode::cbHandleCancel, this, _1),
        std::bind(&MoveRobotServerNode::cbHandleAccepted, this, _1),
        rcl_action_server_get_default_options(), pCallbackGroup_);

    RCLCPP_INFO(this->get_logger(), "Move Robot Server started.");
  }

private:
  rclcpp_action::GoalResponse cbHandleGoal(const rclcpp_action::GoalUUID & /*uuid*/, MoveRobotAction::Goal::ConstSharedPtr pGoal)
  {
    if (pGoal->position < MIN_POSITION)
    {
      RCLCPP_INFO(this->get_logger(), "Rejecting goal because position is below %ld.", MIN_POSITION);
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (pGoal->position > MAX_POSITION)
    {
      RCLCPP_INFO(this->get_logger(), "Rejecting goal because position is above %ld.", MAX_POSITION);
      return rclcpp_action::GoalResponse::REJECT;
    }

    // Policy: preempt previous goal when receiving new goal
    {
      std::lock_guard<std::mutex> lock(mutexGoalHandle_);
      if (pGoalHandle_ && pGoalHandle_->is_active())
      {
        RCLCPP_INFO(this->get_logger(), "Aborting previous goal.");
        preemptingGoalID_ = pGoalHandle_->get_goal_id();
      }
    }

    RCLCPP_INFO(this->get_logger(), "Accepting goal.");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse cbHandleCancel(const std::shared_ptr<GoalHandle> /*pGoalHandle*/)
  {
    RCLCPP_INFO(this->get_logger(), "Received a cancel request.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void cbHandleAccepted(const std::shared_ptr<GoalHandle> pGoalHandle)
  {
    {
      std::lock_guard<std::mutex> lock(mutexGoalHandle_);
      pGoalHandle_ = pGoalHandle;
    }
    executeGoal(pGoalHandle);
  }

  void executeGoal(const std::shared_ptr<GoalHandle> pGoalHandle)
  {
    const int64_t target = pGoalHandle->get_goal()->position;
    const int64_t velocity = pGoalHandle->get_goal()->velocity;

    auto pResult = std::make_shared<MoveRobotAction::Result>();

    auto pFeedback = std::make_shared<MoveRobotAction::Feedback>();
    pFeedback->position = position_;

    pGoalHandle->publish_feedback(pFeedback);

    rclcpp::Rate loopRate(1.0);

    while (position_ != target)
    {
      loopRate.sleep();

      {
        std::lock_guard<std::mutex> lock(mutexGoalHandle_);
        if (pGoalHandle->get_goal_id() == preemptingGoalID_)
        {
          RCLCPP_INFO(this->get_logger(), "Aborting goal.");
          pResult->position = position_;
          pResult->message = "Aborted";
          pGoalHandle->abort(pResult);
          return;
        }
      }

      if (pGoalHandle->is_canceling())
      {
        RCLCPP_INFO(this->get_logger(), "Cancelling goal.");
        pResult->position = position_;
        pResult->message = "Cancelled";
        pGoalHandle->canceled(pResult);
        return;
      }

      if (target > position_)
      {
        position_ += velocity;
        if (position_ > target)
          position_ = target;
      }
      else if (target < position_)
      {
        position_ -= velocity;
        if (position_ < target)
          position_ = target;
      }
      else
      {
        assert(false);
      }

      RCLCPP_INFO(this->get_logger(), "Moved to %ld.", position_);

      pFeedback->position = position_;
      pGoalHandle->publish_feedback(pFeedback);
    }

    RCLCPP_INFO(this->get_logger(), "Suceeded goal.");

    pResult->position = position_;
    pResult->message = "Succeeded";
    pGoalHandle->succeed(pResult);
  }

  rclcpp_action::Server<MoveRobotAction>::SharedPtr pActionServer_;
  rclcpp::CallbackGroup::SharedPtr pCallbackGroup_;

  std::mutex mutexGoalHandle_;
  std::shared_ptr<GoalHandle> pGoalHandle_;

  rclcpp_action::GoalUUID preemptingGoalID_;

  const int64_t MIN_POSITION = 0;
  const int64_t MAX_POSITION = 100;

  int64_t position_ = 0.5 * (MIN_POSITION + MAX_POSITION);
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto pNode = std::make_shared<MoveRobotServerNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(pNode);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
