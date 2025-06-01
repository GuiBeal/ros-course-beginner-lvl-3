#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <my_robot_interfaces/action/count_until.hpp>

using namespace std::placeholders;

class CountUntilServerNode : public rclcpp::Node
{
public:
  CountUntilServerNode() : Node("count_until_server")
  {
    threadGoalQueue_ = std::thread(&CountUntilServerNode::rungGoalQueueThread, this);

    pCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    pActionServer_ =
        rclcpp_action::create_server<my_robot_interfaces::action::CountUntil>(
            this, "count_until",
            std::bind(&CountUntilServerNode::cbHandleGoal, this, _1, _2),
            std::bind(&CountUntilServerNode::cbHandleCancel, this, _1),
            std::bind(&CountUntilServerNode::cbHandleAccepted, this, _1),
            rcl_action_server_get_default_options(),
            pCallbackGroup_);

    RCLCPP_INFO(this->get_logger(), "Count Until Server started.");
  }

  ~CountUntilServerNode()
  {
    threadGoalQueue_.join();
  }

private:
  rclcpp_action::GoalResponse cbHandleGoal(const rclcpp_action::GoalUUID & /*uuid*/, my_robot_interfaces::action::CountUntil::Goal::ConstSharedPtr pGoal)
  {
    RCLCPP_INFO(this->get_logger(), "Received a goal.");

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

    RCLCPP_INFO(this->get_logger(), "Accepting goal.");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse cbHandleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<my_robot_interfaces::action::CountUntil>> /*pGoalHandle*/)
  {
    RCLCPP_INFO(this->get_logger(), "Received a cancel request.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void cbHandleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<my_robot_interfaces::action::CountUntil>> pGoalHandle)
  {
    std::lock_guard<std::mutex> lock(mutexQueue_);
    queueGoalHandle_.push(pGoalHandle);
    RCLCPP_INFO(this->get_logger(), "Queued goal at %lu.", queueGoalHandle_.size());
  }

  void rungGoalQueueThread()
  {
    rclcpp::Rate loopRate(1000.0);
    while (rclcpp::ok())
    {
      std::shared_ptr<rclcpp_action::ServerGoalHandle<my_robot_interfaces::action::CountUntil>> pNextGoalHandle;
      {
        std::lock_guard<std::mutex> lock(mutexQueue_);
        if (!queueGoalHandle_.empty())
        {
          pNextGoalHandle = queueGoalHandle_.front();
          queueGoalHandle_.pop();
        }
      }

      if (pNextGoalHandle)
      {
        RCLCPP_INFO(this->get_logger(), "Executing next goal.");
        executeGoal(pNextGoalHandle);
      }

      loopRate.sleep();
    }
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

  std::mutex mutexQueue_;
  std::queue<std::shared_ptr<rclcpp_action::ServerGoalHandle<my_robot_interfaces::action::CountUntil>>> queueGoalHandle_;
  std::thread threadGoalQueue_;
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
