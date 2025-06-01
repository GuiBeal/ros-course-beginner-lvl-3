#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/empty.hpp>

#include <my_robot_interfaces/action/move_robot.hpp>

using namespace std::placeholders;

using EmptyMsg = std_msgs::msg::Empty;
using MoveRobotAction = my_robot_interfaces::action::MoveRobot;
using GoalHandle = rclcpp_action::ClientGoalHandle<MoveRobotAction>;

class MoveRobotClientNode : public rclcpp::Node
{
public:
  MoveRobotClientNode() : Node("move_robot_client")
  {
    pActionClient_ = rclcpp_action::create_client<MoveRobotAction>(this, "move_robot");
    pSubscriber_ = this->create_subscription<EmptyMsg>("cancel_move_robot", 10, std::bind(&MoveRobotClientNode::cbCancelGoal, this, _1));

    RCLCPP_INFO(this->get_logger(), "Move Robot Client started.");
  }

  void sendGoal(const int64_t target, const int64_t velocity)
  {
    pActionClient_->wait_for_action_server();

    auto goal = MoveRobotAction::Goal();
    goal.position = target;
    goal.velocity = velocity;

    auto options = rclcpp_action::Client<MoveRobotAction>::SendGoalOptions();
    options.goal_response_callback = std::bind(&MoveRobotClientNode::cbGoalResponse, this, _1);
    options.feedback_callback = std::bind(&MoveRobotClientNode::cbGoalFeedback, this, _1, _2);
    options.result_callback = std::bind(&MoveRobotClientNode::cbGoalResult, this, _1);

    RCLCPP_INFO(this->get_logger(), "Sending goal.");
    pActionClient_->async_send_goal(goal, options);
  }

private:
  void cbGoalResponse(const GoalHandle::SharedPtr &pGoalHandle)
  {
    if (!pGoalHandle)
    {
      RCLCPP_INFO(this->get_logger(), "Goal rejected.");
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutexGoalHandle_);
      pGoalHandle_ = pGoalHandle;
    }
    RCLCPP_INFO(this->get_logger(), "Goal accepted.");
  }

  void cbGoalFeedback(const GoalHandle::SharedPtr & /*pGoalHandle*/, const MoveRobotAction::Feedback::ConstSharedPtr pFeedback)
  {
    RCLCPP_INFO(this->get_logger(), "Current position is %ld.", pFeedback->position);
  }

  void cbGoalResult(const GoalHandle::WrappedResult &result)
  {
    {
      std::lock_guard<std::mutex> lock(mutexGoalHandle_);
      pGoalHandle_.reset();
    }

    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Goal succeeded with message '%s'.", result.result->message.c_str());
      break;

    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_WARN(this->get_logger(), "Goal aborted with message '%s'.", result.result->message.c_str());
      break;

    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(this->get_logger(), "Goal canceled with message '%s'.", result.result->message.c_str());
      break;

    default:
      assert(false);
      break;
    }

    RCLCPP_INFO(this->get_logger(), "Final position is %ld.", result.result->position);
  }

  void cbCancelGoal(EmptyMsg::SharedPtr /*pMsg*/)
  {
    std::lock_guard<std::mutex> lock(mutexGoalHandle_);
    if (!pGoalHandle_)
    {
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Cancelling goal.");
    pActionClient_->async_cancel_goal(pGoalHandle_);
    pGoalHandle_.reset();
  }

  rclcpp_action::Client<MoveRobotAction>::SharedPtr pActionClient_;

  std::mutex mutexGoalHandle_;
  GoalHandle::SharedPtr pGoalHandle_;

  rclcpp::Subscription<EmptyMsg>::SharedPtr pSubscriber_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto pNode = std::make_shared<MoveRobotClientNode>();
  pNode->sendGoal(70, 3);
  rclcpp::spin(pNode);

  rclcpp::shutdown();
  return 0;
}