#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <my_robot_interfaces/action/count_until.hpp>

using namespace std::placeholders;

class CountUntilClientNode : public rclcpp::Node
{
public:
  CountUntilClientNode() : Node("count_until_client")
  {
    pActionClient_ = rclcpp_action::create_client<my_robot_interfaces::action::CountUntil>(this, "count_until");

    RCLCPP_INFO(this->get_logger(), "Count Until Client started.");
  }

  void sendGoal(const int64_t target, const double period)
  {
    pActionClient_->wait_for_action_server();

    auto goal = my_robot_interfaces::action::CountUntil::Goal();
    goal.target = target;
    goal.period = period;

    auto options = rclcpp_action::Client<my_robot_interfaces::action::CountUntil>::SendGoalOptions();
    options.goal_response_callback = std::bind(&CountUntilClientNode::cbGoalResponse, this, _1);
    options.feedback_callback = std::bind(&CountUntilClientNode::cbGoalFeedback, this, _1, _2);
    options.result_callback = std::bind(&CountUntilClientNode::cbGoalResult, this, _1);

    RCLCPP_INFO(this->get_logger(), "Sending goal.");
    pActionClient_->async_send_goal(goal, options);

    // pTimer_ = this->create_wall_timer(std::chrono::seconds(2), std::bind(&CountUntilClientNode::cbCancelGoal, this));
  }

private:
  // void cbCancelGoal()
  // {
  //   RCLCPP_INFO(this->get_logger(), "Sending cancel request.");
  //   assert(pGoalHandle_);
  //
  //   pActionClient_->async_cancel_goal(pGoalHandle_);
  //
  //   pTimer_->cancel();
  // }

  void cbGoalResponse(const rclcpp_action::ClientGoalHandle<my_robot_interfaces::action::CountUntil>::SharedPtr &pGoalHandle)
  {
    if (!pGoalHandle)
    {
      RCLCPP_INFO(this->get_logger(), "Goal rejected.");
    }
    else
    {
      pGoalHandle_ = pGoalHandle;
      RCLCPP_INFO(this->get_logger(), "Goal accepted.");
    }
  }

  void cbGoalFeedback(const rclcpp_action::ClientGoalHandle<my_robot_interfaces::action::CountUntil>::SharedPtr & /*pGoalHandle*/,
                      const my_robot_interfaces::action::CountUntil::Feedback::ConstSharedPtr pFeedback)
  {
    RCLCPP_INFO(this->get_logger(), "Got feedback of count to %ld.", pFeedback->count);
  }

  void cbGoalResult(const rclcpp_action::ClientGoalHandle<my_robot_interfaces::action::CountUntil>::WrappedResult &result)
  {
    const auto status = result.code;
    if (status == rclcpp_action::ResultCode::SUCCEEDED)
    {
      RCLCPP_INFO(this->get_logger(), "Goal succeeded.");
    }
    else if (status == rclcpp_action::ResultCode::ABORTED)
    {
      RCLCPP_ERROR(this->get_logger(), "Goal aborted.");
    }
    if (status == rclcpp_action::ResultCode::CANCELED)
    {
      RCLCPP_WARN(this->get_logger(), "Goal canceled.");
    }

    const auto count = result.result->count;
    RCLCPP_INFO(this->get_logger(), "Count result is %ld.", count);
  }

  rclcpp_action::Client<my_robot_interfaces::action::CountUntil>::SharedPtr pActionClient_;
  rclcpp_action::ClientGoalHandle<my_robot_interfaces::action::CountUntil>::SharedPtr pGoalHandle_;

  // rclcpp::TimerBase::SharedPtr pTimer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto pNode = std::make_shared<CountUntilClientNode>();
  pNode->sendGoal(8, 1);
  rclcpp::spin(pNode);

  rclcpp::shutdown();
  return 0;
}