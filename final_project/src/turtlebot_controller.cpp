#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <my_robot_interfaces/action/move_turtle.hpp>

using TwistStamped = geometry_msgs::msg::TwistStamped;
using MoveTurtle = my_robot_interfaces::action::MoveTurtle;
using MoveTurtleGoalHandle = rclcpp_action::ServerGoalHandle<MoveTurtle>;

using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace final_project
{
  class TurtlebotController : public rclcpp_lifecycle::LifecycleNode
  {
  public:
    TurtlebotController(const rclcpp::NodeOptions &options) : LifecycleNode("turtle_controller", options)
    {
      pCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

      RCLCPP_INFO(this->get_logger(), "Turtlebot Controller started.");
    }

    LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &state)
    {
      pPubCmdVel_ = this->create_publisher<TwistStamped>("/cmd_vel", 10);

      pServerMoveTurtle_ = rclcpp_action::create_server<MoveTurtle>(this, "move_turtle",
                                                                    std::bind(&TurtlebotController::cbHandleGoal, this, _1, _2),
                                                                    std::bind(&TurtlebotController::cbHandleCancel, this, _1),
                                                                    std::bind(&TurtlebotController::cbHandleAccepted, this, _1),
                                                                    rcl_action_server_get_default_options(), pCallbackGroup_);

      RCLCPP_INFO(this->get_logger(), "Turtlebot Controller configured.");
      return rclcpp_lifecycle::LifecycleNode::on_configure(state);
    }

    LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &state)
    {
      pPubCmdVel_.reset();
      pServerMoveTurtle_.reset();

      RCLCPP_INFO(this->get_logger(), "Turtlebot Controller unconfigured.");
      return rclcpp_lifecycle::LifecycleNode::on_cleanup(state);
    }

    LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &state)
    {
      serverActivated_ = true;

      RCLCPP_INFO(this->get_logger(), "Turtlebot Controller activated.");
      return rclcpp_lifecycle::LifecycleNode::on_activate(state);
    }

    LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &state)
    {
      serverActivated_ = false;

      RCLCPP_INFO(this->get_logger(), "Turtlebot Controller deactivated.");
      return rclcpp_lifecycle::LifecycleNode::on_deactivate(state);
    }

    LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &state)
    {
      RCLCPP_INFO(this->get_logger(), "Turtlebot Controller shutting down.");

      serverActivated_ = false;

      pPubCmdVel_.reset();
      pServerMoveTurtle_.reset();

      return rclcpp_lifecycle::LifecycleNode::on_shutdown(state);
    }

  private:
    rclcpp_action::GoalResponse cbHandleGoal(const rclcpp_action::GoalUUID & /*uuid*/, MoveTurtle::Goal::ConstSharedPtr pGoal)
    {
      RCLCPP_INFO(this->get_logger(), "Received a goal.");

      if (!serverActivated_)
      {
        RCLCPP_INFO(this->get_logger(), "Goal rejected because server is inactive.");
        return rclcpp_action::GoalResponse::REJECT;
      }

      {
        std::lock_guard<std::mutex> lock(mutexGoalHandle_);
        if (pGoalHandle_ && pGoalHandle_->is_active())
        {
          RCLCPP_INFO(this->get_logger(), "Goal rejected because another goal is active.");
          return rclcpp_action::GoalResponse::REJECT;
        }
      }

      if (std::fabs(pGoal->twist.linear.x) > 3 || std::fabs(pGoal->twist.linear.y) > 3 || std::fabs(pGoal->twist.linear.z) > 3 ||
          std::fabs(pGoal->twist.angular.x) > 2 || std::fabs(pGoal->twist.angular.y) > 2 || std::fabs(pGoal->twist.angular.z) > 2)
      {
        RCLCPP_INFO(this->get_logger(), "Goal rejected because speed is too high.");
        return rclcpp_action::GoalResponse::REJECT;
      }

      if (pGoal->duration <= 0)
      {
        RCLCPP_INFO(this->get_logger(), "Goal rejected because duration is not positive.");
        return rclcpp_action::GoalResponse::REJECT;
      }

      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse cbHandleCancel(const std::shared_ptr<MoveTurtleGoalHandle> /*pGoalHandle*/)
    {
      RCLCPP_INFO(this->get_logger(), "Received a cancel request.");
      return rclcpp_action::CancelResponse::ACCEPT;
    }

    void cbHandleAccepted(const std::shared_ptr<MoveTurtleGoalHandle> pGoalHandle)
    {
      executeGoal(pGoalHandle);
    }

    void executeGoal(const std::shared_ptr<MoveTurtleGoalHandle> pGoalHandle)
    {
      {
        std::lock_guard<std::mutex> lock(mutexGoalHandle_);
        pGoalHandle_ = pGoalHandle;
      }

      const auto pGoal = pGoalHandle->get_goal();

      auto pResult = std::make_shared<MoveTurtle::Result>();

      auto start = now();
      rclcpp::Duration duration(pGoal->duration, 0);
      rclcpp::Rate rate(10);
      while (rclcpp::ok())
      {
        auto twist = TwistStamped();

        if ((now() - start) > duration)
        {
          twist.twist.linear.x = 0;
          twist.twist.angular.z = 0;
          pPubCmdVel_->publish(twist);

          pResult->success = true;
          pResult->message = "Success";
          pGoalHandle->succeed(pResult);

          RCLCPP_INFO(this->get_logger(), "Finished goal.");
          return;
        }

        if (!serverActivated_)
        {
          twist.twist.linear.x = 0;
          twist.twist.angular.z = 0;
          pPubCmdVel_->publish(twist);

          pResult->success = false;
          pResult->message = "Aborted";

          pGoalHandle->abort(pResult);
          RCLCPP_INFO(this->get_logger(), "Aborted goal.");
          return;
        }

        twist.twist = pGoal->twist;
        pPubCmdVel_->publish(twist);
        rate.sleep();
      }
    }

    rclcpp::Publisher<TwistStamped>::SharedPtr pPubCmdVel_;

    rclcpp_action::Server<MoveTurtle>::SharedPtr pServerMoveTurtle_;
    std::shared_ptr<MoveTurtleGoalHandle> pGoalHandle_;
    std::mutex mutexGoalHandle_;
    bool serverActivated_ = false;

    rclcpp::CallbackGroup::SharedPtr pCallbackGroup_;
  };
} // namespace final_project

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(final_project::TurtlebotController)
