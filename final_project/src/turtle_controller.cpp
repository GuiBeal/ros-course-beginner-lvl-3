#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <turtlesim/srv/kill.hpp>
#include <turtlesim/srv/spawn.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <my_robot_interfaces/action/move_turtle.hpp>

using Twist = geometry_msgs::msg::Twist;
using MoveTurtle = my_robot_interfaces::action::MoveTurtle;
using MoveTurtleGoalHandle = rclcpp_action::ServerGoalHandle<MoveTurtle>;

using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace final_project
{
  class TurtleController : public rclcpp_lifecycle::LifecycleNode
  {
  public:
    TurtleController(const rclcpp::NodeOptions &options) : LifecycleNode("turtle_controller", options)
    {
      this->declare_parameter("turtle_name", rclcpp::PARAMETER_STRING);
      pCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

      RCLCPP_INFO(this->get_logger(), "Turtle Controller started.");
    }

    LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &state)
    {
      turtleName_ = this->get_parameter("turtle_name").as_string();

      pPubCmdVel_ = this->create_publisher<Twist>("/" + turtleName_ + "/cmd_vel", 10);

      pClientTurtleKill_ = this->create_client<turtlesim::srv::Kill>("/kill", rclcpp::ServicesQoS(), pCallbackGroup_);
      pClientTurtleSpawn_ = this->create_client<turtlesim::srv::Spawn>("/spawn", rclcpp::ServicesQoS(), pCallbackGroup_);

      pServerMoveTurtle_ = rclcpp_action::create_server<MoveTurtle>(this, turtleName_ + "/move_turtle",
                                                                    std::bind(&TurtleController::cbHandleGoal, this, _1, _2),
                                                                    std::bind(&TurtleController::cbHandleCancel, this, _1),
                                                                    std::bind(&TurtleController::cbHandleAccepted, this, _1),
                                                                    rcl_action_server_get_default_options(), pCallbackGroup_);

      spawnTurtle();

      RCLCPP_INFO(this->get_logger(), "Turtle Controller configured.");
      return rclcpp_lifecycle::LifecycleNode::on_configure(state);
    }

    LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &state)
    {
      killTurtle();

      pPubCmdVel_.reset();
      pClientTurtleKill_.reset();
      pClientTurtleSpawn_.reset();
      pServerMoveTurtle_.reset();

      RCLCPP_INFO(this->get_logger(), "Turtle Controller unconfigured.");
      return rclcpp_lifecycle::LifecycleNode::on_cleanup(state);
    }

    LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &state)
    {
      serverActivated_ = true;

      RCLCPP_INFO(this->get_logger(), "Turtle Controller activated.");
      return rclcpp_lifecycle::LifecycleNode::on_activate(state);
    }

    LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &state)
    {
      serverActivated_ = false;

      RCLCPP_INFO(this->get_logger(), "Turtle Controller deactivated.");
      return rclcpp_lifecycle::LifecycleNode::on_deactivate(state);
    }

    LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &state)
    {
      RCLCPP_INFO(this->get_logger(), "Turtle Controller shutting down.");

      serverActivated_ = false;
      killTurtle();

      pPubCmdVel_.reset();
      pClientTurtleKill_.reset();
      pClientTurtleSpawn_.reset();
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
          std::fabs(pGoal->twist.angular.x) > 3 || std::fabs(pGoal->twist.angular.y) > 3 || std::fabs(pGoal->twist.angular.z) > 3)
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
        if ((now() - start) > duration)
        {
          auto twist = Twist();
          twist.linear.x = 0;
          twist.angular.z = 0;
          pPubCmdVel_->publish(twist);

          pResult->success = true;
          pResult->message = "Success";
          pGoalHandle->succeed(pResult);

          RCLCPP_INFO(this->get_logger(), "Finished goal.");
          return;
        }

        if (!serverActivated_)
        {
          auto twist = Twist();
          twist.linear.x = 0;
          twist.angular.z = 0;
          pPubCmdVel_->publish(twist);

          pResult->success = false;
          pResult->message = "Aborted";

          pGoalHandle->abort(pResult);
          RCLCPP_INFO(this->get_logger(), "Aborted goal.");
          return;
        }

        pPubCmdVel_->publish(pGoal->twist);
        rate.sleep();
      }
    }

    void killTurtle()
    {
      pClientTurtleKill_->wait_for_service();

      auto pRequest = std::make_shared<turtlesim::srv::Kill::Request>();
      pRequest->name = turtleName_;

      auto futureResult = pClientTurtleKill_->async_send_request(pRequest);
      futureResult.wait();
      RCLCPP_INFO(this->get_logger(), "Killed turtle named %s.", turtleName_.c_str());
    }

    void spawnTurtle()
    {
      pClientTurtleSpawn_->wait_for_service();

      auto pRequest = std::make_shared<turtlesim::srv::Spawn::Request>();
      pRequest->name = turtleName_;
      pRequest->x = 5;
      pRequest->y = 5;
      pRequest->theta = 0;

      auto futureResult = pClientTurtleSpawn_->async_send_request(pRequest);
      auto pResult = futureResult.get();
      RCLCPP_INFO(this->get_logger(), "Spawned turtle named %s.", pResult->name.c_str());
    }

    std::string turtleName_;

    rclcpp::Publisher<Twist>::SharedPtr pPubCmdVel_;

    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr pClientTurtleKill_;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr pClientTurtleSpawn_;

    rclcpp_action::Server<MoveTurtle>::SharedPtr pServerMoveTurtle_;
    std::shared_ptr<MoveTurtleGoalHandle> pGoalHandle_;
    std::mutex mutexGoalHandle_;
    bool serverActivated_ = false;

    rclcpp::CallbackGroup::SharedPtr pCallbackGroup_;
  };
} // namespace final_project

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(final_project::TurtleController)
