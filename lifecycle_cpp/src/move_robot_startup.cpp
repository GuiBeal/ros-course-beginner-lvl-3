#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>

class MoveRobotStartup : public rclcpp::Node
{
public:
  MoveRobotStartup() : Node("move_robot_startup")
  {
    this->declare_parameter("managed_node_names", rclcpp::ParameterType::PARAMETER_STRING_ARRAY);
    auto nodeNames = this->get_parameter("managed_node_names").as_string_array();

    for (const auto &nodeName : nodeNames)
    {
      const std::string serviceName = "/" + nodeName + "/change_state";
      mapClients_[nodeName] = this->create_client<lifecycle_msgs::srv::ChangeState>(serviceName);
    }

    RCLCPP_INFO(this->get_logger(), "Move Robot Startup started.");
  }

  void changeState(const lifecycle_msgs::msg::Transition &transition)
  {
    for (const auto &client : mapClients_)
    {
      client.second->wait_for_service();
    }

    auto pRequest = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    pRequest->transition = transition;
    for (const auto &client : mapClients_)
    {
      const auto future = client.second->async_send_request(pRequest);
      rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
    }
  }

  void initialize()
  {
    auto transition = lifecycle_msgs::msg::Transition();

    transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
    transition.label = "configure";
    changeState(transition);

    transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
    transition.label = "activate";
    changeState(transition);
  }

private:
  std::map<std::string, rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr> mapClients_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto pNode = std::make_shared<MoveRobotStartup>();
  pNode->initialize();

  rclcpp::shutdown();
  return 0;
}