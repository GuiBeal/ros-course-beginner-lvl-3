#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>

class LifeCycleNodeManager : public rclcpp::Node
{
public:
    LifeCycleNodeManager() : Node("lifecycle_manager")
    {
        this->declare_parameter("managed_node_name", rclcpp::ParameterType::PARAMETER_STRING);
        const std::string nodeName = this->get_parameter("managed_node_name").as_string();

        const std::string serviceName = "/" + nodeName + "/change_state";
        pClient_ = this->create_client<lifecycle_msgs::srv::ChangeState>(serviceName);
    }

    void changeState(const lifecycle_msgs::msg::Transition &transition)
    {
        pClient_->wait_for_service();

        auto pRequest = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        pRequest->transition = transition;

        auto future = pClient_->async_send_request(pRequest);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
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
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr pClient_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto pNode = std::make_shared<LifeCycleNodeManager>();
    pNode->initialize();

    rclcpp::shutdown();
    return 0;
}