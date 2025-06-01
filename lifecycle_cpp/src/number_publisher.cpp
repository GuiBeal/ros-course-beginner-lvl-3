#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "example_interfaces/msg/int64.hpp"

using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class NumberPublisherNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  NumberPublisherNode() : LifecycleNode("number_publisher")
  {

    RCLCPP_INFO(this->get_logger(), "Number publisher started.");
  }

  LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &state)
  {
    configure();
    RCLCPP_INFO(this->get_logger(), "Number publisher configured.");
    return rclcpp_lifecycle::LifecycleNode::on_configure(state);
  }

  LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &state)
  {
    cleanup();
    RCLCPP_INFO(this->get_logger(), "Number publisher unconfigured.");
    return rclcpp_lifecycle::LifecycleNode::on_cleanup(state);
  }

  LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &state)
  {
    activate();
    RCLCPP_INFO(this->get_logger(), "Number publisher activated.");
    return rclcpp_lifecycle::LifecycleNode::on_activate(state);
  }

  LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &state)
  {
    deactivate();
    RCLCPP_INFO(this->get_logger(), "Number publisher deactivated.");
    return rclcpp_lifecycle::LifecycleNode::on_deactivate(state);
  }

  LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &state)
  {
    deactivate();
    cleanup();
    RCLCPP_INFO(this->get_logger(), "Number publisher shutting down.");
    return rclcpp_lifecycle::LifecycleNode::on_shutdown(state);
  }

  LifecycleCallbackReturn on_error(const rclcpp_lifecycle::State &state)
  {
    deactivate();
    cleanup();
    RCLCPP_INFO(this->get_logger(), "Number publisher on error.");
    return rclcpp_lifecycle::LifecycleNode::on_shutdown(state);
  }

private:
  void configure()
  {
    number_ = 1;
    pNumberPublisher_ = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
    pNumberTimer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000 / publishFrequency_)),
                                            std::bind(&NumberPublisherNode::publishNumber, this));
    pNumberTimer_->cancel();
  }

  void cleanup()
  {
    pNumberPublisher_.reset();
    pNumberTimer_.reset();
  }

  void activate()
  {
    if (pNumberTimer_)
      pNumberTimer_->reset();
  }

  void deactivate()
  {
    if (pNumberTimer_)
      pNumberTimer_->cancel();
  }

  void publishNumber()
  {
    auto msg = example_interfaces::msg::Int64();
    msg.data = number_++;
    pNumberPublisher_->publish(msg);
  }

  int number_ = 1;

  rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr pNumberPublisher_;
  rclcpp::TimerBase::SharedPtr pNumberTimer_;
  const double publishFrequency_ = 1.0;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto pNode = std::make_shared<NumberPublisherNode>();
  rclcpp::spin(pNode->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}
