#include <components_cpp/number_publisher.hpp>

using namespace std::chrono_literals;

namespace components_cpp
{
  NumberPublisher::NumberPublisher(const rclcpp::NodeOptions &options) : Node("number_publisher", options)
  {
    pNumberPublisher_ = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
    pNumberTimer_ = this->create_wall_timer(1000ms,
                                            std::bind(&NumberPublisher::publishNumber, this));
    RCLCPP_INFO(this->get_logger(), "Number publisher has been started.");
  }

  void NumberPublisher::publishNumber()
  {
    auto msg = example_interfaces::msg::Int64();
    msg.data = number_;
    pNumberPublisher_->publish(msg);
  }
} // namespace components_cpp

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(components_cpp::NumberPublisher)
