#include <components_cpp/node2.hpp>

#include <chrono>
#include <thread>

using namespace std::chrono_literals;

Node2::Node2() : Node("node2")
{
  this->pTimer4_ = this->create_wall_timer(1000ms, std::bind(&Node2::callbackTimer4, this));
  this->pTimer5_ = this->create_wall_timer(1000ms, std::bind(&Node2::callbackTimer5, this));
}

void Node2::callbackTimer4()
{
  std::this_thread::sleep_for(2000ms);
  RCLCPP_INFO(this->get_logger(), "cb 4");
}

void Node2::callbackTimer5()
{
  std::this_thread::sleep_for(2000ms);
  RCLCPP_INFO(this->get_logger(), "cb 5");
}
