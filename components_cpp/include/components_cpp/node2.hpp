#include <rclcpp/rclcpp.hpp>

class Node2 : public rclcpp::Node
{
public:
  Node2();

private:
  void callbackTimer4();
  void callbackTimer5();

  rclcpp::TimerBase::SharedPtr pTimer4_;
  rclcpp::TimerBase::SharedPtr pTimer5_;
};
