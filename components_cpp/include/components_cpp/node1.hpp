#include <rclcpp/rclcpp.hpp>

class Node1 : public rclcpp::Node
{
public:
  Node1();

private:
  void callbackTimer1();
  void callbackTimer2();
  void callbackTimer3();

  rclcpp::TimerBase::SharedPtr pTimer1_;
  rclcpp::TimerBase::SharedPtr pTimer2_;
  rclcpp::TimerBase::SharedPtr pTimer3_;
};
