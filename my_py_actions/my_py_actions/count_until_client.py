#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus

from my_robot_interfaces.action import CountUntil


class CountUntilClientNode(Node):
    def __init__(self):
        super().__init__("count_until_client")

        self.count_until_client_ = ActionClient(
            node=self, action_type=CountUntil, action_name="count_until"
        )

        self.get_logger().info("Count Until Client started.")

    def send_goal(self, target, period):
        self.count_until_client_.wait_for_server()

        goal = CountUntil.Goal()
        goal.target = target
        goal.period = period

        self.get_logger().info("Sending goal.")
        self.count_until_client_.send_goal_async(
            goal, feedback_callback=self.goal_feedback_callback
        ).add_done_callback(self.goal_response_callback)

        # self.timer_ = self.create_timer(2.0, self.cancel_goal)

    # def cancel_goal(self):
    #     self.get_logger().info("Sending cancel request.")
    #     self.goal_handle_.cancel_goal_async()
    #     self.timer_.cancel()

    def goal_response_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal accepted.")
            self.goal_handle_.get_result_async().add_done_callback(
                self.goal_result_callback
            )
        else:
            self.get_logger().warn("Goal rejected.")

    def goal_result_callback(self, future):
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal succeeded.")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Goal aborted.")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Goal canceled.")

        result: CountUntil.Result = future.result().result
        self.get_logger().info(f"Count result is {result.count}.")

    def goal_feedback_callback(self, feedback_msg):
        feedback: CountUntil.Feedback = feedback_msg.feedback
        self.get_logger().info(f"Got feedback of count to {feedback.count}.")


def main(args=None):
    rclpy.init(args=args)

    node = CountUntilClientNode()
    node.send_goal(5, 1.0)
    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "main":
    main()
