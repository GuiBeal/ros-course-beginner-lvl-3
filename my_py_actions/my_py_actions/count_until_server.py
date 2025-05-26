#!/usr/bin/env python3

from typing import List
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from my_robot_interfaces.action import CountUntil


class CountUntilServerNode(Node):
    def __init__(self):
        super().__init__("count_until_server")

        self.goal_handle_ = None
        self.goal_lock_ = threading.Lock()

        self.goal_queue_: List[ServerGoalHandle] = []

        self.count_until_server_ = ActionServer(
            node=self,
            action_type=CountUntil,
            action_name="count_until",
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info("Count Until Server started.")

    def goal_callback(self, goal: CountUntil.Goal):
        self.get_logger().info("Received a goal.")

        # # Policy: refuse new goal if another is active
        # with self.goal_lock_:
        #     if self.goal_handle_ is not None and self.goal_handle_.is_active:
        #         self.get_logger().info(
        #             "Rejecting goal because a goal is already active."
        #         )
        #         return GoalResponse.REJECT

        if goal.target <= 0:
            self.get_logger().info("Rejecting goal because target is not positive.")
            return GoalResponse.REJECT

        if goal.period <= 0:
            self.get_logger().info("Rejecting goal because period is not positive.")
            return GoalResponse.REJECT

        # # Policy: preempt previous goal when receiving new goal
        # with self.goal_lock_:
        #     if self.goal_handle_ is not None and self.goal_handle_.is_active:
        #         self.get_logger().info("Aborting previous goal.")
        #         self.goal_handle_.abort()

        self.get_logger().info("Accepting goal.")
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle: ServerGoalHandle):
        with self.goal_lock_:
            if self.goal_handle_ is not None:
                self.get_logger().info("Queuing goal.")
                self.goal_queue_.append(goal_handle)
            else:
                goal_handle.execute()

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Received a cancel request.")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        with self.goal_lock_:
            self.goal_handle_ = goal_handle

        request: CountUntil.Goal = goal_handle.request
        target = request.target
        period = request.period

        self.get_logger().info(f"Counting until {target} with period {period}.")

        result = CountUntil.Result()
        result.count = 0

        feedback = CountUntil.Feedback()
        feedback.count = result.count
        goal_handle.publish_feedback(feedback)

        for i in range(target):
            time.sleep(period)

            if not goal_handle.is_active:
                self.execute_next_goal()
                return result

            if goal_handle.is_cancel_requested:
                self.get_logger().info("Cancelling goal.")
                goal_handle.canceled()
                self.execute_next_goal()
                return result

            result.count += 1
            self.get_logger().info(f"Counted to {result.count}.")

            feedback.count = result.count
            goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        self.execute_next_goal()
        return result

    def execute_next_goal(self):
        with self.goal_lock_:
            if len(self.goal_queue_) > 0:
                self.get_logger().info("Executing next goal.")
                self.goal_queue_.pop(0).execute()
            else:
                self.goal_handle_ = None


def main(args=None):
    rclpy.init(args=args)

    node = CountUntilServerNode()
    rclpy.spin(node, MultiThreadedExecutor())

    rclpy.shutdown()


if __name__ == "main":
    main()
