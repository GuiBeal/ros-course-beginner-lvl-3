#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import time


class Node1(Node):
    def __init__(self):
        super().__init__("node1")

        self.cb_group_reentrant_ = ReentrantCallbackGroup()
        self.cb_group_mutually_exclusive_ = MutuallyExclusiveCallbackGroup()

        self.timer1_ = self.create_timer(
            1.0, self.callback_timer1, self.cb_group_mutually_exclusive_
        )
        self.timer2_ = self.create_timer(
            1.0, self.callback_timer2, self.cb_group_reentrant_
        )
        self.timer3_ = self.create_timer(
            1.0, self.callback_timer3, self.cb_group_reentrant_
        )

    def callback_timer1(self):
        time.sleep(2.0)
        self.get_logger().info("cb 1")

    def callback_timer2(self):
        time.sleep(2.0)
        self.get_logger().info("cb 2")

    def callback_timer3(self):
        time.sleep(2.0)
        self.get_logger().info("cb 3")


class Node2(Node):
    def __init__(self):
        super().__init__("node2")

        self.cb_group_mutually_exclusive_ = MutuallyExclusiveCallbackGroup()

        self.timer1_ = self.create_timer(
            1.0, self.callback_timer4, self.cb_group_mutually_exclusive_
        )
        self.timer3_ = self.create_timer(
            1.0, self.callback_timer5, self.cb_group_mutually_exclusive_
        )

    def callback_timer4(self):
        time.sleep(2.0)
        self.get_logger().info("cb 4")

    def callback_timer5(self):
        time.sleep(2.0)
        self.get_logger().info("cb 5")


def main(args=None):
    rclpy.init(args=args)

    node1 = Node1()
    node2 = Node2()

    executor = MultiThreadedExecutor()
    executor.add_node(node1)
    executor.add_node(node2)
    executor.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
