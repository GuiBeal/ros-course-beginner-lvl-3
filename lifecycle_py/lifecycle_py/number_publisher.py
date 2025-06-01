#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn

from example_interfaces.msg import Int64


class NumberPublisherNode(LifecycleNode):
    def __init__(self):
        super().__init__("number_publisher")
        self.number_ = 1
        self.publish_frequency_ = 1.0

        self.number_publisher_ = None
        self.number_timer_ = None

        self.get_logger().info("Number publisher started.")

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.configure()
        self.get_logger().info("Number publisher configured.")
        return super().on_configure(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.cleanup()
        self.get_logger().info("Number publisher unconfigured.")
        return super().on_cleanup(state)

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.activate()
        self.get_logger().info("Number publisher activated.")
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.deactivate()
        self.get_logger().info("Number publisher deactivated.")
        return super().on_deactivate(state)

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.deactivate()
        self.cleanup()
        self.get_logger().info("Number publisher shutting down.")
        return super().on_shutdown(state)

    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Number publisher on error.")
        self.deactivate()
        self.cleanup()
        return super().on_error(state)

    def configure(self):
        self.number_ = 1
        self.number_publisher_ = self.create_lifecycle_publisher(Int64, "number", 10)
        self.number_timer_ = self.create_timer(
            timer_period_sec=1.0 / self.publish_frequency_,
            callback=self.publish_number,
            autostart=False,
        )

    def cleanup(self):
        if self.number_publisher_ is not None:
            self.destroy_lifecycle_publisher(self.number_publisher_)
        if self.number_timer_ is not None:
            self.destroy_timer(self.number_timer_)

    def activate(self):
        if self.number_timer_:
            self.number_timer_.reset()

    def deactivate(self):
        if self.number_timer_:
            self.number_timer_.cancel()

    def publish_number(self):
        assert self.number_publisher_ is not None
        msg = Int64()
        msg.data = self.number_
        self.number_publisher_.publish(msg)
        self.number_ += 1


def main(args=None):
    rclpy.init(args=args)

    node = NumberPublisherNode()
    rclpy.spin(node)

    rclpy.shutdown()
