#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("py_test")
        self.counter_ = 0
        self.get_logger().info("OOP world")
        self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        self.counter_ += 2
        self.get_logger().info("In timer callback, counter = " + str(self.counter_))



def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    node.get_logger().info("Hello ROS 2 from Python")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
