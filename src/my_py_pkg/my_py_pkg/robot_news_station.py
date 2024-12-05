#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import String


class RobotNewsStation(Node):
    def __init__(self):
        super().__init__("robot_news_station")

        self.robot_name_ = "C3PO"
        self.publisher_ = self.create_publisher(String, "robot_news", 10)
        self.timer_ = self.create_timer(0.5, self.publish_news)
        self.get_logger().info("Robot News Publisher Initialized")

    def publish_news(self):
        msg = String()
        msg.data = "Hi this is..." + str(self.robot_name_)
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotNewsStation()
    rclpy.spin(node)
    node.destroy_node()  # Proper cleanup
    rclpy.shutdown()


if __name__ == "__main__":
    main()
