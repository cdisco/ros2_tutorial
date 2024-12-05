#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
 
 
class NumberPublisherNode(Node):
   def __init__(self):
       super().__init__("number_publisher")
       self.publisher_ = self.create_publisher(Int64, "number", 10)
       self.timer = self.create_timer(0.2, self.publish_number)
       self.counter_ = 0
       self.get_logger().info("Initialized Number Publishing node (PYTHON)")
 
   def publish_number(self):
       self.counter_ += 1
       msg = Int64()
       msg.data = self.counter_
       self.publisher_.publish(msg)
       self.get_logger().info(f"Publishing number = {self.counter_}")

        
 
def main(args=None):
   rclpy.init(args=args)
   node = NumberPublisherNode()
   rclpy.spin(node)
   rclpy.shutdown()
 
 
if __name__ == "__main__":
   main()