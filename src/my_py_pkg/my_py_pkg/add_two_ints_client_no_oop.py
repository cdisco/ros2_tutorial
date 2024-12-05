#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts 
 
def main(args=None):
   rclpy.init(args=args)
   node = Node("add_two_ints_no_oop")
   
   client = node.create_client(AddTwoInts, "add_two_ints")
   
   # wait for server to be up
   while not client.wait_for_service(2.0):
       node.get_logger().warn("WARNING!  Waiting for server AddTwoInts")
   
   request = AddTwoInts.Request()
   request.a = 3
   request.b = 97

   future = client.call_async(request)
   
   rclpy.spin_until_future_complete(node, future)
   
   try:
        response = future.result()
        node.get_logger().info("request.a (" + str(request.a) + ") + request.b (" +str(request.b) + ") = " + str(response.sum))

   except Exception as e:
       node.get_logger().error(f"ERROR!  Service call failed {e}")

   node.get_logger().info("Response received....shuting down")
   
   rclpy.shutdown()
 
 
if __name__ == "__main__":
   main()
