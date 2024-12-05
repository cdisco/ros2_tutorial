#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp" 

class NumberPublisherNode : public rclcpp::Node
{
public:
   NumberPublisherNode() : Node("number_publisher"),number_(64)
   {
         publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
         timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&NumberPublisherNode::publish_number, this));
         RCLCPP_INFO(this->get_logger(), "Number publisher node initialized");
   }

private:
   void publish_number()
   {
      number_++;
      RCLCPP_INFO(this->get_logger(), "number is %li", number_);
      auto msg = example_interfaces::msg::Int64();
      msg.data = number_;
      publisher_->publish(msg);
   }

   // example_interfaces::msg::Int64 number_;
   int64_t number_;
   rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
   rclcpp::TimerBase::SharedPtr timer_;

};
 
int main(int argc, char **argv)
{
   rclcpp::init(argc, argv);
   auto node = std::make_shared<NumberPublisherNode>();
   rclcpp::spin(node);
   rclcpp::shutdown();
   return 0;
}