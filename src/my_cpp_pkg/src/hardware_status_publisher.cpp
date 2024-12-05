#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/hardware_status.hpp"
 
class HardwareStatusPublisherNode : public rclcpp::Node
{
public:
   HardwareStatusPublisherNode() : Node("hardware_status")
   {
    this->publisher_ = this->create_publisher<my_robot_interfaces::msg::HardwareStatus>("hardware_status", 10);
    this->publisher_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000), 
        std::bind(&HardwareStatusPublisherNode::publish_hardware_status,
        this));
    RCLCPP_INFO(this->get_logger(), "HardwareStatus Publisher Node initialized");
   }

private:
    rclcpp::Publisher<my_robot_interfaces::msg::HardwareStatus>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr publisher_timer_;

    void publish_hardware_status()
    {
        auto msg = my_robot_interfaces::msg::HardwareStatus();
        msg.temperature = 99;
        msg.are_motors_ready = false;
        msg.debug_msg = "motors are not ready!!!";

        this->publisher_->publish(msg);
    }
};
 
int main(int argc, char **argv)
{
   rclcpp::init(argc, argv);
   auto node = std::make_shared<HardwareStatusPublisherNode>();
   rclcpp::spin(node);
   rclcpp::shutdown();
   return 0;
}