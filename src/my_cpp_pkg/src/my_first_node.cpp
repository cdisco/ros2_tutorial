#include <rclcpp/rclcpp.hpp>

class MyCppNode:public rclcpp::Node
{
    public:
        MyCppNode():Node("cpp_test"), counter_(0)
        {
            RCLCPP_INFO(this->get_logger(), "C++ OOP Constructor");

            timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&MyCppNode::timerCallback, this));
        }

    private:
        void timerCallback()
        {
            RCLCPP_INFO(this->get_logger(), "Inside callback timer, counter = %i", counter_);
            counter_++;
        }

        rclcpp::TimerBase::SharedPtr timer_;
        int counter_;
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MyCppNode>();

    RCLCPP_INFO(node->get_logger(), "Hello from CPP node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}