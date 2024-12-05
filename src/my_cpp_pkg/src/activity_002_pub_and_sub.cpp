#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/msg/bool.hpp"
#include "example_interfaces/srv/set_bool.hpp"
 
class NumberCounterNode : public rclcpp::Node
{
public:
   NumberCounterNode() : Node("number_counter"), counter_(0)
   {
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);
        subscriber_ = this->create_subscription<example_interfaces::msg::Int64>("number", 10, 
                        std::bind(&NumberCounterNode::callbackNumber, this,
                        std::placeholders::_1));
        // timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&NumberCounterNode::publish_count, this));

        server_ = this->create_service<example_interfaces::srv::SetBool>(
            "reset_counter",
            std::bind(&NumberCounterNode::callbackSetBool, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "NumberCounterNode created with reset service");
   }
 
private:

    void callbackSetBool(const example_interfaces::srv::SetBool_Request::SharedPtr request,
                        const example_interfaces::srv::SetBool_Response::SharedPtr response)
    {
        if(request->data == true)
        {
            response->message = "Counter was at " + this->counter_;
            response->success = true;
            this->counter_ = 0;
            RCLCPP_WARN(this->get_logger(), "WARNING!  Resetting Counter to zero");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Counter NOT reset to zero, value still %ld", this->counter_);
            response->success = false;
            response->message = "Counter was at " + std::to_string(this->counter_) + " and will remain so";
        }
    }

    void callbackNumber(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        this->counter_ += msg->data;
        RCLCPP_INFO(this->get_logger(), "counter value received = %li", msg->data);
        auto new_msg = example_interfaces::msg::Int64();
        new_msg.data = this->counter_;
        this->publisher_->publish(new_msg);
        RCLCPP_INFO(this->get_logger(), "counter value published = %li", new_msg.data);
    }

    // uncomment timer declaration/initialization above and following code if you want to have publisher
    // function separate.  Remove publish call from callbackNumber too
    // void publish_count()
    // {
    //     this->counter_++;
    //     auto msg = example_interfaces::msg::Int64();
    //     msg.data = this->counter_;
    //     publisher_.publish(.....);
    //     RCLCPP_INFO(this->get_logger(), "publishing number count = %li", msg.data);
    // }

    rclcpp::TimerBase::SharedPtr timer_;
    int64_t counter_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;

    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr server_;
};
 
int main(int argc, char **argv)
{
   rclcpp::init(argc, argv);
   auto node = std::make_shared<NumberCounterNode>();
   rclcpp::spin(node);
   rclcpp::shutdown();
   return 0;
}